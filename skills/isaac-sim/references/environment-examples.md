# Environment Examples: Manager-Based Task and Direct Workflow

Moved from SKILL.md. Verified against Isaac Lab 3.0.0 (tag `v3.0.0`) / Isaac Sim 6.2.0,
2026-11-18. Not executed: examples are checked against source, not run.

## Compact manager-based task (verified against 3.0 source)

```python
# my_tasks/cartpole_upright/env_cfg.py
import math
import torch

import isaaclab.envs.mdp as mdp
import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.envs import ManagerBasedEnv, ManagerBasedRLEnvCfg
from isaaclab.managers import EventTermCfg as EventTerm, ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm, RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg, TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.utils import configclass, replace
from isaaclab.utils.math import wrap_to_pi
from isaaclab.utils.noise import UniformNoiseCfg as Unoise
from isaaclab_assets.robots.cartpole import CARTPOLE_CFG
from isaaclab_newton.physics import MJWarpSolverCfg, NewtonCfg
from isaaclab_physx.physics import PhysxCfg
from isaaclab_tasks.utils import PresetCfg

POLE = SceneEntityCfg("robot", joint_names=["cart_to_pole"])
CART = SceneEntityCfg("robot", joint_names=["slider_to_cart"])


def pole_upright(env: ManagerBasedEnv, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """Custom term: (num_envs,) tensor in [0, 1]. Asset data is a ProxyArray -> use .torch."""
    q = wrap_to_pi(env.scene[asset_cfg.name].data.joint_pos.torch[:, asset_cfg.joint_ids])
    return torch.exp(-torch.sum(q**2, dim=1) / 0.1)


@configclass
class PhysicsCfg(PresetCfg):            # selectable with physics=<field name>
    isaacsim_physx: PhysxCfg = PhysxCfg()                                  # needs --extra isaacsim
    newton_mjwarp: NewtonCfg = NewtonCfg(solver_cfg=MJWarpSolverCfg())    # kit-less
    default: PhysxCfg = isaacsim_physx


@configclass
class SceneCfg(InteractiveSceneCfg):
    ground = AssetBaseCfg(prim_path="/World/ground", spawn=sim_utils.GroundPlaneCfg())
    robot: ArticulationCfg = replace(CARTPOLE_CFG, prim_path="{ENV_REGEX_NS}/Robot")
    light = AssetBaseCfg(prim_path="/World/light", spawn=sim_utils.DistantLightCfg(intensity=2000.0))


@configclass
class ActionsCfg:
    effort = mdp.JointEffortActionCfg(asset_name="robot", joint_names=["slider_to_cart"], scale=100.0)


@configclass
class ObservationsCfg:
    @configclass
    class PolicyCfg(ObsGroup):
        joint_pos = ObsTerm(func=mdp.joint_pos_rel, noise=Unoise(n_min=-0.01, n_max=0.01))
        joint_vel = ObsTerm(func=mdp.joint_vel_rel, noise=Unoise(n_min=-0.05, n_max=0.05))

        def __post_init__(self):
            self.enable_corruption = True      # noise is applied only when this is True
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()


@configclass
class EventCfg:
    reset_cart = EventTerm(func=mdp.reset_joints_by_offset, mode="reset",
                           params={"asset_cfg": CART, "position_range": (-1.0, 1.0), "velocity_range": (-0.5, 0.5)})
    reset_pole = EventTerm(func=mdp.reset_joints_by_offset, mode="reset",
                           params={"asset_cfg": POLE, "position_range": (-0.25 * math.pi, 0.25 * math.pi),
                                   "velocity_range": (-0.25 * math.pi, 0.25 * math.pi)})


@configclass
class RewardsCfg:
    alive = RewTerm(func=mdp.is_alive, weight=1.0)
    terminating = RewTerm(func=mdp.is_terminated, weight=-2.0)
    upright = RewTerm(func=pole_upright, weight=1.0, params={"asset_cfg": POLE})
    cart_vel = RewTerm(func=mdp.joint_vel_l1, weight=-0.01, params={"asset_cfg": CART})


@configclass
class TerminationsCfg:
    time_out = DoneTerm(func=mdp.time_out, time_out=True)    # truncation, not failure
    cart_out = DoneTerm(func=mdp.joint_pos_out_of_manual_limit, params={"asset_cfg": CART, "bounds": (-3.0, 3.0)})


@configclass
class CartpoleUprightEnvCfg(ManagerBasedRLEnvCfg):
    scene: SceneCfg = SceneCfg(num_envs=4096, env_spacing=4.0)
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    events: EventCfg = EventCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()

    def __post_init__(self):
        self.decimation = 2                 # policy at 60 Hz
        self.episode_length_s = 5.0
        self.sim.dt = 1 / 120               # physics at 120 Hz
        self.sim.render_interval = self.decimation
        self.sim.physics = PhysicsCfg()
```

```python
# my_tasks/cartpole_upright/__init__.py
import gymnasium as gym

gym.register(
    id="MyOrg-Cartpole-Upright",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.env_cfg:CartpoleUprightEnvCfg",
        "rsl_rl_cfg_entry_point": "isaaclab_tasks.core.cartpole.agents.rsl_rl_ppo_cfg:CartpolePPORunnerCfg",
        "default_agent": "rsl_rl",
    },
)
```

Make `isaaclab train` find the task. Scaffold with `uv run isaaclab --new` (template
generator), or declare the entry point in your package's `pyproject.toml` and reinstall:

```toml
[project.entry-points."isaaclab.tasks"]
my_tasks = "my_tasks"
```

Then check with `uv run isaaclab list_envs --keyword Upright` and
`uv run isaaclab zero_agent --task MyOrg-Cartpole-Upright --num_envs 4`.

Rules this example encodes:

- `sim.dt * decimation` is the control period. Set `render_interval = decimation` so cameras
  and visualizers don't render every physics step.
- Mark time limits `time_out=True` so they are truncations, not failures. The RL side is in
  `robot-reinforcement-learning`.
- Every term function returns a `(num_envs,)` or `(num_envs, k)` torch tensor on `env.device`.
  No Python loops over envs and no `.item()` in the step path, because each sync stalls the GPU.
- Resolve joint/body names once through `SceneEntityCfg(..., joint_names=...)`. Do not
  hard-code indices, because joint order can differ between PhysX and Newton.

## Direct workflow essentials

```python
from isaaclab.envs import DirectRLEnv

class MyEnv(DirectRLEnv):
    cfg: "MyEnvCfg"          # skeleton: DirectRLEnvCfg with action_space, observation_space, scene, sim

    def __init__(self, cfg, render_mode=None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        self.robot = self.scene["robot"]
        self._joint_ids, _ = self.robot.find_joints(["slider_to_cart"])

    def _pre_physics_step(self, actions):  self.actions = actions.clone() * self.cfg.action_scale
    def _apply_action(self):                # called `decimation` times per env step
        self.robot.actuators.target_command.set_effort_index(value=self.actions, joint_ids=self._joint_ids)
    def _get_observations(self):            return {"policy": self.robot.data.joint_pos.torch.clone()}
    def _get_rewards(self):                 ...
    def _get_dones(self):                   return terminated, self.episode_length_buf >= self.max_episode_length
    def _reset_idx(self, env_ids):
        super()._reset_idx(env_ids)
        pose = self.robot.data.default_root_pose.torch[env_ids].clone()
        pose[:, :3] += self.scene.env_origins[env_ids]       # default poses are env-local
        self.robot.write_root_pose_to_sim_index(root_pose=pose, env_ids=env_ids)
```

In 3.0, writes are `write_*_to_sim_index(..., env_ids=)` for partial data and
`write_*_to_sim_mask(..., env_mask=)` for full data. Joint targets go through
`robot.actuators.target_command.set_{position,velocity,effort}_index`. The old
`robot.set_joint_*_target*` still works but warns. The stock 3.0 Cartpole-Direct still calls
it, so don't copy that line.
