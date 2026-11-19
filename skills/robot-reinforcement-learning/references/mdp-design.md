# MDP Design Tables: Observations and Action Spaces

Moved from SKILL.md section 2. The rules that follow each table stay in SKILL.md.

## Observations

| Group | Typical contents | Rules |
|---|---|---|
| Proprioception | Joint positions relative to default pose, joint velocities, base angular velocity, projected gravity, previous action | Express in body frame; include the previous action so the policy can reason about its own command |
| Task / command | Goal pose relative to the robot, velocity command, object pose relative to gripper | Relative, not world-frame: world coordinates make the policy memorise positions |
| Exteroception | Height scan, depth, camera images | Must exist on the real robot at the same rate and latency |
| History | Last N proprio frames or a recurrent state | Needed when latency, contact state or dynamics parameters are unobserved |
| Privileged (critic or teacher only) | Friction, masses, true base velocity, contact forces, terrain heights | Never in the deployed actor unless the robot estimates it |

## Action Spaces

| Action space | Use for | Pros | Cons |
|---|---|---|---|
| Joint position target `q_default + scale * a`, PD at higher rate | Default for legged and humanoid | Stable exploration, transfers well; Isaac Lab velocity task uses `scale=0.5` | Policy cannot command torque directly; PD gains become part of the MDP |
| Joint position delta from current `q` | Arms, slow tasks | Small steps are safe | Drift; integral behaviour harder to learn |
| Joint torque | High-rate, well-identified actuators | Full authority | Needs high control rate and accurate actuator model; transfers worst |
| End-effector delta (pose twist) via IK / impedance / OSC | Real-robot manipulation RL | Safe workspace bounds, small action dim; HIL-SERL uses EE twists with an impedance controller | Controller quality limits the policy |
| Discrete gripper open/close | Grasping | Avoids chattering half-closed grippers | HIL-SERL uses a separate DQN-style critic for it |
