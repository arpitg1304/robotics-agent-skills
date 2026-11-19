# Reward Design: Template and Reward-Hacking Examples

Moved from SKILL.md section 3. The reward structure, sign/termination table and detection
habits stay in SKILL.md.

## Template: Isaac Lab Velocity Task

The Isaac Lab velocity task is a good template: `track_lin_vel_xy_exp` (weight 1.0,
std sqrt(0.25)), `track_ang_vel_z_exp` (0.5), and penalties such as `lin_vel_z_l2` (-2.0),
`action_rate_l2` (-0.01), `dof_torques_l2` (-1e-5), `dof_acc_l2` (-2.5e-7).

## Reward Hacking: Examples and Detection

| Hack | Reward that invited it | Detection |
|---|---|---|
| Feet vibrating or tapping to farm a feet-air-time term | Air-time reward without contact-force or slip penalty | Video; foot contact frequency far above gait frequency |
| High-frequency jitter that exploits contact solver penetration | Velocity tracking with weak action-rate penalty | Action spectrum; joint power; replay at lower sim dt |
| Knocking the object off the table | "Object moved toward goal" without bounds | Object termination counts; final-state images |
| Pushing the object with the arm instead of grasping | Object-pose reward only | Gripper-contact statistics |
| Occluding the camera with the gripper | Image success classifier with no hard negatives | Classifier score vs independent check; review positives |
| Standing still | Penalties dominate a weak task term | Task term near zero while penalties shrink |
| Walking on knees, dragging a link | No undesired-contact penalty | Contact sensor on non-foot bodies (Isaac Lab `undesired_contacts`) |
