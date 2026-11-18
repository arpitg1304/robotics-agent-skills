# Lessons: ARMATURE bench rigs

Facts learned while building `examples/pick_place`, each from a measured run. Ten dev-range
seeds (1-10) per configuration unless noted.

## Both rigs

- **The tracker reports in its own frame.** Apply the `mount` transform from its descriptor
  (`world_T_tracker`, rpy applied Z-Y-X). With it applied, claimed object positions stay
  within about 1 mm of ground truth.
- **First contact is not a grasp.** Lifting as soon as `object_detected` turns true leaves
  the jaw still closing and the part sliding. Waiting out the full jaw travel raised SO-101
  `transfer-blocks` from 8/10 to 10/10 and Panda `stack-blocks` from 7/10 to 9/10.
- **Five seeds are too few to rank changes.** Results on seeds 1-5 and 1-10 disagreed about
  which variant was better more than once; compare on ten before keeping a change.

## SO-101

- **Parts creep in the jaw while held.** The tool-to-part offset grows through lift,
  transfer and lower (about 3 mm, 8 mm, 13 mm along z in one traced episode), and the part
  sags roughly 14 mm below the tool. Hold time is the cost: anything that lengthens the
  carry makes placement worse.
- **Measuring the part in hand before placing made things worse**, 10/10 down to 5/10 on
  `transfer-blocks`, because the measurement adds hold time. Slowing the carry also hurt.
- **Neighbours get knocked.** Parts and targets are 40 mm apart and blocks are 26 mm wide.
  Grasping a part next to an occupied target often displaces the placed part (`disturbed`).
  This is what limits `transfer-crossed`.
- **Stacking is unsolved** (0/10). Untested ideas: a narrower pre-grasp opening, placing
  from the side away from neighbours, releasing a few millimetres above the stack.

## Panda

- **In-hand measurement helps when stacking.** Compensating the measured offset took
  `stack-blocks` from 9/10 to 10/10. For bench placements it brought no gain.
