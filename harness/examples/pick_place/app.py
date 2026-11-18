"""Pick-and-place over the rig device bus.

Runs on any rig exposing `arm` (with an `ik` service), `gripper` and `tracker`,
for tasks whose goal is a set of at(object, target) relations.
"""

from drivers import Arm, Gripper, Tracker

GRASP_DEPTH_M = 0.004  # grasp slightly below the object centre


def run(rig):
    arm, gripper, tracker = Arm(rig), Gripper(rig), Tracker(rig)
    layout = rig.task["layout"]
    clearance = layout["clearance_height_m"]
    parts = {part["name"]: part for part in layout["parts"]}
    targets = {target["name"]: target["position"] for target in layout["targets"]}

    for relation in layout["goal"]:
        if relation["rel"] != "at":
            raise NotImplementedError(f"This app only handles 'at' goals, not {relation['rel']}.")
        part, goal = parts[relation["a"]], targets[relation["b"]]
        name = part["name"]

        rig.mark(f"locate {name}")
        x, y, z = tracker.locate(part["tracker_id"])

        rig.mark(f"approach {name}")
        gripper.open()
        arm.move_to([x, clearance, z], seconds=1.5)
        rig.mark(f"descend {name}")
        arm.move_to([x, y - GRASP_DEPTH_M, z], seconds=2.0)

        rig.mark(f"grasp {name}")
        holding = gripper.grasp()
        rig.claim("holding", id=part["tracker_id"] if holding else None)
        if not holding:
            # Nothing between the fingers: back off and go on to the next part.
            gripper.open()
            arm.move_to([x, clearance, z], seconds=1.5)
            continue

        # Carry high enough to clear whatever already sits at the destination.
        carry = max(clearance, goal[1] + part["height_m"] + 0.03)
        rig.mark(f"lift {name}")
        arm.move_to([x, carry, z], seconds=2.0)
        rig.mark(f"transfer {name}")
        arm.move_to([goal[0], carry, goal[2]], seconds=2.0)
        dx, dy, dz = 0.0, -GRASP_DEPTH_M, 0.0
        if goal[1] > part["height_m"]:
            # Placing on top of another part needs the part, not the tool, on the target:
            # measure where it sits in the fingers. On the bench the extra hold time costs
            # more than it gains (see LESSONS.md).
            rig.mark(f"in-hand check {name}")
            held = tracker.locate(part["tracker_id"], samples=3)
            dx, dy, dz = (t - h for t, h in zip(arm.tcp(), held))
        rig.mark(f"lower {name}")
        arm.move_to([goal[0] + dx, goal[1] + dy, goal[2] + dz], seconds=2.0)

        rig.mark(f"release {name}")
        gripper.open()
        rig.mark(f"withdraw {name}")
        arm.move_to([goal[0], carry, goal[2]], seconds=1.5)

    rig.mark("home")
    arm.move_to(layout["home_tcp"], seconds=1.5)
    rig.mark("settle")
    while not rig.done:
        rig.step()
