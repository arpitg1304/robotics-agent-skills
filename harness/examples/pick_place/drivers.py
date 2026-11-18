"""Device drivers for the pick-and-place example.

Each driver wraps one device on the rig bus and takes everything robot specific
from that device's descriptor, so the application runs unchanged on other rigs.
"""

import math


def _rotation(rpy):
    """Rotation matrix for roll, pitch, yaw applied Z-Y-X: R = Rz(yaw) Ry(pitch) Rx(roll)."""
    (cr, sr), (cp, sp), (cy, sy) = ((math.cos(a), math.sin(a)) for a in rpy)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


class Tracker:
    """Object tracker. Readings arrive in the tracker frame, below the control rate, with noise."""

    def __init__(self, rig):
        self.rig = rig
        mount = rig.devices["tracker"]["mount"]
        self.rotation = _rotation(mount["rpy"])
        self.origin = mount["xyz"]
        self.period = max(1, round(rig.devices["arm"]["control_hz"] / rig.devices["tracker"]["rate_hz"]))

    def _to_world(self, p):
        return [sum(self.rotation[i][j] * p[j] for j in range(3)) + self.origin[i] for i in range(3)]

    def locate(self, tracker_id, samples=5):
        """World position of an object, averaged over fresh samples. Advances time while waiting."""
        total, taken, last_stamp = [0.0, 0.0, 0.0], 0, None
        while taken < samples:
            sample = self.rig.read("tracker")
            if sample["stamp"] != last_stamp:
                last_stamp = sample["stamp"]
                position = next(o["position"] for o in sample["objects"] if o["id"] == tracker_id)
                total = [t + p for t, p in zip(total, self._to_world(position))]
                taken += 1
            if taken < samples:
                self.rig.step(self.period)
        position = [t / samples for t in total]
        self.rig.claim("object_position", id=tracker_id, position=position)
        return position


class Arm:
    """Cartesian moves on top of the arm's joint-position interface and controller IK."""

    def __init__(self, rig):
        self.rig = rig
        self.hz = rig.devices["arm"]["control_hz"]

    def tcp(self):
        return self.rig.read("arm")["tcp"]

    def move_to(self, target, seconds, tolerance=0.004, settle_ticks=20):
        """Straight-line move with smoothstep timing, then wait for the servo to converge."""
        start = self.tcp()
        ticks = max(1, round(seconds * self.hz))
        for i in range(1, ticks + 1):
            s = i / ticks
            s = s * s * (3 - 2 * s)
            waypoint = [a + (b - a) * s for a, b in zip(start, target)]
            self.rig.write("arm", position=self.rig.call("arm", "ik", position=waypoint)["position"])
            self.rig.step()
        for _ in range(settle_ticks):
            if math.dist(self.tcp(), target) < tolerance:
                break
            self.rig.step()
        return math.dist(self.tcp(), target)


class Gripper:
    def __init__(self, rig):
        self.rig = rig
        self.spec = rig.devices["gripper"]
        self.hz = rig.devices["arm"]["control_hz"]

    def _travel_ticks(self, target):
        distance = abs(self.rig.read("gripper")["position"] - target)
        return math.ceil(distance / self.spec["speed"] * self.hz) + 5

    def open(self):
        self.rig.write("gripper", position=self.spec["open"])
        self.rig.step(self._travel_ticks(self.spec["open"]))

    def grasp(self):
        """Close on the part and let the grip settle. Returns False if nothing was caught.

        First contact is not a grasp: the jaw is still travelling, and a part lifted
        at that moment keeps sliding between the fingers for the rest of the move.
        """
        self.rig.write("gripper", position=self.spec["closed"])
        self.rig.step(self._travel_ticks(self.spec["closed"]))
        return self.rig.read("gripper")["object_detected"]
