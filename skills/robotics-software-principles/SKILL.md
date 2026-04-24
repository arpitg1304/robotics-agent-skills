---
name: robotics-software-principles
description: "Applies foundational software design principles to robotics module development. Use this skill when designing robot software modules, structuring codebases, making architecture decisions, reviewing robotics code, or building reusable robotics libraries. Trigger whenever the user mentions SOLID principles for robots, modular robotics software, clean architecture for robots, dependency injection in robotics, interface design for hardware, real-time design constraints, error handling strategies for robots, configuration management, separation of concerns in perception-planning-control, composability of robot behaviors, or software craftsmanship in a robotics context. Also trigger for code reviews of robotics code, refactoring robot software, or designing APIs for robotics libraries."
---

# Robotics Software Design Principles

## Why Robotics Software Is Different

Robotics code operates under constraints that most software never faces:

1. **Physical consequences** — bugs crash robots into walls, not just processes
2. **Real-time deadlines** — missing a 1ms control loop causes oscillation or damage
3. **Sensor uncertainty** — all inputs are noisy, delayed, and occasionally wrong
4. **Hardware diversity** — same algorithm must work across vendors and form factors
5. **Sim-to-real gap** — code must run identically in simulation and on real hardware
6. **Long-running operation** — robots run for hours/days; leaks and drift matter
7. **Safety criticality** — some failures must never happen regardless of software state

These constraints demand disciplined design. The 12 principles below address them directly.

---

## The 12 Principles

### Principle 1: Single Responsibility — One Module, One Job

Each module should have exactly one reason to change. In robotics, coupling perception and control means a camera driver update can break your arm controller — unacceptable in safety-critical systems.

See [detailed examples and code](references/design-principles-detailed.md#principle-1-single-responsibility--one-module-one-job)

### Principle 2: Dependency Inversion — Depend on Abstractions, Not Hardware

High-level modules (planning, behavior) depend on abstract interfaces, not drivers. This is the foundation of sim-to-real: if your planner imports `UR5Driver`, it cannot run in simulation; if it depends on `ArmInterface`, you swap implementations freely.

```python
class ArmInterface(ABC):
    @abstractmethod
    def get_joint_positions(self) -> np.ndarray: ...
    @abstractmethod
    def move_to_joints(self, positions: np.ndarray, velocity: float = 0.5) -> bool: ...
    @abstractmethod
    def stop(self) -> None: ...

class PickPlaceTask:
    """Works with ANY arm — never knows if it's sim or real."""
    def __init__(self, arm: ArmInterface, gripper: GripperInterface):
        self.arm = arm
        self.gripper = gripper
```

See [detailed examples and code](references/design-principles-detailed.md#principle-2-dependency-inversion--depend-on-abstractions-not-hardware)

### Principle 3: Open-Closed — Extend Without Modifying

Add new sensors, robots, or tasks by adding code, not modifying existing modules. Use plugin architectures so adding OAK-D support never touches the core perception pipeline.

See [detailed examples and code](references/design-principles-detailed.md#principle-3-open-closed--extend-without-modifying)

### Principle 4: Interface Segregation — Small, Focused Interfaces

A 1-DOF gripper should not implement a 6-DOF hand interface. Segregate into `RGBCamera`, `DepthCamera`, `PTZCamera` so each device implements only what it supports.

See [detailed examples and code](references/design-principles-detailed.md#principle-4-interface-segregation--small-focused-interfaces)

### Principle 5: Liskov Substitution — Replaceable Implementations

Every implementation of an interface must be fully substitutable. Make implementations self-describing (expose `num_joints`, `joint_limits`) so callers never hard-code hardware assumptions.

See [detailed examples and code](references/design-principles-detailed.md#principle-5-liskov-substitution--replaceable-implementations)

### Principle 6: Separation of Rates — Respect Timing Boundaries

Control loops (100-1000 Hz) must never block on perception (10-30 Hz). Decouple subsystems with async boundaries — buffers, shared variables, or message queues — so each runs at its own rate.

```python
# ❌ BAD: Perception blocks the control loop
def control_loop(self):  # Must run at 100Hz
    objects = self.detector.detect(self.camera.capture())  # 200ms ← BLOCKS!
    cmd = self.controller.compute(objects)
    self.arm.send_command(cmd)  # Now runs at 5Hz instead of 100Hz

# ✅ GOOD: Decoupled — perception in its own thread, control reads latest
def control_loop(self):  # Runs at 100Hz, never blocked
    with self.detection_lock:
        detections = self.latest_detections  # Latest available, never stale
    cmd = self.controller.compute(detections)
    self.arm.send_command(cmd)
```

See [detailed examples and code](references/design-principles-detailed.md#principle-6-separation-of-rates--respect-timing-boundaries)

### Principle 7: Fail-Safe Defaults — Safe Until Proven Otherwise

Default to the safest behavior: crawl speed, collision checks on, conservative workspace limits, explicit operator enable. On communication loss, unknown state, or sensor failure, the default action is stop.

```python
class ArmController:
    def __init__(self):
        self.max_velocity = 0.1              # Crawl speed by default
        self.collision_check = True           # Always on
        self.workspace_limits = DEFAULT_SAFE_WORKSPACE
        self._enabled = False                 # Must be explicitly enabled

    def move_to(self, target: np.ndarray, velocity: float = None):
        if not self._enabled:
            raise SafetyError("Controller not enabled")
        velocity = min(velocity or self.max_velocity, self.max_velocity)
        if not self.workspace_limits.contains(target):
            raise WorkspaceViolation(f"Target {target} outside safe workspace")
        return self._execute_move(target, velocity)
```

See [detailed examples and code](references/design-principles-detailed.md#principle-7-fail-safe-defaults--safe-until-proven-otherwise)

### Principle 8: Configuration Over Code — Externalize Everything That Changes

Robot IPs, joint limits, sensor parameters, safety thresholds, and workspace boundaries belong in config files (YAML/dataclass), not magic numbers in source. Algorithms, control logic, and interfaces stay in code.

See [detailed examples and code](references/design-principles-detailed.md#principle-8-configuration-over-code--externalize-everything-that-changes)

### Principle 9: Idempotent Operations — Safe to Retry

Every command must be safe to send twice. Use absolute targets (not relative deltas) and command-ID deduplication so network retries never move the robot twice as far.

See [detailed examples and code](references/design-principles-detailed.md#principle-9-idempotent-operations--safe-to-retry)

### Principle 10: Observe Everything — You Can't Debug What You Can't See

Emit structured telemetry (key-value, not printf) from every module: state transitions, command executions, safety events, performance metrics, sensor health. When a robot misbehaves at 2 AM, logs are all you have.

See [detailed examples and code](references/design-principles-detailed.md#principle-10-observe-everything--you-cant-debug-what-you-cant-see)

### Principle 11: Composability — Build Complex Behaviors from Simple Ones

Design primitive skills (MoveTo, Grasp, Detect) as composable blocks. Complex behaviors (PickAndPlace) emerge from combining tested primitives via dependency injection.

See [detailed examples and code](references/design-principles-detailed.md#principle-11-composability--build-complex-behaviors-from-simple-ones)

### Principle 12: Graceful Degradation — Work With What You Have

When components fail, degrade capability rather than halting entirely. Map available hardware to the best operational mode (full autonomy, blind manipulation, perception only, safe stop).

See [detailed examples and code](references/design-principles-detailed.md#principle-12-graceful-degradation--work-with-what-you-have)

---

## Applying These Principles

**When designing a new robotics module:**
1. Define the interface (ABC) first — what does the caller need? (Principles 2, 4)
2. Implement for simulation first, validate behavior (Principle 5)
3. Add safe defaults and explicit enable (Principle 7)
4. Externalize all tunable parameters to config (Principle 8)
5. Add structured telemetry before deploying (Principle 10)
6. Run through the checklist below before code review

**When reviewing robotics code:** flag violations by principle number and suggest the specific refactoring pattern from the [detailed reference](references/design-principles-detailed.md).

## Quick Reference: Principle Checklist

Use this during code reviews:

| # | Principle | Check |
|---|-----------|-------|
| 1 | Single Responsibility | Can you describe the module without "and"? |
| 2 | Dependency Inversion | Does high-level code import hardware drivers? |
| 3 | Open-Closed | Does adding a new sensor require modifying existing code? |
| 4 | Interface Segregation | Are implementations forced to stub out unused methods? |
| 5 | Liskov Substitution | Can you swap sim/real without changing caller code? |
| 6 | Separation of Rates | Does perception block the control loop? |
| 7 | Fail-Safe Defaults | What happens on communication loss? |
| 8 | Configuration Over Code | Are there magic numbers in the source? |
| 9 | Idempotent Operations | Is it safe to send every command twice? |
| 10 | Observe Everything | Can you diagnose a 2 AM failure from logs alone? |
| 11 | Composability | Can you build new tasks from existing skills? |
| 12 | Graceful Degradation | What's the robot's behavior when a sensor fails? |
