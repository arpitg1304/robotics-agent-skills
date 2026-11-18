"""The rig: the only surface an application sees.

A rig is a robot plus its peripherals behind a device bus. Applications discover
devices from their descriptors, read sensors, write actuator targets and advance
time. Evaluator state never crosses this interface.
"""

from __future__ import annotations

import json
from pathlib import Path

from .backend import Backend, BackendError


class EpisodeOver(Exception):
    """Raised when an application steps or commands a finished episode."""


def load_json(path: str | Path, required: tuple[str, ...]) -> dict:
    path = Path(path)
    data = json.loads(path.read_text(encoding="utf-8"))
    missing = [key for key in required if key not in data]
    if missing:
        raise BackendError(f"{path}: missing required field(s): {', '.join(missing)}")
    return data


def load_rig(path: str | Path) -> dict:
    return load_json(path, ("name", "backend", "robot"))


def load_task(path: str | Path) -> dict:
    return load_json(path, ("name", "instruction", "scene"))


class Rig:
    def __init__(self, backend: Backend, manifest: dict):
        self._backend = backend
        self.manifest = manifest
        self.name = manifest["name"]
        #: Device descriptors keyed by device name, as reported by the backend.
        configured = backend.request("configure", rig=manifest)
        self.devices: dict[str, dict] = configured["devices"]
        #: Safety envelope the backend enforces; leaving it ends the episode.
        self.safety: dict | None = configured.get("safety")
        #: Public episode context: instruction, cell layout, step budget.
        self.task: dict = {}
        self.done = True

    # -- episode lifecycle (used by the runner, not by applications) --

    def _begin(self, task: dict, seed: int) -> None:
        context = self._backend.request("reset", scene=task["scene"], goal=task.get("goal"), seed=seed)
        self.task = {"name": task["name"], "instruction": task["instruction"], **context}
        self.done = False

    def _finish(self) -> dict:
        self.done = True
        return self._backend.request("finish")

    # -- application interface --

    def read(self, device: str) -> dict:
        """Latest sample from a device. Check `stamp`: slow sensors return held samples."""
        return self._backend.request("read", device=device)

    def write(self, device: str, **command) -> None:
        """Set an actuator target. Takes effect on the next step and holds until replaced."""
        self._check()
        self._backend.request("write", device=device, command=command)

    def call(self, device: str, method: str, **params) -> dict:
        """Invoke a device service listed in its descriptor, such as the arm's `ik`."""
        return self._backend.request("call", device=device, method=method, params=params)

    def step(self, n: int = 1) -> dict:
        """Advance n control ticks. The simulator waits for you; slow code does not skew timing."""
        self._check()
        result = self._backend.request("step", n=n)
        self.done = result["done"]
        return result

    def mark(self, label: str) -> None:
        """Name the phase you are entering. Marks appear in the trace and failure reports."""
        self._backend.request("mark", label=label)

    def claim(self, kind: str, **fields) -> None:
        """State what you believe from your sensors, e.g. claim("object_position", id=0,
        position=[x, y, z]) in the world frame, or claim("holding", id=0). Claims are scored
        against ground truth in the report; nothing is returned here."""
        self._backend.request("claim", kind=kind, **fields)

    def _check(self) -> None:
        if self.done:
            raise EpisodeOver("Episode is over.")
