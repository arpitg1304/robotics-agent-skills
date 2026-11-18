"""Backend process management and the JSON-lines protocol client."""

from __future__ import annotations

import json
import os
import shutil
import subprocess
from pathlib import Path

HARNESS_DIR = Path(__file__).resolve().parent.parent
PROTOCOL = 1


class BackendError(RuntimeError):
    """The backend rejected a request or is not usable."""


def resolve_armature_home(configured: str | None = None) -> Path:
    """Find the ARMATURE checkout: explicit setting, $ARMATURE_HOME, then a sibling directory."""
    candidates = [configured, os.environ.get("ARMATURE_HOME"), HARNESS_DIR.parent.parent / "ARMATURE"]
    for candidate in candidates:
        if not candidate:
            continue
        home = Path(candidate).expanduser().resolve()
        if (home / "src" / "simulation" / "environment.js").is_file():
            return home
    raise BackendError(
        "ARMATURE checkout not found. Clone https://github.com/arpitg1304/armature, run "
        "`npm ci` in it, and set ARMATURE_HOME to its path."
    )


def backend_command(spec: dict) -> list[str]:
    kind = spec.get("type")
    if kind == "armature":
        node = shutil.which("node")
        if not node:
            raise BackendError("The armature backend needs Node.js 22+ on PATH.")
        home = resolve_armature_home(spec.get("home"))
        if not (home / "node_modules" / "esbuild").is_dir():
            raise BackendError(f"Run `npm ci` in {home}: the backend bundles the simulator with esbuild.")
        return [node, str(HARNESS_DIR / "backends" / "armature" / "bridge.mjs"), str(home)]
    if kind == "command":
        # Any executable that speaks the protocol on stdin/stdout.
        return list(spec["argv"])
    raise BackendError(f"Unknown backend type: {kind!r}")


class Backend:
    """A running backend. One request per line out, one response per line back."""

    def __init__(self, spec: dict):
        self._process = subprocess.Popen(
            backend_command(spec),
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        self.info = self.request("hello")
        if self.info.get("protocol") != PROTOCOL:
            self.close()
            raise BackendError(f"Backend speaks protocol {self.info.get('protocol')}, expected {PROTOCOL}.")

    def request(self, op: str, **fields) -> dict:
        process = self._process
        try:
            process.stdin.write(json.dumps({"op": op, **fields}) + "\n")
            process.stdin.flush()
            line = process.stdout.readline()
        except (BrokenPipeError, ValueError):
            line = ""
        if not line:
            stderr = process.stderr.read().strip() if process.poll() is not None else ""
            raise BackendError(f"Backend exited during '{op}'. {stderr[-2000:]}".strip())
        response = json.loads(line)
        if not response.get("ok"):
            raise BackendError(response.get("error", "unknown backend error"))
        return response.get("result", {})

    def close(self) -> None:
        process = self._process
        if process.poll() is None:
            process.stdin.close()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
        for stream in (process.stdout, process.stderr):
            stream.close()

    def __enter__(self) -> "Backend":
        return self

    def __exit__(self, *exc) -> None:
        self.close()
