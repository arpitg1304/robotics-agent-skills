"""Command line for the robotics agent harness."""

from __future__ import annotations

import argparse
import json
import shutil
import sys
import time
from pathlib import Path

from .backend import HARNESS_DIR, Backend, BackendError
from .replay import serve
from .rig import Rig, load_rig, load_task
from .runner import format_summary, run

APP_TEMPLATE = '''"""Application for rig '{rig}', task '{task}'.

Task: {instruction}

Run `rah probe --rig {rig_path}` to see the devices, then iterate with
`rah run --rig {rig_path} --task {task_path} --app app.py`.
"""


def run(rig):
    # rig.devices  - device descriptors (units, frames, limits, rates, services)
    # rig.task     - instruction and public cell layout for this episode
    # rig.read(device) / rig.write(device, **command) / rig.call(device, method, **params)
    # rig.step(n)  - advance n control ticks;  rig.mark(label) - name the current phase
    raise NotImplementedError("Write the application.")
'''


LESSONS_TEMPLATE = """# Lessons: {rig}

Durable facts learned about this rig while building applications. Read before starting a
task; add to it when a run teaches you something the device descriptors did not say.
"""


def cmd_doctor(args) -> int:
    ok = True
    node = shutil.which("node")
    print(f"node: {node or 'MISSING (need Node.js 22+)'}")
    ok &= bool(node)
    try:
        with Backend({"type": "armature", "home": args.armature}) as backend:
            info = backend.info
        print(f"armature: {info['home']} (v{info['version']}), robots: {', '.join(info['robots'])}")
    except BackendError as error:
        print(f"armature: {error}")
        ok = False
    return 0 if ok else 1


def cmd_probe(args) -> int:
    manifest = load_rig(args.rig)
    with Backend(manifest["backend"]) as backend:
        rig = Rig(backend, manifest)
        print(json.dumps({"rig": rig.name, "backend": backend.info, "devices": rig.devices}, indent=2))
    return 0


def cmd_init(args) -> int:
    manifest, task = load_rig(args.rig), load_task(args.task)
    workspace = Path(args.workspace)
    app = workspace / "app.py"
    if app.exists():
        print(f"{app} already exists; not overwriting.", file=sys.stderr)
        return 1
    workspace.mkdir(parents=True, exist_ok=True)
    lessons = workspace / "LESSONS.md"
    if not lessons.exists():
        lessons.write_text(LESSONS_TEMPLATE.format(rig=manifest["name"]), encoding="utf-8")
    app.write_text(
        APP_TEMPLATE.format(
            rig=manifest["name"],
            task=task["name"],
            instruction=task["instruction"],
            rig_path=Path(args.rig).resolve(),
            task_path=Path(args.task).resolve(),
        ),
        encoding="utf-8",
    )
    print(f"Created {app}")
    return 0


def cmd_run(args) -> int:
    manifest, task = load_rig(args.rig), load_task(args.task)
    split = args.split
    seeds = args.seeds or task.get("seeds", {}).get(split)
    if not seeds:
        print(f"error: task '{task['name']}' has no {split} seeds; pass --seeds.", file=sys.stderr)
        return 2
    stamp = f"{time.strftime('%Y%m%d-%H%M%S')}-{task['name']}-{split}"
    out_dir = Path(args.out) if args.out else Path("runs") / stamp
    summary = run(manifest, task, args.app, seeds, out_dir, args.timeout, split, args.note)
    print(format_summary(summary))
    print(f"report: {out_dir / 'report.json'}")
    return 0 if summary["succeeded"] == len(seeds) else 1


def cmd_replay(args) -> int:
    serve(Path(args.run), args.seed, args.armature, args.port, not args.no_open)
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="rah", description="Robotics agent harness.")
    commands = parser.add_subparsers(dest="command", required=True)

    doctor = commands.add_parser("doctor", help="Check that the simulator backend is usable.")
    doctor.add_argument("--armature", help="Path to an ARMATURE checkout (default: $ARMATURE_HOME).")
    doctor.set_defaults(handler=cmd_doctor)

    probe = commands.add_parser("probe", help="Print the device descriptors of a rig.")
    probe.add_argument("--rig", required=True)
    probe.set_defaults(handler=cmd_probe)

    init = commands.add_parser("init", help="Create an application skeleton in a workspace directory.")
    init.add_argument("workspace")
    init.add_argument("--rig", required=True)
    init.add_argument("--task", required=True)
    init.set_defaults(handler=cmd_init)

    for name, split, text in (
        ("run", "dev", "Run an application on the task's dev seeds. Iterate against this."),
        ("verify", "held_out", "Acceptance run on the task's held-out seeds. Do not tune against it."),
    ):
        runner = commands.add_parser(name, help=text)
        runner.add_argument("--rig", required=True)
        runner.add_argument("--task", required=True)
        runner.add_argument("--app", required=True, help="Python file defining run(rig).")
        runner.add_argument("--note", default="", help="What this attempt changes; recorded in the ledger.")
        runner.add_argument("--out", help="Output directory (default: runs/<timestamp>-<task>-<split>).")
        runner.add_argument("--timeout", type=float, default=120, help="Wall-clock seconds per episode; 0 disables.")
        if name == "run":
            runner.add_argument("--seeds", type=int, nargs="+", help="Override the task's dev seeds.")
        runner.set_defaults(handler=cmd_run, split=split, seeds=None)

    replay = commands.add_parser("replay", help="Watch a recorded episode in the ARMATURE studio.")
    replay.add_argument("run", help="Run directory, e.g. runs/<timestamp>-<task>-dev.")
    replay.add_argument("--seed", type=int, help="Default: the first failed episode, else the first.")
    replay.add_argument("--armature", help="Path to an ARMATURE checkout (default: $ARMATURE_HOME).")
    replay.add_argument("--port", type=int, default=0, help="Default: any free port.")
    replay.add_argument("--no-open", action="store_true", help="Print the URL without opening a browser.")
    replay.set_defaults(handler=cmd_replay)

    args = parser.parse_args(argv)
    try:
        return args.handler(args)
    except BackendError as error:
        print(f"error: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
