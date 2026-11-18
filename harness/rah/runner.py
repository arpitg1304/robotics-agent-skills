"""Run an application against a rig and task, one episode per seed."""

from __future__ import annotations

import hashlib
import importlib.util
import json
import signal
import sys
import time
import traceback
from pathlib import Path

from .backend import Backend
from .rig import EpisodeOver, Rig


class EpisodeTimeout(Exception):
    """The application exceeded its wall-clock allowance for one episode."""


def load_app(path: str | Path):
    path = Path(path).resolve()
    spec = importlib.util.spec_from_file_location("rah_app", path)
    module = importlib.util.module_from_spec(spec)
    # Let the app import sibling modules (drivers, helpers) from its own directory.
    sys.path.insert(0, str(path.parent))
    spec.loader.exec_module(module)
    if not callable(getattr(module, "run", None)):
        raise SystemExit(f"{path}: an app must define run(rig).")
    return module


def _raise_timeout(signum, frame):
    raise EpisodeTimeout()


def run_episode(rig: Rig, app, task: dict, seed: int, timeout_s: float) -> dict:
    rig._begin(task, seed)
    app_error = None
    started = time.monotonic()
    use_alarm = timeout_s > 0 and hasattr(signal, "SIGALRM")
    if use_alarm:
        previous = signal.signal(signal.SIGALRM, _raise_timeout)
        signal.setitimer(signal.ITIMER_REAL, timeout_s)
    try:
        app.run(rig)
    except EpisodeOver:
        pass
    except EpisodeTimeout:
        app_error = f"Timed out after {timeout_s:g}s of wall-clock time."
    except Exception:  # noqa: BLE001 - an app crash is an episode outcome, not a harness failure
        app_error = traceback.format_exc()
    finally:
        if use_alarm:
            signal.setitimer(signal.ITIMER_REAL, 0)
            signal.signal(signal.SIGALRM, previous)
    result = rig._finish()
    report = result["report"]
    report["seed"] = seed
    report["wall_seconds"] = round(time.monotonic() - started, 3)
    if app_error:
        report["app_error"] = app_error
        if not report["success"]:
            report["outcome"] = "app_error"
    elif not report["success"] and not report["out_of_budget"] and report["outcome"] not in ("fell", "safety_violation"):
        # The app returned while the task was unfinished and steps remained.
        report["returned_early"] = True
    return {"report": report, "trace": result["trace"], "replay": result.get("replay")}


def app_digest(app_path: str | Path) -> str:
    """Hash of the app's Python sources, so every result is tied to the code that produced it."""
    digest = hashlib.sha256()
    for source in sorted(Path(app_path).resolve().parent.glob("*.py")):
        digest.update(source.name.encode())
        digest.update(source.read_bytes())
    return digest.hexdigest()[:12]


def run(
    rig_manifest: dict,
    task: dict,
    app_path: str,
    seeds: list[int],
    out_dir: Path,
    timeout_s: float,
    split: str = "dev",
    note: str = "",
) -> dict:
    app = load_app(app_path)
    out_dir.mkdir(parents=True, exist_ok=True)
    episodes = []
    with Backend(rig_manifest["backend"]) as backend:
        rig = Rig(backend, rig_manifest)
        for seed in seeds:
            episode = run_episode(rig, app, task, seed, timeout_s)
            with (out_dir / f"episode-{seed}.trace.jsonl").open("w", encoding="utf-8") as handle:
                for row in episode["trace"]:
                    handle.write(json.dumps(row) + "\n")
            if episode["replay"]:
                replay = {
                    "rig": rig_manifest["name"],
                    "task": task["name"],
                    "instruction": task["instruction"],
                    "seed": seed,
                    **episode["replay"],
                    "report": episode["report"],
                    # Ground-truth object positions from the original run, to detect replay drift.
                    "checkpoints": [[row["step"], row["objects"]] for row in episode["trace"]],
                }
                (out_dir / f"episode-{seed}.replay.json").write_text(json.dumps(replay), encoding="utf-8")
            episodes.append(episode["report"])
        summary = {
            "rig": rig_manifest["name"],
            "task": task["name"],
            "app": str(Path(app_path).resolve()),
            "app_digest": app_digest(app_path),
            "split": split,
            "note": note,
            "backend": backend.info,
            "succeeded": sum(e["success"] for e in episodes),
            "mean_progress": round(sum(e["progress"] for e in episodes) / len(episodes), 3),
            "episodes": episodes,
        }
    (out_dir / "report.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    # Append-only record of every attempt: what was tried, on which code, with what result.
    entry = {
        "time": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "run": out_dir.name,
        **{key: summary[key] for key in ("rig", "task", "split", "app_digest", "note", "succeeded", "mean_progress")},
        "episodes": len(episodes),
        "outcomes": {str(e["seed"]): e["outcome"] for e in episodes},
    }
    with (out_dir.parent / "ledger.jsonl").open("a", encoding="utf-8") as handle:
        handle.write(json.dumps(entry) + "\n")
    return summary


def format_summary(summary: dict) -> str:
    lines = [f"{'seed':>5}  {'outcome':<16} {'progress':>8} {'steps':>5}  {'worst goal err':>14}  last mark"]
    for e in summary["episodes"]:
        worst = max(e["goal_error_m"]) * 1000 if e["goal_error_m"] else 0.0
        lines.append(
            f"{e['seed']:>5}  {e['outcome']:<16} {e['progress']:>7.0%} {e['steps']:>5}  {worst:>11.1f} mm  {e['last_mark']}"
        )
        beliefs = e["beliefs"]
        if beliefs["wrong"] or (beliefs["max_position_error_m"] or 0) > 0.005:
            lines.append(
                f"       belief check: {beliefs['wrong']} wrong claim(s), max position error "
                f"{(beliefs['max_position_error_m'] or 0) * 1000:.1f} mm - suspect sensor integration"
            )
        if e.get("safety_violation"):
            v = e["safety_violation"]
            lines.append(f"       tcp left the workspace along {v['axis']} at step {v['step']} during '{v['mark']}'")
        if e.get("app_error"):
            lines.append("       " + e["app_error"].strip().splitlines()[-1])
        elif e.get("returned_early"):
            lines.append("       app returned before the task succeeded or the step budget ran out")
        if not e["success"]:
            for g in e["goal"]:
                if not g["holds"]:
                    lines.append(f"       goal not met: {g['relation']}")
            for event in [ev for ev in e["events"] if "relation" in ev][-4:]:
                state = "became true" if event["holds"] else "became false"
                lines.append(f"       step {event['step']}: {event['relation']} {state} during '{event['mark']}'")
    total = len(summary["episodes"])
    lines.append(
        f"{summary['succeeded']}/{total} episodes succeeded ({summary['split']} seeds, mean progress "
        f"{summary['mean_progress']:.0%}) on {summary['rig']} / {summary['task']} @ app {summary['app_digest']}"
    )
    return "\n".join(lines)
