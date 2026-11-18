"""Replay a recorded episode in the ARMATURE browser studio.

The harness records the action sent on every control tick. The simulator is
deterministic per seed, so stepping the studio with the same actions reproduces
the episode. A local server puts the viewer page and the studio build on one
origin so the viewer can drive the studio's `window.armature` API.
"""

from __future__ import annotations

import json
import webbrowser
from functools import partial
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from .backend import HARNESS_DIR, BackendError, resolve_armature_home

VIEWER = HARNESS_DIR / "viewer" / "index.html"
STUDIO_BUILD = Path("dist") / "robotics-arm-studio.html"


def find_studio(configured: str | None = None) -> Path:
    studio = resolve_armature_home(configured) / STUDIO_BUILD
    if not studio.is_file():
        raise BackendError(f"{studio} not found. Run `npm run build` in the ARMATURE checkout.")
    return studio


def pick_episode(run_dir: Path, seed: int | None) -> Path:
    """The requested seed, else the first failed episode, else the first one."""
    replays = {int(p.name.split("-")[1].split(".")[0]): p for p in run_dir.glob("episode-*.replay.json")}
    if not replays:
        raise BackendError(f"No replay files in {run_dir}. Re-run the app; older runs did not record actions.")
    if seed is not None:
        if seed not in replays:
            raise BackendError(f"No episode for seed {seed} in {run_dir}. Available: {sorted(replays)}")
        return replays[seed]
    report = run_dir / "report.json"
    if report.is_file():
        for episode in json.loads(report.read_text(encoding="utf-8"))["episodes"]:
            if not episode["success"] and episode["seed"] in replays:
                return replays[episode["seed"]]
    return replays[min(replays)]


class Handler(BaseHTTPRequestHandler):
    def __init__(self, routes: dict[str, tuple[Path, str]], *args, **kwargs):
        self.routes = routes
        super().__init__(*args, **kwargs)

    def do_GET(self):  # noqa: N802 - http.server naming
        route = self.routes.get(self.path.split("?")[0])
        if not route:
            self.send_error(404)
            return
        path, content_type = route
        body = path.read_bytes()
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *args):
        pass


def make_server(replay_file: Path, studio: Path, port: int = 0) -> ThreadingHTTPServer:
    routes = {
        "/": (VIEWER, "text/html; charset=utf-8"),
        "/studio.html": (studio, "text/html; charset=utf-8"),
        "/replay.json": (replay_file, "application/json"),
    }
    # Loopback only: this serves local files and is not meant to be reachable from the network.
    return ThreadingHTTPServer(("127.0.0.1", port), partial(Handler, routes))


def serve(run_dir: Path, seed: int | None, armature: str | None, port: int, open_browser: bool) -> None:
    replay_file = pick_episode(run_dir, seed)
    server = make_server(replay_file, find_studio(armature), port)
    url = f"http://127.0.0.1:{server.server_address[1]}/"
    print(f"Replaying {replay_file.name} from {run_dir}")
    print(f"Open {url}  (Ctrl-C to stop)")
    if open_browser:
        webbrowser.open(url)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
