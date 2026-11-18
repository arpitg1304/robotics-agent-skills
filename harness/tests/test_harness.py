"""Harness tests. Backend tests need an ARMATURE checkout and are skipped without one.

Run from the harness directory: python3 -m unittest discover -s tests
"""

import json
import sys
import tempfile
import threading
import unittest
import urllib.request
from pathlib import Path

HARNESS = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HARNESS))

from rah.backend import Backend, BackendError, resolve_armature_home  # noqa: E402
from rah.rig import EpisodeOver, Rig, load_rig, load_task  # noqa: E402
from rah.replay import find_studio, make_server, pick_episode  # noqa: E402
from rah.runner import app_digest, run  # noqa: E402

try:
    resolve_armature_home()
    ARMATURE = True
except BackendError:
    ARMATURE = False

PANDA = HARNESS / "rigs" / "armature-panda-bench.json"
EXAMPLE = HARNESS / "examples" / "pick_place" / "app.py"


class ManifestTests(unittest.TestCase):
    def test_shipped_manifests_load(self):
        for path in (HARNESS / "rigs").glob("*.json"):
            rig = load_rig(path)
            self.assertEqual(rig["name"], path.stem)
        for path in (HARNESS / "tasks").glob("*.json"):
            task = load_task(path)
            self.assertEqual(task["name"], path.stem)
            seeds = task["seeds"]
            self.assertFalse(set(seeds["dev"]) & set(seeds["held_out"]), "dev and held-out seeds overlap")

    def test_missing_field_is_reported(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "rig.json"
            path.write_text(json.dumps({"name": "x"}))
            with self.assertRaisesRegex(BackendError, "backend, robot"):
                load_rig(path)

    def test_app_digest_tracks_source_changes(self):
        with tempfile.TemporaryDirectory() as tmp:
            app = Path(tmp) / "app.py"
            app.write_text("def run(rig): pass\n")
            before = app_digest(app)
            (Path(tmp) / "drivers.py").write_text("X = 1\n")
            self.assertNotEqual(before, app_digest(app))


@unittest.skipUnless(ARMATURE, "ARMATURE checkout not found (set ARMATURE_HOME)")
class ArmatureBackendTests(unittest.TestCase):
    def setUp(self):
        self.manifest = load_rig(PANDA)
        self.backend = Backend(self.manifest["backend"])
        self.addCleanup(self.backend.close)
        self.rig = Rig(self.backend, self.manifest)
        self.task = load_task(HARNESS / "tasks" / "transfer-blocks.json")

    def test_probe_describes_devices(self):
        self.assertEqual(set(self.rig.devices), {"arm", "gripper", "tracker"})
        self.assertEqual(len(self.rig.devices["arm"]["joints"]), 7)
        self.assertEqual(self.rig.devices["tracker"]["mount"]["xyz"], self.manifest["peripherals"]["tracker"]["mount"]["xyz"])

    def test_app_sees_no_evaluator_state(self):
        self.rig._begin(self.task, 1)
        self.assertNotIn("goal_error_m", json.dumps(self.rig.task))
        self.assertEqual(self.rig.claim("holding", id=None), None)
        self.assertEqual(set(self.rig.step()), {"step", "stamp", "done"})

    def test_tracker_is_rate_limited_and_in_its_own_frame(self):
        self.rig._begin(self.task, 1)
        first = self.rig.read("tracker")
        self.rig.step()
        self.assertEqual(self.rig.read("tracker")["stamp"], first["stamp"])
        self.rig.step()
        self.assertGreater(self.rig.read("tracker")["stamp"], first["stamp"])
        # Blocks rest on the bench at y ~ 0.02 in the world; the mounted tracker must not report that.
        self.assertGreater(abs(first["objects"][0]["position"][1] - 0.02), 0.05)

    def test_same_seed_reproduces(self):
        samples = []
        for _ in range(2):
            self.rig._begin(self.task, 7)
            self.rig.step(5)
            samples.append(self.rig.read("tracker"))
            self.rig._finish()
        self.assertEqual(samples[0], samples[1])

    def test_invalid_command_is_rejected(self):
        self.rig._begin(self.task, 1)
        with self.assertRaisesRegex(BackendError, "7 finite numbers"):
            self.rig.write("arm", position=[0.0])

    def test_leaving_workspace_ends_episode(self):
        self.rig._begin(self.task, 1)
        below_bench = self.rig.call("arm", "ik", position=[0.1, -0.05, 0.0])["position"]
        self.rig.write("arm", position=below_bench)
        with self.assertRaises(EpisodeOver):
            for _ in range(300):
                self.rig.step()
        report = self.rig._finish()["report"]
        self.assertEqual(report["outcome"], "safety_violation")
        self.assertEqual(report["safety_violation"]["axis"], "y")

    def test_wrong_belief_is_scored(self):
        self.rig._begin(self.task, 1)
        self.rig.claim("object_position", id=0, position=[1.0, 1.0, 1.0])
        self.rig.claim("holding", id=0)
        beliefs = self.rig._finish()["report"]["beliefs"]
        self.assertEqual(beliefs["wrong"], 1)
        self.assertGreater(beliefs["max_position_error_m"], 0.5)

    def test_unknown_goal_is_rejected(self):
        task = {**self.task, "goal": [{"rel": "at", "a": "object_9", "b": "target_0"}]}
        with self.assertRaisesRegex(BackendError, "object_9"):
            self.rig._begin(task, 1)


@unittest.skipUnless(ARMATURE, "ARMATURE checkout not found (set ARMATURE_HOME)")
class EndToEndTests(unittest.TestCase):
    def run_app(self, app: Path, task_name: str, seeds):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / "runs" / "one"
            task = load_task(HARNESS / "tasks" / f"{task_name}.json")
            summary = run(load_rig(PANDA), task, str(app), seeds, out, timeout_s=120, note="test")
            ledger = [json.loads(line) for line in (out.parent / "ledger.jsonl").read_text().splitlines()]
            self.assertTrue((out / f"episode-{seeds[0]}.trace.jsonl").stat().st_size)
        self.assertEqual(ledger[-1]["app_digest"], summary["app_digest"])
        return summary

    def test_reference_app_solves_custom_goal_task(self):
        summary = self.run_app(EXAMPLE, "transfer-crossed", [1])
        episode = summary["episodes"][0]
        self.assertTrue(episode["success"], episode)
        self.assertEqual(episode["beliefs"]["wrong"], 0)
        self.assertLess(episode["beliefs"]["max_position_error_m"], 0.005)

    def test_replay_is_recorded_and_served(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / "runs" / "one"
            app = Path(tmp) / "app.py"
            app.write_text("def run(rig):\n    rig.mark('wait')\n    rig.step(4)\n")
            task = load_task(HARNESS / "tasks" / "transfer-blocks.json")
            run(load_rig(PANDA), task, str(app), [2, 1], out, timeout_s=60)
            # Both episodes fail, so the first failed one in run order is picked.
            chosen = pick_episode(out, None)
            self.assertEqual(chosen.name, "episode-2.replay.json")
            self.assertEqual(pick_episode(out, 1).name, "episode-1.replay.json")
            with self.assertRaisesRegex(BackendError, "seed 9"):
                pick_episode(out, 9)
            replay = json.loads(chosen.read_text())
            self.assertEqual(len(replay["actions"]), 4)
            self.assertEqual(len(replay["actions"][0]), 8)
            self.assertEqual(replay["marks"], [[0, "start"], [0, "wait"]])
            self.assertEqual(replay["config"]["seed"], 2)

            studio = Path(tmp) / "studio.html"
            studio.write_text("<html>studio</html>")
            server = make_server(chosen, studio)
            self.addCleanup(server.server_close)
            threading.Thread(target=server.serve_forever, daemon=True).start()
            self.addCleanup(server.shutdown)
            base = f"http://127.0.0.1:{server.server_address[1]}"
            with urllib.request.urlopen(base + "/") as response:
                self.assertIn(b"rah replay", response.read())
            with urllib.request.urlopen(base + "/replay.json") as response:
                self.assertEqual(json.loads(response.read())["seed"], 2)
            with self.assertRaises(urllib.error.HTTPError):
                urllib.request.urlopen(base + "/../rah/cli.py")

    def test_app_crash_is_an_outcome_not_a_harness_failure(self):
        with tempfile.TemporaryDirectory() as tmp:
            app = Path(tmp) / "app.py"
            app.write_text("def run(rig):\n    rig.step(3)\n    raise ValueError('boom')\n")
            episode = self.run_app(app, "transfer-blocks", [1])["episodes"][0]
        self.assertEqual(episode["outcome"], "app_error")
        self.assertIn("boom", episode["app_error"])
        self.assertFalse(episode["success"])


if __name__ == "__main__":
    unittest.main()
