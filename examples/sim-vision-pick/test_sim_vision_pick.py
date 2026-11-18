"""Offline tests: no server needed.  python -m unittest -v

The robot model is checked against the same cases as the engine's own
collision tests (crates/kinematics/src/collision.rs), so the client-side
port and the engine agree.
"""

import math
import pathlib
import random
import tomllib
import unittest

from robot_model import RobotModel, segment_distance
from sim_vision import OBJECT_HEIGHT_MM, Camera, FakeDetector, Scene
from sim_vision_pick import movej_params

PROFILES = pathlib.Path(__file__).resolve().parents[2] / "robot-profiles"


def config_from_profile(name: str) -> dict:
    """The subset of GET /api/config the model uses, built from a profile."""
    t = tomllib.loads((PROFILES / name).read_text(encoding="utf-8"))
    axes = []
    for a in t["axis"]:
        g = a.get("geometry")
        axes.append({
            "id": a["id"], "kind": a["kind"], "min": a["min"], "max": a["max"], "max_velocity": a["max_velocity"],
            "geometry": None if g is None else {
                "parent": g.get("parent"), "origin_mm": g.get("origin", [0.0] * 3),
                "rotation_deg": g.get("rotation", [0.0] * 3), "axis": g["axis"],
                "tip_mm": g.get("tip"), "radius_mm": g.get("radius")},
        })
    return {"name": t["robot"]["name"], "axes": axes,
            "collision_checking": all(a["geometry"] and a["geometry"]["radius_mm"] for a in axes)}


DUMMY = RobotModel.from_config(config_from_profile("dummy-arm-virtual.toml"))
REST = [0.0, -73.0, 180.0, 0.0, 0.0, 0.0]
DOWN = (0.0, 0.0, -1.0)


class ModelMatchesEngine(unittest.TestCase):
    def test_segment_distance(self):
        self.assertAlmostEqual(segment_distance((0, 0, 0), (1, 0, 0), (0, 1, 0), (1, 1, 0)), 1.0)
        self.assertAlmostEqual(segment_distance((0, 0, 0), (2, 0, 0), (1, -1, 1), (1, 1, 1)), 1.0)
        self.assertAlmostEqual(segment_distance((0, 0, 0), (1, 0, 0), (3, 0, 0), (4, 0, 0)), 2.0)

    def test_forward_kinematics_matches_dummy_firmware(self):
        up, _ = DUMMY.tool([0.0] * 6)
        self.assertAlmostEqual(up[0], -17.0, places=6)
        self.assertAlmostEqual(up[2], 442.0, places=6)
        rest, _ = DUMMY.tool(REST)
        self.assertAlmostEqual(rest[0], 89.41, delta=0.01)
        self.assertAlmostEqual(rest[2], 146.74, delta=0.01)

    def test_known_poses(self):
        self.assertTrue(DUMMY.collision_checking)
        self.assertGreater(DUMMY.clearance([0.0, 0.0, 90.0, 0.0, 0.0, 0.0]).gap, 50.0)
        self.assertGreater(DUMMY.clearance(REST).gap, 2.0)
        bad = DUMMY.clearance([-170.0, -73.0, 180.0, -180.0, -120.0, -720.0])
        self.assertLess(bad.gap, 0.0)
        self.assertEqual((bad.a, bad.b), ("j2", "j6"))
        floor = DUMMY.clearance([0.0, 90.0, 90.0, 0.0, 0.0, 0.0])
        self.assertEqual(floor.b, "floor")
        self.assertLess(floor.gap, 0.0)

    def test_sweeps(self):
        hit = DUMMY.check_sweep(REST, 4, 60.0)
        self.assertIsNotNone(hit)
        self.assertEqual(hit.b, "j6")
        self.assertIsNone(DUMMY.check_sweep(REST, 4, -60.0))
        into = [0.0, -73.0, 180.0, 0.0, 25.0, 0.0]
        self.assertIsNone(DUMMY.check_sweep(into, 4, 0.0))  # backing out is allowed
        self.assertIsNotNone(DUMMY.check_sweep(into, 4, 40.0))


class InverseKinematics(unittest.TestCase):
    def test_reaches_table_points_pointing_down(self):
        rng = random.Random(1)
        seed = [0.0, 0.0, 90.0, 0.0, 0.0, 0.0]
        for x, y, z in [(200, 0, 25), (150, -170, 25), (120, 140, 40), (260, -70, 85)]:
            q = DUMMY.solve_ik((x, y, z), DOWN, seed, rng=rng)
            self.assertIsNotNone(q, (x, y, z))
            p, d = DUMMY.tool(q)
            self.assertLess(math.dist(p, (x, y, z)), 0.5)
            self.assertLess(math.degrees(math.acos(-d[2])), 1.0)
            for v, j in zip(q, DUMMY.joints):
                self.assertTrue(j.min <= v <= j.max)
            self.assertGreater(DUMMY.clearance(q).gap, 2.0)

    def test_out_of_reach_returns_none(self):
        self.assertIsNone(DUMMY.solve_ik((600.0, 0.0, 25.0), DOWN, REST, attempts=4))

    def test_single_axis_order_avoids_collisions(self):
        a = DUMMY.solve_ik((200, 0, 180), DOWN, [0.0, 0.0, 90.0, 0.0, 0.0, 0.0])
        b = DUMMY.solve_ik((150, -170, 25), DOWN, a)
        order = DUMMY.single_axis_order(a, b)
        self.assertIsNotNone(order)
        q = list(a)
        for i in order:
            self.assertIsNone(DUMMY.check_sweep(q, i, b[i]))
            q[i] = b[i]

    def test_rail_arm_profile_builds(self):
        m = RobotModel.from_config(config_from_profile("rail-arm-virtual.toml"))
        self.assertEqual(len(m.joints), 3)


class Vision(unittest.TestCase):
    def test_camera_round_trip(self):
        cam = Camera()
        for x, y, z in [(200, 0, 25), (120, 140, 40), (260, -70, 25)]:
            u, v = cam.project(x, y, z)
            self.assertTrue(cam.sees(u, v))
            bx, by = cam.unproject(u, v, z)
            self.assertAlmostEqual(bx, x)
            self.assertAlmostEqual(by, y)

    def test_detector_noise_and_misses(self):
        rng = random.Random(2)
        scene = Scene.random(5, rng)
        perfect = FakeDetector(Camera(), rng, noise_px=0, miss_rate=0, false_positive_rate=0, latency_s=0)
        dets = perfect.detect(scene)
        self.assertEqual(len(dets), 5)
        for d in dets:
            x, y = Camera().unproject(d.u, d.v, OBJECT_HEIGHT_MM[d.label])
            self.assertIsNotNone(scene.object_at(x, y, OBJECT_HEIGHT_MM[d.label], 0.01, 0.01))
        blind = FakeDetector(Camera(), rng, miss_rate=1.0, false_positive_rate=0, latency_s=0)
        self.assertEqual(blind.detect(scene), [])

    def test_scene_spacing(self):
        s = Scene.random(6, random.Random(3))
        for i, a in enumerate(s.objects):
            for b in s.objects[i + 1:]:
                self.assertGreaterEqual(math.hypot(a.x - b.x, a.y - b.y), 60.0)


class MoveJContract(unittest.TestCase):
    def test_params_name_every_joint(self):
        self.assertEqual(movej_params(["j1", "j2"], [1.23456, -2.0]), {"targets": {"j1": 1.2346, "j2": -2.0}})


if __name__ == "__main__":
    unittest.main()
