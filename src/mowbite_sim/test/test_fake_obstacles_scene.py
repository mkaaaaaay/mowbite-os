"""The scene logic of fake_obstacles without ROS: what is there when, what lies in the view, where it is reported.

Run: python3 -m unittest src/mowbite_sim/test/test_fake_obstacles_scene.py (or python3 on the file).
"""
import io
import json
import math
import os
import random
import re
import tempfile
import unittest
from importlib.machinery import SourceFileLoader
from importlib.util import module_from_spec, spec_from_loader

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPT = os.path.join(HERE, "..", "scripts", "fake_obstacles")
EXAMPLE = os.path.join(HERE, "..", "config", "obstacles_scene.example.json")
MSG = os.path.join(HERE, "..", "..", "mowbite_msgs", "msg")


def load_script():
    loader = SourceFileLoader("fake_obstacles", SCRIPT)
    module = module_from_spec(spec_from_loader("fake_obstacles", loader))
    loader.exec_module(module)
    return module


fo = load_script()


def constants(msg_file, prefix):
    """The values of the string constants of a .msg file starting with prefix."""
    with open(os.path.join(MSG, msg_file)) as f:
        return set(re.findall(rf"^string {prefix}\w+=(\w+)", f.read(), re.MULTILINE))


KINDS = constants("Obstacle.msg", "KIND_")
REASONS = constants("Obstacles.msg", "REASON_")
HALF_FOV = math.radians(43.0)
VIEW = fo.View(0.466, 0.0, HALF_FOV, -HALF_FOV, 0.47, 4.0)
EXACT = [0.0, 0.0]


def scene(obstacles, sight=()):
    with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False) as f:
        json.dump({"obstacles": list(obstacles), "sight": list(sight)}, f)
    try:
        return fo.load_scene(f.name, KINDS, REASONS)
    finally:
        os.unlink(f.name)


def thing(**fields):
    return dict({"kind": "OBJECT", "class_name": "box", "score": 0.8, "radius": 0.2, "height": 0.5}, **fields)


def seen(things, t=0.0, pose=(0.0, 0.0, 0.0), image=1, sigma=EXACT, seed=1):
    return fo.observe(things, t, pose, VIEW, image, sigma, random.Random(seed))


class Scene(unittest.TestCase):
    def test_example_loads(self):
        loaded = fo.load_scene(EXAMPLE, KINDS, REASONS)
        self.assertEqual(len(loaded["obstacles"]), 4)
        self.assertEqual(loaded["sight"], [(30.0, 35.0, "TOO_DARK")])
        unknown = loaded["obstacles"][3]
        self.assertEqual((unknown["class_name"], unknown["score"]), ("", 1.0))

    def test_no_scene_is_empty(self):
        self.assertEqual(fo.load_scene("", KINDS, REASONS), {"obstacles": [], "sight": []})

    def test_wrong_scene_is_refused(self):
        for bad in ([thing(kind="CAT", x=1, y=0)], [thing()], [thing(x=1, y=0, radius=0)],
                    [thing(path=[[5, 0, 0], [5, 1, 0]])]):
            with self.assertRaises(ValueError):
                scene(bad)
        with self.assertRaises(ValueError):
            scene([], [{"from_s": 0, "until_s": 1, "reason": "FOG"}])
        with self.assertRaises(ValueError):
            fo.load_scene("/nonexistent/scene.json", KINDS, REASONS)


class View(unittest.TestCase):
    def test_ahead_is_reported(self):
        found = seen(scene([thing(x=2.5, y=0.1)])["obstacles"])
        self.assertEqual(len(found), 1)
        o = found[0]
        self.assertAlmostEqual(o["x"], 2.5)
        self.assertAlmostEqual(o["y"], 0.1)
        self.assertEqual((o["kind"], o["class_name"], o["radius"], o["agrees"], o["cut_bottom"]),
                         ("OBJECT", "box", 0.2, True, False))
        self.assertAlmostEqual(o["score"], 0.8)

    def test_aside_is_not(self):
        self.assertEqual(seen(scene([thing(x=0.8, y=2.0)])["obstacles"]), [])
        # Middle inside the view, but the disc crosses its left edge.
        bearing = HALF_FOV - 0.02
        d = 2.0
        self.assertEqual(seen(scene([thing(x=0.466 + d * math.cos(bearing), y=d * math.sin(bearing))])["obstacles"]),
                         [])

    def test_near_and_far_are_not(self):
        self.assertEqual(seen(scene([thing(x=0.466 + 0.6, y=0.0)])["obstacles"]), [])
        self.assertEqual(seen(scene([thing(x=0.466 + 3.9, y=0.0)])["obstacles"]), [])
        self.assertEqual(len(seen(scene([thing(x=0.466 + 0.7, y=0.0)])["obstacles"])), 1)

    def test_heading_90_turns_into_base_link(self):
        pose = (1.0, 1.0, math.pi / 2)
        found = seen(scene([thing(x=1.0, y=3.5)])["obstacles"], pose=pose)
        self.assertEqual(len(found), 1)
        self.assertAlmostEqual(found[0]["x"], 2.5)
        self.assertAlmostEqual(found[0]["y"], 0.0)
        found = seen(scene([thing(x=0.5, y=3.0)])["obstacles"], pose=pose)
        self.assertAlmostEqual(found[0]["x"], 2.0)
        self.assertAlmostEqual(found[0]["y"], 0.5)
        # Ahead in the map frame is to the right of the mower now.
        self.assertEqual(seen(scene([thing(x=3.5, y=1.0)])["obstacles"], pose=pose), [])


class Time(unittest.TestCase):
    def test_moving_thing(self):
        things = scene([thing(path=[[10, 2.0, 1.0], [20, 3.0, -1.0]])])["obstacles"]
        self.assertEqual(fo.thing_at(things[0], 0.0), (2.0, 1.0))
        x, y = fo.thing_at(things[0], 15.0)
        self.assertAlmostEqual(x, 2.5)
        self.assertAlmostEqual(y, 0.0)
        self.assertEqual(fo.thing_at(things[0], 30.0), (3.0, -1.0))
        found = seen(things, t=15.0)
        self.assertEqual(len(found), 1)
        self.assertAlmostEqual(found[0]["x"], 2.5)
        self.assertAlmostEqual(found[0]["y"], 0.0)

    def test_from_until(self):
        things = scene([thing(x=2.0, y=0.0, from_s=5, until_s=10)])["obstacles"]
        self.assertEqual(seen(things, t=4.9), [])
        self.assertEqual(len(seen(things, t=5.0)), 1)
        self.assertEqual(len(seen(things, t=9.9)), 1)
        self.assertEqual(seen(things, t=10.0), [])

    def test_sight_plan(self):
        plan = scene([], [{"from_s": 3, "until_s": 6, "reason": "TOO_DARK"},
                          {"from_s": 8, "until_s": 9, "reason": "TOO_BRIGHT"}])["sight"]
        self.assertEqual([fo.sight_reason(plan, t) for t in (2.9, 3.0, 5.9, 6.0, 8.5, 9.0)],
                         ["", "TOO_DARK", "TOO_DARK", "", "TOO_BRIGHT", ""])

    def test_score_drop(self):
        things = scene([thing(x=2.0, y=0.0, score_drop=[3])])["obstacles"]
        self.assertEqual([len(seen(things, image=i)) for i in range(1, 7)], [1, 1, 0, 1, 1, 0])

    def test_carry_true_pose(self):
        state = {"x": 1.0, "y": 2.0, "yaw": math.pi / 2, "linear": 0.5, "angular": 0.0, "clock": 100.0}
        x, y, yaw = fo.carry(state, 100.2)
        self.assertAlmostEqual(x, 1.0)
        self.assertAlmostEqual(y, 2.1)
        self.assertAlmostEqual(yaw, math.pi / 2)
        self.assertIsNone(fo.carry({}, 100.0))


class Error(unittest.TestCase):
    def test_noise_from_seed_and_covariance_along_sight(self):
        things = scene([thing(x=2.466, y=0.0)])["obstacles"]
        a, b = seen(things, sigma=[0.03, 0.02], seed=7), seen(things, sigma=[0.03, 0.02], seed=7)
        self.assertEqual(a, b)
        along = 0.03 + 0.02 * 2.0
        self.assertAlmostEqual(a[0]["cov_xx"], along ** 2)
        self.assertAlmostEqual(a[0]["cov_yy"], (along / 4) ** 2)
        self.assertAlmostEqual(a[0]["cov_xy"], 0.0)
        # Across the line of sight at 90 deg from the view point the spread turns with it.
        cxx, cxy, cyy = fo.noisy(VIEW, 0.466, 0.0001, [0.1, 0.0], random.Random(1))[2:]
        self.assertAlmostEqual(cyy, 0.01, places=6)
        self.assertAlmostEqual(cxx, 0.01 / 16, places=6)

    def test_box_in_image(self):
        for x, y in ((1.0, 0.0), (3.5, 1.5), (3.5, -1.5)):
            box_x, box_y, box_w, box_h = fo.box(VIEW, x, y, 0.2, 0.5)
            self.assertTrue(0 < box_x < fo.IMAGE_WIDTH and 0 < box_y < fo.IMAGE_HEIGHT)
            self.assertGreater(box_w, 0)
            self.assertGreater(box_h, 0)
        self.assertGreater(fo.box(VIEW, 1.0, 0.0, 0.2, 0.5)[2], fo.box(VIEW, 3.0, 0.0, 0.2, 0.5)[2])


class Photo(unittest.TestCase):
    def test_jpeg_is_valid(self):
        jpeg = fo.plain_jpeg()
        self.assertEqual(jpeg[:2], b"\xff\xd8")
        self.assertEqual(jpeg[-2:], b"\xff\xd9")
        self.assertLess(len(jpeg), 100 * 1024)
        try:
            from PIL import Image
        except ImportError:
            self.skipTest("no PIL to decode")
        image = Image.open(io.BytesIO(jpeg))
        image.load()
        self.assertEqual(image.size, (fo.IMAGE_WIDTH, fo.IMAGE_HEIGHT))


if __name__ == "__main__":
    unittest.main()
