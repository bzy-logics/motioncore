"""Simulated vision: a table scene, an overhead camera and a fake AI
detector. Nothing here talks to the robot.

The detector behaves like a real model would: it reports objects in *image
pixels* with a confidence, adds pixel noise, sometimes misses an object and
sometimes reports one that is not there, and takes time. The control code
only sees these detections, so it is exercised against imperfect vision.
Swap `FakeDetector` for a real model (same `detect()` output) without
touching the control code.
"""

from __future__ import annotations

import math
import random
import time
from dataclasses import dataclass, field

# Known object types: height above the table (mm). The grasp point is the
# centre of the top face.
OBJECT_HEIGHT_MM = {"cube": 25.0, "cylinder": 40.0}


@dataclass
class SceneObject:
    label: str
    x: float  # mm, robot base frame (table = z 0)
    y: float

    @property
    def height(self) -> float:
        return OBJECT_HEIGHT_MM[self.label]


@dataclass
class Scene:
    objects: list[SceneObject] = field(default_factory=list)

    @classmethod
    def random(cls, n: int, rng: random.Random, r_min: float = 170.0, r_max: float = 250.0,
               max_angle_deg: float = 70.0, min_spacing: float = 60.0) -> "Scene":
        """n objects in a ring in front of the robot (toward +X)."""
        objs: list[SceneObject] = []
        while len(objs) < n:
            r = rng.uniform(r_min, r_max)
            a = math.radians(rng.uniform(-max_angle_deg, max_angle_deg))
            x, y = r * math.cos(a), r * math.sin(a)
            if all(math.hypot(x - o.x, y - o.y) >= min_spacing for o in objs):
                objs.append(SceneObject(rng.choice(sorted(OBJECT_HEIGHT_MM)), round(x, 1), round(y, 1)))
        return cls(objs)

    def object_at(self, x: float, y: float, z: float, tol_xy: float, tol_z: float) -> SceneObject | None:
        """The object whose grasp point is within tolerance of (x, y, z)."""
        for o in self.objects:
            if math.hypot(o.x - x, o.y - y) <= tol_xy and abs(o.height - z) <= tol_z:
                return o
        return None


@dataclass
class Camera:
    """Fixed overhead pinhole camera looking straight down at the table.
    Image u grows with robot -Y, v with robot -X (camera mounted facing the
    robot). Calibration = these numbers; a real setup would measure them."""

    x: float = 210.0  # mm, camera centre above the table, robot base frame
    y: float = 0.0
    z: float = 900.0
    f_px: float = 800.0  # focal length in pixels
    width: int = 640
    height: int = 480

    def project(self, x: float, y: float, z: float) -> tuple[float, float]:
        s = self.f_px / (self.z - z)
        return self.width / 2 - (y - self.y) * s, self.height / 2 - (x - self.x) * s

    def unproject(self, u: float, v: float, z: float) -> tuple[float, float]:
        """Pixel -> robot (x, y) on the plane at height z."""
        s = (self.z - z) / self.f_px
        return self.x - (v - self.height / 2) * s, self.y - (u - self.width / 2) * s

    def sees(self, u: float, v: float) -> bool:
        return 0 <= u < self.width and 0 <= v < self.height


@dataclass
class Detection:
    label: str
    u: float  # bounding-box centre, pixels
    v: float
    w: float  # bounding-box size, pixels
    h: float
    confidence: float


@dataclass
class FakeDetector:
    """Stand-in for an object-detection model."""

    camera: Camera
    rng: random.Random
    noise_px: float = 1.5  # std-dev of the box centre
    miss_rate: float = 0.1  # chance a real object is not reported
    false_positive_rate: float = 0.15  # chance of one phantom per frame
    latency_s: float = 0.15  # inference time

    def detect(self, scene: Scene) -> list[Detection]:
        time.sleep(self.latency_s)
        out = []
        for o in scene.objects:
            if self.rng.random() < self.miss_rate:
                continue
            u, v = self.camera.project(o.x, o.y, o.height)
            if not self.camera.sees(u, v):
                continue
            size = 30.0 * self.camera.f_px / (self.camera.z - o.height)
            out.append(Detection(o.label, self.rng.gauss(u, self.noise_px), self.rng.gauss(v, self.noise_px),
                                 size, size, round(self.rng.uniform(0.7, 0.99), 2)))
        if self.rng.random() < self.false_positive_rate:
            out.append(Detection(self.rng.choice(sorted(OBJECT_HEIGHT_MM)),
                                 self.rng.uniform(0, self.camera.width), self.rng.uniform(0, self.camera.height),
                                 25.0, 25.0, round(self.rng.uniform(0.3, 0.75), 2)))
        return out
