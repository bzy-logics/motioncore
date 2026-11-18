"""Kinematics of the robot described by ratavyr's GET /api/config.

Built only from `axes[].geometry` (URDF joint convention, mm / deg):

    frame(child) = frame(parent) * T(origin) * RPY(rotation) * Joint(q about axis)

Provides forward kinematics, a numerical inverse kinematics solver and a port
of the engine's capsule collision model (crates/kinematics/src/collision.rs),
so the client can plan moves the engine will accept. The engine re-checks
every command and has the final word.

Standard library only.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass

Vec3 = tuple[float, float, float]
Mat4 = list[list[float]]

# Same surface clearance the engine requires (COLLISION_MARGIN_MM).
ENGINE_MARGIN_MM = 2.0


# ----------------------------------------------------------------- math

def _ident() -> Mat4:
    return [[1.0 if i == j else 0.0 for j in range(4)] for i in range(4)]


def _mul(a: Mat4, b: Mat4) -> Mat4:
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def _translate(v) -> Mat4:
    m = _ident()
    m[0][3], m[1][3], m[2][3] = v
    return m


def _rotate(u, angle: float) -> Mat4:
    s, c = math.sin(angle), math.cos(angle)
    t = 1.0 - c
    x, y, z = u
    return [
        [t * x * x + c, t * x * y - s * z, t * x * z + s * y, 0.0],
        [t * x * y + s * z, t * y * y + c, t * y * z - s * x, 0.0],
        [t * x * z - s * y, t * y * z + s * x, t * z * z + c, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _rpy(r) -> Mat4:
    d = math.pi / 180.0
    return _mul(_rotate((0, 0, 1), r[2] * d), _mul(_rotate((0, 1, 0), r[1] * d), _rotate((1, 0, 0), r[0] * d)))


def _apply(m: Mat4, p) -> Vec3:
    return (
        m[0][0] * p[0] + m[0][1] * p[1] + m[0][2] * p[2] + m[0][3],
        m[1][0] * p[0] + m[1][1] * p[1] + m[1][2] * p[2] + m[1][3],
        m[2][0] * p[0] + m[2][1] * p[1] + m[2][2] * p[2] + m[2][3],
    )


def _sub(a, b) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _dot(a, b) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _norm(a) -> float:
    return math.sqrt(_dot(a, a))


def _cross(a, b) -> Vec3:
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def segment_distance(p1, q1, p2, q2) -> float:
    """Exact minimum distance between segments p1-q1 and p2-q2."""
    d1, d2, r = _sub(q1, p1), _sub(q2, p2), _sub(p1, p2)
    a, e, f = _dot(d1, d1), _dot(d2, d2), _dot(d2, r)
    eps = 1e-12
    if a <= eps and e <= eps:
        return _norm(r)
    if a <= eps:
        s, t = 0.0, min(max(f / e, 0.0), 1.0)
    else:
        c = _dot(d1, r)
        if e <= eps:
            t, s = 0.0, min(max(-c / a, 0.0), 1.0)
        else:
            b = _dot(d1, d2)
            denom = a * e - b * b
            s = min(max((b * f - c * e) / denom, 0.0), 1.0) if denom > eps else 0.0
            t = (b * s + f) / e
            if t < 0.0:
                t, s = 0.0, min(max(-c / a, 0.0), 1.0)
            elif t > 1.0:
                t, s = 1.0, min(max((b - c) / a, 0.0), 1.0)
    c1 = (p1[0] + d1[0] * s, p1[1] + d1[1] * s, p1[2] + d1[2] * s)
    c2 = (p2[0] + d2[0] * t, p2[1] + d2[1] * t, p2[2] + d2[2] * t)
    return _norm(_sub(c1, c2))


def _solve(a: list[list[float]], b: list[float]) -> list[float]:
    """Solve a x = b (small dense system, partial pivoting)."""
    n = len(b)
    m = [row[:] + [b[i]] for i, row in enumerate(a)]
    for col in range(n):
        piv = max(range(col, n), key=lambda r: abs(m[r][col]))
        m[col], m[piv] = m[piv], m[col]
        p = m[col][col]
        if abs(p) < 1e-15:
            raise ZeroDivisionError("singular system")
        for r in range(col + 1, n):
            f = m[r][col] / p
            for c in range(col, n + 1):
                m[r][c] -= f * m[col][c]
    x = [0.0] * n
    for r in range(n - 1, -1, -1):
        x[r] = (m[r][n] - sum(m[r][c] * x[c] for c in range(r + 1, n))) / m[r][r]
    return x


# ----------------------------------------------------------------- model

@dataclass
class Joint:
    id: str
    parent: int | None  # index into RobotModel.joints (parents come first)
    origin: Vec3
    rotation: Vec3
    axis: Vec3
    tip: Vec3 | None
    radius: float
    prismatic: bool
    min: float
    max: float
    max_velocity: float


@dataclass
class Collision:
    a: str
    b: str  # "floor" for floor contact
    gap: float  # smallest surface gap, mm


class NoGeometry(Exception):
    """The robot configuration has no [axis.geometry]."""


class RobotModel:
    def __init__(self, joints: list[Joint], collision_checking: bool):
        self.joints = joints
        self.ids = [j.id for j in joints]
        self.collision_checking = collision_checking
        n = len(joints)
        # Collision bodies, exactly as the engine builds them.
        self._ends: list[list[Vec3]] = [[] for _ in range(n)]
        for i, j in enumerate(joints):
            if j.parent is not None:
                self._ends[j.parent].append(j.origin)
            if j.tip is not None:
                self._ends[i].append(j.tip)
        vol = [any(_norm(e) > 1e-6 for e in self._ends[i]) for i in range(n)]

        def touching(a: int, b: int) -> bool:
            for lo, hi in ((a, b), (b, a)):
                cur, empty = joints[hi].parent, True
                while cur is not None:
                    if cur == lo:
                        return empty
                    if vol[cur]:
                        empty = False
                    cur = joints[cur].parent
            return False

        self._pairs = [(a, b) for a in range(n) for b in range(a + 1, n)
                       if vol[a] and vol[b] and not touching(a, b)]
        # End effector: the joint with a tip, else the last joint.
        tipped = [i for i, j in enumerate(joints) if j.tip is not None]
        self.ee = tipped[-1] if tipped else n - 1
        chain, cur = [], self.ee
        while cur is not None:
            chain.append(cur)
            cur = joints[cur].parent
        self.chain = sorted(chain)  # joints that move the end effector

    @classmethod
    def from_config(cls, config: dict) -> "RobotModel":
        """Build from the JSON of GET /api/config. Raises NoGeometry."""
        axes = config["axes"]
        if not axes or any(a.get("geometry") is None for a in axes):
            raise NoGeometry("robot profile has no [axis.geometry]; IK needs it")
        order: list[int] = []
        while len(order) < len(axes):
            before = len(order)
            for i, a in enumerate(axes):
                if i in order:
                    continue
                p = a["geometry"]["parent"]
                if p is None or any(axes[o]["id"] == p for o in order):
                    order.append(i)
            if len(order) == before:
                raise ValueError("geometry parents do not form a tree")
        joints = []
        for i in order:
            a, g = axes[i], axes[i]["geometry"]
            parent = None if g["parent"] is None else next(k for k, o in enumerate(order) if axes[o]["id"] == g["parent"])
            joints.append(Joint(
                id=a["id"], parent=parent,
                origin=tuple(g["origin_mm"]), rotation=tuple(g["rotation_deg"]), axis=tuple(g["axis"]),
                tip=None if g["tip_mm"] is None else tuple(g["tip_mm"]),
                radius=g["radius_mm"] or 0.0, prismatic=a["kind"] == "prismatic",
                min=a["min"], max=a["max"], max_velocity=a["max_velocity"],
            ))
        return cls(joints, bool(config.get("collision_checking")))

    # ---- forward kinematics

    def frames(self, q: list[float]) -> list[Mat4]:
        out: list[Mat4] = []
        for i, j in enumerate(self.joints):
            base = out[j.parent] if j.parent is not None else _ident()
            v = q[i]
            if j.prismatic:
                motion = _translate((j.axis[0] * v, j.axis[1] * v, j.axis[2] * v))
            else:
                motion = _rotate(j.axis, math.radians(v))
            out.append(_mul(base, _mul(_mul(_translate(j.origin), _rpy(j.rotation)), motion)))
        return out

    def tool(self, q: list[float]) -> tuple[Vec3, Vec3]:
        """Tool point and pointing direction (unit) of the end effector."""
        f = self.frames(q)[self.ee]
        j = self.joints[self.ee]
        local = j.tip if j.tip is not None else (0.0, 0.0, 0.0)
        p = _apply(f, local)
        if j.tip is not None and _norm(j.tip) > 1e-9:
            o = _apply(f, (0.0, 0.0, 0.0))
            d = _sub(p, o)
        else:
            d = (f[0][2], f[1][2], f[2][2])  # frame z axis
        n = _norm(d)
        return p, (d[0] / n, d[1] / n, d[2] / n)

    # ---- collision (port of the engine's CollisionModel)

    def clearance(self, q: list[float]) -> Collision:
        f = self.frames(q)
        segs = []
        for i, ends in enumerate(self._ends):
            o = _apply(f[i], (0.0, 0.0, 0.0))
            segs.append([(o, _apply(f[i], e)) for e in ends if _norm(e) > 1e-6])
        worst = Collision("", "", math.inf)
        for a, b in self._pairs:
            for p1, q1 in segs[a]:
                for p2, q2 in segs[b]:
                    gap = segment_distance(p1, q1, p2, q2) - self.joints[a].radius - self.joints[b].radius
                    if gap < worst.gap:
                        worst = Collision(self.joints[a].id, self.joints[b].id, gap)
        for i, j in enumerate(self.joints):
            if j.parent is None:
                continue
            for p, e in segs[i]:
                gap = min(p[2], e[2]) - j.radius
                if gap < worst.gap:
                    worst = Collision(j.id, "floor", gap)
        return worst

    def check_sweep(self, q: list[float], axis: int, to: float, margin: float = ENGINE_MARGIN_MM) -> Collision | None:
        """Like the engine: moving one joint, others held. None = clear."""
        if not self.collision_checking:
            return None
        frm = q[axis]
        step = 1.0 if self.joints[axis].prismatic else 0.5
        n = min(max(math.ceil(abs(to - frm) / step), 1), 10_000)
        start = self.clearance(q).gap
        qs = list(q)
        for k in range(1, n + 1):
            qs[axis] = frm + (to - frm) * k / n
            c = self.clearance(qs)
            if c.gap < margin and c.gap < start - 1e-6:
                return c
        return None

    def check_path(self, q0: list[float], q1: list[float], margin: float = ENGINE_MARGIN_MM, samples: int = 200) -> Collision | None:
        """Straight joint-space path (what a coordinated MoveJ follows)."""
        if not self.collision_checking:
            return None
        start = self.clearance(q0).gap
        for k in range(1, samples + 1):
            q = [a + (b - a) * k / samples for a, b in zip(q0, q1)]
            c = self.clearance(q)
            if c.gap < margin and c.gap < start - 1e-6:
                return c
        return None

    def single_axis_order(self, q0: list[float], q1: list[float], margin: float = ENGINE_MARGIN_MM) -> list[int] | None:
        """An order of one-joint moves from q0 to q1 where every sweep is
        clear, or None. Depth-first search over the joints that change."""
        todo = [i for i in range(len(q0)) if abs(q1[i] - q0[i]) > 1e-6]

        def search(q, left):
            if not left:
                return []
            for i in left:
                if self.check_sweep(q, i, q1[i], margin) is None:
                    nq = list(q)
                    nq[i] = q1[i]
                    rest = search(nq, [k for k in left if k != i])
                    if rest is not None:
                        return [i] + rest
            return None

        return search(list(q0), todo)

    # ---- inverse kinematics

    def solve_ik(self, target: Vec3, direction: Vec3, seed: list[float], *,
                 tol_mm: float = 0.5, tol_deg: float = 1.0, attempts: int = 12,
                 min_clearance: float = ENGINE_MARGIN_MM + 3.0, rng: random.Random | None = None) -> list[float] | None:
        """Joint values putting the tool point at `target` (mm) pointing along
        `direction`, within joint limits and with collision clearance.
        Tries `seed` first, then random restarts; returns the solution
        closest to `seed`, or None."""
        rng = rng or random.Random(0)
        best, best_cost = None, math.inf
        for k in range(attempts):
            start = list(seed) if k == 0 else [
                rng.uniform(j.min, j.max) if i in self.chain else seed[i] for i, j in enumerate(self.joints)]
            q = self._dls(start, target, direction)
            if q is None:
                continue
            p, d = self.tool(q)
            ang = math.degrees(math.acos(max(-1.0, min(1.0, _dot(d, direction)))))
            if _norm(_sub(p, target)) > tol_mm or ang > tol_deg:
                continue
            if self.collision_checking and self.clearance(q).gap < min_clearance:
                continue
            cost = sum(abs(a - b) / max(j.max_velocity, 1e-9) for a, b, j in zip(q, seed, self.joints))
            if cost < best_cost:
                best, best_cost = q, cost
            if k == 0:
                break  # the seed converged: keep the nearby solution
        return best

    def _dls(self, q: list[float], target: Vec3, direction: Vec3, iters: int = 200) -> list[float] | None:
        """Damped least squares on [position error; direction error]."""
        w_rot = 100.0  # mm per rad: weighs pointing against position
        lam = 2.0
        chain = self.chain
        q = list(q)

        def error(qq):
            p, d = self.tool(qq)
            ep = _sub(target, p)
            er = _cross(d, direction)  # small-angle rotation taking d to direction
            if _dot(d, direction) < 0:  # pointing the wrong way: push harder
                er = tuple(x + y for x, y in zip(er, _cross(d, (1e-3, 2e-3, 1.0))))
            return [ep[0], ep[1], ep[2], w_rot * er[0], w_rot * er[1], w_rot * er[2]]

        best_err, stalled = math.inf, 0
        for _ in range(iters):
            e = error(q)
            if math.sqrt(sum(x * x for x in e[:3])) < 0.05 and math.sqrt(sum(x * x for x in e[3:])) < 0.05:
                return q
            err = math.sqrt(sum(x * x for x in e))
            if err < best_err * 0.99:
                best_err, stalled = err, 0
            else:
                stalled += 1
                if stalled >= 15:  # stuck at a joint limit or out of reach
                    break
            # numerical Jacobian, columns per chain joint (per deg or mm)
            cols = []
            for i in chain:
                h = 1e-4
                qh = list(q)
                qh[i] += h
                eh = error(qh)
                cols.append([(e[r] - eh[r]) / h for r in range(6)])  # d(tool)/dq = -d(err)/dq
            jjt = [[sum(c[r] * c[s] for c in cols) + (lam * lam if r == s else 0.0) for s in range(6)] for r in range(6)]
            try:
                y = _solve(jjt, e)
            except ZeroDivisionError:
                return None
            dq = [sum(c[r] * y[r] for r in range(6)) for c in cols]
            scale = max(1.0, max(abs(x) for x in dq) / 10.0)  # at most 10 deg/mm per step
            for k, i in enumerate(chain):
                j = self.joints[i]
                q[i] = min(max(q[i] + dq[k] / scale, j.min), j.max)
        e = error(q)
        return q if math.sqrt(sum(x * x for x in e[:3])) < 1.0 else None
