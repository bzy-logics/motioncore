#!/usr/bin/env python3
"""sim-vision-pick: simulated camera + fake AI detector driving a MotionCore
robot through ratavyr's public API.

Runs entirely on the client PC. The MotionCore host only sees ordinary REST
commands (the same ones sindrava sends) and validates every one of them.

    MOTIONCORE_URL=http://127.0.0.1:8080 MOTIONCORE_TOKEN=... python sim_vision_pick.py --prepare

See README.md. Standard library only; Python 3.10+.
"""

from __future__ import annotations

import argparse
import math
import os
import random
import sys
import time

from motioncore_client import Api, ApiError, StaleTelemetry, Telemetry
from robot_model import NoGeometry, RobotModel
from sim_vision import OBJECT_HEIGHT_MM, Camera, FakeDetector, Scene

__version__ = "0.1.0"

DOWN = (0.0, 0.0, -1.0)  # tool points at the table
POS_TOL = 0.05  # deg / mm: "arrived" tolerance on measured positions
CMD_TOL = 0.01  # commanded position vs. target (step quantisation)
MIN_MOVE = 0.02  # joint changes smaller than this are not sent
BAD_STATES = {"faulted", "emergency_stopped", "disconnected"}


def log(msg: str) -> None:
    print(f"{time.strftime('%H:%M:%S')}  {msg}", flush=True)


class RobotFault(Exception):
    pass


class Unreachable(Exception):
    pass


class Robot:
    """Joint-space motion through ratavyr, waiting on live telemetry.

    Uses a coordinated MoveJ command when the server has one (all joints
    start and finish together on a straight joint-space path); otherwise
    moves one joint at a time, in an order that the client-side collision
    model says is clear. Either way the engine re-validates every command.
    """

    def __init__(self, api: Api, tel: Telemetry, model: RobotModel, ids: list[str],
                 speed: float, motion: str, movej_command: str, dry_run: bool):
        self.api, self.tel, self.model = api, tel, model
        self.ids = ids  # model joint order -> axis ids
        self.speed = speed
        self.movej = motion in ("auto", "movej")
        self.motion = motion
        self.movej_command = movej_command
        self.dry_run = dry_run
        self.dry_q: list[float] | None = None

    # ---- telemetry helpers

    def _axes(self, t: dict) -> dict:
        return {a["id"]: a for a in t["axes"]}

    def q(self) -> list[float]:
        if self.dry_run and self.dry_q is not None:
            return list(self.dry_q)
        t = self.tel.latest()
        if t is None:
            raise StaleTelemetry("no live telemetry")
        ax = self._axes(t)
        return [ax[i]["measured_position"] for i in self.ids]

    def _check(self, t: dict) -> None:
        if t["state"] in BAD_STATES:
            f = t.get("fault") or {}
            raise RobotFault(f"robot is {t['state']}" + (f": {f.get('code')} {f.get('message', '')}" if f else ""))

    def _wait_at(self, target: list[float], moved: list[int], timeout: float) -> None:
        """Wait until the move is done. The engine sets an axis's
        commanded_position to the target when it accepts the move, in the
        same step as entering `running`, so `ready` + commanded at target
        can only be seen after this move (not a frame from before it)."""
        def arrived(t):
            self._check(t)
            if t["state"] != "ready":
                return False
            ax = self._axes(t)
            if any(abs(ax[self.ids[i]]["commanded_position"] - target[i]) > CMD_TOL for i in moved):
                return False
            return all(abs(ax[i]["measured_position"] - v) <= POS_TOL and abs(ax[i]["velocity"]) < 1e-6
                       for i, v in zip(self.ids, target))
        self.tel.wait(arrived, timeout)

    def _require_ready(self) -> None:
        t = self.tel.latest()
        if t is None:
            raise StaleTelemetry("no live telemetry")
        self._check(t)
        if t["state"] != "ready":
            raise RobotFault(f"robot is {t['state']}, not ready")

    def _duration(self, i: int, frm: float, to: float) -> float:
        v = self.model.joints[i].max_velocity * self.speed
        return abs(to - frm) / max(v, 1e-9)

    # ---- motion

    def move(self, target: list[float], what: str) -> None:
        q0 = self.q()
        if all(abs(a - b) <= MIN_MOVE for a, b in zip(q0, target)):
            return
        if self.dry_run:
            log(f"  [dry-run] {what}: " + ", ".join(f"{i}={v:.1f}" for i, v in zip(self.ids, target)))
            self.dry_q = list(target)
            return
        if self.movej and self._try_movej(q0, target, what):
            return
        target = [t if abs(t - c) > MIN_MOVE else c for t, c in zip(target, q0)]
        order = self.model.single_axis_order(q0, target)
        if order is None:
            raise Unreachable(f"{what}: no collision-free order of single-joint moves")
        log(f"  {what}: joints one at a time ({' -> '.join(self.ids[i] for i in order)})")
        for i in order:
            self._require_ready()
            self.api.command("move_to", {"axis": self.ids[i], "position": round(target[i], 4),
                                         "velocity": round(self.model.joints[i].max_velocity * self.speed, 3)})
            partial = self.q()  # the other joints stay where they are
            partial[i] = target[i]
            self._wait_at(partial, [i], self._duration(i, q0[i], target[i]) + 10.0)

    def _try_movej(self, q0: list[float], target: list[float], what: str) -> bool:
        """Coordinated move. Returns False (and stops trying) when the
        server has no such command or its contract differs from this one."""
        c = self.model.check_path(q0, target)
        if c is not None:
            raise Unreachable(f"{what}: straight joint path collides ({c.a} / {c.b}, gap {c.gap:.1f} mm)")
        self._require_ready()
        try:
            self.api.command(self.movej_command, movej_params(self.ids, target))
        except ApiError as e:
            if e.code == "unknown_command" or (e.code == "bad_request" and self.motion == "auto"):
                log(f"  coordinated MoveJ unavailable ({e.code}: {e.message}); moving joints one at a time from now on")
                self.movej = False
                return False
            raise
        log(f"  {what}: coordinated MoveJ")
        slowest = max(self._duration(i, q0[i], target[i]) for i in range(len(target)))
        moved = [i for i in range(len(target)) if abs(target[i] - q0[i]) > MIN_MOVE]
        self._wait_at(target, moved, slowest + 10.0)
        return True

    def stop(self) -> None:
        try:
            self.api.command("stop")
        except ApiError as e:
            log(f"stop failed: {e} -- use the hardware E-stop if the robot is moving")


def movej_params(ids: list[str], target: list[float]) -> dict:
    """Body of the coordinated MoveJ command.

    MoveJ is being added in milestone v0.3 (coordinated motion); its REST
    contract is not final. This is the only place to adapt once it is:
    with --motion auto, a 404 unknown_command or 400 bad_request falls back
    to single-joint moves, so a mismatch never moves the robot.
    """
    return {"targets": {i: round(v, 4) for i, v in zip(ids, target)}}


# ----------------------------------------------------------------- setup

def prepare(api: Api, tel: Telemetry, config: dict, do_prepare: bool, assume_yes: bool) -> None:
    t = tel.wait(lambda t: True, 5.0)
    if t["state"] in ("faulted", "emergency_stopped"):
        raise RobotFault(f"robot is {t['state']}; check it and press Reset fault in sindrava first")
    need_enable = t["state"] == "disabled"
    need_home = not all(a["homed"] for a in t["axes"])
    if not (need_enable or need_home):
        if t["state"] != "ready":
            raise RobotFault(f"robot is {t['state']}, not ready")
        return
    todo = [s for s, n in (("enable the drives", need_enable), ("home all axes", need_home)) if n]
    if not do_prepare:
        raise RobotFault(f"robot needs to {' and '.join(todo)}; do it in sindrava, or run with --prepare")
    if need_home:
        plan = config.get("home_all") or {}
        if plan.get("start_pose"):
            log(f"home-all start pose: {plan['start_pose']}")
    if not assume_yes:
        if input(f"About to {' and '.join(todo)}. Robot clear and in the start pose? [y/N] ").strip().lower() != "y":
            raise SystemExit("aborted")
    if need_enable:
        log("enabling drives")
        api.command("enable")
        tel.wait(lambda t: t["state"] not in ("disabled", "initializing"), 10.0)
    if need_home:
        log("homing all axes")
        api.command("home_all")
        def homed(t):
            if t["state"] in BAD_STATES:
                raise RobotFault(f"homing stopped: robot is {t['state']}")
            return t["state"] == "ready" and all(a["homed"] for a in t["axes"])
        tel.wait(homed, 180.0)
    log("robot ready")


# ----------------------------------------------------------------- main

def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0], formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--url", default=os.environ.get("MOTIONCORE_URL", "http://127.0.0.1:8080"),
                   help="ratavyr base URL (env MOTIONCORE_URL)")
    p.add_argument("--token", default=os.environ.get("MOTIONCORE_TOKEN"),
                   help="operator token (env MOTIONCORE_TOKEN; prefer the env var)")
    p.add_argument("--objects", type=int, default=4, help="objects in the simulated scene")
    p.add_argument("--seed", type=int, default=None, help="random seed for scene and detector")
    p.add_argument("--max-radius", type=float, default=250.0,
                   help="objects are placed up to this distance from the base, mm")
    p.add_argument("--noise-px", type=float, default=1.5, help="detector box-centre noise (std-dev, px)")
    p.add_argument("--miss-rate", type=float, default=0.1, help="chance the detector misses an object")
    p.add_argument("--false-positives", type=float, default=0.15, help="chance of a phantom detection per frame")
    p.add_argument("--min-confidence", type=float, default=0.6)
    p.add_argument("--speed", type=float, default=0.5, help="fraction of each joint's max velocity (0-1]")
    p.add_argument("--motion", choices=("auto", "movej", "single"), default="auto",
                   help="auto: coordinated MoveJ if the server has it, else one joint at a time")
    p.add_argument("--movej-command", default="move-j", help="command name of the coordinated move")
    p.add_argument("--max-cycles", type=int, default=20)
    p.add_argument("--prepare", action="store_true", help="enable and home the robot if needed")
    p.add_argument("--yes", action="store_true", help="don't ask before enabling / homing")
    p.add_argument("--dry-run", action="store_true", help="vision and planning only; send no commands")
    p.add_argument("--allow-hardware", action="store_true",
                   help="run against a real MCU (default: refuse unless the MCU transport is virtual)")
    a = p.parse_args(argv)
    if not 0 < a.speed <= 1:
        p.error("--speed must be in (0, 1]")

    rng = random.Random(a.seed)
    api = Api(a.url, a.token)
    try:
        info = api.get("/api/info")
    except ApiError as e:
        log(f"cannot reach ratavyr: {e}")
        return 2
    if info.get("engine") is None:
        log(f"ratavyr is up but the engine is not connected: {info.get('engine_error')}")
        return 2
    log(f"ratavyr {info['server_version']}, engine {info['engine']['engine_version']} (API {info['engine']['api_version']})")
    try:
        session = api.get("/api/session")
        config = api.get("/api/config")
    except ApiError as e:
        log(f"{e}" + (" -- set MOTIONCORE_TOKEN to an operator token" if e.status == 401 else ""))
        return 2
    if not a.dry_run and not session["permissions"]["command"]:
        log(f"your role is {session['role']}; moving the robot needs operator or admin")
        return 2
    try:
        model = RobotModel.from_config(config)
    except NoGeometry as e:
        log(f"{config['name']}: {e} (try robot-profiles/dummy-arm-virtual.toml)")
        return 2
    ids = model.ids
    log(f"robot {config['name']}: {len(ids)} joints, collision checking "
        f"{'on' if model.collision_checking else 'off'}")

    tel = Telemetry(api, info["auth_required"], log)
    robot = Robot(api, tel, model, ids, a.speed, a.motion, a.movej_command, a.dry_run)
    picked = missed = skipped = 0
    try:
        t = tel.wait(lambda t: True, 5.0)
        if t["mcu"]["transport"] != "virtual" and not a.allow_hardware:
            log(f"MCU transport is {t['mcu']['transport']!r}, not virtual: this is a simulation demo. "
                "Use --allow-hardware only on a guarded machine with a hardware E-stop.")
            return 2
        if not a.dry_run:
            prepare(api, tel, config, a.prepare, a.yes)

        camera = Camera()
        detector = FakeDetector(camera, rng, noise_px=a.noise_px, miss_rate=a.miss_rate,
                                false_positive_rate=a.false_positives)
        scene = Scene.random(a.objects, rng, r_max=max(a.max_radius, 180.0))
        bin_xy, bin_z = (-60.0, -220.0), 120.0
        log("scene (ground truth, hidden from the controller): " +
            ", ".join(f"{o.label}@({o.x:.0f},{o.y:.0f})" for o in scene.objects))

        home_q = model.solve_ik((200.0, 0.0, 180.0), DOWN, robot.q())
        if home_q is None:
            log("cannot find an observation pose for this robot")
            return 2
        failed_spots: list[tuple[float, float]] = []
        idle_frames = 0  # frames in a row with nothing new to pick

        for cycle in range(1, a.max_cycles + 1):
            robot.move(home_q, "observe pose")
            dets = [d for d in detector.detect(scene) if d.confidence >= a.min_confidence]
            log(f"cycle {cycle}: {len(dets)} detection(s) >= {a.min_confidence}: " +
                ", ".join(f"{d.label} {d.confidence:.2f}" for d in dets))
            cands = []
            for d in dets:
                h = OBJECT_HEIGHT_MM[d.label]
                x, y = camera.unproject(d.u, d.v, h)
                if any(math.hypot(x - fx, y - fy) < 15 for fx, fy in failed_spots):
                    continue
                cands.append((d, x, y, h))
            if not cands:
                # Decided from what the camera sees, never from the scene's
                # ground truth (the controller can't know it).
                idle_frames += 1
                if idle_frames >= 3:
                    log("nothing to pick in 3 frames in a row: table looks clear")
                    break
                log("nothing (new) to pick this frame; looking again")
                continue
            idle_frames = 0
            d, x, y, h = max(cands, key=lambda c: c[0].confidence)
            log(f"target {d.label} ({d.confidence:.2f}) at ({x:.1f}, {y:.1f}) mm, grasp height {h:.0f} mm")
            q = robot.q()
            above = model.solve_ik((x, y, h + 60.0), DOWN, q, rng=rng)
            grasp = above and model.solve_ik((x, y, h), DOWN, above, rng=rng)
            if not grasp:
                log("  out of reach (no IK solution with clearance); skipping")
                skipped += 1
                failed_spots.append((x, y))
                continue
            try:
                robot.move(above, "approach")
                robot.move(grasp, "descend")
            except (Unreachable, ApiError) as e:
                if isinstance(e, ApiError) and e.code not in ("collision", "validation_failed", "not_homed"):
                    raise
                log(f"  refused: {e}; skipping this target")
                skipped += 1
                failed_spots.append((x, y))
                continue
            # Simulated gripper: grasp succeeds only if the tool, at the
            # robot's *measured* pose, is really at an object.
            (tx, ty, tz), _ = model.tool(robot.q())
            obj = scene.object_at(tx, ty, tz, tol_xy=8.0, tol_z=5.0)
            log(f"  gripper close at ({tx:.1f}, {ty:.1f}, {tz:.1f}) -> {'got ' + obj.label if obj else 'nothing there'}")
            robot.move(above, "lift")
            if obj is None:
                missed += 1
                failed_spots.append((x, y))
                continue
            scene.objects.remove(obj)
            drop = model.solve_ik((bin_xy[0], bin_xy[1], bin_z), DOWN, robot.q(), rng=rng)
            if drop is None:
                log("  bin out of reach; dropping the object back at the observe pose")
            else:
                robot.move(drop, "to bin")
            log(f"  gripper open: {obj.label} placed")
            picked += 1
        robot.move(home_q, "observe pose")
    except KeyboardInterrupt:
        log("interrupted: sending stop")
        robot.stop()
        return 130
    except (RobotFault, StaleTelemetry, TimeoutError, Unreachable, ApiError) as e:
        log(f"aborting: {e}")
        if not isinstance(e, RobotFault):
            robot.stop()
        return 1
    except BaseException:
        robot.stop()  # never leave the robot moving on an unexpected error
        raise
    finally:
        tel.close()
    log(f"done: {picked} picked, {missed} missed grasps, {skipped} skipped, {len(scene.objects)} left on the table")
    return 0


if __name__ == "__main__":
    sys.exit(main())
