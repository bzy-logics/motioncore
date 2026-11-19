# sim-vision-pick

A simulated camera and a fake AI object detector drive a robot arm through MotionCore's public API. The client finds objects on a table, picks them up and drops them in a bin.

It runs **entirely on the client PC**. The MotionCore host keeps running what it always runs (engine + ratavyr, with the virtual MCU or a real board) and only sees ordinary REST commands: the same ones sindrava sends, validated the same way.

```text
CLIENT PC                                        MOTIONCORE HOST
┌──────────────────────────────────┐             ┌──────────────────────────────┐
│ sim_vision.py                    │             │ ratavyr (REST + WebSocket)   │
│   table scene, overhead camera,  │             │   │ gRPC, local only         │
│   fake detector (noise, misses,  │ HTTP + token│ motioncore engine            │
│   phantoms, latency)             │ ──────────▶ │   state, soft limits,        │
│ robot_model.py                   │ ◀────────── │   collision check            │
│   FK / IK / collision model      │  telemetry  │   │                          │
│ sim_vision_pick.py               │             │ virtual MCU (or real board)  │
│   pick loop, waits on telemetry  │             └──────────────────────────────┘
│ sindrava in a browser (watch)    │
└──────────────────────────────────┘
```

**Performance.** The detector, IK and planning cost the host nothing. Per move, the host handles a few small HTTP requests, the same as an operator clicking in sindrava. Step timing stays on the MCU. Live state comes from the WebSocket stream (about 20 Hz) rather than polling.

## Requirements

- **Python 3.11 or newer, standard library only.** Nothing to `pip install`.
- **A robot profile with `[axis.geometry]`:** IK is built from the geometry ratavyr reports in `GET /api/config`. This example is tuned for `robot-profiles/dummy-arm-virtual.toml`, a six-axis desktop arm.
- **An operator token**, or authentication off when everything runs on one PC.

## Run it on one PC

```bash
# 1. engine with the six-axis arm on the virtual MCU (motioncore repo)
cargo build
target/debug/motioncore start --config robot-profiles/dummy-arm-virtual.toml

# 2. gateway (ratavyr repo), here with an operator token
MOTIONCORE_OPERATOR_TOKEN=<long random token> go run ./cmd/server     # http://127.0.0.1:8080

# 3. optional: sindrava (npm run dev) to watch the arm in the 3D viewer

# 4. the client (this directory)
export MOTIONCORE_URL=http://127.0.0.1:8080
export MOTIONCORE_TOKEN=<the operator token>
python sim_vision_pick.py --prepare
```

`--prepare` enables the drives and runs home-all if needed, after asking you to confirm that the robot is in the home-all start pose. Without it, the client expects a robot that is already enabled and homed, for example from sindrava.

## Run it from another PC

On the **MotionCore host**, let ratavyr accept connections from the network. Configuration only, no code changes:

```bash
MOTIONCORE_API_ADDR=0.0.0.0:8080 \
MOTIONCORE_OPERATOR_TOKEN=<long random token> \
go run ./cmd/server
```

- ratavyr refuses a non-loopback address without a token, on purpose.
- For anything beyond a lab network, put a TLS reverse proxy in front: see [ratavyr: deployment](https://ratavyr.bzy-logics.com/guide/deployment). The client accepts `https://` URLs and then uses `wss://` for telemetry.

On the **client PC**:

```bash
export MOTIONCORE_URL=http://<host-ip>:8080
export MOTIONCORE_TOKEN=<the operator token>
python sim_vision_pick.py
```

A Python client sends no `Origin` header, so `MOTIONCORE_ALLOWED_ORIGINS` doesn't apply. That setting is only for browsers.

## What a run looks like

```text
ratavyr 0.3.0, engine 0.3.0 (API 0.3.0)
robot dummy-arm: 6 joints, collision checking on
scene (ground truth, hidden from the controller): cube@(119,136), cylinder@(190,-2), ...
  observe pose: MoveJ
...
cycle 4: 2 detection(s) >= 0.6: cube 0.81, cube 0.92
target cube (0.92) at (190.0, -117.3) mm, grasp height 25 mm
  approach: MoveJ
  descend: MoveL
  gripper close at (190.0, -117.3, 25.0) -> got cube
  lift: MoveL
  to bin: MoveJ
  gripper open: cube placed
  observe pose: MoveJ
...
```

With more detector noise (`--noise-px 6`), some grasps miss: `gripper close at ... -> nothing there`. The client lifts, looks again and retries.

Each cycle:

1. **Observe:** go to the observe pose and run the detector.
2. **Choose a target:** take the most confident detection and turn its pixel position into table coordinates using the camera calibration.
3. **Approach:** solve IK for a point 60 mm above the object with the tool pointing down, and go there with a coordinated MoveJ.
4. **Descend:** a MoveL straight down to the grasp point, orientation held.
5. **Grasp:** the simulated gripper closes. It only succeeds if the robot's measured pose, from telemetry, really is at an object.
6. **Place:** a MoveL straight up, a MoveJ to the bin, and release.

The controller never reads the scene's ground truth. It decides that the table is clear when three frames in a row show nothing new to pick.

## Options

| Option | Default | |
|---|---|---|
| `--url` / `MOTIONCORE_URL` | `http://127.0.0.1:8080` | ratavyr base URL |
| `--token` / `MOTIONCORE_TOKEN` | – | Operator token (prefer the env var) |
| `--objects` | 4 | Objects in the scene |
| `--max-radius` | 250 | Objects are placed up to this far from the base, mm. Go higher to see out-of-reach handling |
| `--noise-px` | 1.5 | Detector box-centre noise (std-dev, px) |
| `--miss-rate` | 0.1 | Chance an object is not detected |
| `--false-positives` | 0.15 | Chance of a phantom detection per frame |
| `--min-confidence` | 0.6 | Ignore weaker detections |
| `--speed` | 0.5 | Fraction of each joint's `max_velocity` |
| `--motion` | `auto` | `auto`: MoveJ, plus MoveL for descend and lift. `movej`: MoveJ only. `single`: one joint at a time |
| `--line-speed` | 40 | MoveL tool speed for descend and lift, mm/s |
| `--seed` | random | Reproducible scene and detector |
| `--prepare` / `--yes` | off | Enable and home if needed / without asking |
| `--dry-run` | off | Vision and planning only; sends no commands (a viewer token is enough) |
| `--allow-hardware` | off | See Safety below |

## Safety

- **Simulation only by default.** The client refuses to run unless the MCU transport is `virtual`. `--allow-hardware` exists for a guarded machine with a hardware E-stop, not for a first try.
- **The engine has the final word.**
  - The client plans with its own copy of the engine's collision model (`robot_model.py`, tested against the engine's own test cases).
  - Every command is still checked by ratavyr: the token's role, and rate limits.
  - The engine checks each command again: robot state, homing, soft limits and collisions. A refused move (`409 collision`, `400 validation_failed`) skips that target.
- **It stops on anything unexpected.**
  - It aborts and sends `stop` on stale telemetry (no data for 1.5 s), timeouts, errors and Ctrl+C.
  - On a fault or emergency stop it aborts at once, and it never resets faults itself: that is a person's decision, made in sindrava.
- **The hardware E-stop is the real safety layer.** The client is just another operator.

## Motion commands

| Move | Command | Since |
|---|---|---|
| Observe, approach, to bin | `move_j` with `targets` and `speed_scale` (`--speed`): all joints start and finish together on a straight joint-space path | engine-api 0.2 (MotionCore 0.3.0) |
| Descend, lift | `move_l` with `position_mm` and `speed_mm_s` (`--line-speed`): the tool point moves in a straight line, orientation held | engine-api 0.3 (MotionCore 0.3.0) |

- **Before each `move_j`** the client checks the straight joint-space path against its collision model. The engine checks it again.
- **A `move_l` is done** when telemetry shows the robot `ready` with `tool.position_mm` within 0.5 mm of the target.
- **Older engines** answer `404 unknown_command`. The client then falls back for the rest of the run:
  - joint moves for descend and lift;
  - single-joint `move_to` commands, ordered so that every sweep passes the collision model as the engine checks it.

## Use a real detector

`FakeDetector.detect(scene)` returns `Detection(label, u, v, w, h, confidence)` in image pixels. Replace it with a real model and camera that return the same thing, and update `Camera` with your calibration (or replace `unproject` with a proper hand-eye calibration). The control code doesn't change.

## Limits (inputs for the roadmap)

- **No IK API:** the client solves IK itself for the approach and bin poses. `move_l` solves the straight lines engine-side.
- **No gripper or I/O in MotionCore:** the gripper is simulated in the client.
- **Detections can't be shown in sindrava's 3D viewer:** that would need a scene-markers feature.
- **It is not a plugin:** it uses the public API like any other client. A future out-of-process plugin could do the same once plugins can read telemetry and command motion.

## Tests

```bash
python -m unittest -v      # offline, no server needed
```

They cover:
- forward kinematics against the Dummy arm firmware;
- the collision model against the engine's own test cases;
- IK reach, limits and clearance;
- the order of single-joint moves;
- the camera round trip;
- the detector;
- the `move_j` request body.
