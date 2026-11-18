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
robot dummy-arm: 6 joints, collision checking on
scene (ground truth, hidden from the controller): cube@(148,-170), cylinder@(182,16), ...
cycle 1: 4 detection(s) >= 0.5: cube 0.86, cylinder 0.75, cube 0.84, cylinder 0.97
target cylinder (0.97) at (121.0, 135.5) mm, grasp height 40 mm
  coordinated MoveJ unavailable (unknown_command: ...); moving joints one at a time from now on
  approach: joints one at a time (j1 -> j2 -> j3 -> j5)
  descend: joints one at a time (j2 -> j3 -> j5)
  gripper close at (121.0, 135.5, 40.0) -> got cylinder
  ...
cycle 4: ... target cube (0.90) at (251.6, -63.8) mm
  gripper close at (251.6, -63.9, 25.0) -> nothing there        <- 6 px of detector noise
...
done: 5 picked, 1 missed grasps, 0 skipped, 0 left on the table
```

Each cycle:

1. **Observe:** go to the observe pose and run the detector.
2. **Choose a target:** take the most confident detection and turn its pixel position into table coordinates using the camera calibration.
3. **Approach and descend:** solve IK for a point above the object, then for the grasp point, with the tool pointing down.
4. **Grasp:** the simulated gripper closes. It only succeeds if the robot's measured pose, from telemetry, really is at an object.
5. **Place:** lift, move to the bin and release.

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
| `--motion` | `auto` | `auto`: MoveJ if the server has it, else one joint at a time. Also `movej` or `single` |
| `--movej-command` | `move-j` | Command name of the coordinated move |
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

## Coordinated motion (MoveJ)

Today the engine moves **one joint per command**. The client therefore orders single-joint moves so that every intermediate pose stays clear: it searches for an order in which each sweep passes the collision model, as the engine checks it.

Coordinated MoveJ is being added in milestone **v0.3** (`POST /api/commands/move-j`). With `--motion auto`, the client tries it first:

- **404 `unknown_command` or 400 `bad_request`:** it falls back to single-joint moves for the rest of the run. A missing or different contract never moves the robot.
- **The command exists:** all joints start and finish together. Before sending it, the client checks the straight joint-space path against the collision model.

The request body is built in one place, `movej_params()` in `sim_vision_pick.py`. When the v0.3 contract is final, adapt that function and its test.

## Use a real detector

`FakeDetector.detect(scene)` returns `Detection(label, u, v, w, h, confidence)` in image pixels. Replace it with a real model and camera that return the same thing, and update `Camera` with your calibration (or replace `unproject` with a proper hand-eye calibration). The control code doesn't change.

## Limits (inputs for the roadmap)

- **Joints move one at a time** until MoveJ lands, so approach and descend are not straight lines.
- **No Cartesian or IK API:** the client does its own IK.
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
- the MoveJ request body.
