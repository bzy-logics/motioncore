# CLI reference

`motioncore` is the headless runtime and tooling. It runs the engine and talks to a running engine; neither the Go gateway nor the web UI is required.

```text
motioncore [--engine ADDR] <command>
```

## Global option

| Option | Env | Meaning |
|---|---|---|
| `--engine ADDR` | `MOTIONCORE_ENGINE_ADDR` | Engine gRPC address for client commands: `unix:///path` or `tcp://host:port`. Default: `unix:///run/motioncore/engine.sock` on Linux, `tcp://127.0.0.1:7600` elsewhere. |

## Running the engine

### `start`

Run the engine: state authority, motion pipeline, MCU link and gRPC API.

```bash
motioncore start [--config FILE] [--listen ADDR] [--plugins DIR] [--repo PATH] [--enable-fault-injection]
```

| Option | Default | Meaning |
|---|---|---|
| `--config FILE` | `robot-profiles/single-axis-virtual.toml` | Robot configuration |
| `--listen ADDR` | `--engine` address or platform default | gRPC listen address |
| `--plugins DIR` | – | Directory with external plugin manifests (`*/plugin.toml`) |
| `--repo PATH` | auto-detected from the working directory | motioncore source tree for firmware builds. Without it, builds are disabled |
| `--enable-fault-injection` | off | Enable debug fault-injection RPCs. **Never with real hardware** |

Log verbosity follows `RUST_LOG` (default `info`). On Ctrl-C the engine issues an emergency stop before exiting, leaving hardware safe.

### `simulator`

Run a virtual MCU on TCP. Each connection is a fresh boot.

```bash
motioncore simulator [--listen 127.0.0.1:7700] [--config FILE] [--mcu main]
```

With `--config`, the simulator takes the axis geometry (endstop positions, etc.) from the profile's MCU `--mcu`, forcing it to be virtual. Point an engine at it with `transport = "tcp://127.0.0.1:7700"`.

## Inspecting

| Command | Prints |
|---|---|
| `motioncore info` | Engine version, API version, MCU protocol version, plugin API version, fault injection on/off |
| `motioncore status` | Current telemetry: state, fault, MCU link, axes |
| `motioncore config` | The loaded robot configuration |
| `motioncore plugins` | Each plugin's id, version, kind and state (with error, if failed) |
| `motioncore watch` | Streams telemetry and log events until Ctrl-C |

## Commanding

| Command | Effect |
|---|---|
| `motioncore enable` | Energise drives: `disabled` → `ready` |
| `motioncore disable` | Controlled stop if moving, then de-energise |
| `motioncore stop` | Controlled stop; drives stay enabled |
| `motioncore estop` | Software emergency stop: drives disabled immediately, latched. Complements, never replaces, a hardware E-stop |
| `motioncore reset-fault` | Clear a latched fault. Never resumes motion; axes must be re-homed |
| `motioncore home AXIS` | Run the axis's homing procedure |
| `motioncore home-all` | Home every axis, following the profile's `[home_all]` plan (one at a time without one) |
| `motioncore jog AXIS DELTA [--velocity V]` | Relative move in axis units. Negative values allowed: `jog j1 -5` |
| `motioncore move AXIS POSITION [--velocity V]` | Absolute move in axis units. The axis must be homed |

Commands return when the engine accepts them. Use `status` or `watch` to follow progress. See [Moving axes](/guide/motion).

## MCUs and firmware

### `mcu list`

List serial ports, to find your board's device.

### `firmware targets`

List supported boards: id, firmware target, Rust target, step channels, maximum step rate, flash method, and whether the board is provisional.

### `firmware configure`

```bash
motioncore firmware configure --config FILE [--mcu NAME]
```

Resolve the configuration and write the lockfile. Prints the lockfile path, build hash and contents. Useful to validate a profile.

### `firmware build`

```bash
motioncore firmware build --config FILE [--mcu NAME]   # → build/firmware/<mcu>/
motioncore firmware build --board BOARD                # reference profile → build/firmware/<board>/
```

`--config` and `--board` are mutually exclusive; one is required.

### `firmware flash`

```bash
motioncore firmware flash --artifact DIR --yes
motioncore firmware flash --board BOARD [--uf2-volume PATH] --yes
```

| Option | Meaning |
|---|---|
| `--artifact DIR` | Build output directory to flash |
| `--board BOARD` | Use the artifact of `firmware build --board BOARD` |
| `--uf2-volume PATH` | Mount point of the UF2 bootloader drive (default: auto-detect) |
| `--yes` | Confirm that drives are powered off. Required |

Uses probe-rs for STM32F411 and UF2 copy for RP2040. Refused while a reachable engine reports `ready`, `homing` or `running`.

### `firmware verify`

```bash
motioncore firmware verify --artifact DIR --device PORT
motioncore firmware verify --board BOARD --device PORT
```

Compare the running firmware (board, build hash, capabilities) with an artifact. The only way to confirm a flash.

## Engine API

### `engine-api proto`

Print the gRPC contract (`.proto`) this binary was built with.

## Run from source

`cargo build` produces `target/debug/motioncore`. Firmware commands that compile or locate artifacts must run inside the motioncore source tree.
