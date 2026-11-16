# Quick start (simulation)

This runs the engine against a **virtual MCU** — the same firmware runtime that runs on the boards, driven by a software clock. No hardware is needed.

## Requirements

- Rust stable. The repository's `rust-toolchain.toml` selects the version and also installs the `thumbv7em-none-eabihf` and `thumbv6m-none-eabi` targets, which you only need for firmware builds.

## 1. Build

```bash
git clone https://github.com/bzy-logics/motioncore
cd motioncore
cargo build
```

The CLI binary is `target/debug/motioncore`. The examples below call it `motioncore`; add `target/debug` to your `PATH` or use the full path.

## 2. Start the engine

In one terminal:

```bash
motioncore start --config robot-profiles/single-axis-virtual.toml
```

This loads a one-axis bench robot (`j1`, a revolute joint with soft limits of ±170° and an endstop at −170°) on a virtual MCU.

The engine listens on:

| Platform | Default address |
|---|---|
| Linux | `unix:///run/motioncore/engine.sock` |
| Windows, macOS | `tcp://127.0.0.1:7600` |

Override it with `--listen` when starting, and with `--engine` or `MOTIONCORE_ENGINE_ADDR` for client commands.

## 3. Drive the axis

In a second terminal:

```bash
motioncore info          # engine and API versions
motioncore status        # state, MCU link, axis positions
motioncore enable        # energise drives: disabled → ready
motioncore home j1       # search for the endstop; assigns -170°
motioncore move j1 -90   # absolute move (requires a homed axis)
motioncore jog j1 5      # relative move of +5°
motioncore jog j1 -5 --velocity 10
motioncore watch         # live telemetry and log; Ctrl-C to quit
```

Stop and shut down:

```bash
motioncore stop          # controlled deceleration; drives stay enabled
motioncore estop         # software emergency stop; drives disabled, latched
motioncore reset-fault   # clear the latch; lands in "disabled", axes unhomed
motioncore disable
```

::: tip Why is `move` refused?
Absolute moves need a homed axis. Before homing, only relative `jog` is allowed, at 25 % of the axis maximum speed. Any fault or E-stop clears the homed flag, so home again after `reset-fault`. See [Moving axes](/guide/motion).
:::

## 4. Try a fault: unplug the simulator

The simulator can also run as a separate process on TCP, which lets you test what happens when the MCU link drops.

```bash
motioncore simulator --listen 127.0.0.1:7700 --config robot-profiles/single-axis-virtual.toml
```

Copy the profile, change the MCU transport, and start the engine with it:

```toml
[mcu.main]
board = "virtual"
transport = "tcp://127.0.0.1:7700"
watchdog_ms = 500
```

```bash
motioncore start --config my-tcp-profile.toml
motioncore enable
motioncore jog j1 90
# now kill the simulator process
motioncore status        # state: faulted, fault: communication_lost
```

Each TCP connection to the simulator is a fresh boot of the virtual MCU.

## Next steps

- Put a browser UI on top: [Running the full stack](/guide/full-stack)
- Write a profile for your own robot: [Robot configuration](/guide/configuration)
- All CLI commands: [CLI reference](/reference/cli)
