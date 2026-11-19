# Firmware: build, flash, verify

Firmware is built from your robot configuration by the restricted builder in `crates/firmware-builder`. Builds are reproducible, artifacts are checksummed, and the running build can be verified over the wire.

```text
configure ──▶ build ──▶ flash ──▶ verify
 lockfile     ELF/UF2   probe-rs   MCU reports board,
 build hash   manifest  or UF2     build hash, capabilities
```

## Prerequisites

- The motioncore source tree (builds compile the in-repo board crates).
- The Rust targets from `rust-toolchain.toml`: `thumbv7em-none-eabihf` (STM32F411) and `thumbv6m-none-eabi` (RP2040). `rustup` installs them automatically inside the repository.
- For STM32F411: [`probe-rs`](https://probe.rs) and an SWD probe (ST-Link V2 or DAPLink).
- For RP2040: nothing extra. Flashing copies a UF2 file to the BOOTSEL drive.

## Build

From a robot configuration:

```bash
motioncore firmware build --config robot-profiles/single-axis-stm32f411.toml
# artifacts in build/firmware/<mcu name>/, e.g. build/firmware/main/
```

From a board's reference profile:

```bash
motioncore firmware build --board rp2040-pico
# artifacts in build/firmware/rp2040-pico/
```

Use `--mcu NAME` with `--config` to choose the MCU when the profile defines more than one.

### Bench test builds

Hardware validation needs two test builds. Both use one spare pin (GP28 on the RP2040, PB1 on the STM32F411), so build one at a time:

| `--bench` | What the spare pin does | Used for |
|---|---|---|
| `bench-isr-probe` | output, high while the motion tick interrupt runs | interrupt time and period |
| `bench-panic` | input; tied to GND, the firmware panics in the next tick | the panic safe state |

```bash
motioncore firmware build --board rp2040-pico --bench bench-isr-probe
# artifacts in build/firmware/rp2040-pico-bench-isr-probe/
motioncore firmware flash --board rp2040-pico --bench bench-isr-probe --yes
motioncore firmware verify --board rp2040-pico --bench bench-isr-probe --device COM5
```

- **Build hash:** the feature is part of the lockfile, so each bench build has its own build hash.
- **Identity:** the firmware reports the feature as a capability (`bench.isr_probe`, `bench.panic`).
- **Warnings:** `verify` prints the build type, and the engine logs a warning when it connects to a bench build.

Never leave a bench build on a machine that drives motors: reflash the production build afterwards.

### Step-rate test

The engine caps every move at the board's advertised step rate, so it can't measure the firmware's real limit. `firmware rate-test` talks to the MCU directly, without the engine:
- it enables the drivers;
- it runs all channels through rising rate stages (by default 10 to 250 kHz in 10 kHz steps, 1 s each);
- it stops at the first fault, or at the first stage that ends at the wrong step count.

```bash
motioncore firmware rate-test --device COM5 --no-motor-power
```

It needs the interlock input closed and refuses to run without `--no-motor-power` or while an engine is reachable. **The motor supply must be off.**

A build writes:

| File | Contents |
|---|---|
| `firmware.elf` | The image |
| `firmware.uf2` | RP2040 only: UF2 generated from the ELF and validated block by block |
| `manifest.json` | Board, firmware and Rust targets, firmware version, build hash, image SHA-256, toolchain, MCU protocol version, required capabilities, build time |
| `motioncore.lock.toml` | The pinned build configuration |
| `build.log` | Compiler output |

To only resolve the configuration and write the lockfile, without compiling:

```bash
motioncore firmware configure --config my-robot.toml
```

### Reproducibility

- Builds run `cargo --locked`, on in-repo board crates only, with the `firmware` profile (`opt-level = "s"`, LTO, `panic = "abort"`).
- The lockfile's hash is the **build hash**. It is embedded in the image and reported by the MCU at handshake, so you can always tell which build is running.
- Builds never run plugin-supplied scripts. Through the API, clients can only choose which MCU of the engine's loaded configuration to build for; they cannot supply paths, scripts or toolchains.

## Flash

::: danger Drives off
Disable the drives and remove driver power before flashing. The tool refuses to flash while a reachable engine reports `ready`, `homing` or `running`, but if no engine is reachable it cannot check — it is up to you.
:::

```bash
motioncore disable

# STM32F411, via probe-rs over SWD
motioncore firmware flash --artifact build/firmware/main --yes

# RP2040, via UF2: hold BOOTSEL while plugging in USB first
motioncore firmware flash --board rp2040-pico --yes
motioncore firmware flash --board rp2040-pico --uf2-volume /media/$USER/RPI-RP2 --yes   # if not auto-detected
```

`--yes` confirms that drives are powered off; without it the tool refuses. For UF2, the tool copies `firmware.uf2` to the drive and waits for the drive to disappear (the bootloader took the image and rebooted). If it is still there after 15 s, it reports failure.

## Verify

A successful flash command does **not** prove what is running. Only `verify` does:

```bash
motioncore firmware verify --artifact build/firmware/main --device /dev/ttyACM0
motioncore firmware verify --board rp2040-pico --device COM5        # Windows
```

It connects to the MCU, reads its identity, and compares board, build hash, capabilities and bench features with the artifact's manifest. Any mismatch is printed as `MISMATCH: …` and the command fails. Example of a successful run:

```text
MCU reports: target rp2040 board rp2040-pico fw 0.3.0 build dcdfcb82 caps [stepper.step_dir, endstop, watchdog]
build type: production
verified: running firmware matches build/firmware/rp2040-pico
```

Find the serial port with:

```bash
motioncore mcu list
```

## From the web UI or REST API

Through ratavyr, firmware jobs run in the engine and always use the robot configuration the engine was started with.

| Step | REST | Minimum role |
|---|---|---|
| List boards | `GET /api/firmware/targets` | viewer |
| Build | `POST /api/firmware/builds` `{"mcu":"main"}` → `202` job | operator |
| Poll | `GET /api/firmware/jobs/{id}` | viewer |
| Flash | `POST /api/firmware/flashes` `{"build_job_id":"build-3","confirm":true}` → `202` job | **admin** |

- Building for a virtual board → `400 bad_request`.
- Flashing without `"confirm": true` → `400`; a job that is not a succeeded build → `400`; drives energised or another job running → `409 invalid_state`.
- Job progress is polled; the event stream does not carry job updates.
- **Verification is CLI-only.** The engine API has no RPC for it yet.

Firmware builds are only available when the engine can find the motioncore source tree: it auto-detects it from the working directory, or use `motioncore start --repo PATH`. Otherwise the engine logs `firmware builds disabled`.

## Recovery

- **STM32F411:** probe-rs over SWD works even if the running firmware is broken.
- **RP2040:** hold BOOTSEL while plugging in USB to reach the ROM bootloader regardless of the flashed image. A second Pico running debugprobe also works with `probe-rs download --chip RP2040 firmware.elf`.
- Keep the artifact directory of every known-good build; `manifest.json` records exactly what it was.
