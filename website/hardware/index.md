# Hardware overview

::: warning Unvalidated on hardware
Both physical boards are **provisional**: their firmware compiles and passes host tests, but flashing, USB enumeration, pulse timing and stop latency have not been measured on real hardware. Follow your board's validation procedure before connecting a motor.
:::

## Supported boards

| Board id | MCU | Step channels | Step output | Max step rate (advertised) | Flashing | Status |
|---|---|---|---|---|---|---|
| `virtual` | simulated | 6 | software, shared firmware core | 50 000 /s | – | host-tested |
| `stm32f411-blackpill` | STM32F411CEU6 | 1 | GPIO in a 20 kHz timer interrupt | 20 000 /s | probe-rs (SWD) | provisional |
| `rp2040-pico` | RP2040 | 6 | PIO, one state machine per axis | 50 000 /s per axis | UF2 (BOOTSEL) or probe-rs | provisional |

`motioncore firmware targets` prints the current table from the builder.

## What every setup needs

- **External step/dir drivers** — TMC2209 in step/dir mode, DRV8825, A4988, or a servo/stepper drive with step/dir inputs. The MCU never powers motors.
- **3.3 V-compatible logic.** Neither board is 5 V tolerant. Use drivers whose inputs accept 3.3 V, or level shifters. Never connect a 5/12/24 V signal directly.
- **A pull-up on ENABLE** to the driver's logic supply (e.g. 10 kΩ). ENABLE is active-low, so with the pull-up, drivers stay disabled whenever the MCU is in reset, in bootloader mode, or unprogrammed.
- **A hardware E-stop** that removes driver power (contactor or safety relay), independent of all software.
- **A logic analyser or oscilloscope** for the validation procedure.
- **Normally-open endstops to GND** (internal pull-ups are enabled; pressed = low), or normally-closed with `invert = true`.

## Board guides

- [STM32F411 Black Pill](/hardware/stm32f411) — single axis, the first bring-up target
- [RP2040 (Raspberry Pi Pico)](/hardware/rp2040) — up to six axes with PIO-timed pulses
- [Firmware: build, flash, verify](/hardware/firmware) — the workflow for both

## How boards share code

Every board, and the simulator, runs the same firmware runtime. A board crate supplies only pins, a tick timer and the transport.

```text
boards/stm32f4          boards/rp2040            crates/simulator
  TIM3 tick 20 kHz        TIMER alarm 5 kHz        tokio tick
  USB CDC, IWDG           USB CDC, watchdog        in-process / TCP
  GPIO STEP/DIR           PIO STEP/DIR (6 SMs)
         └──────────────────────┼─────────────────────────┘
                                ▼
             firmware-core     protocol, watchdog, faults, status
             firmware-stepper  segments, homing, stop, limits
             firmware-hal      Hal trait (+ flush_pulses hook)
```

The host does not know which MCU it talks to: every board speaks the same protocol, segment format, fault codes and identity message. The engine checks the board id the MCU reports against the configuration and refuses a mismatch — for example, a Pico plugged in where an STM32 is configured.

## Adding a board

Board support means a crate under `boards/` that implements the `firmware-hal` traits, plus an entry in `crates/device-graph/src/boards.rs` with the board's channels, pins, fixed channel pin map, maximum step rate and flash method. The resolver then validates robot profiles against it automatically. ESP32-S3 and Teensy 4.1 are possible future targets, once their Rust toolchain and HAL support are validated.
