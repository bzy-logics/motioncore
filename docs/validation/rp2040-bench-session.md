# RP2040 bench session (tests 1–16)

How to run the RP2040 hardware validation of [`docs/rp2040.md`](../rp2040.md) section 9 in one or two bench days, and what to record so the results count as **hardware-validated** under the project's Definition of Done. The test definitions and pass criteria stay in `rp2040.md` section 9; this page is the practical order, setup and record keeping.

| Issue | Tests |
|---|---|
| #25 RP2040 six-axis STEP/DIR bench validation | 1–14 |
| #13 Coordinated-move skew | 15 |
| #68 Safe state on firmware panic | 16 |
| #26 Six-axis MoveJ on hardware | after 1–16, see section 6 |

> **No motor power during tests 1–16.** Drivers unpowered, or no drivers at all. Motors come only after every test passes (section 6), at low current and with no load.

## 1. Equipment

- [ ] Raspberry Pi Pico (RP2040) and a micro-USB cable to the host
- [ ] Logic analyser, **≥ 24 MHz, ≥ 8 channels** (sigrok/PulseView or Saleae). At 24 MHz one sample is 42 ns, enough for the ±0.1 µs of test 3.
- [ ] Optional: a second Pico as an SWD debug probe (debugprobe firmware). Needed for test 12 (halting the core) and convenient for flashing the test builds.
- [ ] 10 kΩ pull-up resistors from each ENABLE pin to 3.3 V (or the driver's logic supply). Without them, ENABLE floats during reset and test 1 is meaningless.
- [ ] Jumper wires: GP27 (interlock) to GND, and one per endstop input to touch GND by hand while homing
- [ ] Step/dir drivers are optional for 1–16. If fitted: 3.3 V logic inputs, **motor supply off**.

## 2. Firmware builds

Record the **build hash** of every image you flash. `motioncore firmware verify` prints the hash the MCU reports.

| Build | For | How |
|---|---|---|
| **A: release** | 1–6, 8, 10–15 | `motioncore firmware build --board rp2040-pico`, then `motioncore firmware flash --board rp2040-pico --yes` and `motioncore firmware verify --board rp2040-pico --device <port>` |
| **B: tick-ISR probe** | 7, 9 (ISR time) | A temporary build that toggles GP28 at the start and end of `TIMER_IRQ_0`. **There is no build option for this yet (#114)**; until there is, it means a local edit to `boards/rp2040/src/main.rs` that is never committed. Note the edit in the record. |
| **C: bench-panic** | 16 | `cargo build -p motioncore-board-rp2040 --profile firmware --target thumbv6m-none-eabi --features bench-panic`. The firmware builder has no feature option yet (#114), so flash the ELF `target/thumbv6m-none-eabi/firmware/motioncore-board-rp2040` with `probe-rs download --chip RP2040 <elf>` (SWD probe), or convert it with `picotool uf2 convert <elf> firmware.uf2 --family rp2040` and copy it to the BOOTSEL drive. |

**Never** leave build B or C on a machine that drives motors. Reflash build A after the session.

## 3. Robot profile and engine

Use **`robot-profiles/six-axis-rp2040-pico-timing.toml`**. It has the same wiring as the reference profile, with limits opened up for the timing tests:
- 50 000 steps/s per axis (1125°/s at 44.444444 steps/°);
- soft limits of ±20 000°;
- **no motors.**

The reference profile tops out at 2 000 steps/s, too slow for tests 3–9.

```bash
# Linux: serial:///dev/ttyACM0 (as in the profile). Windows: copy the profile
# and set transport = "serial://COM5" (your port).
motioncore start --config robot-profiles/six-axis-rp2040-pico-timing.toml
motioncore enable          # GP27 must be jumpered to GND
motioncore home-all        # touch each endstop pin to GND while its axis searches
```

Jog velocities for the single-axis rates (`motioncore jog j1 <distance> --velocity <v>`):

| Step rate | Velocity (°/s) | Distance for ≥ 1 s at constant speed (°) |
|---|---|---|
| 1 kHz | 22.5 | 30 |
| 10 kHz | 225 | 300 |
| 50 kHz | 1125 | 1500 |

## 4. Analyser channels

**Layout S: one axis (tests 1–7, 10–14)**

| Ch | Signal | Pin |
|---|---|---|
| D0 | STEP0 | GP0 |
| D1 | DIR0 | GP1 |
| D2 | ENABLE0 | GP12 |
| D3 | ENDSTOP0 | GP18 |
| D4 | Interlock | GP27 |
| D5 | ISR probe (build B) | GP28 |
| D6 | USB D+ or a host marker (optional, for test 10 frame receipt) | – |

**Layout M: six axes (tests 9, 15, 16, #26)**

| Ch | Signal | Pin |
|---|---|---|
| D0–D5 | STEP0–STEP5 | GP0, GP2, GP4, GP6, GP8, GP10 |
| D6 | ENABLE0 | GP12 (all ENABLEs switch together) |
| D7 | GP28: ISR probe (build B), or the panic trigger (build C) | GP28 |

## 5. Order

The order keeps each build and wiring change to a minimum.

| Step | Tests | Build | Layout | Notes |
|---|---|---|---|---|
| 1 | 1 Safe start, 2 Identity | A | S | Test 1 also covers BOOTSEL mode and flashing itself |
| 2 | 3–6 pulse timing, DIR, frequency | A | S | Jog j1 at 1, 10 and 50 kHz; reversing jog for test 5 |
| 3 | 8 Max sustainable rate | A | S | See the limitation below |
| 4 | 10 Stop latency, 11 Watchdog, 13 Interlock, 14 Endstop | A | S | Test 11: unplug USB while enabled. Test 13: pull the GP27 jumper while moving |
| 5 | 12 Hang | A | S | Halt core 0 with the SWD probe |
| 6 | 7 Jitter, 9 Six axes | B | S then M | Test 9: `motioncore movej` all six axes the same distance at `--speed 1` (50 kHz each) |
| 7 | 15 Skew | A | M | `rp2040.md` section 9.1; analyse with `tools/step-skew` (default limits) |
| 8 | 16 Panic safe state | C | M | Tie GP28 to GND during a six-axis move |
| 9 | Reflash build A, then `motioncore firmware verify` | A | – | – |

**Test 8 limitation.** The engine refuses profiles above the board's advertised 50 kHz, so through the engine the rate can't be raised past it. Record whether 50 kHz on six axes runs without `StepRateExceeded`, and note the ISR time from test 9. Finding the real maximum needs a build with a higher advertised rate (#114).

## 6. After tests 1–16: #26 and motors

- **#26:** with build A and layout M, run a six-axis `movej`, a `movej-path` with reversals, and a Stop mid-move, at 25% and 100% speed. Compare the exact step counts per channel with the planned targets; analyse skew with `tools/step-skew`.
- **Only after everything passes:** motors at low current, no load. Jog ±1°, home, Stop mid-move, E-stop. The hardware E-stop must remove driver power (`rp2040.md` section 6).

## 7. Recording

Copy [`rp2040-record-template.md`](rp2040-record-template.md) to `docs/validation/rp2040-YYYY-MM-DD.md` and fill it in during the session.

- **Captures:** name them `YYYYMMDD-tNN-<what>.sr` (sigrok) or `.sal` (Saleae), plus a CSV export for 15 and #26. Zip them and attach the zip to the issue (#25, #13, #68, #26); link it from the record.
- **Everything needed to reproduce:** date, board revision, every build hash, analyser model and sample rate, driver model (or "none"), host OS, engine version (`motioncore info`).
- **Fail honestly:** a test that doesn't pass gets a new issue, linked from the record. Don't adjust the pass criterion after the fact.
- **Then:**
  - commit the filled record;
  - summarise the results in `rp2040.md` section 9 (replacing "None of the following has been carried out yet");
  - move each issue to **Done**;
  - set the changelog validation level to *hardware-validated* only for what was measured.
