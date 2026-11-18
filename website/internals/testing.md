# Testing

## motioncore

```bash
cargo fmt --all -- --check    # formatting (rustfmt.toml: max_width 160)
cargo test --workspace        # all host crates: protocol, firmware runtime, planner, graph, plugins, link, gRPC end-to-end
cargo clippy --workspace --all-targets --exclude motioncore-board-stm32f4 --exclude motioncore-board-rp2040 -- -D warnings
cargo clippy -p motioncore-board-rp2040 --target thumbv6m-none-eabi -- -D warnings
cargo build -p motioncore-board-stm32f4 --profile firmware --target thumbv7em-none-eabihf
cargo build -p motioncore-board-rp2040  --profile firmware --target thumbv6m-none-eabi
python tools/step-skew/step_skew.py --self-test
(cd examples/sim-vision-pick && python -m unittest)
```

### Continuous integration

`.github/workflows/ci.yml` runs all of the above on every pull request and every push to `main`: rustfmt, clippy and the test suite on the host; clippy and a `firmware`-profile build for each board; the Python tools and example. A pull request merges only with a green CI run.

**Formatting** is enforced with rustfmt using the repository's `rustfmt.toml` (`max_width = 160`, `use_small_heuristics = "Max"`, matching the existing wide-line style). Run `cargo fmt --all` before committing.

Because the firmware runtime (`firmware-core`, `firmware-stepper`, `firmware-pio-step`) is `no_std` and hardware-independent, it is tested on the host. `firmware-pio-step` includes an emulator for its PIO program, so pulse encoding and windowing are tested without a Pico. `crates/core/tests/e2e.rs` exercises the gRPC API end to end against the virtual MCU.

Hardware validation is manual; see [STM32F411 bring-up](/hardware/stm32f411#bring-up-procedure) and [RP2040 validation](/hardware/rp2040#hardware-validation).

### Fault injection

Start the engine with `--enable-fault-injection` to enable two debug RPCs:

- `DebugDisconnectMcu` — simulate a lost MCU link
- `DebugFailPlugin { id }` — make a plugin fail

Never use this with real hardware. The engine logs a warning when it is enabled, and `motioncore info` shows it.

## Gateway and UI

ratavyr and sindrava have their own test suites, including integration and end-to-end tests that run against this engine with its virtual MCU:

- [ratavyr: testing](https://ratavyr.bzy-logics.com/development/testing)
- [sindrava: testing](https://sindrava.bzy-logics.com/development/testing)

## What the test strategy covers

| Kind | Examples |
|---|---|
| Unit | Kinematics, trajectory constraints, graph resolution, manifest validation |
| Protocol | Framing, CRC failures, sequence behaviour, version mismatches |
| Simulation | The virtual MCU executes the same commands as hardware |
| Fault injection | Unplugged MCU, stopped heartbeat, plugin crash, buffer underrun |
| Hardware-in-the-loop | One motor, limit switch, E-stop / drive-disable verification (manual, pending) |
| Timing | Step jitter, maximum sustainable step rate, clock drift, queue latency (manual, pending) |
| Build reproducibility | Same pinned inputs produce auditable artifacts |
