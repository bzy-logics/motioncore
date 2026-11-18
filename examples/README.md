# Examples

## Headless session with the CLI

```bash
cargo build
target/debug/motioncore start --config robot-profiles/single-axis-virtual.toml &

target/debug/motioncore info        # engine + API versions
target/debug/motioncore enable
target/debug/motioncore home j1     # switch at -170 deg in the simulator
target/debug/motioncore move j1 -90
target/debug/motioncore jog j1 -5 --velocity 10
target/debug/motioncore status
target/debug/motioncore stop
target/debug/motioncore disable
```

## Out-of-process simulator ("unplug" test)

```bash
target/debug/motioncore simulator --listen 127.0.0.1:7700 --config robot-profiles/single-axis-virtual.toml
# copy the profile, set: transport = "tcp://127.0.0.1:7700"
target/debug/motioncore start --config my-tcp-profile.toml
# enable + jog, then kill the simulator: the engine faults with communication_lost
```

## Any gRPC client

The contract is `crates/engine-api/proto/motioncore/engine/v1/engine.proto`
(`motioncore engine-api proto` prints it). Errors carry the MotionCore code
in the `motioncore-error-code` trailer.

## Vision-guided pick and place (Python client)

[`sim-vision-pick/`](sim-vision-pick/): a simulated camera and fake AI
detector drive the six-axis arm (`robot-profiles/dummy-arm-virtual.toml`)
through ratavyr's REST/WebSocket API. Runs entirely on a client PC,
standard library only. See its README.
