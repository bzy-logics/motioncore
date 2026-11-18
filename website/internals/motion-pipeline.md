# Motion pipeline

What happens between `motioncore move j1 -90` (or `motioncore movej j1=-90 j2=30`) and the first step pulse.

```text
command ─▶ supervisor ─▶ planner ─▶ segmenter ─▶ feeder ─▶ MCU queue ─▶ stepper tick ─▶ STEP/DIR
           (state ok?)   (profile,   (≤ 20 ms,    (as space   (re-validate,
                          validate)   MCU time)    frees up)   ACK/NACK)
```

## 1. Command

`Jog`, `MoveTo`, `MoveJ`, `Home`, `HomeAll`, `Stop` or `EmergencyStop` arrives over gRPC.

## 2. Supervisor check

The safety supervisor checks the transition. Motion requires `ready`, and only one motion runs at a time. The MCU link must be alive. For `MoveTo` and `MoveJ`, every axis moved must be homed (`not_homed` otherwise).

For an **unhomed** axis the planner limits are replaced: soft limits become ± one travel range (`max − min`) around the power-on position, and velocity is capped at 25 % of `max_velocity`.

## 3. Planning

`crates/motion` computes a symmetric trapezoidal velocity profile in axis units — or a triangular one if the move is too short to reach cruise velocity. It validates finite values, soft limits, velocity and acceleration. A violation returns `validation_failed`. A zero-length move returns immediately.

For `MoveJ`, `plan_synchronized_scaled` plans every listed joint with **shared phase times**: it takes the slowest joint's acceleration and cruise times and stretches them by the smallest factor `k ≥ 1` that keeps every joint within its limits. With phase times `k·ta` and `k·tc`, a joint moving `d` needs `v = d / (k·(ta+tc))` and `a = d / (k²·ta·(ta+tc))`, which gives `k` in closed form. Every joint is then at the same fraction of its distance at every instant: a straight line in joint space. With collision checking, that line is checked against the link model before planning continues.

## 4. Segmentation

The profile is converted into constant-acceleration MCU segments:

- A segment **never straddles a profile phase** (accelerate / cruise / decelerate), so `p(t) = v0·t + ½·a·t²` is exact within it.
- Step counts come from rounded **absolute** positions, so rounding never accumulates over a long move.
- Each segment is at most **20 ms** long.
- Segments are timestamped on the MCU clock, starting **100 ms** ahead of now, using the offset measured by clock sync.
- The final segment is marked `last`.
- For `MoveJ`, all joints' segments share **one time grid**: the profiles have the same phase times, so every joint's segments start and end at the same MCU timestamps. Joints that do not move get no segments.

```rust
pub struct Segment {
    pub axis: u8,
    pub start_us: u64,      // absolute MCU time
    pub duration_us: u32,
    pub steps: i32,         // exact integer step delta
    pub v0: f32,            // steps/s
    pub accel: f32,         // steps/s²
    pub last: bool,         // axis at rest afterwards
}
```

## 5. Feeding

A feeder streams segments to the MCU as queue space frees up. The MCU has a queue **per axis**, so the feeder keeps up to `queue_capacity − 4` segments in flight per axis. Pending segments are in time order across axes and the feeder stops at the first axis whose queue is full, so every axis is fed equally far ahead. Segments, `Stop` and `EmergencyStop` are all submitted under the engine lock, so a stop is never overtaken by a stale segment.

## 6. MCU execution

The MCU re-validates every segment and NACKs anything wrong. On each tick, the stepper module works out which steps are due, checks soft limits and endstops before every step, and calls the board's `Hal::step`. After `duration_us`, the axis has moved exactly `steps` steps — the firmware snaps to that integer.

| Situation | MCU response |
|---|---|
| Queue empty after a segment not marked `last` | `queue_underrun` fault |
| Segment starts later than the late tolerance | `segment_late` fault |
| Step would leave soft limits | `soft_limit` fault |
| Endstop pressed while moving toward it | `limit_hit` fault |

The late tolerance is 5 ms on hardware and 40 ms on the virtual MCU, because desktop OS timers are coarse (about 15 ms on Windows).

## 7. Completion

When the MCU reports every axis idle, the engine re-syncs the commanded positions from the MCU step counters and the robot returns to `ready`.

## Stops

| | Host action | MCU action |
|---|---|---|
| `Stop` | Drop pending and in-flight segments, send `Stop`. A stopped homing cycle leaves the axis unhomed | Discard queued segments; decelerate every moving axis at its `stop_decel`; drives stay enabled |
| `Disable` | `Stop`, then disable drives | Same, then ENABLE released |
| `EmergencyStop` | Send `EmergencyStop` on the priority lane | Halt step generation and disable all drives immediately; latch `EmergencyStop` |

## Homing

`Home` sends `HomeParams` to the MCU: search direction, speed in steps/s, maximum travel in steps, and the position to assign at the trigger. The MCU moves until the endstop triggers, sets the position, and reports `HomingDone`. If the switch isn't found within the maximum travel, it faults with `homing_failed`.

The strategy is a single-switch search. A back-off and slow second touch for better repeatability is planned.

## Not yet implemented

- **Cartesian moves.** `MoveJ` is straight in joint space; straight-line tool paths (MoveL) need inverse kinematics.
- **Blending.** Each move starts and ends at rest; consecutive moves are not blended.
- **Kinematics in the loop.** Only the joint-space identity solver exists.
- **Encoder feedback.** Measured position is the step counter.
