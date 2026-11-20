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

### Blended paths (`MoveJPath`)

`motion::blend::plan_blended` joins the waypoints with constant-velocity segments and rounds every corner with a constant-acceleration blend shared by all joints. With τₖ the nominal time of waypoint k and dₖ = τₖ₊₁ − τₖ, joint j moves at vⱼₖ = Δⱼₖ / dₖ on segment k; the blend at waypoint k lasts tₖ = maxⱼ |vⱼₖ − vⱼ,ₖ₋₁| / aⱼ and is centred on τₖ (the first ends at t₀ and the last starts at T − tₙ, so the path starts and ends exactly on the waypoints). Durations start time-optimal and are stretched until dₖ ≥ (tₖ + tₖ₊₁) / 2. The result is one piecewise-constant-acceleration profile per joint with shared piece times, segmented exactly like a trapezoid.

### Cartesian lines (`MoveL`)

`kinematics::chain` builds a serial chain from the profile geometry (forward kinematics of the tool point and a Levenberg–Marquardt inverse-kinematics solver). `Chain::linear_path` samples the line every 0.5 mm / 0.25° and solves each sample from the previous one; it refuses lines that are out of reach, need a joint beyond its limits, or make a joint jump (a singularity or branch flip). `motion::sampled::time_scale` then picks a trapezoidal profile for the path parameter s(t) such that every joint respects its limits, using q̇ = q′(s)·ṡ and q̈ = q″(s)·ṡ² + q′(s)·s̈, and checks the sampled result.

### Planning off the engine lock

Every motion command (jog, MoveTo, MoveJ, MoveJPath, MoveL) runs in three phases:

1. **Snapshot**, under the engine lock: state check, start positions, limits and the engine's *state revision*.
2. **Plan**, outside the lock on a blocking worker: profile, collision check and MCU segments timed from zero. Telemetry, Stop and E-stop are never held up by planning.
3. **Commit**, under the lock: the move is refused with `invalid_state` ("the robot state changed while the move was planned; nothing was sent") if the revision changed; otherwise the segments are placed on the MCU clock (100 ms ahead) and queued.

The revision changes with every supervisor transition (including Stop, E-stop, faults, enable/disable, homing and motion start/end), whenever planned motion is discarded, when commanded positions are resynced from the MCU, and when an axis is homed. A Stop that arrives while a move is being planned therefore always wins.

## 4. Segmentation

The profile is converted into constant-acceleration MCU segments:

- A segment **never straddles a profile phase** (accelerate / cruise / decelerate), so `p(t) = v0·t + ½·a·t²` is exact within it.
- Step counts come from rounded **absolute** positions, so rounding never accumulates over a long move.
- Each segment is at most **20 ms** long.
- Segments are timestamped on the MCU clock, starting **100 ms** ahead of now, using the offset measured by clock sync.
- The final segment is marked `last`.
- `MoveL` trajectories are sampled every **10 ms**: each knot interval becomes a segment whose step count comes from rounded absolute positions and whose start velocity is the exact joint velocity at the knot. The acceleration is the smooth one between the knot velocities while the MCU's consistency rule allows, otherwise the one that lands exactly on the step count.
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

- **Blending between separate commands.** `MoveJPath` blends the waypoints of one command; a new command still waits until the robot is `ready`.
- **Cartesian paths other than lines.** `MoveL` moves along a straight line; arcs and blended Cartesian paths are not planned.
- **Closed-form inverse kinematics.** Serial chains use a numerical solver that follows the current branch; all solutions and configuration choice come with #81. Geometries that are not one serial chain have forward kinematics (for collision checking) but no inverse.
- **Encoder feedback.** Measured position is the step counter.
