# ADR 0003: Motion architecture

- **Status:** Accepted (2026-11-19, approved by the project owner on [#79](https://github.com/bzy-logics/motioncore/issues/79))
- **Issue:** [#79](https://github.com/bzy-logics/motioncore/issues/79) · Epic [#60](https://github.com/bzy-logics/motioncore/issues/60) · Stages [#90](https://github.com/bzy-logics/motioncore/issues/90)–[#97](https://github.com/bzy-logics/motioncore/issues/97)
- **Applies to:** `crates/kinematics`, `crates/motion`, `crates/core` (engine), the execution path to `crates/protocol`; the HAL is governed by [ADR 0002](0002-hal-architecture.md)
- **Related:** [stop specification](../stop-behaviour.md), [safety concept](../safety-concept.md)

## Context

MotionCore plans single-axis moves, coordinated joint moves (MoveJ), blended joint paths (MoveJPath) and Cartesian lines (MoveL), checks them against limits and a collision model, and streams timestamped segments to the MCU. The responsibilities exist, but the seams between them are informal, so every new move type, solver or algorithm means editing the engine's command handlers.

### Current state (2026-11-21)

- **Kinematics** (`crates/kinematics`): a `Kinematics` trait that only the unused `JointIdentity` implements; the real solver is the separate `chain::Chain` (FK, numerical IK, `linear_path`). Forward kinematics exists twice in Rust (`Chain::forward` and `CollisionModel::frames`) and once more as a TypeScript port in sindrava.
- **Collision** (`crates/kinematics/src/collision.rs`): a capsule model with a certified path check since #77 (`check_path`, `check_sweep`, `check_path_within`), with stated guarantees.
- **Planning** (`crates/motion`): three trajectory representations, `Trapezoid`, `blend::Piecewise` and `sampled::Sampled`, and two segmenters (`to_segments`, `Sampled::segments`). The crate depends on `motioncore-protocol` and produces MCU `Segment`s itself.
- **Engine** (`crates/core/src/engine.rs`, 1,690 lines): `start_move`, `move_joints`, `move_j_path` and `move_l` each assemble their own sequence of limit checks, planner calls, collision checks and segmentation. Since #22 they share `plan_off_lock` and `commit` (state-revision check), but nothing else. The kinematic and collision models are built in the engine from the resolved profile (`serial_chain`, `collision_model`, `joint_specs`). The feeder loop (per-axis budget, in-flight tracking, completion) is also in `engine.rs`.
- **What depends on this:** S-curves (#80), all-solution IK (#81), singularity handling (#84), plan conformance (#86), TOPP-RA (#87), Cartesian blending (#88), the coordinated stop (#102), feed hold and Resume (#29), and position confidence (#103).

### Owner requirements (2026-11-18)

1. Motion planning stays independent of hardware-specific HAL implementations.
2. Trajectory generation is separate from deterministic real-time trajectory execution.
3. Safety validation is an explicit pipeline stage with clearly defined guarantees and assumptions.
4. The architecture supports future motion types, kinematics solvers and trajectory-planning algorithms without duplicating implementations.
5. The refactoring is split into small, dependency-ordered issues, with existing tests passing after every stage.

### Success criterion (owner, 2026-11-19)

Adding a new robot, kinematics solver or motion planner does not require editing `engine.rs`. Closing the stage issues is not the goal. Each stage must be independently mergeable, tested and useful on its own, and the refactor must not freeze feature work.

## Decision

Motion is a pipeline of five layers with one data type between planning and execution:

```text
API request ─▶ MotionRequest ─▶ Planner ─▶ Trajectory ─▶ Validator ─▶ commit ─▶ Executor ─▶ MCU segments ─▶ HAL
                                   │                         │           (revision)    │
                            KinematicModel            KinematicModel                   └─ live Trajectory: feed hold,
                                                      + CollisionWorld                    coordinated stop, conformance
```

### 1. Kinematics: one `KinematicModel` interface

`crates/kinematics` defines `KinematicModel`: link and tool frames for a joint vector (the only FK implementation), the Jacobian, `ik_near(pose, seed)`, optional `ik_all(pose)` with the configuration of each solution, and joint limits. `Chain` implements it; the unused `Kinematics` trait and `JointIdentity` are removed.

Models are built from the robot profile by a factory keyed on the geometry kind (serial chain today), so a new robot type adds a factory entry and an implementation, not engine code. The collision model takes its link frames from the `KinematicModel`. Stage #90.

### 2. One `Trajectory` type

Every planner returns a `Trajectory`: per-joint piecewise polynomials (degree ≤ 3, so jerk-limited profiles fit) on shared breakpoints, with metadata (motion kind, start pose, duration, the limits and scales used). It can be evaluated for position, velocity, acceleration and jerk at any time, and **under a time warp** `t → τ(t)` common to all joints. The coordinated stop (#102) and feed hold (#29) are time warps of the committed trajectory, which keeps the path exactly. The three existing representations convert to it without loss. Stage #91.

### 3. Planners behind one interface

`Planner::plan(&MotionRequest, &PlanningContext) -> Result<Trajectory, PlanError>`. `PlanningContext` carries the start state, limits, scales and a `&dyn KinematicModel`; it contains no engine state, no locks and no hardware types. Implementations: single-axis, synchronised joint, blended joint path, Cartesian line. A registry maps request kinds to planners, so a new motion type or algorithm (S-curve, TOPP-RA, Cartesian blending) is a new implementation and a registration. Stage #92.

### 4. Validation is an explicit stage

`Validator::validate(&Trajectory, &ValidationContext) -> Result<ValidationReport, Refusal>`:

- position, velocity, acceleration (and jerk where limited) against the limits, at every breakpoint and interior extremum;
- the certified collision check (#77) over the whole trajectory, on a `CollisionWorld` (robot capsules, tool, and obstacles once #32 adds them);
- position confidence (#103): checks that need a known position refuse on Uncertain or Lost axes.

The `ValidationReport` lists each check that ran, with its guarantee and assumptions (for example "capsule model from the profile's radii, rigid links, planned positions; certified minimum clearance 3.1 mm"), and each check that could not run and why (no collision model, unhomed axes). Reports never claim a check that did not run. They are logged and attached to the motion (motion IDs, #28). Resume (#29, stop specification section 7.5) revalidates the remaining trajectory through this same stage. Stage #94.

### 5. Execution is separate and deterministic

The executor owns everything after commit: converting the `Trajectory` plus a start time into MCU segments (one segmenter, with today's exactness rules), feeding them with the per-axis budget, tracking completion and aborts. It is deterministic for a given trajectory and start time, and never re-plans. It keeps the committed trajectory and its time base, which the conformance monitor (#86), the coordinated stop (#102) and feed hold (#29) need. `motioncore-motion` loses its dependency on `motioncore-protocol`. Stage #93.

Below the executor, the HAL is unchanged and follows ADR 0002: it receives timestamped segments, and nothing above it knows the board.

### 6. One engine pipeline

Every motion command follows one path: request → `MotionRequest` → plan (off the engine lock, `spawn_blocking`) → validate (off the lock) → commit (state-revision check, #22) → execute. Engine handlers only translate API requests into `MotionRequest`s. Stop and E-stop are never blocked by planning or validation. Stage #95.

### 7. Dependency rules, enforced in CI

| Crate or module | May depend on | Must not depend on |
|---|---|---|
| `motioncore-kinematics` | maths, error types | motion, protocol, firmware, HAL, boards, engine |
| `motioncore-motion` (planners, `Trajectory`, validator) | kinematics | protocol, firmware, HAL, boards, engine |
| Execution | motion (`Trajectory` only), protocol | planner internals |
| Engine | all of the above | — |
| HAL and firmware crates | protocol | anything above them |

A CI check over `cargo metadata` fails with a message naming the broken rule. Stage #96.

### How the requirements are met

| Requirement | Met by | Stage |
|---|---|---|
| 1. Planning independent of the HAL | Planners see only `PlanningContext` and `KinematicModel`; the dependency rules are checked in CI | #92, #93, #96 |
| 2. Generation separate from execution | `Trajectory` is the only interface; segmentation and feeding move to the executor; `motion` loses its protocol dependency | #91, #93 |
| 3. Validation as an explicit stage | `Validator` with a `ValidationReport` stating each guarantee and assumption, and every check that did not run | #94 |
| 4. Extensibility without duplication | `KinematicModel` factory, planner registry, validator checks as plug-ins; proven by a test-only planner and solver added without engine edits | #90, #92, #94, #97 |
| 5. Small, ordered, always green | Eight stages, each mergeable alone; existing e2e, property and simulated-timing suites pass after every stage; golden segment streams for recorded moves | #90–#97 |

### Stages

| # | Stage | Depends on | Useful on its own because |
|---|---|---|---|
| [#90](https://github.com/bzy-logics/motioncore/issues/90) | `KinematicModel` and one FK | — | Removes a duplicate FK; unblocks all-solution IK (#81) |
| [#91](https://github.com/bzy-logics/motioncore/issues/91) | One `Trajectory` type | — | One evaluation and test surface for every planner |
| [#92](https://github.com/bzy-logics/motioncore/issues/92) | Planners behind one interface | #90, #91 | Unblocks S-curves (#80), TOPP-RA (#87), Cartesian blending (#88) |
| [#93](https://github.com/bzy-logics/motioncore/issues/93) | Separate execution | #91 | Unblocks plan conformance (#86), coordinated stop (#102), feed hold (#29) |
| [#94](https://github.com/bzy-logics/motioncore/issues/94) | Explicit validation stage | #92, #77 | One place for every safety check, with a report |
| [#95](https://github.com/bzy-logics/motioncore/issues/95) | One engine pipeline | #92, #93, #94 | Meets the success criterion |
| [#96](https://github.com/bzy-logics/motioncore/issues/96) | Dependency rules in CI | #95 | Keeps the layering from eroding |
| [#97](https://github.com/bzy-logics/motioncore/issues/97) | Prove extensibility | #95, #96 | Developer guide and an end-to-end example |

#90 and #91 can run in parallel; so can #93 and #92 once #91 is in. Features that do not depend on a stage keep moving.

## Consequences

- **Positive:** a new robot type, solver, planner or validation check is an addition, not an edit to `engine.rs`. Safety checks live in one place with an explicit report. Feed hold, the coordinated stop and plan conformance get the committed trajectory they need instead of per-motion-type special cases.
- **Cost:** about two to three weeks of refactoring spread over eight stages, with regression risk. This is mitigated by the e2e suite, the property tests (#23), the simulated timing tests (#24), and golden segment streams that must stay identical through #93.
- **No external change:** the engine API, the MCU protocol and the robot profile format stay as they are. Each stage is a patch-level internal change.
- **Performance:** one extra conversion to `Trajectory`, negligible next to IK and collision checks. Planning stays off the engine lock.
- **Constraint:** validation stays a check of the **plan** on a model. The [safety concept](../safety-concept.md) rules apply: no report may be described as a safety function.

## Alternatives considered

- **Keep per-command handlers and add helpers.** This is the smallest change, but every new move type still edits the engine, and validation stays scattered. It fails the owner's success criterion.
- **One big rewrite.** It would reach the target sooner on paper, but it freezes features, cannot be merged in useful steps, and puts every regression in one change. It fails requirement 5.
- **Sampled trajectories everywhere** (fixed-rate joint samples instead of polynomials). This is simpler to produce, but loses exact derivatives for validation, makes certified checks and time warps approximate, and multiplies data for long moves.
- **Planning in the firmware.** It would give the lowest latency for stop and feed hold, but duplicates kinematics on every MCU, conflicts with requirement 1 and ADR 0002, and the coordinated stop can be met by a host-side time warp or a simple MCU feed-rate ramp instead (stop specification section 4.2).

## Follow-up work

- Stages #90–#97, in the order above.
- Features that build on the pipeline: #80, #81, #84, #86, #87, #88, #29, #102, #103.
- The developer guide from #97 is linked here when it exists.
