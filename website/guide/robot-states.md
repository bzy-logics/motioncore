# Robot states

The engine's safety supervisor tracks exactly one robot state. Every client sees it as the `state` field of telemetry.

| State | Meaning | Drives |
|---|---|---|
| `disconnected` | No MCU link | off |
| `initializing` | Link up; handshake, clock sync and axis configuration in progress | off |
| `disabled` | MCU ready, drives not energised | off |
| `ready` | Drives energised, idle; motion commands accepted | **on** |
| `homing` | A homing move is running | **on** |
| `running` | A jog or move is running | **on** |
| `faulted` | A fault is latched | off |
| `emergency_stopped` | A software E-stop is latched | off |

## Transitions

The normal path:

```text
disconnected ──connect──▶ initializing ──initialized──▶ disabled ──enable──▶ ready ⇄ homing
                                                            ▲                  │ ⇄ running
                                                            └─────disable──────┘
```

Exceptions:

```text
any connected state ──fault / link lost──▶ faulted ───────────reset_fault──▶ disabled (link up)
any state ───────────estop──────────────▶ emergency_stopped ─reset_fault──▶ disconnected (link down)
```

| From | Event | To |
|---|---|---|
| any | `estop` | `emergency_stopped` |
| `emergency_stopped` | `reset_fault` | `disabled` if linked, else `disconnected` |
| `emergency_stopped` | fault, link lost, disable, motion complete | `emergency_stopped` (unchanged) |
| `disconnected` | connect | `initializing` |
| `initializing` | initialized | `disabled` |
| `initializing` | link lost | `disconnected` |
| `initializing` | fault | `faulted` |
| other connected states | fault or link lost | `faulted` |
| `faulted` | `reset_fault` | `disabled` if linked, else `disconnected` |
| `disabled` | `enable` | `ready` |
| `ready`, `homing`, `running` | `disable` | `disabled` |
| `ready` | `home` | `homing` → `ready` when done |
| `ready` | `jog` / `move` | `running` → `ready` when done |

The rules, from the transition table in `crates/safety`:

- **E-stop always wins.** `estop` is accepted in every state and moves to `emergency_stopped`. While E-stopped, faults, link loss and disable requests do not change the state.
- **Faults and link loss** move any connected state to `faulted`. The *first* fault is kept; later ones do not overwrite it. During `initializing`, link loss returns to `disconnected` instead.
- **Reset never resumes motion.** `reset_fault` from `faulted` or `emergency_stopped` lands in `disabled` if the MCU is still connected, otherwise `disconnected`. Drives stay off and **every axis must be re-homed.**
- **One motion at a time.** `home`, `jog` and `move` are only accepted in `ready`. A new motion command while `running` or `homing` is rejected with `invalid_state`.
- **Disable** from `ready`, `homing` or `running` performs a controlled stop if moving, then de-energises the drives.
- **Nothing** except `connect` is accepted while `disconnected`.

A rejected command returns `invalid_state` (gRPC `FAILED_PRECONDITION`, HTTP `409`).

## Stop vs. E-stop vs. disable

| Command | Motion | Drives | Latches? | Who may send it (gateway) |
|---|---|---|---|---|
| `stop` | Discards queued segments, decelerates at `stop_deceleration` | stay **on** | no | viewer and up |
| `disable` | Controlled stop if moving | turned **off** | no | operator and up |
| `estop` | Halts step generation immediately | turned **off** immediately | **yes** — `emergency_stopped` | viewer and up |

`stop` and `estop` are open to every authenticated role and are never rate-limited: anyone who can see the robot may stop it.

## Confirming a command

A successful response means the engine **accepted** the command, not that it finished. Watch telemetry for the outcome. For `estop` in particular, a client should only show the stop as confirmed once telemetry reports `"state": "emergency_stopped"`. The sindrava UI does exactly this, and tells the operator to use the hardware E-stop if no confirmation arrives within 2 s.
