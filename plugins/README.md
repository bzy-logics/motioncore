# External plugins

The engine discovers `*/plugin.toml` under the directory passed with
`motioncore start --plugins DIR`. v0.1 validates external (WASM) plugins fully — id, semver,
API version, permissions, entrypoint path and SHA-256 — and resolves their
dependencies, but **does not execute them**: the Wasmtime runtime is a later
phase. Such plugins appear as `failed` with an explanatory error.

Builtin reference plugins ([`builtin/`](builtin), compiled into the engine and built only against `crates/plugin-sdk`):

| id | provides | requires |
|---|---|---|
| `org.motioncore.robot.single_axis` | `robot.profile` | `robot.joint_model` (core) |
| `org.motioncore.actuator.step_dir` | `actuator.step_dir` | `mcu.stepper` (core) |
| `org.motioncore.kinematics.joint_identity` | `kinematics.fk`, `kinematics.ik` | `robot.profile` |

## Manifest

```toml
id = "org.example.kinematics.articulated6"   # reverse-DNS, lowercase
version = "0.1.0"                            # semver
api_version = "0"                            # must equal the engine's plugin API version
kind = "wasm"
description = "6-DOF articulated arm kinematics"
entrypoint = "plugin.wasm"                   # relative, inside the plugin directory
sha256 = "<hex sha-256 of plugin.wasm>"
provides = ["kinematics.fk", "kinematics.ik"]
requires = ["robot.profile"]
permissions = ["robot.read_geometry"]       # robot.read_geometry | robot.read_state |
                                             # motion.propose | telemetry.read | config.read
```

Plugins never command hardware. Anything motion-related a plugin produces is
a *proposal* that passes the engine's planner validation and the MCU's own
checks before a single step is emitted.
