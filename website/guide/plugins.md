# Plugins

MotionCore is plugin-first: robot profiles, actuator types and kinematics are provided by plugins, which declare what they provide, what they require, and what they are allowed to do.

::: info Current status
In v0.2 the shipped plugins are **builtin**: compiled into the engine, built only against `crates/plugin-sdk`. External **WASM** plugins are discovered and fully validated, and their dependencies resolved, but **not executed**. The Wasmtime runtime comes in a later phase. External plugins therefore show up as `failed` with an explanatory error.
:::

## Builtin plugins

| id | provides | requires |
|---|---|---|
| `org.motioncore.robot.single_axis` | `robot.profile` | `robot.joint_model` (core) |
| `org.motioncore.actuator.step_dir` | `actuator.step_dir` | `mcu.stepper` (core) |
| `org.motioncore.kinematics.joint_identity` | `kinematics.fk`, `kinematics.ik` | `robot.profile` |

A robot configuration refers to plugins by id: `robot.profile` names a robot-profile plugin and each axis's `driver.actuator` names an actuator plugin. Both must be active.

List plugins and their states:

```bash
motioncore plugins
curl -H "Authorization: Bearer $TOKEN" http://127.0.0.1:8080/api/plugins
```

## Lifecycle

```text
discovered → validated → resolved → initialized → active
                                                   │
                                     failed ◀──────┤──▶ deactivated
```

If a plugin fails, it and every plugin that depends on it are marked `failed`. If the robot requires it, the robot faults with `plugin_failed`. A plugin failure never silently changes robot state.

## Writing an external plugin

Start the engine with a plugin directory:

```bash
motioncore start --config my-robot.toml --plugins ./plugins
```

The engine looks for `*/plugin.toml` under that directory, so each plugin lives in its own folder:

```text
plugins/
└── articulated6/
    ├── plugin.toml
    └── plugin.wasm
```

### Manifest

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
permissions = ["robot.read_geometry"]
```

Validation checks the id format, semver, `api_version` (`motioncore info` prints the engine's plugin API version), the permissions, that the entrypoint is a relative path inside the plugin directory, and the SHA-256 of the entrypoint.

### Permissions

| Permission | Grants |
|---|---|
| `robot.read_geometry` | Read robot geometry |
| `robot.read_state` | Read robot state |
| `motion.propose` | Propose motion |
| `telemetry.read` | Read telemetry |
| `config.read` | Read configuration |

### Plugins never command hardware

There is no permission to move an axis. Anything motion-related a plugin produces is a **proposal**: it passes the engine's planner validation and the MCU's own checks before a single step is emitted.

## Other extension points

The specification describes three extension layers:

| Extension | Packaging | Status |
|---|---|---|
| Host plugin | Versioned WASM component + manifest | Builtin only; WASM validated, not executed |
| Firmware module | Rust crate compiled into the MCU firmware | `firmware-stepper`, `firmware-pio-step`; encoder and GPIO modules not yet created |
| Web extension | Versioned UI extension contract | Planned |
