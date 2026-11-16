# Configuration schema

Robot configuration files are TOML, schema version 1, validated by `crates/device-graph`. For a walkthrough, see [Robot configuration](/guide/configuration).

General rules:

- **Unknown keys are errors.**
- All problems are reported at once, each with a path such as `axis[0](j1).driver.dir_pin`.
- Units follow the axis: `deg` for revolute, `mm` for prismatic. Velocities are units/s, accelerations units/s².

## Top level

| Key | Type | Required | Rules |
|---|---|---|---|
| `schema` | integer | yes | Must be `1` |
| `[robot]` | table | yes | |
| `[mcu.<name>]` | table | yes | Exactly one MCU in this version |
| `[[axis]]` | array of tables | yes | One per axis |

## `[robot]`

| Key | Type | Required | Rules |
|---|---|---|---|
| `name` | string | yes | Display name |
| `profile` | string | yes | Robot-profile plugin id; the plugin must be active. E.g. `org.motioncore.robot.single_axis` |

## `[mcu.<name>]`

The table name (e.g. `main`) is the MCU's name, which axes refer to.

| Key | Type | Required | Rules |
|---|---|---|---|
| `board` | string | yes | `virtual`, `stm32f411-blackpill` or `rp2040-pico` (see `motioncore firmware targets`) |
| `transport` | string | yes | `virtual`, `tcp://HOST:PORT` or `serial:///dev/ttyACM0` |
| `watchdog_ms` | integer | no | Default `500`; 10–10000. MCU communication watchdog |

## `[[axis]]`

| Key | Type | Required | Rules |
|---|---|---|---|
| `id` | string | yes | `[A-Za-z0-9_]`, unique |
| `name` | string | yes | Display name |
| `kind` | string | yes | `revolute` or `prismatic` |
| `unit` | string | yes | `deg` for revolute, `mm` for prismatic |
| `mcu` | string | yes | Name of an `[mcu.<name>]` table |
| `channel` | integer | yes | Below the board's step-channel count; unique per MCU |
| `steps_per_unit` | float | yes | Motor steps × microsteps × gearing / unit |
| `min`, `max` | float | yes | Soft limits |
| `max_velocity` | float | yes | `max_velocity × steps_per_unit` ≤ board max step rate |
| `max_acceleration` | float | yes | |
| `stop_deceleration` | float | no | Default `2 × max_acceleration`. Used for controlled stops |
| `invert_dir` | bool | no | Default `false` |

### `[axis.driver]`

| Key | Type | Required | Rules |
|---|---|---|---|
| `actuator` | string | yes | Actuator plugin id; must be active. E.g. `org.motioncore.actuator.step_dir` |
| `step_pin` | string | yes | Must exist on the board, be unique per MCU, and match the board's firmware pin map for this channel |
| `dir_pin` | string | yes | Same |
| `enable_pin` | string | yes | Same |

### `[axis.endstop]` (optional)

| Key | Type | Required | Rules |
|---|---|---|---|
| `pin` | string | yes | Board pin, matching the firmware pin map |
| `invert` | bool | no | Default `false`. `true` for normally-closed switches |

### `[axis.homing]` (optional, requires `[axis.endstop]`)

| Key | Type | Required | Rules |
|---|---|---|---|
| `direction` | string | yes | `negative` or `positive` |
| `speed` | float | yes | ≤ `max_velocity` |
| `position` | float | yes | Assigned at the switch; within `[min, max]` |
| `max_travel` | float | yes | Homing fails after this much travel without a trigger |

## Board pin maps

| Board | Channel → STEP / DIR / ENABLE / ENDSTOP |
|---|---|
| `virtual` | No fixed map |
| `stm32f411-blackpill` | 0 → PA0 / PA1 / PA2 / PB0 |
| `rp2040-pico` | 0 → GP0/GP1/GP12/GP18 · 1 → GP2/GP3/GP13/GP19 · 2 → GP4/GP5/GP14/GP20 · 3 → GP6/GP7/GP15/GP21 · 4 → GP8/GP9/GP16/GP22 · 5 → GP10/GP11/GP17/GP26 |

## Full template

```toml
schema = 1

[robot]
name = "single-axis-bench"
profile = "org.motioncore.robot.single_axis"

[mcu.main]
board = "virtual"                 # | "stm32f411-blackpill" | "rp2040-pico"
transport = "virtual"             # | "tcp://HOST:PORT" | "serial:///dev/ttyACM0"
watchdog_ms = 500

[[axis]]
id = "j1"
name = "Base"
kind = "revolute"                 # | "prismatic"
unit = "deg"                      # | "mm"
mcu = "main"
channel = 0
steps_per_unit = 44.444444
min = -170.0
max = 170.0
max_velocity = 90.0
max_acceleration = 360.0
stop_deceleration = 720.0         # optional
invert_dir = false                # optional

[axis.driver]
actuator = "org.motioncore.actuator.step_dir"
step_pin = "STEP0"
dir_pin = "DIR0"
enable_pin = "EN0"

[axis.endstop]                    # optional
pin = "ENDSTOP0"
invert = false

[axis.homing]                     # optional, requires [axis.endstop]
direction = "negative"            # | "positive"
speed = 30.0
position = -170.0
max_travel = 360.0
```
