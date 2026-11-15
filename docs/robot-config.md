# Robot configuration — schema 1

TOML, validated by `crates/device-graph`. Unknown keys are errors.
All problems are reported at once with a path such as `axis[0](j1).driver.dir_pin`.

```toml
schema = 1                                   # required, must be 1

[robot]
name = "single-axis-bench"
profile = "org.motioncore.robot.single_axis" # robot-profile plugin id (must be active)

[mcu.<name>]                                 # exactly one MCU in v0.1
board = "virtual" | "stm32f411-blackpill" | "rp2040-pico"   # see `motioncore firmware targets`
transport = "virtual" | "tcp://HOST:PORT" | "serial:///dev/ttyACM0"
watchdog_ms = 500                            # 10..=10000, MCU communication watchdog

[[axis]]
id = "j1"                                    # [A-Za-z0-9_], unique
name = "Base"
kind = "revolute" | "prismatic"
unit = "deg" | "mm"                          # revolute → deg, prismatic → mm
mcu = "<name>"
channel = 0                                  # < board step channels, unique per MCU
steps_per_unit = 44.444444                   # motor steps × microsteps × gearing / unit
min = -170.0                                 # soft limits, axis units
max = 170.0
max_velocity = 90.0                          # units/s; × steps_per_unit ≤ board max step rate
max_acceleration = 360.0                     # units/s²
stop_deceleration = 720.0                    # optional, default 2 × max_acceleration
invert_dir = false                           # optional

[axis.driver]
actuator = "org.motioncore.actuator.step_dir" # actuator plugin id (must be active)
step_pin = "PA0"                             # must exist on the board, unique per MCU
dir_pin = "PA1"
enable_pin = "PA2"

[axis.endstop]                               # optional
pin = "PB0"
invert = false

[axis.homing]                                # optional, requires [axis.endstop]
direction = "negative" | "positive"
speed = 30.0                                 # units/s, ≤ max_velocity
position = -170.0                            # assigned at the switch, within [min, max]
max_travel = 360.0                           # fail homing after this much travel
```
