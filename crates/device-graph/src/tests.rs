use super::*;

const GOOD: &str = r#"
schema = 1

[robot]
name = "bench"
profile = "org.motioncore.robot.single_axis"

[mcu.main]
board = "stm32f411-blackpill"
transport = "serial:///dev/ttyACM0"

[[axis]]
id = "j1"
name = "Base"
kind = "revolute"
unit = "deg"
mcu = "main"
channel = 0
steps_per_unit = 44.444
min = -170.0
max = 170.0
max_velocity = 90.0
max_acceleration = 360.0

[axis.driver]
actuator = "org.motioncore.actuator.step_dir"
step_pin = "PA0"
dir_pin = "PA1"
enable_pin = "PA2"

[axis.endstop]
pin = "PB0"

[axis.homing]
direction = "negative"
speed = 20.0
position = -170.0
max_travel = 360.0
"#;

#[test]
fn resolves_valid_config() {
    let r = parse(GOOD).unwrap();
    let m = &r.mcus["main"];
    assert_eq!(m.transport, Transport::Serial("/dev/ttyACM0".into()));
    assert!(m.required_firmware_capabilities.contains(&"endstop"));
    assert!(m.required_firmware_capabilities.contains(&"stepper.step_dir"));
    assert_eq!(r.required_plugins, ["org.motioncore.actuator.step_dir", "org.motioncore.robot.single_axis"]);
    assert_eq!(r.axis("j1").unwrap().stop_deceleration(), 720.0);
}

fn issues(text: &str) -> Vec<Issue> {
    match parse(text) {
        Err(ConfigError::Invalid(v)) => v,
        other => panic!("expected validation failure, got {other:?}"),
    }
}

#[test]
fn reports_all_problems_at_once() {
    let bad = GOOD
        .replace("dir_pin = \"PA1\"", "dir_pin = \"PA0\"") // duplicate pin
        .replace("channel = 0", "channel = 7") // board has 1 channel
        .replace("unit = \"deg\"", "unit = \"mm\"") // wrong unit for revolute
        .replace("position = -170.0", "position = -500.0"); // home outside limits
    let v = issues(&bad);
    let text: Vec<String> = v.iter().map(|i| i.to_string()).collect();
    assert!(text.iter().any(|t| t.contains("already assigned")), "{text:?}");
    assert!(text.iter().any(|t| t.contains("1 step channel(s)")), "{text:?}");
    assert!(text.iter().any(|t| t.contains("not a valid unit")), "{text:?}");
    assert!(text.iter().any(|t| t.contains("homing.position")), "{text:?}");
    assert_eq!(v.len(), 4);
}

#[test]
fn rejects_unknown_pins_boards_and_rates() {
    let v = issues(&GOOD.replace("\"PB0\"", "\"PZ9\""));
    assert!(v[0].message.contains("not available"));

    let v = issues(&GOOD.replace("stm32f411-blackpill", "arduino-uno"));
    assert!(v[0].message.contains("unknown board"));

    let v = issues(&GOOD.replace("steps_per_unit = 44.444", "steps_per_unit = 1000.0"));
    assert!(v.iter().any(|i| i.message.contains("steps/s")));
}

#[test]
fn homing_requires_endstop() {
    let text = GOOD.replace("[axis.endstop]\npin = \"PB0\"\n", "");
    let v = issues(&text);
    assert!(v.iter().any(|i| i.message.contains("requires an endstop")));
}

#[test]
fn virtual_transport_needs_virtual_board() {
    let v = issues(&GOOD.replace("serial:///dev/ttyACM0", "virtual"));
    assert!(v[0].message.contains("virtual transport"));
    let ok = GOOD.replace("serial:///dev/ttyACM0", "virtual").replace("stm32f411-blackpill", "virtual");
    assert!(parse(&ok).is_ok());
}

#[test]
fn unknown_fields_are_rejected() {
    assert!(matches!(parse(&GOOD.replace("min = -170.0", "min = -170.0\nfoo = 1")), Err(ConfigError::Parse(_))));
}

#[test]
fn shipped_profiles_are_valid() {
    let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../robot-profiles");
    let mut n = 0;
    for entry in std::fs::read_dir(&dir).unwrap() {
        let path = entry.unwrap().path();
        if path.extension().is_some_and(|e| e == "toml") {
            load(&path).unwrap_or_else(|e| panic!("{}: {e}", path.display()));
            n += 1;
        }
    }
    assert!(n >= 2, "expected shipped robot profiles in {}", dir.display());
}
