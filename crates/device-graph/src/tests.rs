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
    assert!(v.iter().any(|i| i.message.contains("not available")), "{v:?}");

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

#[test]
fn profile_must_match_firmware_pin_map() {
    // PB1 exists on the board but the firmware wires channel 0's endstop to PB0.
    let v = issues(&GOOD.replace("\"PB0\"", "\"PB1\""));
    assert_eq!(v.len(), 1, "{v:?}");
    assert_eq!(v[0].path, "axis[0](j1).endstop.pin");
    assert!(v[0].message.contains("wires channel 0 endstop.pin to PB0"), "{v:?}");
}

fn rp2040_axis(channel: u8) -> String {
    let b = boards::find("rp2040-pico").unwrap();
    let pins = b.channel_pins[channel as usize];
    format!(
        "[[axis]]
id = \"a{channel}\"
name = \"A\"
kind = \"revolute\"
unit = \"deg\"
mcu = \"main\"
channel = {channel}
         steps_per_unit = 10.0
min = -90.0
max = 90.0
max_velocity = 90.0
max_acceleration = 360.0
         [axis.driver]
actuator = \"org.motioncore.actuator.step_dir\"
step_pin = \"{}\"
dir_pin = \"{}\"
enable_pin = \"{}\"
",
        pins.step, pins.dir, pins.enable
    )
}

const RP_HEAD: &str = "schema = 1
[robot]
name = \"r\"
profile = \"p\"
[mcu.main]
board = \"rp2040-pico\"
transport = \"serial:///dev/ttyACM0\"
";

#[test]
fn rp2040_routes_six_channels_and_rejects_a_seventh() {
    let six: String = (0..6).map(rp2040_axis).collect();
    let r = parse(&format!("{RP_HEAD}{six}")).unwrap();
    assert_eq!(r.file.axes.len(), 6);
    let mut channels: Vec<u8> = r.file.axes.iter().map(|a| a.channel).collect();
    channels.dedup();
    assert_eq!(channels, [0, 1, 2, 3, 4, 5]);

    let seventh = rp2040_axis(0).replace("channel = 0", "channel = 6").replace("id = \"a0\"", "id = \"a6\"");
    let v = issues(&format!("{RP_HEAD}{six}{seventh}"));
    assert!(v.iter().any(|i| i.message.contains("6 step channel(s)")), "{v:?}");
}

#[test]
fn rp2040_rejects_swapped_step_and_dir() {
    let swapped = rp2040_axis(1).replace("step_pin = \"GP2\"", "step_pin = \"GPX\"").replace("dir_pin = \"GP3\"", "dir_pin = \"GP2\"").replace("GPX", "GP3");
    let v = issues(&format!("{RP_HEAD}{swapped}"));
    assert!(v.iter().any(|i| i.path == "axis[0](a1).driver.step_pin"), "{v:?}");
    assert!(v.iter().any(|i| i.path == "axis[0](a1).driver.dir_pin"), "{v:?}");
}

const GEOMETRY: &str = "\n[axis.geometry]\norigin = [0.0, 0.0, 100.0]\naxis = [0.0, 0.0, 2.0]\n";

#[test]
fn geometry_is_optional_and_normalised() {
    let r = parse(GOOD).unwrap();
    assert!(r.file.axes[0].geometry.is_none());
    let r = parse(&format!("{GOOD}{GEOMETRY}")).unwrap();
    let g = r.file.axes[0].geometry.as_ref().unwrap();
    assert_eq!(g.parent, None);
    assert_eq!(g.origin, [0.0, 0.0, 100.0]);
    assert_eq!(g.rotation, [0.0, 0.0, 0.0]);
    assert_eq!(g.axis, [0.0, 0.0, 1.0], "axis is normalised");
}

#[test]
fn rail_arm_profile_forms_a_tree() {
    let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../robot-profiles/rail-arm-virtual.toml");
    let r = load(&path).unwrap();
    let parent = |id: &str| r.axis(id).unwrap().geometry.as_ref().unwrap().parent.clone();
    assert_eq!(parent("x"), None);
    assert_eq!(parent("j1").as_deref(), Some("x"));
    assert_eq!(parent("j2").as_deref(), Some("j1"));
}

fn rail_arm_with(edit: impl Fn(&str) -> String) -> Vec<Issue> {
    let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../robot-profiles/rail-arm-virtual.toml");
    issues(&edit(&std::fs::read_to_string(path).unwrap()))
}

#[test]
fn geometry_errors_are_reported() {
    // Unknown parent.
    let v = rail_arm_with(|t| t.replace("parent = \"j1\"", "parent = \"j9\""));
    assert!(v.iter().any(|i| i.path.ends_with("geometry.parent") && i.message.contains("unknown axis 'j9'")), "{v:?}");
    // Self parent.
    let v = rail_arm_with(|t| t.replace("parent = \"j1\"", "parent = \"j2\""));
    assert!(v.iter().any(|i| i.message.contains("own parent")), "{v:?}");
    // Cycle: x -> j2 -> j1 -> x.
    let v = rail_arm_with(|t| t.replacen("# On the robot base; slides along +X.", "parent = \"j2\"", 1));
    assert!(v.iter().any(|i| i.message.contains("cycle")), "{v:?}");
    // Zero axis.
    let v = rail_arm_with(|t| t.replacen("axis = [1.0, 0.0, 0.0]", "axis = [0.0, 0.0, 0.0]", 1));
    assert!(v.iter().any(|i| i.path.ends_with("geometry.axis")), "{v:?}");
    // Tip must be finite.
    let v = rail_arm_with(|t| t.replace("tip = [200.0, 0.0, 0.0]", "tip = [nan, 0.0, 0.0]"));
    assert!(v.iter().any(|i| i.path.ends_with("geometry.tip")), "{v:?}");
    // All or nothing.
    let v = rail_arm_with(|t| {
        t.replacen(
            "[axis.geometry]\nparent = \"j1\"\norigin = [250.0, 0.0, 0.0]\naxis = [0.0, 0.0, 1.0]\n# Forearm ends at the tool flange, 200 mm out.\ntip = [200.0, 0.0, 0.0]\n",
            "",
            1,
        )
    });
    assert!(v.iter().any(|i| i.path == "axis[2](j2).geometry" && i.message.contains("every axis")), "{v:?}");
    // Unknown keys are rejected like everywhere else.
    assert!(matches!(parse(&format!("{GOOD}{GEOMETRY}length = 3\n")), Err(ConfigError::Parse(_))));
}
