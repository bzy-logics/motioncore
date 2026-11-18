//! Robot configuration, board metadata and device-graph resolution.
//!
//! A robot is described as typed components (axes → driver → MCU channel,
//! endstops, homing) rather than a hard-coded six-motor structure. [`resolve`]
//! validates the whole graph and reports **every** problem it finds — duplicate
//! pins, channels the board does not have, missing firmware capabilities,
//! inconsistent limits — instead of stopping at the first.
//!
//! Automated checks cannot replace electrical verification or a physical
//! safety assessment.

use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, HashMap, HashSet};
use std::path::Path;

pub mod boards;

pub use boards::{Board, ChannelPins, FlashMethod, BOARDS};

// ---------------------------------------------------------------------------
// Configuration file schema (TOML)
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RobotFile {
    /// Schema version of this file.
    pub schema: u32,
    pub robot: RobotSection,
    pub mcu: BTreeMap<String, McuSection>,
    #[serde(rename = "axis")]
    pub axes: Vec<AxisSection>,
    /// How "home all" proceeds (optional). Without it every homeable axis is
    /// homed on its own, in profile order.
    pub home_all: Option<HomeAllSection>,
}

/// Home-all plan: the axes of one group home at the same time, groups run in
/// order. The plan is only as safe as its design: before homing, joint
/// positions are unknown, so the engine cannot check the paths. `start_pose`
/// tells the operator what pose the robot must be in first.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct HomeAllSection {
    pub groups: Vec<Vec<String>>,
    pub start_pose: Option<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RobotSection {
    pub name: String,
    /// Robot-profile plugin that provides this robot definition.
    pub profile: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct McuSection {
    /// Board id from [`BOARDS`].
    pub board: String,
    /// `virtual`, `tcp://host:port`, or `serial:///dev/ttyACM0`.
    pub transport: String,
    #[serde(default = "default_watchdog")]
    pub watchdog_ms: u32,
}

fn default_watchdog() -> u32 {
    500
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum JointKind {
    Revolute,
    Prismatic,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Unit {
    Deg,
    Mm,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AxisSection {
    pub id: String,
    pub name: String,
    pub kind: JointKind,
    pub unit: Unit,
    pub mcu: String,
    pub channel: u8,
    /// Motor steps (incl. microstepping and gearing) per axis unit.
    pub steps_per_unit: f64,
    pub min: f64,
    pub max: f64,
    pub max_velocity: f64,
    pub max_acceleration: f64,
    /// Deceleration for controlled stops; defaults to 2 × `max_acceleration`.
    pub stop_deceleration: Option<f64>,
    #[serde(default)]
    pub invert_dir: bool,
    pub driver: DriverSection,
    pub endstop: Option<EndstopSection>,
    pub homing: Option<HomingSection>,
    /// Where the joint sits in the robot's kinematic tree (optional; if any
    /// axis has geometry, every axis must).
    pub geometry: Option<GeometrySection>,
}

/// Kinematic placement of one joint, URDF-style:
///
/// `frame(child) = frame(parent) · Translate(origin) · RPY(rotation) · Joint(q about axis)`
///
/// where `frame(parent)` already includes the parent's own joint motion and
/// the base (no parent) is the robot's world frame.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct GeometrySection {
    /// Axis this joint is mounted on; omitted = the robot base.
    pub parent: Option<String>,
    /// Joint frame origin in the parent's frame, in mm.
    #[serde(default)]
    pub origin: [f64; 3],
    /// Fixed roll, pitch, yaw (degrees, about X, Y, Z of the parent frame,
    /// applied in that order) of the joint frame.
    #[serde(default)]
    pub rotation: [f64; 3],
    /// Joint axis in the joint frame: rotation axis (revolute) or slide
    /// direction (prismatic). Normalised by the resolver. Default +Z.
    #[serde(default = "default_joint_axis")]
    pub axis: [f64; 3],
    /// End point of this joint's link in its joint frame, mm, for joints
    /// with nothing mounted on them (e.g. the tool flange).
    pub tip: Option<[f64; 3]>,
    /// Radius of this joint's link (capsules from the joint to its children
    /// and tip), mm. When every axis has one, the engine checks moves for
    /// self-collision and floor contact.
    pub radius: Option<f64>,
}

fn default_joint_axis() -> [f64; 3] {
    [0.0, 0.0, 1.0]
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DriverSection {
    /// Actuator plugin that drives this axis, e.g. `org.motioncore.actuator.step_dir`.
    pub actuator: String,
    pub step_pin: String,
    pub dir_pin: String,
    pub enable_pin: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct EndstopSection {
    pub pin: String,
    #[serde(default)]
    pub invert: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Direction {
    Positive,
    Negative,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct HomingSection {
    pub direction: Direction,
    /// Search speed in axis units/s.
    pub speed: f64,
    /// Axis position assigned at the switch.
    pub position: f64,
    /// Maximum search travel in axis units before homing fails.
    pub max_travel: f64,
}

// ---------------------------------------------------------------------------
// Resolution
// ---------------------------------------------------------------------------

/// A validated robot ready for the engine.
#[derive(Debug, Clone)]
pub struct ResolvedRobot {
    pub file: RobotFile,
    pub mcus: BTreeMap<String, ResolvedMcu>,
    /// Plugin capabilities required by this configuration.
    pub required_capabilities: Vec<String>,
    /// Plugin ids referenced (profile + actuators).
    pub required_plugins: Vec<String>,
}

#[derive(Debug, Clone)]
pub struct ResolvedMcu {
    pub name: String,
    pub board: &'static Board,
    pub transport: Transport,
    pub watchdog_ms: u32,
    /// MCU capability names the firmware must report.
    pub required_firmware_capabilities: Vec<&'static str>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Transport {
    Virtual,
    Tcp(String),
    Serial(String),
}

impl Transport {
    pub fn parse(s: &str) -> Option<Self> {
        if s == "virtual" {
            Some(Self::Virtual)
        } else if let Some(addr) = s.strip_prefix("tcp://") {
            (!addr.is_empty()).then(|| Self::Tcp(addr.to_string()))
        } else if let Some(dev) = s.strip_prefix("serial://") {
            (!dev.is_empty()).then(|| Self::Serial(dev.to_string()))
        } else {
            None
        }
    }

    pub fn kind(&self) -> &'static str {
        match self {
            Self::Virtual => "virtual",
            Self::Tcp(_) => "tcp",
            Self::Serial(_) => "serial",
        }
    }
}

/// One validation problem, with a dotted path to the offending field.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Issue {
    pub path: String,
    pub message: String,
}

impl std::fmt::Display for Issue {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}: {}", self.path, self.message)
    }
}

#[derive(Debug, thiserror::Error)]
pub enum ConfigError {
    #[error("cannot read {path}: {source}")]
    Io { path: String, source: std::io::Error },
    #[error("invalid TOML: {0}")]
    Parse(#[from] toml::de::Error),
    #[error("configuration invalid:\n  {}", .0.iter().map(|i| i.to_string()).collect::<Vec<_>>().join("\n  "))]
    Invalid(Vec<Issue>),
}

pub const SCHEMA_VERSION: u32 = 1;

pub fn load(path: &Path) -> Result<ResolvedRobot, ConfigError> {
    let text = std::fs::read_to_string(path).map_err(|e| ConfigError::Io { path: path.display().to_string(), source: e })?;
    parse(&text)
}

pub fn parse(text: &str) -> Result<ResolvedRobot, ConfigError> {
    let file: RobotFile = toml::from_str(text)?;
    resolve(file).map_err(ConfigError::Invalid)
}

/// Validate a robot description against board metadata.
pub fn resolve(mut file: RobotFile) -> Result<ResolvedRobot, Vec<Issue>> {
    let mut issues = Vec::new();

    if file.schema != SCHEMA_VERSION {
        add(&mut issues, "schema".into(), format!("unsupported schema {} (expected {SCHEMA_VERSION})", file.schema));
    }
    if file.axes.is_empty() {
        add(&mut issues, "axis".into(), "at least one axis is required".into());
    }

    let mut mcus = BTreeMap::new();
    for (name, m) in &file.mcu {
        let path = format!("mcu.{name}");
        let board = boards::find(&m.board);
        if board.is_none() {
            let known: Vec<_> = BOARDS.iter().map(|b| b.id).collect();
            add(&mut issues, format!("{path}.board"), format!("unknown board '{}' (known: {})", m.board, known.join(", ")));
        }
        let transport = Transport::parse(&m.transport);
        if transport.is_none() {
            add(&mut issues, format!("{path}.transport"), format!("invalid transport '{}'", m.transport));
        }
        if !(10..=10_000).contains(&m.watchdog_ms) {
            add(&mut issues, format!("{path}.watchdog_ms"), "must be within 10..=10000".into());
        }
        if let (Some(board), Some(transport)) = (board, transport) {
            if transport == Transport::Virtual && !board.is_virtual {
                add(&mut issues, format!("{path}.transport"), format!("board '{}' is physical; virtual transport requires the 'virtual' board", board.id));
            }
            mcus.insert(
                name.clone(),
                ResolvedMcu { name: name.clone(), board, transport, watchdog_ms: m.watchdog_ms, required_firmware_capabilities: vec!["watchdog"] },
            );
        }
    }

    let mut ids = HashSet::new();
    let mut channels: HashMap<(String, u8), String> = HashMap::new();
    let mut pins: HashMap<(String, String), String> = HashMap::new();
    let mut required_plugins = vec![file.robot.profile.clone()];
    let mut required_capabilities = vec!["robot.profile".to_string()];

    for (i, a) in file.axes.iter().enumerate() {
        let p = format!("axis[{i}]({})", a.id);
        if a.id.is_empty() || !a.id.chars().all(|c| c.is_ascii_alphanumeric() || c == '_') {
            add(&mut issues, format!("{p}.id"), "must be non-empty [A-Za-z0-9_]".into());
        }
        if !ids.insert(a.id.clone()) {
            add(&mut issues, format!("{p}.id"), "duplicate axis id".into());
        }
        match (a.kind, a.unit) {
            (JointKind::Revolute, Unit::Deg) | (JointKind::Prismatic, Unit::Mm) => {}
            (k, u) => add(&mut issues, format!("{p}.unit"), format!("{u:?} is not a valid unit for a {k:?} joint")),
        }
        for (field, v) in [
            ("steps_per_unit", a.steps_per_unit),
            ("max_velocity", a.max_velocity),
            ("max_acceleration", a.max_acceleration),
            ("stop_deceleration", a.stop_deceleration.unwrap_or(1.0)),
        ] {
            if !(v.is_finite() && v > 0.0) {
                add(&mut issues, format!("{p}.{field}"), "must be a positive finite number".into());
            }
        }
        if !(a.min.is_finite() && a.max.is_finite() && a.min < a.max) {
            add(&mut issues, format!("{p}.min"), "min must be finite and less than max".into());
        }
        let range_steps = (a.max - a.min).abs() * a.steps_per_unit;
        if range_steps.is_finite() && range_steps > i32::MAX as f64 / 2.0 {
            add(&mut issues, format!("{p}.steps_per_unit"), "travel range exceeds MCU step counter".into());
        }

        required_plugins.push(a.driver.actuator.clone());
        required_capabilities.push("actuator.step_dir".into());

        let Some(m) = mcus.get_mut(&a.mcu) else {
            if !file.mcu.contains_key(&a.mcu) {
                add(&mut issues, format!("{p}.mcu"), format!("unknown mcu '{}'", a.mcu));
            }
            continue;
        };
        let board = m.board;
        if a.channel as usize >= board.step_channels as usize {
            add(&mut issues, format!("{p}.channel"), format!("board '{}' has {} step channel(s)", board.id, board.step_channels));
        }
        if let Some(other) = channels.insert((a.mcu.clone(), a.channel), a.id.clone()) {
            add(&mut issues, format!("{p}.channel"), format!("channel {} already used by axis '{other}'", a.channel));
        }
        let rate = a.max_velocity * a.steps_per_unit;
        if rate > board.max_step_rate as f64 {
            add(&mut issues, format!("{p}.max_velocity"), format!("needs {rate:.0} steps/s; board limit is {}", board.max_step_rate));
        }
        if !m.required_firmware_capabilities.contains(&"stepper.step_dir") {
            m.required_firmware_capabilities.push("stepper.step_dir");
        }

        let mut claim = |pin: &str, role: &str, issues: &mut Vec<Issue>| {
            let path = format!("{p}.{role}");
            if !board.has_pin(pin) {
                issues.push(Issue { path: path.clone(), message: format!("pin '{pin}' not available on board '{}'", board.id) });
            }
            if let Some(owner) = pins.insert((a.mcu.clone(), pin.to_string()), format!("{}.{role}", a.id)) {
                issues.push(Issue { path, message: format!("pin '{pin}' already assigned to {owner}") });
            }
        };
        // Boards with a fixed firmware pin map: the profile must match it.
        if let Some(fixed) = board.channel_pins.get(a.channel as usize) {
            let fixed_pin = |pin: &str, want: &str, role: &str, issues: &mut Vec<Issue>| {
                if pin != want {
                    issues.push(Issue {
                        path: format!("{p}.{role}"),
                        message: format!("board '{}' firmware wires channel {} {role} to {want}, not '{pin}'", board.id, a.channel),
                    });
                }
            };
            fixed_pin(&a.driver.step_pin, fixed.step, "driver.step_pin", &mut issues);
            fixed_pin(&a.driver.dir_pin, fixed.dir, "driver.dir_pin", &mut issues);
            fixed_pin(&a.driver.enable_pin, fixed.enable, "driver.enable_pin", &mut issues);
            if let Some(e) = &a.endstop {
                fixed_pin(&e.pin, fixed.endstop, "endstop.pin", &mut issues);
            }
        }
        claim(&a.driver.step_pin, "driver.step_pin", &mut issues);
        claim(&a.driver.dir_pin, "driver.dir_pin", &mut issues);
        claim(&a.driver.enable_pin, "driver.enable_pin", &mut issues);
        if let Some(e) = &a.endstop {
            claim(&e.pin, "endstop.pin", &mut issues);
            if !m.required_firmware_capabilities.contains(&"endstop") {
                m.required_firmware_capabilities.push("endstop");
            }
        }

        if let Some(h) = &a.homing {
            let hp = format!("{p}.homing");
            if a.endstop.is_none() {
                issues.push(Issue { path: hp.clone(), message: "homing requires an endstop".into() });
            }
            if !(h.speed.is_finite() && h.speed > 0.0 && h.speed <= a.max_velocity) {
                issues.push(Issue { path: format!("{hp}.speed"), message: "must be positive and ≤ max_velocity".into() });
            }
            if !(h.position >= a.min && h.position <= a.max) {
                issues.push(Issue { path: format!("{hp}.position"), message: "must lie within [min, max]".into() });
            }
            if !(h.max_travel.is_finite() && h.max_travel > 0.0) {
                issues.push(Issue { path: format!("{hp}.max_travel"), message: "must be positive".into() });
            }
        }
    }

    check_geometry(&mut file, &mut issues);
    check_home_all(&file, &mut issues);

    required_plugins.sort();
    required_plugins.dedup();
    required_capabilities.sort();
    required_capabilities.dedup();

    if issues.is_empty() {
        Ok(ResolvedRobot { file, mcus, required_capabilities, required_plugins })
    } else {
        Err(issues)
    }
}

/// Validate the kinematic tree: all-or-nothing, known parents, no cycles,
/// finite numbers, non-zero joint axes (normalised in place).
fn check_geometry(file: &mut RobotFile, issues: &mut Vec<Issue>) {
    let with = file.axes.iter().filter(|a| a.geometry.is_some()).count();
    if with == 0 {
        return;
    }
    for (i, a) in file.axes.iter().enumerate() {
        if a.geometry.is_none() {
            issues.push(Issue { path: format!("axis[{i}]({}).geometry", a.id), message: "missing: when one axis has geometry, every axis needs it".into() });
        }
    }
    let ids: HashSet<String> = file.axes.iter().map(|a| a.id.clone()).collect();
    let parent_of: HashMap<String, Option<String>> = file.axes.iter().filter_map(|a| a.geometry.as_ref().map(|g| (a.id.clone(), g.parent.clone()))).collect();
    for (i, a) in file.axes.iter_mut().enumerate() {
        let Some(g) = a.geometry.as_mut() else { continue };
        let p = format!("axis[{i}]({}).geometry", a.id);
        if let Some(parent) = &g.parent {
            if parent == &a.id {
                issues.push(Issue { path: format!("{p}.parent"), message: "an axis cannot be its own parent".into() });
            } else if !ids.contains(parent) {
                issues.push(Issue { path: format!("{p}.parent"), message: format!("unknown axis '{parent}'") });
            } else {
                // Walk up; a chain longer than the axis count is a cycle.
                let mut cur = Some(parent.clone());
                let mut steps = 0;
                while let Some(c) = cur {
                    if c == a.id || steps > ids.len() {
                        issues.push(Issue { path: format!("{p}.parent"), message: "parent chain forms a cycle".into() });
                        break;
                    }
                    cur = parent_of.get(&c).cloned().flatten();
                    steps += 1;
                }
            }
        }
        if let Some(r) = g.radius {
            if !(r.is_finite() && r > 0.0) {
                issues.push(Issue { path: format!("{p}.radius"), message: "must be a positive number (mm)".into() });
            }
        }
        for (field, v) in [("origin", g.origin), ("rotation", g.rotation), ("axis", g.axis), ("tip", g.tip.unwrap_or_default())] {
            if !v.iter().all(|x| x.is_finite()) {
                issues.push(Issue { path: format!("{p}.{field}"), message: "must contain finite numbers".into() });
            }
        }
        let n = (g.axis[0] * g.axis[0] + g.axis[1] * g.axis[1] + g.axis[2] * g.axis[2]).sqrt();
        if n.is_finite() && n > 1e-9 {
            g.axis = [g.axis[0] / n, g.axis[1] / n, g.axis[2] / n];
        } else {
            issues.push(Issue { path: format!("{p}.axis"), message: "must be a non-zero vector".into() });
        }
    }
}

/// Validate the home-all plan: known, homeable axes, each exactly once, and
/// every homeable axis covered.
fn check_home_all(file: &RobotFile, issues: &mut Vec<Issue>) {
    let with_radius = file.axes.iter().filter(|a| a.geometry.as_ref().is_some_and(|g| g.radius.is_some())).count();
    if with_radius > 0 && with_radius < file.axes.len() {
        for (i, a) in file.axes.iter().enumerate() {
            if !a.geometry.as_ref().is_some_and(|g| g.radius.is_some()) {
                issues.push(Issue {
                    path: format!("axis[{i}]({}).geometry.radius", a.id),
                    message: "missing: when one axis has a link radius, every axis needs one".into(),
                });
            }
        }
    }
    let Some(plan) = &file.home_all else { return };
    let mut seen = HashSet::new();
    for (gi, group) in plan.groups.iter().enumerate() {
        let gp = format!("home_all.groups[{gi}]");
        if group.is_empty() {
            issues.push(Issue { path: gp.clone(), message: "empty group".into() });
        }
        for id in group {
            match file.axes.iter().find(|a| &a.id == id) {
                None => issues.push(Issue { path: gp.clone(), message: format!("unknown axis '{id}'") }),
                Some(a) if a.homing.is_none() => issues.push(Issue { path: gp.clone(), message: format!("axis '{id}' has no [axis.homing]") }),
                Some(_) => {}
            }
            if !seen.insert(id.clone()) {
                issues.push(Issue { path: gp.clone(), message: format!("axis '{id}' appears more than once") });
            }
        }
    }
    for a in file.axes.iter().filter(|a| a.homing.is_some()) {
        if !seen.contains(&a.id) {
            issues.push(Issue { path: "home_all.groups".into(), message: format!("homeable axis '{}' is in no group", a.id) });
        }
    }
}

impl ResolvedRobot {
    /// The home-all plan as groups of axis indices: the profile's
    /// `[home_all]` groups, or one group per homeable axis in profile order.
    pub fn home_all_plan(&self) -> Vec<Vec<usize>> {
        let index = |id: &str| self.file.axes.iter().position(|a| a.id == id);
        match &self.file.home_all {
            Some(plan) => plan.groups.iter().map(|g| g.iter().filter_map(|id| index(id)).collect()).collect(),
            None => (0..self.file.axes.len()).filter(|&i| self.file.axes[i].homing.is_some()).map(|i| vec![i]).collect(),
        }
    }

    /// True when every axis has geometry with a link radius, i.e. the
    /// engine can check moves for self-collision.
    pub fn has_collision_model(&self) -> bool {
        !self.file.axes.is_empty() && self.file.axes.iter().all(|a| a.geometry.as_ref().is_some_and(|g| g.radius.is_some()))
    }

    pub fn axis(&self, id: &str) -> Option<&AxisSection> {
        self.file.axes.iter().find(|a| a.id == id)
    }
}

impl AxisSection {
    pub fn stop_deceleration(&self) -> f64 {
        self.stop_deceleration.unwrap_or(2.0 * self.max_acceleration)
    }
}

#[cfg(test)]
mod tests;

fn add(issues: &mut Vec<Issue>, path: String, message: String) {
    issues.push(Issue { path, message });
}
