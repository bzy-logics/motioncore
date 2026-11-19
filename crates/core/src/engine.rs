//! The MotionCore engine: single authority for robot state.
//!
//! All state lives in [`Inner`] behind a `std::sync::Mutex` that is never held
//! across an `.await`. Commands follow the motion pipeline of the spec (section 7.1):
//! check state/ownership → resolve geometry → plan → validate → convert to
//! timestamped MCU segments → queue → observe feedback.

use std::collections::{BTreeSet, VecDeque};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use motioncore_device_graph::{AxisSection, Direction, JointKind, ResolvedMcu, ResolvedRobot, Transport, Unit};
use motioncore_engine_api::v1 as pb;
use motioncore_kinematics::chain::{pose_from, Chain, ToolPose};
use motioncore_kinematics::collision::{CollisionModel, JointSpec};
use motioncore_mcu_manager::{McuError, McuEvent, McuLink};
use motioncore_motion::blend::plan_blended;
use motioncore_motion::sampled::time_scale;
use motioncore_motion::{plan_axis_move, plan_synchronized_scaled, to_segments, AxisLimits, MotionError, Profile};
use motioncore_plugin_runtime::{PluginKind, PluginState, Registry};
use motioncore_protocol::{capability_names, AxisConfig, FaultCode as McuFault, HomeParams, HostMsg, Identity, NackReason, Segment, Status};
use motioncore_safety::{Event, Fault, FaultCode, RobotState, Supervisor};
use tokio::sync::{broadcast, mpsc};

use crate::error::ApiError;

/// Lead time between planning a move and its first segment on the MCU clock.
const LEAD_US: u64 = 100_000;
/// Maximum duration of one MCU segment.
const MAX_SEGMENT_US: u32 = 20_000;
/// Unhomed axes are limited to this fraction of their maximum velocity.
const UNHOMED_SPEED_FACTOR: f64 = 0.25;
const RECONNECT_PERIOD: Duration = Duration::from_secs(2);
/// How long a controlled stop may take before the engine gives up and faults.
const STOP_CONFIRM_TIMEOUT: Duration = Duration::from_secs(3);
/// How long fresh telemetry may take to confirm a homed position.
const HOMING_CONFIRM_TIMEOUT: Duration = Duration::from_millis(500);

#[derive(Debug, Clone, Default)]
pub struct Options {
    pub fault_injection: bool,
    /// Test hook: wait this long before planning a move (outside the engine
    /// lock), to exercise commands that arrive while a plan is in flight.
    /// Zero in production.
    pub planning_delay: Duration,
}

struct AxisRt {
    cfg: AxisSection,
    homed: bool,
    enabled: bool,
    /// Target of the current/last motion (axis units).
    commanded: f64,
}

struct Inner {
    sup: Supervisor,
    link: Option<McuLink>,
    identity: Option<Identity>,
    /// Aborts the in-process virtual MCU (fault injection: "unplug").
    virtual_task: Option<tokio::task::AbortHandle>,
    axes: Vec<AxisRt>,
    /// Segments planned but not yet sent to the MCU.
    pending: VecDeque<Segment>,
    /// MCU channel and end time (MCU clock) of segments sent and possibly
    /// still queued. The MCU queues segments per axis.
    in_flight: VecDeque<(u8, u64)>,
    last_end_us: u64,
    /// Bumped whenever planned motion is discarded (stop, fault, e-stop).
    motion_gen: u64,
    /// Bumped whenever anything a motion plan depends on changes: every
    /// supervisor transition, discarded motion, resync from the MCU, homing
    /// results. A plan is committed only if it is unchanged (#22).
    revision: u64,
    telemetry_seq: u64,
    /// Homing in progress (Home = one group of one axis, HomeAll = the plan).
    homing: Option<HomingRun>,
}

/// A homing run: the axes of one group home at the same time, groups in order.
struct HomingRun {
    groups: Vec<Vec<usize>>,
    /// Index of the next group to start.
    next: usize,
    /// Axes of the current group still homing.
    pending: BTreeSet<usize>,
}

/// Required surface clearance for the collision check, mm.
const COLLISION_MARGIN_MM: f64 = 2.0;

pub struct Engine {
    robot: ResolvedRobot,
    mcu: ResolvedMcu,
    opts: Options,
    inner: Mutex<Inner>,
    plugins: Mutex<Registry>,
    started: Instant,
    logs: broadcast::Sender<pb::LogEntry>,
    telemetry: broadcast::Sender<Arc<pb::Telemetry>>,
    /// Capsule model (when every axis has a link radius) and the axis index
    /// of each model joint (the model lists parents first).
    collision: Option<Arc<(CollisionModel, Vec<usize>)>>,
    /// Serial chain for Cartesian moves and the tool pose, and the axis
    /// index of each chain joint.
    chain: Option<Arc<(Chain, Vec<usize>)>>,
}

/// Target of a single-axis move.
enum MoveTarget {
    Absolute(f64),
    Relative(f64),
}

/// A motion planned off the engine lock: segments timed from 0 (placed on the
/// MCU clock at commit), the commanded positions at its end, a log line.
struct PlannedMotion {
    segs: Vec<Segment>,
    ends: Vec<(usize, f64)>,
    log: String,
}

fn axis_limits(c: &AxisSection) -> AxisLimits {
    AxisLimits { min: c.min, max: c.max, max_velocity: c.max_velocity, max_acceleration: c.max_acceleration }
}

/// Build the collision model from the profile geometry, parents first.
fn collision_model(robot: &ResolvedRobot) -> Option<(CollisionModel, Vec<usize>)> {
    if !robot.has_collision_model() {
        return None;
    }
    let (joints, order) = joint_specs(robot)?;
    Some((CollisionModel::new(joints, COLLISION_MARGIN_MM), order))
}

/// The serial chain for Cartesian moves, when the geometry is one.
fn serial_chain(robot: &ResolvedRobot) -> Option<(Chain, Vec<usize>)> {
    let (joints, order) = joint_specs(robot)?;
    let axes = &robot.file.axes;
    let limits = order.iter().map(|&i| (axes[i].min, axes[i].max)).collect();
    Chain::new(joints, limits).ok().map(|c| (c, order))
}

/// Joints of the profile geometry, parents first, and the axis index of each.
fn joint_specs(robot: &ResolvedRobot) -> Option<(Vec<JointSpec>, Vec<usize>)> {
    let axes = &robot.file.axes;
    let mut order: Vec<usize> = Vec::with_capacity(axes.len());
    while order.len() < axes.len() {
        let before = order.len();
        for (i, a) in axes.iter().enumerate() {
            if order.contains(&i) {
                continue;
            }
            let g = a.geometry.as_ref()?;
            let parent_ok = match &g.parent {
                None => true,
                Some(p) => order.iter().any(|&o| &axes[o].id == p),
            };
            if parent_ok {
                order.push(i);
            }
        }
        if order.len() == before {
            return None; // validated away by the resolver; be defensive
        }
    }
    let joints = order
        .iter()
        .map(|&i| {
            let a = &axes[i];
            let g = a.geometry.as_ref().unwrap();
            JointSpec {
                id: a.id.clone(),
                parent: g.parent.as_ref().and_then(|p| order.iter().position(|&o| &axes[o].id == p)),
                origin: g.origin,
                rotation_deg: g.rotation,
                axis: g.axis,
                tip: g.tip,
                radius: g.radius.unwrap_or(0.0),
                prismatic: a.kind == JointKind::Prismatic,
            }
        })
        .collect();
    Some((joints, order))
}

fn vec3(v: [f64; 3]) -> pb::Vec3 {
    pb::Vec3 { x: v[0], y: v[1], z: v[2] }
}

/// Planning steps of a Cartesian line: tool travel and rotation per sample.
const LINE_STEP_MM: f64 = 0.5;
const LINE_STEP_DEG: f64 = 0.25;
/// Default and maximum tool speed of MoveL, mm/s.
const LINE_SPEED_MM_S: f64 = 50.0;
const LINE_SPEED_MAX_MM_S: f64 = 2000.0;
/// Tool acceleration, rotation speed and acceleration of MoveL.
const LINE_ACCEL_MM_S2: f64 = 500.0;
const LINE_TURN_DEG_S: f64 = 45.0;
const LINE_TURN_DEG_S2: f64 = 180.0;
/// Most waypoints in one MoveJPath.
const MAX_WAYPOINTS: usize = 256;

impl Engine {
    pub fn new(robot: ResolvedRobot, plugins: Registry, opts: Options) -> anyhow::Result<Arc<Self>> {
        if robot.mcus.len() != 1 {
            anyhow::bail!("v0.1 supports exactly one MCU per robot (multi-MCU sync is a later phase)");
        }
        for p in &robot.required_plugins {
            if !plugins.is_active(p) {
                anyhow::bail!("required plugin '{p}' is not active");
            }
        }
        let mcu = robot.mcus.values().next().unwrap().clone();
        let axes = robot.file.axes.iter().map(|a| AxisRt { cfg: a.clone(), homed: false, enabled: false, commanded: 0.0 }).collect();
        let collision = collision_model(&robot).map(Arc::new);
        let chain = serial_chain(&robot).map(Arc::new);
        Ok(Arc::new(Self {
            robot,
            mcu,
            opts,
            inner: Mutex::new(Inner {
                sup: Supervisor::new(),
                link: None,
                identity: None,
                virtual_task: None,
                axes,
                pending: VecDeque::new(),
                in_flight: VecDeque::new(),
                last_end_us: 0,
                motion_gen: 0,
                revision: 0,
                telemetry_seq: 0,
                homing: None,
            }),
            collision,
            chain,
            plugins: Mutex::new(plugins),
            started: Instant::now(),
            logs: broadcast::channel(256).0,
            telemetry: broadcast::channel(16).0,
        }))
    }

    pub fn time_ms(&self) -> u64 {
        self.started.elapsed().as_millis() as u64
    }

    pub fn log(&self, level: &'static str, message: impl Into<String>) {
        let message = message.into();
        match level {
            "error" => tracing::error!("{message}"),
            "warn" => tracing::warn!("{message}"),
            "debug" => tracing::debug!("{message}"),
            _ => tracing::info!("{message}"),
        }
        let level = match level {
            "error" => pb::LogLevel::Error,
            "warn" => pb::LogLevel::Warn,
            "debug" => pb::LogLevel::Debug,
            _ => pb::LogLevel::Info,
        };
        let _ = self.logs.send(pb::LogEntry { time_ms: self.time_ms(), level: level as i32, message });
    }

    pub fn subscribe_logs(&self) -> broadcast::Receiver<pb::LogEntry> {
        self.logs.subscribe()
    }

    pub fn subscribe_telemetry(&self) -> broadcast::Receiver<Arc<pb::Telemetry>> {
        self.telemetry.subscribe()
    }

    pub fn state(&self) -> RobotState {
        self.inner.lock().unwrap().sup.state()
    }

    /// Start background tasks: connection management, MCU feeder, telemetry.
    pub fn spawn(self: &Arc<Self>) {
        tokio::spawn(self.clone().connection_loop());
        tokio::spawn(self.clone().feeder_loop());
        tokio::spawn(self.clone().telemetry_loop());
    }

    // ------------------------------------------------------------------
    // Connection management
    // ------------------------------------------------------------------

    async fn connection_loop(self: Arc<Self>) {
        loop {
            let needs = {
                let g = self.inner.lock().unwrap();
                g.link.is_none() && g.sup.state() == RobotState::Disconnected
            };
            if needs {
                if let Err(e) = self.connect().await {
                    self.log("warn", format!("MCU connection failed: {e}"));
                    let mut g = self.inner.lock().unwrap();
                    if let Some(t) = g.virtual_task.take() {
                        t.abort();
                    }
                    g.link = None;
                    if g.sup.state() == RobotState::Initializing {
                        let _ = g.apply(Event::ConnectionLost);
                    }
                }
            }
            tokio::time::sleep(RECONNECT_PERIOD).await;
        }
    }

    async fn open_transport(&self) -> anyhow::Result<(McuLink, mpsc::UnboundedReceiver<McuEvent>, Option<tokio::task::AbortHandle>)> {
        match &self.mcu.transport {
            Transport::Virtual => {
                let (host, mcu) = tokio::io::duplex(8192);
                let opts = motioncore_simulator::VirtualOptions::from_robot(&self.robot, &self.mcu.name);
                let task = tokio::spawn(async move {
                    let _ = motioncore_simulator::run(mcu, opts).await;
                });
                let (link, ev) = McuLink::start(host);
                Ok((link, ev, Some(task.abort_handle())))
            }
            Transport::Tcp(addr) => {
                let sock = tokio::time::timeout(Duration::from_secs(2), tokio::net::TcpStream::connect(addr)).await??;
                sock.set_nodelay(true)?;
                let (link, ev) = McuLink::start(sock);
                Ok((link, ev, None))
            }
            Transport::Serial(dev) => {
                use tokio_serial::SerialPortBuilderExt;
                // USB CDC-ACM ignores the baud rate; UART bridges need it.
                let port = tokio_serial::new(dev, 250_000).open_native_async()?;
                let (link, ev) = McuLink::start(port);
                Ok((link, ev, None))
            }
        }
    }

    async fn connect(self: &Arc<Self>) -> anyhow::Result<()> {
        self.log("info", format!("connecting to MCU '{}' via {}", self.mcu.name, self.mcu.transport.kind()));
        let (link, events, vtask) = self.open_transport().await?;
        {
            let mut g = self.inner.lock().unwrap();
            g.apply(Event::Connect)?;
            g.link = Some(link.clone());
            g.virtual_task = vtask;
        }
        tokio::spawn(self.clone().event_loop(events));

        let id = link.identify().await?;
        check_identity(&self.mcu, &self.robot.file.axes, &id).map_err(|e| anyhow::anyhow!(e))?;
        let clock = link.sync_clock(8).await?;
        link.command(HostMsg::SetWatchdog { timeout_ms: self.mcu.watchdog_ms }).await?;
        for i in 0..self.robot.file.axes.len() {
            let cfg = self.mcu_axis_config(i, false, 0);
            link.command(HostMsg::ConfigureAxis(cfg)).await?;
            link.command(HostMsg::SetPosition { axis: cfg.axis, steps: 0 }).await?;
        }
        self.log(
            "info",
            format!(
                "MCU '{}' ready: {} {} fw {}.{}.{} build {:08x}, caps [{}], clock offset {} µs (rtt {} µs)",
                self.mcu.name,
                id.target,
                id.board,
                id.firmware_version.0,
                id.firmware_version.1,
                id.firmware_version.2,
                id.build_hash,
                capability_names(id.capabilities).collect::<Vec<_>>().join(", "),
                clock.offset_us,
                clock.rtt_us
            ),
        );
        let mut g = self.inner.lock().unwrap();
        g.identity = Some(id);
        for a in g.axes.iter_mut() {
            a.homed = false;
            a.enabled = false;
            a.commanded = 0.0;
        }
        g.apply(Event::Initialized)?;
        Ok(())
    }

    /// MCU-side axis configuration. Before homing the soft limits are a
    /// window of one full travel range around the current position, because
    /// absolute position is unknown.
    fn mcu_axis_config(&self, i: usize, homed: bool, current_steps: i32) -> AxisConfig {
        let a = &self.robot.file.axes[i];
        let spu = a.steps_per_unit;
        let (min_steps, max_steps) = if homed {
            ((a.min * spu).floor() as i32, (a.max * spu).ceil() as i32)
        } else {
            let range = ((a.max - a.min) * spu).ceil() as i32;
            (current_steps.saturating_sub(range), current_steps.saturating_add(range))
        };
        AxisConfig {
            axis: a.channel,
            min_steps,
            max_steps,
            max_rate: (a.max_velocity * spu * 1.05) as f32,
            stop_decel: (a.stop_deceleration() * spu) as f32,
            invert_dir: a.invert_dir,
            invert_endstop: a.endstop.as_ref().is_some_and(|e| e.invert),
        }
    }

    async fn event_loop(self: Arc<Self>, mut events: mpsc::UnboundedReceiver<McuEvent>) {
        while let Some(ev) = events.recv().await {
            match ev {
                McuEvent::Fault { code, axis } => {
                    let axis_id = axis.and_then(|ch| self.axis_by_channel(ch));
                    let (fc, what) = map_mcu_fault(code);
                    let mut f = Fault::new(fc, format!("MCU fault: {what}"));
                    if let Some(id) = &axis_id {
                        f = f.on_axis(id.clone());
                    }
                    self.log("error", format!("{}{}", f.message, axis_id.map(|a| format!(" on axis {a}")).unwrap_or_default()));
                    self.fault(f);
                }
                McuEvent::HomingDone { axis, position_steps } => {
                    let Some(i) = self.axis_index_by_channel(axis) else { continue };
                    let link = self.inner.lock().unwrap().link.clone();
                    // Install the real soft limits now that the frame is known.
                    let cfg = self.mcu_axis_config(i, true, position_steps);
                    let res = match link {
                        Some(l) => match l.command(HostMsg::ConfigureAxis(cfg)).await {
                            Err(e) => Err(format!("failed to apply homed limits: {e}")),
                            // Don't report "homed" until fresh telemetry shows
                            // the axis idle at the homed position.
                            Ok(()) => fresh_status(&l, HOMING_CONFIRM_TIMEOUT, |s| {
                                s.axes.get(axis as usize).is_some_and(|a| !a.homing && !a.moving && a.position_steps == position_steps)
                            })
                            .await
                            .map(|_| ())
                            .map_err(|e| format!("homed position not confirmed: {e}")),
                        },
                        None => Err(McuError::LinkDown.to_string()),
                    };
                    let mut g = self.inner.lock().unwrap();
                    if g.sup.state() != RobotState::Homing {
                        continue;
                    }
                    if let Err(e) = res {
                        drop(g);
                        let id = self.robot.file.axes[i].id.clone();
                        self.log("error", format!("homing axis {id}: {e}"));
                        self.fault(Fault::new(FaultCode::HomingFailed, e).on_axis(id));
                        continue;
                    }
                    let a = &mut g.axes[i];
                    a.homed = true;
                    a.commanded = position_steps as f64 / a.cfg.steps_per_unit;
                    let id = a.cfg.id.clone();
                    let pos = a.commanded;
                    g.revision += 1;
                    let group_done = match g.homing.as_mut() {
                        Some(run) => {
                            run.pending.remove(&i);
                            run.pending.is_empty()
                        }
                        None => true,
                    };
                    let more = g.homing.as_ref().is_some_and(|r| r.next < r.groups.len());
                    if group_done && !more {
                        g.homing = None;
                        let _ = g.apply(Event::HomingComplete);
                    }
                    drop(g);
                    self.log("info", format!("axis {id} homed at {pos:.3}"));
                    if group_done && more {
                        let this = self.clone();
                        tokio::spawn(async move {
                            let _ = this.start_homing_group().await;
                        });
                    }
                }
                McuEvent::LinkLost { reason } => {
                    self.log("error", format!("MCU link lost: {reason}"));
                    let mut g = self.inner.lock().unwrap();
                    g.link = None;
                    g.identity = None;
                    if let Some(t) = g.virtual_task.take() {
                        t.abort();
                    }
                    clear_motion(&mut g);
                    let _ = g.apply(Event::ConnectionLost);
                    return;
                }
            }
        }
    }

    /// Latch a fault: discard planned motion, forget homing, stop the MCU.
    fn fault(&self, f: Fault) {
        let mut g = self.inner.lock().unwrap();
        clear_motion(&mut g);
        g.homing = None;
        for a in g.axes.iter_mut() {
            a.homed = false;
            a.enabled = false;
        }
        let _ = g.apply(Event::Fault(f));
        if let Some(l) = &g.link {
            // The MCU already disabled its drives for MCU-side faults; for
            // host-side faults make sure it does too.
            let _ = l.send(HostMsg::EmergencyStop);
        }
    }

    fn axis_index_by_channel(&self, ch: u8) -> Option<usize> {
        self.robot.file.axes.iter().position(|a| a.channel == ch)
    }

    fn axis_by_channel(&self, ch: u8) -> Option<String> {
        self.axis_index_by_channel(ch).map(|i| self.robot.file.axes[i].id.clone())
    }

    fn axis_index(&self, id: &str) -> Result<usize, ApiError> {
        self.robot.file.axes.iter().position(|a| a.id == id).ok_or_else(|| ApiError::new("unknown_axis", format!("unknown axis '{id}'")))
    }

    fn link(&self) -> Result<McuLink, ApiError> {
        self.inner.lock().unwrap().link.clone().filter(|l| l.is_alive()).ok_or_else(|| ApiError::mcu("MCU not connected"))
    }

    // ------------------------------------------------------------------
    // Motion feeding
    // ------------------------------------------------------------------

    async fn feeder_loop(self: Arc<Self>) {
        let mut iv = tokio::time::interval(Duration::from_millis(5));
        loop {
            iv.tick().await;
            // Segments are *submitted* under the state lock so that a Stop /
            // E-stop (also submitted under the lock) can never be overtaken.
            let (generation, acks) = {
                let mut g = self.inner.lock().unwrap();
                let Some(link) = g.link.clone() else { continue };
                let cap = g.identity.as_ref().map(|i| i.queue_capacity as usize).unwrap_or(0);
                let now = link.mcu_now_us();
                g.in_flight.retain(|&(_, end)| end >= now);
                // Each axis has its own MCU queue; keep a few slots of slack
                // per axis. Pending segments are in time order across axes,
                // so stopping at the first full axis keeps every axis fed
                // equally far ahead.
                let per_axis = cap.saturating_sub(4);
                let mut queued = [0usize; 256];
                for &(ch, _) in &g.in_flight {
                    queued[ch as usize] += 1;
                }
                let mut acks = Vec::new();
                while let Some(&s) = g.pending.front() {
                    if queued[s.axis as usize] >= per_axis {
                        break;
                    }
                    g.pending.pop_front();
                    match link.submit(HostMsg::QueueSegment(s)) {
                        Ok(a) => {
                            g.in_flight.push_back((s.axis, s.end_us()));
                            queued[s.axis as usize] += 1;
                            acks.push(a);
                        }
                        Err(McuError::Backpressure) => {
                            // Outbound queue full: retry on the next tick.
                            g.pending.push_front(s);
                            break;
                        }
                        Err(_) => break, // link loss is handled by the event loop
                    }
                }
                if acks.is_empty() {
                    self.check_motion_complete(&mut g, &link);
                    continue;
                }
                (g.motion_gen, acks)
            };
            for ack in acks {
                if let Err(e) = ack.wait().await {
                    let current = self.inner.lock().unwrap().motion_gen;
                    if current == generation && !matches!(e, McuError::LinkDown) {
                        self.log("error", format!("segment rejected: {e}"));
                        self.fault(Fault::new(FaultCode::McuFault, format!("motion segment rejected: {e}")));
                    }
                    break;
                }
            }
        }
    }

    fn check_motion_complete(&self, g: &mut Inner, link: &McuLink) {
        if g.sup.state() != RobotState::Running || !g.pending.is_empty() || !g.in_flight.is_empty() {
            return;
        }
        let Some(st) = link.latest_status() else { return };
        if st.mcu_time_us < g.last_end_us || st.axes.iter().any(|a| a.moving) || st.queue_depth > 0 {
            return;
        }
        // The MCU is authoritative for where the axes actually are.
        for a in g.axes.iter_mut() {
            if let Some(s) = st.axes.get(a.cfg.channel as usize) {
                a.commanded = s.position_steps as f64 / a.cfg.steps_per_unit;
            }
        }
        let _ = g.apply(Event::MotionComplete);
    }

    // ------------------------------------------------------------------
    // Telemetry
    // ------------------------------------------------------------------

    async fn telemetry_loop(self: Arc<Self>) {
        let mut iv = tokio::time::interval(Duration::from_millis(50));
        loop {
            iv.tick().await;
            let t = Arc::new(self.telemetry_snapshot());
            let _ = self.telemetry.send(t);
        }
    }

    pub fn telemetry_snapshot(&self) -> pb::Telemetry {
        let mut g = self.inner.lock().unwrap();
        g.telemetry_seq += 1;
        let link = g.link.clone();
        let status = link.as_ref().and_then(|l| l.latest_status());
        let mut mcu = pb::McuTelemetry { transport: self.mcu.transport.kind().into(), ..Default::default() };
        if let Some(l) = &link {
            let s = l.stats();
            mcu.connected = l.is_alive() && g.identity.is_some();
            mcu.clock_offset_us = s.clock.offset_us;
            mcu.rtt_us = s.clock.rtt_us;
            mcu.tx_frames = s.tx_frames;
            mcu.rx_frames = s.rx_frames;
            mcu.crc_errors = s.crc_errors;
            mcu.last_heartbeat_ms_ago = Some(s.last_rx_ms_ago);
        }
        if let Some(id) = &g.identity {
            mcu.target = id.target.to_string();
            mcu.board = id.board.to_string();
            mcu.firmware_version = format!("{}.{}.{}", id.firmware_version.0, id.firmware_version.1, id.firmware_version.2);
            mcu.protocol_version = id.protocol_version as u32;
            mcu.build_hash = format!("{:08x}", id.build_hash);
            mcu.capabilities = capability_names(id.capabilities).map(String::from).collect();
            mcu.queue_capacity = id.queue_capacity as u32;
        }
        if let Some(s) = &status {
            mcu.queue_depth = s.queue_depth as u32;
        }
        let homing_axes: BTreeSet<usize> = g.homing.as_ref().map(|r| r.pending.clone()).unwrap_or_default();
        let axes: Vec<pb::AxisTelemetry> = g
            .axes
            .iter()
            .enumerate()
            .map(|(i, a)| {
                let st = status.as_ref().and_then(|s| s.axes.get(a.cfg.channel as usize));
                let spu = a.cfg.steps_per_unit;
                pb::AxisTelemetry {
                    id: a.cfg.id.clone(),
                    homed: a.homed,
                    enabled: st.map(|s| s.enabled).unwrap_or(false),
                    commanded_position: a.commanded,
                    measured_position: st.map(|s| s.position_steps as f64 / spu).unwrap_or(a.commanded),
                    velocity: st.map(|s| s.velocity as f64 / spu).unwrap_or(0.0),
                    endstop: st.map(|s| s.endstop).unwrap_or(false),
                    homing: homing_axes.contains(&i),
                }
            })
            .collect();
        let tool = self.chain.as_deref().map(|(c, order)| {
            let q: Vec<f64> = order.iter().map(|&k| axes[k].measured_position).collect();
            let p = c.forward(&q);
            pb::ToolPose { position_mm: Some(vec3(p.position)), rotation_deg: Some(vec3(p.rpy_deg())) }
        });
        pb::Telemetry {
            seq: g.telemetry_seq,
            time_ms: self.time_ms(),
            state: state_pb(g.sup.state()) as i32,
            fault: g.sup.fault().map(fault_pb),
            mcu: Some(mcu),
            axes,
            tool,
        }
    }

    pub fn robot(&self) -> &ResolvedRobot {
        &self.robot
    }

    pub fn fault_injection_enabled(&self) -> bool {
        self.opts.fault_injection
    }

    /// Drives may be energised (flashing must be refused).
    pub fn drives_energised(&self) -> bool {
        self.state().drives_enabled()
    }

    pub fn robot_config(&self) -> pb::RobotConfig {
        let f = &self.robot.file;
        pb::RobotConfig {
            name: f.robot.name.clone(),
            profile: f.robot.profile.clone(),
            mcus: f
                .mcu
                .iter()
                .map(|(name, m)| pb::McuConfig { name: name.clone(), board: m.board.clone(), transport: m.transport.clone(), watchdog_ms: m.watchdog_ms })
                .collect(),
            axes: f
                .axes
                .iter()
                .map(|a| pb::AxisConfig {
                    id: a.id.clone(),
                    name: a.name.clone(),
                    kind: match a.kind {
                        JointKind::Revolute => pb::JointKind::Revolute,
                        JointKind::Prismatic => pb::JointKind::Prismatic,
                    } as i32,
                    unit: match a.unit {
                        Unit::Deg => "deg",
                        Unit::Mm => "mm",
                    }
                    .into(),
                    min: a.min,
                    max: a.max,
                    max_velocity: a.max_velocity,
                    max_acceleration: a.max_acceleration,
                    home_position: a.homing.as_ref().map(|h| h.position),
                    steps_per_unit: a.steps_per_unit,
                    mcu: a.mcu.clone(),
                    mcu_channel: a.channel as u32,
                    actuator: a.driver.actuator.clone(),
                    step_pin: a.driver.step_pin.clone(),
                    dir_pin: a.driver.dir_pin.clone(),
                    enable_pin: a.driver.enable_pin.clone(),
                    endstop_pin: a.endstop.as_ref().map(|e| e.pin.clone()).unwrap_or_default(),
                    geometry: a.geometry.as_ref().map(|g| {
                        let v = |x: [f64; 3]| pb::Vec3 { x: x[0], y: x[1], z: x[2] };
                        pb::AxisGeometry {
                            parent: g.parent.clone().unwrap_or_default(),
                            origin_mm: Some(v(g.origin)),
                            rotation_deg: Some(v(g.rotation)),
                            axis: Some(v(g.axis)),
                            tip_mm: g.tip.map(v),
                            radius_mm: g.radius,
                        }
                    }),
                })
                .collect(),
            home_all: Some(pb::HomeAllPlan {
                groups: self
                    .robot
                    .home_all_plan()
                    .into_iter()
                    .map(|grp| pb::HomeAllGroup { axes: grp.into_iter().map(|i| f.axes[i].id.clone()).collect() })
                    .collect(),
                start_pose: f.home_all.as_ref().and_then(|h| h.start_pose.clone()).unwrap_or_default(),
                from_profile: f.home_all.is_some(),
            }),
            collision_checking: self.collision.is_some(),
        }
    }

    pub fn plugins(&self) -> Vec<pb::Plugin> {
        self.plugins
            .lock()
            .unwrap()
            .info()
            .into_iter()
            .map(|p| pb::Plugin {
                id: p.id,
                version: p.version,
                api_version: p.api_version,
                kind: match p.kind {
                    PluginKind::Builtin => "builtin",
                    PluginKind::Wasm => "wasm",
                }
                .into(),
                description: p.description,
                provides: p.provides,
                requires: p.requires,
                permissions: p.permissions,
                state: match p.state {
                    PluginState::Discovered => pb::PluginState::Discovered,
                    PluginState::Validated => pb::PluginState::Validated,
                    PluginState::Resolved => pb::PluginState::Resolved,
                    PluginState::Initialized => pb::PluginState::Initialized,
                    PluginState::Active => pb::PluginState::Active,
                    PluginState::Failed => pb::PluginState::Failed,
                    PluginState::Deactivated => pb::PluginState::Deactivated,
                } as i32,
                error: p.error.unwrap_or_default(),
            })
            .collect()
    }

    // ------------------------------------------------------------------
    // Commands
    // ------------------------------------------------------------------

    fn require(&self, event: &Event) -> Result<(), ApiError> {
        self.inner.lock().unwrap().sup.check(event).map(|_| ()).map_err(|e| ApiError::invalid_state(e.to_string()))
    }

    pub async fn enable(&self) -> Result<(), ApiError> {
        self.require(&Event::Enable)?;
        let link = self.link()?;
        for a in &self.robot.file.axes {
            link.command(HostMsg::Enable { axis: a.channel }).await.map_err(ApiError::mcu)?;
        }
        let mut g = self.inner.lock().unwrap();
        g.apply(Event::Enable).map_err(|e| ApiError::invalid_state(e.to_string()))?;
        g.axes.iter_mut().for_each(|a| a.enabled = true);
        drop(g);
        self.log("info", "drives enabled");
        Ok(())
    }

    pub async fn disable(&self) -> Result<(), ApiError> {
        self.require(&Event::Disable)?;
        let link = self.link()?;
        let moving = self.state() != RobotState::Ready && self.state() != RobotState::Disabled;
        if moving {
            let stopped = async {
                let ack = {
                    let mut g = self.inner.lock().unwrap();
                    clear_motion(&mut g);
                    link.submit(HostMsg::Stop).map_err(|e| e.to_string())?
                };
                ack.wait().await.map_err(|e| e.to_string())?;
                self.wait_idle(&link, STOP_CONFIRM_TIMEOUT).await
            }
            .await;
            if let Err(why) = stopped {
                // The controlled stop could not be confirmed: fall back to the
                // emergency stop path, which cuts the drives immediately.
                let msg = format!("disable: could not confirm the robot stopped: {why}");
                self.log("error", msg.clone());
                self.fault(Fault::new(FaultCode::McuFault, msg.clone()));
                return Err(ApiError::mcu(msg));
            }
        }
        for a in &self.robot.file.axes {
            link.command(HostMsg::Disable { axis: a.channel }).await.map_err(ApiError::mcu)?;
        }
        let mut g = self.inner.lock().unwrap();
        clear_motion(&mut g);
        resync_from_mcu(&mut g, &link);
        g.axes.iter_mut().for_each(|a| a.enabled = false);
        let _ = g.apply(Event::Disable);
        drop(g);
        self.log("info", "drives disabled");
        Ok(())
    }

    /// Wait until fresh MCU telemetry shows every axis at rest with an empty
    /// segment queue. Errors on timeout, MCU fault or link loss.
    async fn wait_idle(&self, link: &McuLink, timeout: Duration) -> Result<(), String> {
        fresh_status(link, timeout, |s| s.queue_depth == 0 && !s.axes.iter().any(|a| a.moving)).await.map(|_| ())
    }

    pub async fn jog(&self, axis: &str, delta: f64, velocity: Option<f64>) -> Result<(), ApiError> {
        let i = self.axis_index(axis)?;
        if !delta.is_finite() {
            return Err(ApiError::new("validation_failed", "delta must be finite"));
        }
        self.start_move(i, MoveTarget::Relative(delta), velocity).await
    }

    pub async fn move_to(&self, axis: &str, position: f64, velocity: Option<f64>) -> Result<(), ApiError> {
        let i = self.axis_index(axis)?;
        self.start_move(i, MoveTarget::Absolute(position), velocity).await
    }

    /// Run planning work off the engine lock, on a blocking worker, so
    /// telemetry, Stop and E-stop are never held up by it.
    async fn plan_off_lock<T: Send + 'static>(&self, plan: impl FnOnce() -> Result<T, ApiError> + Send + 'static) -> Result<T, ApiError> {
        if !self.opts.planning_delay.is_zero() {
            tokio::time::sleep(self.opts.planning_delay).await;
        }
        tokio::task::spawn_blocking(plan).await.map_err(|e| ApiError::new("internal", format!("planning task failed: {e}")))?
    }

    /// Commit a motion planned from the state at `revision`: refused if
    /// anything that planning depended on changed meanwhile (another move,
    /// Stop, a fault, homing, MCU reconnection); otherwise its segments are
    /// placed on the MCU clock and queued.
    fn commit(&self, revision: u64, planned: PlannedMotion) -> Result<(), ApiError> {
        let mut g = self.inner.lock().unwrap();
        g.sup.check(&Event::StartMotion).map_err(|e| ApiError::invalid_state(e.to_string()))?;
        if g.revision != revision {
            return Err(ApiError::invalid_state("the robot state changed while the move was planned; nothing was sent"));
        }
        let link = g.link.clone().filter(|l| l.is_alive()).ok_or_else(|| ApiError::mcu("MCU not connected"))?;
        let start_us = link.mcu_now_us() + LEAD_US;
        let segs = planned
            .segs
            .into_iter()
            .map(|mut s| {
                s.start_us += start_us;
                s
            })
            .collect();
        self.start_planned(&mut g, segs, start_us, &planned.ends)?;
        drop(g);
        self.log("debug", planned.log);
        Ok(())
    }

    async fn start_move(&self, i: usize, target: MoveTarget, velocity: Option<f64>) -> Result<(), ApiError> {
        let cfg = self.robot.file.axes[i].clone();
        let relative = matches!(target, MoveTarget::Relative(_));
        let (revision, start, target, limits, velocity, pose) = {
            let g = self.inner.lock().unwrap();
            g.sup.check(&Event::StartMotion).map_err(|e| ApiError::invalid_state(e.to_string()))?;
            g.link.clone().filter(|l| l.is_alive()).ok_or_else(|| ApiError::mcu("MCU not connected"))?;
            let a = &g.axes[i];
            if matches!(target, MoveTarget::Absolute(_)) && !a.homed {
                return Err(ApiError::new("not_homed", format!("axis {} must be homed before absolute moves", cfg.id)));
            }
            let mut limits = AxisLimits { min: cfg.min, max: cfg.max, max_velocity: cfg.max_velocity, max_acceleration: cfg.max_acceleration };
            let mut velocity = velocity;
            if !a.homed {
                // Position unknown: allow relative jogs within one travel range
                // of the power-on position, at reduced speed.
                let range = cfg.max - cfg.min;
                limits.min = -range;
                limits.max = range;
                let cap = cfg.max_velocity * UNHOMED_SPEED_FACTOR;
                limits.max_velocity = cap;
                velocity = Some(velocity.unwrap_or(cap).min(cap));
            }
            let target = match target {
                MoveTarget::Absolute(p) => p,
                MoveTarget::Relative(d) => a.commanded + d,
            };
            // The collision check needs every position, so only once all are homed.
            let pose: Option<Vec<f64>> = g.axes.iter().all(|x| x.homed).then(|| g.axes.iter().map(|x| x.commanded).collect());
            (g.revision, a.commanded, target, limits, velocity, pose)
        };
        let collision = self.collision.clone();
        let planned = self
            .plan_off_lock(move || {
                if let (Some(m), Some(pose)) = (&collision, &pose) {
                    let (model, order) = m.as_ref();
                    let q: Vec<f64> = order.iter().map(|&k| pose[k]).collect();
                    let mi = order.iter().position(|&k| k == i).expect("every axis is in the model");
                    let to = if relative { target.clamp(cfg.min, cfg.max) } else { target };
                    if let Err(c) = model.check_sweep(&q, mi, to) {
                        let at = c.at[mi];
                        return Err(ApiError::new(
                            "collision",
                            format!("refused: {c} when {} reaches {at:.1} {}", cfg.id, if cfg.kind == JointKind::Prismatic { "mm" } else { "deg" }),
                        ));
                    }
                }
                let profile = plan_axis_move(start, target, velocity, &limits).map_err(motion_err)?;
                if profile.duration() == 0.0 {
                    return Ok(None);
                }
                Ok(Some(PlannedMotion {
                    segs: to_segments(&profile, cfg.channel, cfg.steps_per_unit, 0, MAX_SEGMENT_US),
                    ends: vec![(i, target)],
                    log: format!("move {} -> {target:.3} ({:.3} s)", cfg.id, profile.duration()),
                }))
            })
            .await?;
        match planned {
            Some(p) => self.commit(revision, p),
            None => Ok(()),
        }
    }

    /// Coordinated joint move (MoveJ): every listed axis starts and finishes
    /// together on a straight line in joint space, using the synchronised
    /// planner; unlisted axes hold their position.
    pub async fn move_joints(&self, targets: &[(String, f64)], speed_scale: Option<f64>) -> Result<(), ApiError> {
        if targets.is_empty() {
            return Err(ApiError::new("bad_request", "at least one joint target is required"));
        }
        let mut idx = Vec::with_capacity(targets.len());
        for (axis, pos) in targets {
            let i = self.axis_index(axis)?;
            if idx.contains(&i) {
                return Err(ApiError::new("bad_request", format!("axis {axis} is listed twice")));
            }
            if !pos.is_finite() {
                return Err(ApiError::new("validation_failed", format!("target of {axis} must be finite")));
            }
            idx.push(i);
        }
        let scale = speed_scale.unwrap_or(1.0);
        if !(scale.is_finite() && scale > 0.0 && scale <= 1.0) {
            return Err(ApiError::new("validation_failed", "speed_scale must be in (0, 1]"));
        }
        let axes: Vec<AxisSection> = self.robot.file.axes.clone();
        let goals: Vec<f64> = targets.iter().map(|(_, p)| *p).collect();
        for (&i, &t) in idx.iter().zip(&goals) {
            let c = &axes[i];
            if t < c.min || t > c.max {
                return Err(ApiError::new("validation_failed", format!("target {t} of {} outside limits [{}, {}]", c.id, c.min, c.max)));
            }
        }
        let (revision, pose, all_homed) = {
            let g = self.inner.lock().unwrap();
            g.sup.check(&Event::StartMotion).map_err(|e| ApiError::invalid_state(e.to_string()))?;
            g.link.clone().filter(|l| l.is_alive()).ok_or_else(|| ApiError::mcu("MCU not connected"))?;
            if let Some(&i) = idx.iter().find(|&&i| !g.axes[i].homed) {
                return Err(ApiError::new("not_homed", format!("axis {} must be homed before a joint move", g.axes[i].cfg.id)));
            }
            (g.revision, g.axes.iter().map(|a| a.commanded).collect::<Vec<f64>>(), g.axes.iter().all(|a| a.homed))
        };
        let collision = self.collision.clone();
        let planned = self
            .plan_off_lock(move || {
                let starts: Vec<f64> = idx.iter().map(|&i| pose[i]).collect();
                let limits: Vec<AxisLimits> = idx.iter().map(|&i| axis_limits(&axes[i])).collect();
                let profiles = plan_synchronized_scaled(&starts, &goals, &limits, scale).map_err(motion_err)?;
                let duration = profiles.iter().map(|p| p.duration()).fold(0.0, f64::max);
                if duration == 0.0 {
                    return Ok(None);
                }
                // Self-collision check along the whole joint-space path, once
                // every axis is homed (the model needs the full pose).
                if let (Some(m), true) = (&collision, all_homed) {
                    let (model, order) = m.as_ref();
                    let target_of = |k: usize| idx.iter().position(|&i| i == k).map_or(pose[k], |n| goals[n]);
                    let from: Vec<f64> = order.iter().map(|&k| pose[k]).collect();
                    let to: Vec<f64> = order.iter().map(|&k| target_of(k)).collect();
                    if let Err(c) = model.check_path(&from, &to) {
                        let at: Vec<String> =
                            order.iter().enumerate().filter(|(_, k)| idx.contains(k)).map(|(m, &k)| format!("{} {:.1}", axes[k].id, c.at[m])).collect();
                        return Err(ApiError::new("collision", format!("refused: {c} at {}", at.join(", "))));
                    }
                }
                // One time grid for every axis: the profiles share their phase
                // times, so their segments start and end at the same instants.
                let mut segs: Vec<Segment> = Vec::new();
                for (p, &i) in profiles.iter().zip(&idx) {
                    if p.distance != 0.0 {
                        segs.extend(to_segments(p, axes[i].channel, axes[i].steps_per_unit, 0, MAX_SEGMENT_US));
                    }
                }
                if segs.is_empty() {
                    return Ok(None);
                }
                let summary: Vec<String> = idx.iter().zip(&goals).map(|(&i, t)| format!("{} {t:.2}", axes[i].id)).collect();
                Ok(Some(PlannedMotion {
                    segs,
                    ends: idx.iter().copied().zip(goals.iter().copied()).collect(),
                    log: format!("move_j {} ({duration:.3} s at {:.0}% speed)", summary.join(", "), scale * 100.0),
                }))
            })
            .await?;
        match planned {
            Some(p) => self.commit(revision, p),
            None => Ok(()),
        }
    }

    /// Queue planned segments as one motion: time order across axes, state
    /// to running, commanded positions to the move's end.
    fn start_planned(&self, g: &mut Inner, mut segs: Vec<Segment>, start_us: u64, ends: &[(usize, f64)]) -> Result<(), ApiError> {
        segs.sort_by_key(|s| (s.start_us, s.axis));
        g.apply(Event::StartMotion).map_err(|e| ApiError::invalid_state(e.to_string()))?;
        g.last_end_us = segs.iter().map(|s| s.end_us()).max().unwrap_or(start_us);
        g.pending.extend(segs);
        for &(i, p) in ends {
            g.axes[i].commanded = p;
        }
        Ok(())
    }

    /// Blended joint path (MoveJPath): through the waypoints in order
    /// without stopping at intermediate ones; corners are rounded.
    pub async fn move_j_path(&self, waypoints: &[Vec<(String, f64)>], speed_scale: Option<f64>) -> Result<(), ApiError> {
        if waypoints.is_empty() || waypoints.iter().any(|w| w.is_empty()) {
            return Err(ApiError::new("bad_request", "every waypoint needs at least one joint target"));
        }
        if waypoints.len() > MAX_WAYPOINTS {
            return Err(ApiError::new("bad_request", format!("at most {MAX_WAYPOINTS} waypoints")));
        }
        // Resolve axis ids; `used` lists every axis any waypoint moves.
        let mut used: Vec<usize> = Vec::new();
        let mut resolved: Vec<Vec<(usize, f64)>> = Vec::with_capacity(waypoints.len());
        for (n, w) in waypoints.iter().enumerate() {
            let mut r = Vec::with_capacity(w.len());
            for (axis, pos) in w {
                let i = self.axis_index(axis)?;
                if r.iter().any(|&(k, _)| k == i) {
                    return Err(ApiError::new("bad_request", format!("waypoint {}: axis {axis} is listed twice", n + 1)));
                }
                if !pos.is_finite() {
                    return Err(ApiError::new("validation_failed", format!("waypoint {}: target of {axis} must be finite", n + 1)));
                }
                if !used.contains(&i) {
                    used.push(i);
                }
                r.push((i, *pos));
            }
            resolved.push(r);
        }
        used.sort();
        let scale = speed_scale.unwrap_or(1.0);
        if !(scale.is_finite() && scale > 0.0 && scale <= 1.0) {
            return Err(ApiError::new("validation_failed", "speed_scale must be in (0, 1]"));
        }
        let axes: Vec<AxisSection> = self.robot.file.axes.clone();
        for (n, r) in resolved.iter().enumerate() {
            for &(i, p) in r {
                let c = &axes[i];
                if p < c.min || p > c.max {
                    return Err(ApiError::new(
                        "validation_failed",
                        format!("waypoint {}: target {p} of {} outside limits [{}, {}]", n + 1, c.id, c.min, c.max),
                    ));
                }
            }
        }
        let (revision, pose, all_homed) = {
            let g = self.inner.lock().unwrap();
            g.sup.check(&Event::StartMotion).map_err(|e| ApiError::invalid_state(e.to_string()))?;
            g.link.clone().filter(|l| l.is_alive()).ok_or_else(|| ApiError::mcu("MCU not connected"))?;
            if let Some(&i) = used.iter().find(|&&i| !g.axes[i].homed) {
                return Err(ApiError::new("not_homed", format!("axis {} must be homed before a joint path", g.axes[i].cfg.id)));
            }
            (g.revision, g.axes.iter().map(|a| a.commanded).collect::<Vec<f64>>(), g.axes.iter().all(|a| a.homed))
        };
        let collision = self.collision.clone();
        let count = waypoints.len();
        let planned = self
            .plan_off_lock(move || {
                // Full joint vectors of the used axes, waypoint by waypoint.
                let mut wps: Vec<Vec<f64>> = vec![used.iter().map(|&i| pose[i]).collect()];
                for r in &resolved {
                    let mut next = wps.last().unwrap().clone();
                    for &(i, p) in r {
                        next[used.iter().position(|&k| k == i).unwrap()] = p;
                    }
                    wps.push(next);
                }
                let limits: Vec<AxisLimits> = used.iter().map(|&i| axis_limits(&axes[i])).collect();
                let profiles = plan_blended(&wps, &limits, scale).map_err(motion_err)?;
                let duration = profiles.first().map_or(0.0, |p| p.duration());
                if duration == 0.0 {
                    return Ok(None);
                }
                // Collision check of the rounded path, sampled finely in time.
                if let (Some(m), true) = (&collision, all_homed) {
                    let (model, order) = m.as_ref();
                    let pose_at = |t: f64| -> Vec<f64> {
                        order.iter().map(|&k| used.iter().position(|&i| i == k).map_or(pose[k], |n| profiles[n].position(t))).collect()
                    };
                    let steps = ((duration / 0.005).ceil() as usize).max(1);
                    let mut prev = pose_at(0.0);
                    for k in 1..=steps {
                        let t = duration * k as f64 / steps as f64;
                        let cur = pose_at(t);
                        if let Err(c) = model.check_path(&prev, &cur) {
                            return Err(ApiError::new("collision", format!("refused: {c} at {:.0} % of the path", 100.0 * t / duration)));
                        }
                        prev = cur;
                    }
                }
                let mut segs = Vec::new();
                for (p, &i) in profiles.iter().zip(&used) {
                    let (lo, hi) = p.range();
                    if hi - lo > 0.0 {
                        segs.extend(to_segments(p, axes[i].channel, axes[i].steps_per_unit, 0, MAX_SEGMENT_US));
                    }
                }
                if segs.is_empty() {
                    return Ok(None);
                }
                Ok(Some(PlannedMotion {
                    segs,
                    ends: used.iter().copied().zip(wps.last().unwrap().iter().copied()).collect(),
                    log: format!("move_j_path {count} waypoints ({duration:.3} s at {:.0}% speed)", scale * 100.0),
                }))
            })
            .await?;
        match planned {
            Some(p) => self.commit(revision, p),
            None => Ok(()),
        }
    }

    /// Cartesian straight-line move of the tool point (MoveL).
    pub async fn move_l(&self, position: [f64; 3], rotation_deg: Option<[f64; 3]>, speed_mm_s: Option<f64>, speed_scale: Option<f64>) -> Result<(), ApiError> {
        let Some(chain) = self.chain.clone() else {
            return Err(ApiError::new("bad_request", "MoveL needs a robot profile whose geometry is a single serial chain"));
        };
        if position.iter().chain(rotation_deg.iter().flatten()).any(|v| !v.is_finite()) {
            return Err(ApiError::new("validation_failed", "position and rotation must be finite"));
        }
        let speed = speed_mm_s.unwrap_or(LINE_SPEED_MM_S);
        if !(speed.is_finite() && speed > 0.0 && speed <= LINE_SPEED_MAX_MM_S) {
            return Err(ApiError::new("validation_failed", format!("speed_mm_s must be in (0, {LINE_SPEED_MAX_MM_S}]")));
        }
        let scale = speed_scale.unwrap_or(1.0);
        if !(scale.is_finite() && scale > 0.0 && scale <= 1.0) {
            return Err(ApiError::new("validation_failed", "speed_scale must be in (0, 1]"));
        }
        let axes: Vec<AxisSection> = self.robot.file.axes.clone();
        let (revision, pose) = {
            let g = self.inner.lock().unwrap();
            g.sup.check(&Event::StartMotion).map_err(|e| ApiError::invalid_state(e.to_string()))?;
            g.link.clone().filter(|l| l.is_alive()).ok_or_else(|| ApiError::mcu("MCU not connected"))?;
            if let Some(a) = g.axes.iter().find(|a| !a.homed) {
                return Err(ApiError::new("not_homed", format!("axis {} must be homed before a Cartesian move", a.cfg.id)));
            }
            (g.revision, g.axes.iter().map(|a| a.commanded).collect::<Vec<f64>>())
        };
        let collision = self.collision.clone();
        let planned = self
            .plan_off_lock(move || {
                let (chain, order) = chain.as_ref();
                let q0: Vec<f64> = order.iter().map(|&k| pose[k]).collect();
                let limits: Vec<AxisLimits> = order.iter().map(|&k| axis_limits(&axes[k])).collect();
                let start = chain.forward(&q0);
                let target = match rotation_deg {
                    Some(r) if chain.controls_orientation() => pose_from(position, r),
                    _ => ToolPose { position, rotation: start.rotation },
                };
                let dist = (0..3).map(|k| (target.position[k] - start.position[k]).powi(2)).sum::<f64>().sqrt();
                let turn = {
                    let r = &target.rotation;
                    let s = &start.rotation;
                    // Angle of r·sᵀ.
                    let tr: f64 = (0..3).map(|i| (0..3).map(|k| r[i][k] * s[i][k]).sum::<f64>()).sum();
                    ((tr - 1.0) / 2.0).clamp(-1.0, 1.0).acos().to_degrees()
                };
                if dist < 1e-3 && turn < 1e-3 {
                    return Ok(None);
                }
                let path =
                    chain.linear_path(&q0, &target, LINE_STEP_MM, LINE_STEP_DEG).map_err(|e| ApiError::new("validation_failed", format!("refused: {e}")))?;
                if let Some(m) = &collision {
                    // Same joint order as the chain (both from the profile geometry).
                    let (model, _) = m.as_ref();
                    for (k, w) in path.windows(2).enumerate() {
                        if let Err(c) = model.check_path(&w[0], &w[1]) {
                            return Err(ApiError::new(
                                "collision",
                                format!("refused: {c} at {:.0} % of the line", 100.0 * (k + 1) as f64 / (path.len() - 1) as f64),
                            ));
                        }
                    }
                }
                let s_speed =
                    [(dist > 1e-9).then(|| speed / dist), (turn > 1e-9).then(|| LINE_TURN_DEG_S / turn)].into_iter().flatten().fold(f64::INFINITY, f64::min);
                let s_accel = [(dist > 1e-9).then(|| LINE_ACCEL_MM_S2 / dist), (turn > 1e-9).then(|| LINE_TURN_DEG_S2 / turn)]
                    .into_iter()
                    .flatten()
                    .fold(f64::INFINITY, f64::min);
                let traj = time_scale(&path, &limits, scale, s_speed, s_accel).map_err(motion_err)?;
                let mut segs = Vec::new();
                for (j, &k) in order.iter().enumerate() {
                    segs.extend(traj.segments(j, axes[k].channel, axes[k].steps_per_unit, 0));
                }
                if segs.is_empty() {
                    return Ok(None);
                }
                Ok(Some(PlannedMotion {
                    segs,
                    ends: order.iter().copied().zip(path.last().unwrap().iter().copied()).collect(),
                    log: format!(
                        "move_l to ({:.1}, {:.1}, {:.1}) mm: {dist:.1} mm, {turn:.1} deg, {:.3} s",
                        position[0],
                        position[1],
                        position[2],
                        traj.duration()
                    ),
                }))
            })
            .await?;
        match planned {
            Some(p) => self.commit(revision, p),
            None => Ok(()),
        }
    }

    pub async fn home(&self, axis: &str) -> Result<(), ApiError> {
        let i = self.axis_index(axis)?;
        let a = &self.robot.file.axes[i];
        if a.homing.is_none() {
            return Err(ApiError::bad_request(format!("axis {} has no homing configuration", a.id)));
        }
        self.start_homing(vec![vec![i]]).await
    }

    /// Run the home-all plan: each group's axes home at the same time.
    pub async fn home_all(&self) -> Result<(), ApiError> {
        let plan: Vec<Vec<usize>> = self.robot.home_all_plan().into_iter().filter(|g| !g.is_empty()).collect();
        if plan.is_empty() {
            return Err(ApiError::bad_request("no axis has a homing configuration"));
        }
        let desc: Vec<String> = plan.iter().map(|g| g.iter().map(|&i| self.robot.file.axes[i].id.as_str()).collect::<Vec<_>>().join("+")).collect();
        self.log("info", format!("home all: {}", desc.join(" -> ")));
        self.start_homing(plan).await
    }

    async fn start_homing(&self, groups: Vec<Vec<usize>>) -> Result<(), ApiError> {
        self.require(&Event::StartHoming)?;
        self.link()?;
        {
            let mut g = self.inner.lock().unwrap();
            g.apply(Event::StartHoming).map_err(|e| ApiError::invalid_state(e.to_string()))?;
            g.homing = Some(HomingRun { groups, next: 0, pending: BTreeSet::new() });
        }
        self.start_homing_group().await
    }

    /// Start every axis of the next group of the current homing run.
    async fn start_homing_group(&self) -> Result<(), ApiError> {
        let (group, link) = {
            let mut g = self.inner.lock().unwrap();
            if g.sup.state() != RobotState::Homing {
                return Ok(()); // stopped or faulted meanwhile
            }
            let link = g.link.clone().filter(|l| l.is_alive()).ok_or_else(|| ApiError::mcu("MCU not connected"))?;
            let Some(run) = g.homing.as_mut() else { return Ok(()) };
            let Some(group) = run.groups.get(run.next).cloned() else { return Ok(()) };
            run.next += 1;
            run.pending = group.iter().copied().collect();
            for &i in &group {
                g.axes[i].homed = false;
            }
            (group, link)
        };
        for &i in &group {
            let a = &self.robot.file.axes[i];
            let h = a.homing.as_ref().expect("plan holds homeable axes only");
            let spu = a.steps_per_unit;
            let params = HomeParams {
                axis: a.channel,
                positive: h.direction == Direction::Positive,
                speed: (h.speed * spu) as f32,
                max_travel_steps: (h.max_travel * spu).ceil() as u32,
                home_steps: (h.position * spu).round() as i32,
            };
            if let Err(e) = link.command(HostMsg::Home(params)).await {
                self.fault(Fault::new(FaultCode::HomingFailed, format!("homing rejected: {e}")).on_axis(a.id.clone()));
                return Err(ApiError::mcu(e));
            }
        }
        let ids: Vec<&str> = group.iter().map(|&i| self.robot.file.axes[i].id.as_str()).collect();
        self.log("info", format!("homing {}", ids.join(", ")));
        Ok(())
    }

    pub async fn stop(&self) -> Result<(), ApiError> {
        let link = self.link()?;
        let ack = {
            let mut g = self.inner.lock().unwrap();
            clear_motion(&mut g);
            g.homing = None;
            if g.sup.state() == RobotState::Homing {
                // A stopped homing cycle leaves the axis un-homed; the
                // supervisor returns to ready once the MCU is idle.
                let _ = g.apply(Event::HomingComplete);
                let _ = g.apply(Event::StartMotion);
            }
            link.submit(HostMsg::Stop).map_err(ApiError::mcu)?
        };
        ack.wait().await.map_err(ApiError::mcu)?;
        self.log("info", "stop requested");
        Ok(())
    }

    /// Software emergency stop. Works in every state, even without a link.
    /// This complements — never replaces — a hardware E-stop circuit.
    pub async fn estop(&self) -> Result<(), ApiError> {
        let link = {
            let mut g = self.inner.lock().unwrap();
            clear_motion(&mut g);
            for a in g.axes.iter_mut() {
                a.homed = false;
                a.enabled = false;
            }
            let _ = g.apply(Event::EmergencyStop);
            g.link.clone()
        };
        if let Some(l) = link {
            let _ = l.send(HostMsg::EmergencyStop);
        }
        self.log("error", "EMERGENCY STOP");
        Ok(())
    }

    pub async fn reset_fault(&self) -> Result<(), ApiError> {
        let link = self.inner.lock().unwrap().link.clone().filter(|l| l.is_alive());
        self.require(&Event::ResetFault { connected: link.is_some() })?;
        if let Some(l) = &link {
            l.command(HostMsg::ResetFault).await.map_err(|e| match e {
                McuError::Nack(NackReason::InvalidState) => ApiError::invalid_state(
                    "MCU refused to clear the fault: the hazard is still present \
                     (endstop still pressed, safety interlock open, or a drive still active)",
                ),
                e => ApiError::mcu(e),
            })?;
            // Re-install un-homed limits around wherever the axes are now.
            let status = l.latest_status();
            for i in 0..self.robot.file.axes.len() {
                let ch = self.robot.file.axes[i].channel as usize;
                let pos = status.as_ref().and_then(|s| s.axes.get(ch)).map(|s| s.position_steps).unwrap_or(0);
                l.command(HostMsg::ConfigureAxis(self.mcu_axis_config(i, false, pos))).await.map_err(ApiError::mcu)?;
            }
        }
        let mut g = self.inner.lock().unwrap();
        if let Some(l) = &link {
            resync_from_mcu(&mut g, l);
        }
        g.apply(Event::ResetFault { connected: link.is_some() }).map_err(|e| ApiError::invalid_state(e.to_string()))?;
        drop(g);
        self.log("info", "fault reset — drives remain disabled, axes must be re-homed");
        Ok(())
    }

    // ------------------------------------------------------------------
    // Fault injection (only with --enable-fault-injection)
    // ------------------------------------------------------------------

    fn require_injection(&self) -> Result<(), ApiError> {
        if self.opts.fault_injection {
            Ok(())
        } else {
            Err(ApiError::bad_request("fault injection disabled (start engine with --enable-fault-injection)"))
        }
    }

    /// Simulate unplugging the virtual MCU.
    pub fn debug_disconnect_mcu(&self) -> Result<(), ApiError> {
        self.require_injection()?;
        let task = self.inner.lock().unwrap().virtual_task.take();
        match task {
            Some(t) => {
                t.abort();
                self.log("warn", "fault injection: virtual MCU unplugged");
                Ok(())
            }
            None => Err(ApiError::bad_request("only the in-process virtual MCU can be unplugged")),
        }
    }

    /// Simulate a plugin crash.
    pub fn debug_fail_plugin(&self, id: &str) -> Result<(), ApiError> {
        self.require_injection()?;
        let changed = self.plugins.lock().unwrap().mark_failed(id, "fault injection: simulated crash");
        if changed.is_empty() {
            return Err(ApiError::bad_request(format!("plugin '{id}' not active")));
        }
        self.log("error", format!("plugin failure: {}", changed.join(", ")));
        if changed.iter().any(|id| self.robot.required_plugins.contains(id)) {
            self.fault(Fault::new(FaultCode::PluginFailed, format!("required plugin failed: {}", changed.join(", "))));
        }
        Ok(())
    }
}

/// Is the connected firmware the one this configuration was built for?
/// Compares only the identity the MCU reports (board id, capabilities,
/// channel count), never architecture details, so the host stays
/// independent of the MCU family.
fn check_identity(mcu: &ResolvedMcu, axes: &[AxisSection], id: &Identity) -> Result<(), String> {
    if id.board.as_str() != mcu.board.id {
        return Err(format!(
            "MCU '{}' reports board '{}' (target {}), but the configuration expects '{}'; flash the matching firmware",
            mcu.name, id.board, id.target, mcu.board.id
        ));
    }
    let caps: Vec<&str> = capability_names(id.capabilities).collect();
    for need in &mcu.required_firmware_capabilities {
        if !caps.contains(need) {
            return Err(format!("firmware lacks required capability '{need}' (has: {})", caps.join(", ")));
        }
    }
    if let Some(a) = axes.iter().filter(|a| a.mcu == mcu.name).find(|a| a.channel >= id.axes) {
        return Err(format!("axis '{}' uses channel {} but firmware reports {} channels", a.id, a.channel, id.axes));
    }
    Ok(())
}

/// Wait for an MCU status report that arrives **after** this call and
/// satisfies `ok`. Telemetry received earlier never counts; a reported MCU
/// fault, link loss or the timeout is an error.
async fn fresh_status(link: &McuLink, timeout: Duration, ok: impl Fn(&Status) -> bool) -> Result<Status, String> {
    let mut w = link.status_watch();
    w.borrow_and_update();
    let _ = link.send(HostMsg::QueryStatus);
    let deadline = tokio::time::Instant::now() + timeout;
    let mut seen = false;
    loop {
        match tokio::time::timeout_at(deadline, w.changed()).await {
            Err(_) if seen => return Err(format!("MCU telemetry did not confirm within {timeout:?}")),
            Err(_) => return Err(format!("no MCU telemetry within {timeout:?}")),
            Ok(Err(_)) => return Err("MCU link closed".into()),
            Ok(Ok(())) => {
                let Some(s) = w.borrow_and_update().clone() else { continue };
                if let Some(code) = s.fault {
                    return Err(format!("MCU reported fault {code:?}"));
                }
                if ok(&s) {
                    return Ok(s);
                }
                seen = true;
            }
        }
    }
}

impl Inner {
    /// Supervisor transition that also invalidates in-flight motion plans.
    fn apply(&mut self, event: Event) -> Result<RobotState, motioncore_safety::TransitionError> {
        let r = self.sup.apply(event);
        if r.is_ok() {
            self.revision += 1;
        }
        r
    }
}

fn clear_motion(g: &mut Inner) {
    g.pending.clear();
    g.in_flight.clear();
    g.last_end_us = 0;
    g.motion_gen += 1;
    g.revision += 1;
}

fn resync_from_mcu(g: &mut Inner, link: &McuLink) {
    g.revision += 1;
    if let Some(st) = link.latest_status() {
        for a in g.axes.iter_mut() {
            if let Some(s) = st.axes.get(a.cfg.channel as usize) {
                a.commanded = s.position_steps as f64 / a.cfg.steps_per_unit;
            }
        }
    }
}

fn motion_err(e: MotionError) -> ApiError {
    ApiError::new("validation_failed", e.to_string())
}

fn map_mcu_fault(c: McuFault) -> (FaultCode, &'static str) {
    match c {
        McuFault::CommunicationLost => (FaultCode::CommunicationLost, "communication watchdog expired"),
        McuFault::QueueUnderrun => (FaultCode::QueueUnderrun, "motion queue underrun"),
        McuFault::SegmentLate => (FaultCode::SegmentLate, "motion segment arrived late"),
        McuFault::LimitHit => (FaultCode::LimitHit, "endstop triggered during motion"),
        McuFault::SoftLimit => (FaultCode::SoftLimit, "soft limit reached"),
        McuFault::HomingFailed => (FaultCode::HomingFailed, "endstop not found within max travel"),
        McuFault::EmergencyStop => (FaultCode::EmergencyStop, "emergency stop"),
        McuFault::StepRateExceeded => (FaultCode::McuFault, "step rate exceeded"),
    }
}

fn state_pb(s: RobotState) -> pb::RobotState {
    match s {
        RobotState::Disconnected => pb::RobotState::Disconnected,
        RobotState::Initializing => pb::RobotState::Initializing,
        RobotState::Disabled => pb::RobotState::Disabled,
        RobotState::Ready => pb::RobotState::Ready,
        RobotState::Homing => pb::RobotState::Homing,
        RobotState::Running => pb::RobotState::Running,
        RobotState::Faulted => pb::RobotState::Faulted,
        RobotState::EmergencyStopped => pb::RobotState::EmergencyStopped,
    }
}

fn fault_pb(f: &Fault) -> pb::Fault {
    let code = match f.code {
        FaultCode::CommunicationLost => pb::FaultCode::CommunicationLost,
        FaultCode::QueueUnderrun => pb::FaultCode::QueueUnderrun,
        FaultCode::SegmentLate => pb::FaultCode::SegmentLate,
        FaultCode::LimitHit => pb::FaultCode::LimitHit,
        FaultCode::SoftLimit => pb::FaultCode::SoftLimit,
        FaultCode::HomingFailed => pb::FaultCode::HomingFailed,
        FaultCode::McuFault => pb::FaultCode::McuFault,
        FaultCode::PluginFailed => pb::FaultCode::PluginFailed,
        FaultCode::EmergencyStop => pb::FaultCode::EmergencyStop,
    };
    pb::Fault { code: code as i32, message: f.message.clone(), axis: f.axis.clone().unwrap_or_default() }
}

#[cfg(test)]
mod identity_tests {
    use super::*;
    use motioncore_protocol::caps;

    fn robot(text: &str) -> ResolvedRobot {
        motioncore_device_graph::parse(text).unwrap()
    }

    fn id(board: &str, target: &str, capabilities: u32, axes: u8) -> Identity {
        Identity {
            protocol_version: motioncore_protocol::PROTOCOL_VERSION,
            firmware_version: (0, 2, 0),
            target: target.try_into().unwrap(),
            board: board.try_into().unwrap(),
            build_hash: 0,
            capabilities,
            axes,
            queue_capacity: 32,
        }
    }

    const ALL: u32 = caps::STEPPER_STEP_DIR | caps::ENDSTOP | caps::WATCHDOG;

    fn profile(name: &str) -> String {
        let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../robot-profiles");
        std::fs::read_to_string(dir.join(name)).unwrap()
    }

    #[test]
    fn matching_firmware_is_accepted() {
        let r = robot(&profile("six-axis-rp2040-pico.toml"));
        let m = &r.mcus["main"];
        assert_eq!(check_identity(m, &r.file.axes, &id("rp2040-pico", "rp2040", ALL, 6)), Ok(()));
        let r = robot(&profile("single-axis-stm32f411.toml"));
        let m = &r.mcus["main"];
        assert_eq!(check_identity(m, &r.file.axes, &id("stm32f411-blackpill", "stm32f4", ALL, 1)), Ok(()));
    }

    #[test]
    fn wrong_board_is_rejected() {
        let r = robot(&profile("single-axis-stm32f411.toml"));
        let m = &r.mcus["main"];
        let e = check_identity(m, &r.file.axes, &id("rp2040-pico", "rp2040", ALL, 6)).unwrap_err();
        assert!(e.contains("reports board 'rp2040-pico'") && e.contains("expects 'stm32f411-blackpill'"), "{e}");
    }

    #[test]
    fn missing_capability_or_channels_are_rejected() {
        let r = robot(&profile("six-axis-rp2040-pico.toml"));
        let m = &r.mcus["main"];
        let e = check_identity(m, &r.file.axes, &id("rp2040-pico", "rp2040", caps::STEPPER_STEP_DIR | caps::WATCHDOG, 6)).unwrap_err();
        assert!(e.contains("endstop"), "{e}");
        let e = check_identity(m, &r.file.axes, &id("rp2040-pico", "rp2040", ALL, 4)).unwrap_err();
        assert!(e.contains("firmware reports 4 channels"), "{e}");
    }
}
