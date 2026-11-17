//! The MotionCore engine: single authority for robot state.
//!
//! All state lives in [`Inner`] behind a `std::sync::Mutex` that is never held
//! across an `.await`. Commands follow the motion pipeline of the spec (§7.1):
//! check state/ownership → resolve geometry → plan → validate → convert to
//! timestamped MCU segments → queue → observe feedback.

use std::collections::{BTreeSet, VecDeque};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use motioncore_device_graph::{AxisSection, Direction, JointKind, ResolvedMcu, ResolvedRobot, Transport, Unit};
use motioncore_kinematics::collision::{CollisionModel, JointSpec};
use motioncore_mcu_manager::{McuError, McuEvent, McuLink};
use motioncore_motion::{plan_axis_move, to_segments, AxisLimits, MotionError};
use motioncore_engine_api::v1 as pb;
use motioncore_plugin_runtime::{PluginKind, PluginState, Registry};
use motioncore_protocol::{
    capability_names, AxisConfig, FaultCode as McuFault, HomeParams, HostMsg, Identity, NackReason, Segment, Status,
};
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

pub struct Options {
    pub fault_injection: bool,
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
    /// End times (MCU clock) of segments sent and possibly still queued.
    in_flight: VecDeque<u64>,
    last_end_us: u64,
    /// Bumped whenever planned motion is discarded (stop, fault, e-stop).
    motion_gen: u64,
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
    collision: Option<(CollisionModel, Vec<usize>)>,
}

/// Build the collision model from the profile geometry, parents first.
fn collision_model(robot: &ResolvedRobot) -> Option<(CollisionModel, Vec<usize>)> {
    if !robot.has_collision_model() {
        return None;
    }
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
    Some((CollisionModel::new(joints, COLLISION_MARGIN_MM), order))
}

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
        let axes = robot
            .file
            .axes
            .iter()
            .map(|a| AxisRt { cfg: a.clone(), homed: false, enabled: false, commanded: 0.0 })
            .collect();
        let collision = collision_model(&robot);
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
                telemetry_seq: 0,
                homing: None,
            }),
            collision,
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
                        let _ = g.sup.apply(Event::ConnectionLost);
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
            g.sup.apply(Event::Connect)?;
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
        g.sup.apply(Event::Initialized)?;
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
                        let _ = g.sup.apply(Event::HomingComplete);
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
                    let _ = g.sup.apply(Event::ConnectionLost);
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
        let _ = g.sup.apply(Event::Fault(f));
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
        self.robot
            .file
            .axes
            .iter()
            .position(|a| a.id == id)
            .ok_or_else(|| ApiError::new("unknown_axis", format!("unknown axis '{id}'")))
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
                while g.in_flight.front().is_some_and(|&end| end < now) {
                    g.in_flight.pop_front();
                }
                let budget = cap.saturating_sub(4).saturating_sub(g.in_flight.len());
                let mut acks = Vec::new();
                for _ in 0..budget {
                    let Some(s) = g.pending.pop_front() else { break };
                    match link.submit(HostMsg::QueueSegment(s)) {
                        Ok(a) => {
                            g.in_flight.push_back(s.end_us());
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
        let _ = g.sup.apply(Event::MotionComplete);
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
        let axes = g
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
        pb::Telemetry {
            seq: g.telemetry_seq,
            time_ms: self.time_ms(),
            state: state_pb(g.sup.state()) as i32,
            fault: g.sup.fault().map(fault_pb),
            mcu: Some(mcu),
            axes,
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
                .map(|(name, m)| pb::McuConfig {
                    name: name.clone(),
                    board: m.board.clone(),
                    transport: m.transport.clone(),
                    watchdog_ms: m.watchdog_ms,
                })
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
        g.sup.apply(Event::Enable).map_err(|e| ApiError::invalid_state(e.to_string()))?;
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
        let _ = g.sup.apply(Event::Disable);
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
        let start = self.inner.lock().unwrap().axes[i].commanded;
        self.start_move(i, start + delta, velocity, false).await
    }

    pub async fn move_to(&self, axis: &str, position: f64, velocity: Option<f64>) -> Result<(), ApiError> {
        let i = self.axis_index(axis)?;
        self.start_move(i, position, velocity, true).await
    }

    async fn start_move(&self, i: usize, target: f64, velocity: Option<f64>, absolute: bool) -> Result<(), ApiError> {
        let mut g = self.inner.lock().unwrap();
        g.sup.check(&Event::StartMotion).map_err(|e| ApiError::invalid_state(e.to_string()))?;
        let link = g.link.clone().filter(|l| l.is_alive()).ok_or_else(|| ApiError::mcu("MCU not connected"))?;
        let a = &g.axes[i];
        let cfg = &a.cfg;
        if absolute && !a.homed {
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
        // Self-collision check: only meaningful once every position is known.
        if let Some((model, order)) = &self.collision {
            if g.axes.iter().all(|x| x.homed) {
                let q: Vec<f64> = order.iter().map(|&k| g.axes[k].commanded).collect();
                let mi = order.iter().position(|&k| k == i).expect("every axis is in the model");
                let to = if absolute { target } else { target.clamp(cfg.min, cfg.max) };
                if let Err(c) = model.check_sweep(&q, mi, to) {
                    let at = c.at[mi];
                    return Err(ApiError::new(
                        "collision",
                        format!("refused: {c} when {} reaches {at:.1} {}", cfg.id, if a.cfg.kind == JointKind::Prismatic { "mm" } else { "deg" }),
                    ));
                }
            }
        }
        let profile = plan_axis_move(a.commanded, target, velocity, &limits).map_err(motion_err)?;
        if profile.duration() == 0.0 {
            return Ok(());
        }
        let start_us = link.mcu_now_us() + LEAD_US;
        let segs = to_segments(&profile, cfg.channel, cfg.steps_per_unit, start_us, MAX_SEGMENT_US);
        let id = cfg.id.clone();
        g.sup.apply(Event::StartMotion).map_err(|e| ApiError::invalid_state(e.to_string()))?;
        g.last_end_us = segs.last().map(|s| s.end_us()).unwrap_or(start_us);
        g.pending.extend(segs);
        g.axes[i].commanded = target;
        drop(g);
        self.log("debug", format!("move {id} -> {target:.3} ({:.3} s)", profile.duration()));
        Ok(())
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
        let desc: Vec<String> = plan
            .iter()
            .map(|g| g.iter().map(|&i| self.robot.file.axes[i].id.as_str()).collect::<Vec<_>>().join("+"))
            .collect();
        self.log("info", format!("home all: {}", desc.join(" -> ")));
        self.start_homing(plan).await
    }

    async fn start_homing(&self, groups: Vec<Vec<usize>>) -> Result<(), ApiError> {
        self.require(&Event::StartHoming)?;
        self.link()?;
        {
            let mut g = self.inner.lock().unwrap();
            g.sup.apply(Event::StartHoming).map_err(|e| ApiError::invalid_state(e.to_string()))?;
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
                let _ = g.sup.apply(Event::HomingComplete);
                let _ = g.sup.apply(Event::StartMotion);
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
            let _ = g.sup.apply(Event::EmergencyStop);
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
        g.sup.apply(Event::ResetFault { connected: link.is_some() }).map_err(|e| ApiError::invalid_state(e.to_string()))?;
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

fn clear_motion(g: &mut Inner) {
    g.pending.clear();
    g.in_flight.clear();
    g.last_end_us = 0;
    g.motion_gen += 1;
}

fn resync_from_mcu(g: &mut Inner, link: &McuLink) {
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
