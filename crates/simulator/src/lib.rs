//! Virtual MCU: the shared firmware runtime ([`motioncore_firmware_core`])
//! driven by a simulated clock and a simulated physical world.
//!
//! It speaks the exact same framed byte protocol as real hardware, so the
//! host cannot tell the difference — any stream (in-process duplex pipe, TCP
//! socket) can carry it.

use std::time::{Duration, Instant};

use motioncore_firmware_core::{BoardInfo, Hal, McuCore};
use motioncore_protocol::{caps, encode_frame, FrameDecoder, HostFrame, MAX_FRAME};
use tokio::io::{AsyncRead, AsyncReadExt, AsyncWrite, AsyncWriteExt};

pub const AXES: usize = 6;
pub const QUEUE: usize = 32;
/// Core tick period. 1 kHz is plenty for simulation: the core emits every
/// step that became due since the previous tick.
pub const TICK: Duration = Duration::from_millis(1);
/// Status report period.
pub const STATUS_PERIOD_US: u64 = 50_000;

/// Simulated endstop: triggers when physical position is at/beyond `at_steps`
/// on the given side.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SimEndstop {
    pub at_steps: i64,
    pub positive_side: bool,
}

#[derive(Debug, Clone, Default)]
pub struct SimAxis {
    /// Physical position at power-on (steps).
    pub start_steps: i64,
    pub endstop: Option<SimEndstop>,
}

#[derive(Debug, Clone, Default)]
pub struct VirtualOptions {
    pub axes: Vec<SimAxis>,
}

impl VirtualOptions {
    /// Build a simulated world from a robot configuration: each homing switch
    /// is placed exactly at the configured home position, so that after
    /// homing the logical and physical frames coincide.
    pub fn from_robot(robot: &motioncore_device_graph::ResolvedRobot, mcu: &str) -> Self {
        use motioncore_device_graph::Direction;
        let mut axes = vec![SimAxis::default(); AXES];
        for a in robot.file.axes.iter().filter(|a| a.mcu == mcu) {
            if let (Some(h), Some(_)) = (&a.homing, &a.endstop) {
                if let Some(slot) = axes.get_mut(a.channel as usize) {
                    slot.endstop =
                        Some(SimEndstop { at_steps: (h.position * a.steps_per_unit).round() as i64, positive_side: h.direction == Direction::Positive });
                }
            }
        }
        Self { axes }
    }
}

/// Simulated physical world.
pub struct SimHal {
    pub physical: [i64; AXES],
    pub enabled: [bool; AXES],
    endstops: [Option<SimEndstop>; AXES],
    pub steps_while_disabled: u64,
}

impl SimHal {
    pub fn new(opts: &VirtualOptions) -> Self {
        let mut physical = [0; AXES];
        let mut endstops = [None; AXES];
        for (i, a) in opts.axes.iter().take(AXES).enumerate() {
            physical[i] = a.start_steps;
            endstops[i] = a.endstop;
        }
        Self { physical, enabled: [false; AXES], endstops, steps_while_disabled: 0 }
    }
}

impl Hal for SimHal {
    fn step(&mut self, axis: u8, forward: bool) {
        let i = axis as usize;
        if !self.enabled[i] {
            // A real driver would ignore the pulse; count it as a defect.
            self.steps_while_disabled += 1;
            return;
        }
        self.physical[i] += if forward { 1 } else { -1 };
    }

    fn set_enable(&mut self, axis: u8, enabled: bool) {
        if let Some(e) = self.enabled.get_mut(axis as usize) {
            *e = enabled;
        }
    }

    fn endstop_raw(&mut self, axis: u8) -> bool {
        let i = axis as usize;
        match self.endstops.get(i).copied().flatten() {
            Some(e) if e.positive_side => self.physical[i] >= e.at_steps,
            Some(e) => self.physical[i] <= e.at_steps,
            None => false,
        }
    }
}

pub fn board_info() -> BoardInfo {
    BoardInfo {
        target: "virtual",
        board: "virtual",
        firmware_version: (parse_u8(env!("CARGO_PKG_VERSION_MAJOR")), parse_u8(env!("CARGO_PKG_VERSION_MINOR")), parse_u8(env!("CARGO_PKG_VERSION_PATCH"))),
        build_hash: 0x5157_0001,
        capabilities: caps::STEPPER_STEP_DIR | caps::ENDSTOP | caps::WATCHDOG,
        // Host OS timers are coarse (≈15 ms on Windows), unlike a hardware tick.
        late_tolerance_us: 40_000,
    }
}

/// Run a virtual MCU on `io` until the stream closes.
pub async fn run<S>(io: S, opts: VirtualOptions) -> std::io::Result<()>
where
    S: AsyncRead + AsyncWrite + Unpin,
{
    let (mut rd, mut wr) = tokio::io::split(io);
    let mut core: McuCore<AXES, QUEUE> = McuCore::new(board_info());
    let mut hal = SimHal::new(&opts);
    let epoch = Instant::now();
    // Start the MCU clock at a non-zero arbitrary value, like real hardware
    // that has been running for a while, so clock-offset bugs show up.
    let base_us: u64 = 7_000_000;
    let now_us = || base_us + epoch.elapsed().as_micros() as u64;

    let mut decoder = FrameDecoder::new();
    let mut rx = [0u8; 256];
    let mut tx = Vec::with_capacity(1024);
    let mut ticker = tokio::time::interval(TICK);
    ticker.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
    let mut next_status = now_us();

    loop {
        tokio::select! {
            r = rd.read(&mut rx) => {
                let n = r?;
                if n == 0 {
                    return Ok(());
                }
                let t = now_us();
                for &b in &rx[..n] {
                    if let Some(Ok(frame)) = decoder.push::<HostFrame>(b) {
                        core.handle(frame, t, &mut hal);
                    }
                    // Corrupted frames are dropped; the host times out and
                    // decides (configuration is idempotent, motion is not).
                }
            }
            _ = ticker.tick() => {
                let t = now_us();
                core.tick(t, &mut hal);
                if t >= next_status {
                    core.report_status(t, &mut hal);
                    next_status = t + STATUS_PERIOD_US;
                }
            }
        }
        tx.clear();
        let mut buf = [0u8; MAX_FRAME];
        while let Some(frame) = core.poll_outgoing() {
            if let Ok(n) = encode_frame(&frame, &mut buf) {
                tx.extend_from_slice(&buf[..n]);
            }
        }
        if !tx.is_empty() {
            wr.write_all(&tx).await?;
            wr.flush().await?;
        }
    }
}

/// Serve a virtual MCU on TCP, one host at a time (like a serial port). Each
/// new connection is a fresh MCU boot; killing the process is a realistic
/// "unplug" test.
pub async fn serve_tcp(listen: &str, opts: VirtualOptions) -> std::io::Result<()> {
    let listener = tokio::net::TcpListener::bind(listen).await?;
    eprintln!("virtual MCU listening on {listen}");
    loop {
        let (sock, peer) = listener.accept().await?;
        sock.set_nodelay(true)?;
        eprintln!("host connected from {peer} (fresh MCU state)");
        match run(sock, opts.clone()).await {
            Ok(()) => eprintln!("host disconnected"),
            Err(e) => eprintln!("connection ended: {e}"),
        }
    }
}

fn parse_u8(s: &str) -> u8 {
    s.parse().unwrap_or(0)
}
