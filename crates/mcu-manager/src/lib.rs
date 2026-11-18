//! Host side of the MCU link.
//!
//! [`McuLink`] owns one framed byte stream to one MCU and provides:
//! * request/ack correlation by sequence number with timeouts,
//! * the identification handshake with protocol-version check,
//! * NTP-style clock synchronisation (best-of-N by round-trip time),
//! * periodic heartbeats feeding the MCU's communication watchdog,
//! * link-loss detection (stream closed or MCU silent), and
//! * link statistics for diagnostics.
//!
//! Non-idempotent commands (motion segments) are never retransmitted; a
//! timeout surfaces as an error and the engine decides to stop safely.
//!
//! Outbound traffic uses two **bounded** lanes. The priority lane carries only
//! messages that are safe to send ahead of queued traffic (emergency stop,
//! heartbeat, status query, identify, clock sync); everything else keeps its
//! submission order on the normal lane. A full lane is reported as
//! [`McuError::Backpressure`] instead of growing without limit, and each
//! transport write is capped at [`MAX_BATCH_BYTES`] so priority frames never
//! wait behind a large batch. The physical E-stop must stay independent of
//! this software path.

use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use motioncore_protocol::{
    encode_frame, FaultCode, FrameDecoder, FrameError, HostFrame, HostMsg, Identity, McuFrame, McuMsg, NackReason, Status, MAX_FRAME, PROTOCOL_VERSION,
};
use tokio::io::{AsyncRead, AsyncReadExt, AsyncWrite, AsyncWriteExt};
use tokio::sync::{mpsc, oneshot, watch};

pub const ACK_TIMEOUT: Duration = Duration::from_millis(500);
pub const HEARTBEAT_PERIOD: Duration = Duration::from_millis(100);
/// The MCU reports status every 50 ms; silence for this long means the link is dead.
pub const SILENCE_TIMEOUT: Duration = Duration::from_millis(1000);
/// Normal-lane capacity in frames.
pub const OUTBOUND_CAPACITY: usize = 256;
/// Priority-lane capacity in frames.
pub const PRIORITY_CAPACITY: usize = 32;
/// Upper bound on the bytes written to the transport in one write, so a
/// priority frame waits for at most one such write.
pub const MAX_BATCH_BYTES: usize = 256;

#[derive(Debug, thiserror::Error, Clone, PartialEq)]
pub enum McuError {
    #[error("MCU rejected command: {0:?}")]
    Nack(NackReason),
    #[error("MCU did not answer within {0:?}")]
    Timeout(Duration),
    #[error("MCU link is down")]
    LinkDown,
    #[error("outbound MCU queue is full")]
    Backpressure,
    #[error("protocol version mismatch: host {host}, mcu {mcu}")]
    Version { host: u16, mcu: u16 },
}

/// Asynchronous notifications from the MCU.
#[derive(Debug, Clone, PartialEq)]
pub enum McuEvent {
    Fault { code: FaultCode, axis: Option<u8> },
    HomingDone { axis: u8, position_steps: i32 },
    LinkLost { reason: String },
}

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct ClockSync {
    /// `mcu_time_us = host_time_us + offset_us`
    pub offset_us: i64,
    pub rtt_us: u32,
}

#[derive(Debug, Clone, Default)]
pub struct LinkStats {
    pub tx_frames: u64,
    pub rx_frames: u64,
    pub crc_errors: u64,
    pub decode_errors: u64,
    pub last_rx_ms_ago: u64,
    pub clock: ClockSync,
}

enum Waiter {
    Ack(oneshot::Sender<Result<(), NackReason>>),
}

struct Shared {
    epoch: Instant,
    pending: Mutex<HashMap<u32, Waiter>>,
    identity: Mutex<Option<oneshot::Sender<Identity>>>,
    clock_waiters: Mutex<HashMap<u64, oneshot::Sender<u64>>>,
    clock: Mutex<ClockSync>,
    seq: AtomicU32,
    tx_frames: AtomicU64,
    rx_frames: AtomicU64,
    crc_errors: AtomicU64,
    decode_errors: AtomicU64,
    last_rx_us: AtomicU64,
    alive: AtomicBool,
}

impl Shared {
    fn host_us(&self) -> u64 {
        self.epoch.elapsed().as_micros() as u64
    }
}

/// Handle to a connected MCU. Cheap to clone.
#[derive(Clone)]
pub struct McuLink {
    shared: Arc<Shared>,
    out: mpsc::Sender<HostFrame>,
    prio: mpsc::Sender<HostFrame>,
    status: watch::Receiver<Option<Status>>,
}

/// Messages that may overtake queued normal traffic. Each is either
/// idempotent or makes the MCU *safer* (an emergency stop that overtakes
/// queued segments only causes them to be rejected).
fn is_priority(msg: &HostMsg) -> bool {
    matches!(msg, HostMsg::EmergencyStop | HostMsg::Heartbeat | HostMsg::QueryStatus | HostMsg::Identify | HostMsg::ClockSync { .. })
}

impl McuLink {
    /// Start the link tasks on `io`. Returns the handle and the event stream.
    pub fn start<S>(io: S) -> (Self, mpsc::UnboundedReceiver<McuEvent>)
    where
        S: AsyncRead + AsyncWrite + Send + 'static,
    {
        let shared = Arc::new(Shared {
            epoch: Instant::now(),
            pending: Mutex::new(HashMap::new()),
            identity: Mutex::new(None),
            clock_waiters: Mutex::new(HashMap::new()),
            clock: Mutex::new(ClockSync::default()),
            seq: AtomicU32::new(0),
            tx_frames: AtomicU64::new(0),
            rx_frames: AtomicU64::new(0),
            crc_errors: AtomicU64::new(0),
            decode_errors: AtomicU64::new(0),
            last_rx_us: AtomicU64::new(0),
            alive: AtomicBool::new(true),
        });
        let (rd, wr) = tokio::io::split(io);
        let (out_tx, out_rx) = mpsc::channel(OUTBOUND_CAPACITY);
        let (prio_tx, prio_rx) = mpsc::channel(PRIORITY_CAPACITY);
        let (ev_tx, ev_rx) = mpsc::unbounded_channel();
        let (st_tx, st_rx) = watch::channel(None);

        tokio::spawn(writer(wr, prio_rx, out_rx, shared.clone(), ev_tx.clone()));
        tokio::spawn(reader(rd, shared.clone(), ev_tx.clone(), st_tx));
        tokio::spawn(heartbeat(prio_tx.clone(), shared.clone(), ev_tx));

        (Self { shared, out: out_tx, prio: prio_tx, status: st_rx }, ev_rx)
    }

    pub fn is_alive(&self) -> bool {
        self.shared.alive.load(Ordering::SeqCst)
    }

    /// Host monotonic clock in µs (link epoch).
    pub fn host_us(&self) -> u64 {
        self.shared.host_us()
    }

    pub fn clock(&self) -> ClockSync {
        *self.shared.clock.lock().unwrap()
    }

    /// Current MCU time estimate.
    pub fn mcu_now_us(&self) -> u64 {
        (self.host_us() as i64 + self.clock().offset_us).max(0) as u64
    }

    pub fn latest_status(&self) -> Option<Status> {
        self.status.borrow().clone()
    }

    pub fn status_watch(&self) -> watch::Receiver<Option<Status>> {
        self.status.clone()
    }

    pub fn stats(&self) -> LinkStats {
        let s = &self.shared;
        let last = s.last_rx_us.load(Ordering::Relaxed);
        LinkStats {
            tx_frames: s.tx_frames.load(Ordering::Relaxed),
            rx_frames: s.rx_frames.load(Ordering::Relaxed),
            crc_errors: s.crc_errors.load(Ordering::Relaxed),
            decode_errors: s.decode_errors.load(Ordering::Relaxed),
            last_rx_ms_ago: s.host_us().saturating_sub(last) / 1000,
            clock: self.clock(),
        }
    }

    fn next_frame(&self, msg: HostMsg) -> HostFrame {
        HostFrame { seq: self.shared.seq.fetch_add(1, Ordering::Relaxed).wrapping_add(1), msg }
    }

    fn enqueue(&self, frame: HostFrame) -> Result<(), McuError> {
        let lane = if is_priority(&frame.msg) { &self.prio } else { &self.out };
        lane.try_send(frame).map_err(|e| match e {
            mpsc::error::TrySendError::Full(_) => McuError::Backpressure,
            mpsc::error::TrySendError::Closed(_) => McuError::LinkDown,
        })
    }

    /// Fire-and-forget (heartbeats, best-effort e-stop).
    pub fn send(&self, msg: HostMsg) -> Result<(), McuError> {
        if !self.is_alive() {
            return Err(McuError::LinkDown);
        }
        self.enqueue(self.next_frame(msg))
    }

    /// Enqueue a command **synchronously** and return a handle to await its
    /// Ack/Nack. Normal-lane frames are transmitted in submission order, so
    /// callers can order commands under their own lock (e.g. "no segment
    /// after Stop"). Fails with [`McuError::Backpressure`] when the lane is
    /// full; nothing is sent in that case.
    pub fn submit(&self, msg: HostMsg) -> Result<PendingAck, McuError> {
        if !self.is_alive() {
            return Err(McuError::LinkDown);
        }
        let frame = self.next_frame(msg);
        let (tx, rx) = oneshot::channel();
        let seq = frame.seq;
        self.shared.pending.lock().unwrap().insert(seq, Waiter::Ack(tx));
        if let Err(e) = self.enqueue(frame) {
            self.shared.pending.lock().unwrap().remove(&seq);
            return Err(e);
        }
        Ok(PendingAck { rx, seq, shared: self.shared.clone() })
    }

    /// Send a command and wait for its Ack/Nack.
    pub async fn command(&self, msg: HostMsg) -> Result<(), McuError> {
        self.submit(msg)?.wait().await
    }

    /// Identification handshake (idempotent, retried).
    pub async fn identify(&self) -> Result<Identity, McuError> {
        for _ in 0..3 {
            let (tx, rx) = oneshot::channel();
            *self.shared.identity.lock().unwrap() = Some(tx);
            self.send(HostMsg::Identify)?;
            if let Ok(Ok(id)) = tokio::time::timeout(ACK_TIMEOUT, rx).await {
                if id.protocol_version != PROTOCOL_VERSION {
                    return Err(McuError::Version { host: PROTOCOL_VERSION, mcu: id.protocol_version });
                }
                return Ok(id);
            }
        }
        Err(McuError::Timeout(ACK_TIMEOUT * 3))
    }

    /// Clock synchronisation: take `samples` probes and keep the one with the
    /// smallest round-trip time (least queueing noise).
    pub async fn sync_clock(&self, samples: usize) -> Result<ClockSync, McuError> {
        let mut best: Option<ClockSync> = None;
        for _ in 0..samples {
            let sent = self.host_us();
            let (tx, rx) = oneshot::channel();
            self.shared.clock_waiters.lock().unwrap().insert(sent, tx);
            self.send(HostMsg::ClockSync { host_time_us: sent })?;
            let Ok(Ok(mcu_time)) = tokio::time::timeout(ACK_TIMEOUT, rx).await else {
                self.shared.clock_waiters.lock().unwrap().remove(&sent);
                continue;
            };
            let recv = self.host_us();
            let rtt = recv - sent;
            let offset = mcu_time as i64 - (sent + rtt / 2) as i64;
            let cand = ClockSync { offset_us: offset, rtt_us: rtt as u32 };
            if best.is_none_or(|b| cand.rtt_us < b.rtt_us) {
                best = Some(cand);
            }
        }
        let best = best.ok_or(McuError::Timeout(ACK_TIMEOUT))?;
        *self.shared.clock.lock().unwrap() = best;
        Ok(best)
    }
}

/// An in-flight command awaiting its Ack/Nack.
pub struct PendingAck {
    rx: oneshot::Receiver<Result<(), NackReason>>,
    seq: u32,
    shared: Arc<Shared>,
}

impl PendingAck {
    pub async fn wait(self) -> Result<(), McuError> {
        match tokio::time::timeout(ACK_TIMEOUT, self.rx).await {
            Ok(Ok(Ok(()))) => Ok(()),
            Ok(Ok(Err(reason))) => Err(McuError::Nack(reason)),
            Ok(Err(_)) => Err(McuError::LinkDown),
            Err(_) => {
                self.shared.pending.lock().unwrap().remove(&self.seq);
                Err(McuError::Timeout(ACK_TIMEOUT))
            }
        }
    }
}

fn link_lost(shared: &Shared, ev: &mpsc::UnboundedSender<McuEvent>, reason: String) {
    if shared.alive.swap(false, Ordering::SeqCst) {
        shared.pending.lock().unwrap().clear();
        shared.clock_waiters.lock().unwrap().clear();
        let _ = ev.send(McuEvent::LinkLost { reason });
    }
}

async fn writer<W: AsyncWrite + Unpin>(
    mut wr: W,
    mut prio: mpsc::Receiver<HostFrame>,
    mut normal: mpsc::Receiver<HostFrame>,
    shared: Arc<Shared>,
    ev: mpsc::UnboundedSender<McuEvent>,
) {
    let mut buf = [0u8; MAX_FRAME];
    let mut batch = Vec::with_capacity(MAX_BATCH_BYTES + MAX_FRAME);
    loop {
        let first = tokio::select! {
            biased;
            Some(f) = prio.recv() => f,
            Some(f) = normal.recv() => f,
            else => return,
        };
        batch.clear();
        let mut push = |f: &HostFrame, batch: &mut Vec<u8>| {
            if let Ok(n) = encode_frame(f, &mut buf) {
                batch.extend_from_slice(&buf[..n]);
                shared.tx_frames.fetch_add(1, Ordering::Relaxed);
            }
        };
        push(&first, &mut batch);
        // Priority frames first, then normal frames, never exceeding the
        // batch budget (one frame may overshoot it).
        while batch.len() < MAX_BATCH_BYTES {
            match prio.try_recv() {
                Ok(f) => push(&f, &mut batch),
                Err(_) => break,
            }
        }
        while batch.len() < MAX_BATCH_BYTES {
            match normal.try_recv() {
                Ok(f) => push(&f, &mut batch),
                Err(_) => break,
            }
        }
        let res = async {
            wr.write_all(&batch).await?;
            wr.flush().await
        }
        .await;
        if let Err(e) = res {
            link_lost(&shared, &ev, format!("write failed: {e}"));
            return;
        }
        if !shared.alive.load(Ordering::SeqCst) {
            return;
        }
    }
}

async fn reader<R: AsyncRead + Unpin>(mut rd: R, shared: Arc<Shared>, ev: mpsc::UnboundedSender<McuEvent>, status: watch::Sender<Option<Status>>) {
    let mut dec = FrameDecoder::new();
    let mut buf = [0u8; 512];
    loop {
        let n = match rd.read(&mut buf).await {
            Ok(0) => {
                link_lost(&shared, &ev, "MCU closed the connection".into());
                return;
            }
            Ok(n) => n,
            Err(e) => {
                link_lost(&shared, &ev, format!("read failed: {e}"));
                return;
            }
        };
        for &b in &buf[..n] {
            match dec.push::<McuFrame>(b) {
                None => {}
                Some(Err(FrameError::Crc)) => {
                    shared.crc_errors.fetch_add(1, Ordering::Relaxed);
                }
                Some(Err(_)) => {
                    shared.decode_errors.fetch_add(1, Ordering::Relaxed);
                }
                Some(Ok(frame)) => {
                    shared.rx_frames.fetch_add(1, Ordering::Relaxed);
                    shared.last_rx_us.store(shared.host_us(), Ordering::Relaxed);
                    dispatch(frame.msg, &shared, &ev, &status);
                }
            }
        }
    }
}

fn dispatch(msg: McuMsg, shared: &Shared, ev: &mpsc::UnboundedSender<McuEvent>, status: &watch::Sender<Option<Status>>) {
    match msg {
        McuMsg::Ack { seq } => {
            if let Some(Waiter::Ack(tx)) = shared.pending.lock().unwrap().remove(&seq) {
                let _ = tx.send(Ok(()));
            }
        }
        McuMsg::Nack { seq, reason } => {
            if let Some(Waiter::Ack(tx)) = shared.pending.lock().unwrap().remove(&seq) {
                let _ = tx.send(Err(reason));
            }
        }
        McuMsg::Identity(id) => {
            if let Some(tx) = shared.identity.lock().unwrap().take() {
                let _ = tx.send(id);
            }
        }
        McuMsg::ClockSyncReply { host_time_us, mcu_time_us } => {
            if let Some(tx) = shared.clock_waiters.lock().unwrap().remove(&host_time_us) {
                let _ = tx.send(mcu_time_us);
            }
        }
        McuMsg::Status(s) => {
            let _ = status.send(Some(s));
        }
        McuMsg::Fault { code, axis } => {
            let _ = ev.send(McuEvent::Fault { code, axis });
        }
        McuMsg::HomingDone { axis, position_steps } => {
            let _ = ev.send(McuEvent::HomingDone { axis, position_steps });
        }
    }
}

async fn heartbeat(out: mpsc::Sender<HostFrame>, shared: Arc<Shared>, ev: mpsc::UnboundedSender<McuEvent>) {
    let mut iv = tokio::time::interval(HEARTBEAT_PERIOD);
    let started = shared.host_us();
    loop {
        iv.tick().await;
        if !shared.alive.load(Ordering::SeqCst) {
            return;
        }
        // Keep half of the priority lane free for emergency stops: if the
        // transport is stalled, more heartbeats would not help anyway.
        if out.capacity() > PRIORITY_CAPACITY / 2 {
            let seq = shared.seq.fetch_add(1, Ordering::Relaxed).wrapping_add(1);
            if out.try_send(HostFrame { seq, msg: HostMsg::Heartbeat }).is_err() && out.is_closed() {
                return;
            }
        }
        let now = shared.host_us();
        let last = shared.last_rx_us.load(Ordering::Relaxed).max(started);
        if now.saturating_sub(last) > SILENCE_TIMEOUT.as_micros() as u64 {
            link_lost(&shared, &ev, format!("no data from MCU for {} ms", (now - last) / 1000));
            return;
        }
    }
}

#[cfg(test)]
mod tests;
