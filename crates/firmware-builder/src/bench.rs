//! Bench test tools that talk to the MCU directly, without the engine.
//!
//! [`rate_test`] measures the highest step rate the firmware sustains (RP2040
//! test 8). Profile validation caps every move at the board's advertised
//! rate, so the engine cannot exceed the very limit this test measures; the
//! test therefore drives the MCU protocol itself. **Drivers must have no motor
//! power**: the axes are enabled and stepped far above any usable speed.

use std::time::Duration;

use anyhow::{bail, Context, Result};
use motioncore_mcu_manager::{McuEvent, McuLink};
use motioncore_protocol::{AxisConfig, FaultCode, HostMsg, Segment};
use tokio::sync::mpsc;

/// Parameters of [`rate_test`].
#[derive(Debug, Clone)]
pub struct RateTestOptions {
    /// Step channels to drive together (default: every channel of the MCU).
    pub axes: Option<Vec<u8>>,
    /// First stage rate (steps/s).
    pub from: f32,
    /// Last stage rate (steps/s).
    pub to: f32,
    /// Rate increment between stages (steps/s).
    pub step: f32,
    /// Time at the stage rate (after a ramp up, before a ramp down).
    pub hold: Duration,
    /// Duration of each ramp.
    pub ramp: Duration,
}

impl Default for RateTestOptions {
    fn default() -> Self {
        Self { axes: None, from: 10_000.0, to: 250_000.0, step: 10_000.0, hold: Duration::from_secs(1), ramp: Duration::from_millis(50) }
    }
}

/// Outcome of one stage.
#[derive(Debug, Clone, PartialEq)]
pub struct StageResult {
    pub rate: f32,
    /// `None`: every axis ran the stage and ended at the exact step count.
    pub failure: Option<String>,
}

#[derive(Debug, Clone)]
pub struct RateTestReport {
    pub board: String,
    pub axes: Vec<u8>,
    pub stages: Vec<StageResult>,
}

impl RateTestReport {
    /// Highest stage rate that passed (0 if none did).
    pub fn max_sustained(&self) -> f32 {
        self.stages.iter().filter(|s| s.failure.is_none()).map(|s| s.rate).fold(0.0, f32::max)
    }

    /// `true` if the sweep ended without a failing stage: the real limit lies
    /// above [`Self::max_sustained`].
    pub fn limit_not_reached(&self) -> bool {
        self.stages.iter().all(|s| s.failure.is_none())
    }
}

/// Stage rates from `from` to `to` (inclusive) in increments of `step`.
pub fn stage_rates(o: &RateTestOptions) -> Result<Vec<f32>> {
    if !(o.from > 0.0 && o.to >= o.from && o.step > 0.0) || !(o.from.is_finite() && o.to.is_finite() && o.step.is_finite()) {
        bail!("need 0 < from <= to and step > 0 (got from {}, to {}, step {})", o.from, o.to, o.step);
    }
    let n = ((o.to - o.from) / o.step).floor() as usize + 1;
    if n > 1000 {
        bail!("{n} stages: use a larger step");
    }
    Ok((0..n).map(|i| o.from + i as f32 * o.step).collect())
}

/// Ramp up, hold and ramp down at `rate` for one axis, starting at
/// `start_us`, in direction `sign`. Returns the segments and their total
/// step count. Each segment's integer step count is its rounded kinematic
/// distance, so the MCU accepts it.
pub fn stage_segments(axis: u8, rate: f32, sign: f32, start_us: u64, ramp: Duration, hold: Duration) -> (Vec<Segment>, i64) {
    let ramp_us = ramp.as_micros().max(1) as u32;
    let hold_us = hold.as_micros().max(1) as u32;
    let tr = ramp_us as f64 * 1e-6;
    let th = hold_us as f64 * 1e-6;
    let r = rate as f64;
    let accel = r / tr;
    let ramp_steps = (0.5 * r * tr).round() as i32;
    let hold_steps = (r * th).round() as i32;
    let s = sign as f64;
    let segs = vec![
        Segment { axis, start_us, duration_us: ramp_us, steps: (s * ramp_steps as f64) as i32, v0: 0.0, accel: (s * accel) as f32, last: false },
        Segment {
            axis,
            start_us: start_us + ramp_us as u64,
            duration_us: hold_us,
            steps: (s * hold_steps as f64) as i32,
            v0: (s * r) as f32,
            accel: 0.0,
            last: false,
        },
        Segment {
            axis,
            start_us: start_us + ramp_us as u64 + hold_us as u64,
            duration_us: ramp_us,
            steps: (s * ramp_steps as f64) as i32,
            v0: (s * r) as f32,
            accel: (-s * accel) as f32,
            last: true,
        },
    ];
    let total = (s as i64) * (2 * ramp_steps as i64 + hold_steps as i64);
    (segs, total)
}

/// Lead time between queueing a stage and its start.
const LEAD: Duration = Duration::from_millis(150);
/// Extra wait after a stage's planned end before checking the result.
const SETTLE: Duration = Duration::from_millis(250);

/// [`rate_test`] on the MCU at serial port `device`.
pub async fn rate_test_serial(device: &str, o: &RateTestOptions, log: &mut dyn FnMut(&str)) -> Result<RateTestReport> {
    use tokio_serial::SerialPortBuilderExt;
    let port = tokio_serial::new(device, 250_000).open_native_async().with_context(|| format!("opening {device}"))?;
    let (link, mut events) = McuLink::start(port);
    rate_test(&link, &mut events, o, log).await
}

/// Run the sweep on a connected MCU. Every listed axis is configured with
/// wide soft limits and a rate ceiling above `to`, enabled, and driven
/// through one stage per rate until a stage fails. Drives are disabled when
/// the test ends, also on error. `log` gets one line per stage.
pub async fn rate_test(
    link: &McuLink,
    events: &mut mpsc::UnboundedReceiver<McuEvent>,
    o: &RateTestOptions,
    log: &mut dyn FnMut(&str),
) -> Result<RateTestReport> {
    let rates = stage_rates(o)?;
    let id = link.identify().await.context("identifying the MCU")?;
    let axes: Vec<u8> = match &o.axes {
        Some(a) => a.clone(),
        None => (0..id.axes).collect(),
    };
    if axes.is_empty() || axes.iter().any(|&a| a >= id.axes) {
        bail!("axes {axes:?}: the MCU has {} channels", id.axes);
    }
    log(&format!("MCU: {} {} build {:08x}, {} channels; driving {:?}", id.target, id.board, id.build_hash, id.axes, axes));
    let result = sweep(link, events, o, &rates, &axes, log).await;
    for &a in &axes {
        let _ = link.command(HostMsg::Disable { axis: a }).await;
    }
    let stages = result?;
    Ok(RateTestReport { board: id.board.to_string(), axes, stages })
}

async fn sweep(
    link: &McuLink,
    events: &mut mpsc::UnboundedReceiver<McuEvent>,
    o: &RateTestOptions,
    rates: &[f32],
    axes: &[u8],
    log: &mut dyn FnMut(&str),
) -> Result<Vec<StageResult>> {
    link.command(HostMsg::ResetFault).await.context("clearing a latched fault")?;
    link.sync_clock(8).await.context("clock sync")?;
    link.command(HostMsg::SetWatchdog { timeout_ms: 500 }).await?;
    let ceiling = o.to * 1.1;
    for &axis in axes {
        let cfg = AxisConfig {
            axis,
            min_steps: i32::MIN / 2,
            max_steps: i32::MAX / 2,
            max_rate: ceiling,
            stop_decel: ceiling / 0.05,
            invert_dir: false,
            invert_endstop: false,
        };
        link.command(HostMsg::ConfigureAxis(cfg)).await.with_context(|| format!("configuring axis {axis}"))?;
        link.command(HostMsg::SetPosition { axis, steps: 0 }).await?;
    }
    while events.try_recv().is_ok() {}

    let mut position = 0i64;
    let mut stages = Vec::new();
    for (k, &rate) in rates.iter().enumerate() {
        for &axis in axes {
            link.command(HostMsg::Enable { axis }).await.with_context(|| format!("enabling axis {axis} (interlock closed?)"))?;
        }
        // Alternate direction so the position stays near zero.
        let sign = if k % 2 == 0 { 1.0 } else { -1.0 };
        let start = link.mcu_now_us() + LEAD.as_micros() as u64;
        let mut delta = 0;
        for &axis in axes {
            let (segs, total) = stage_segments(axis, rate, sign, start, o.ramp, o.hold);
            for s in segs {
                link.command(HostMsg::QueueSegment(s)).await.with_context(|| format!("queueing a {rate} steps/s segment on axis {axis}"))?;
            }
            delta = total;
        }
        let end = start + (2 * o.ramp + o.hold).as_micros() as u64;
        let failure = watch_stage(link, events, end, position + delta, axes).await?;
        match &failure {
            None => {
                position += delta;
                log(&format!("{rate:>9.0} steps/s  ok"));
            }
            Some(f) => log(&format!("{rate:>9.0} steps/s  FAILED: {f}")),
        }
        let failed = failure.is_some();
        stages.push(StageResult { rate, failure });
        if failed {
            let _ = link.command(HostMsg::ResetFault).await;
            break;
        }
    }
    Ok(stages)
}

/// Wait for the stage to end; report a fault, or a position other than
/// `expected` on any axis.
async fn watch_stage(link: &McuLink, events: &mut mpsc::UnboundedReceiver<McuEvent>, end_mcu_us: u64, expected: i64, axes: &[u8]) -> Result<Option<String>> {
    let wait = Duration::from_micros(end_mcu_us.saturating_sub(link.mcu_now_us())) + SETTLE;
    let deadline = tokio::time::Instant::now() + wait;
    loop {
        tokio::select! {
            ev = events.recv() => match ev {
                Some(McuEvent::Fault { code, axis }) => return Ok(Some(fault_text(code, axis))),
                Some(McuEvent::LinkLost { reason }) => bail!("MCU link lost: {reason}"),
                Some(_) => {}
                None => bail!("MCU link closed"),
            },
            _ = tokio::time::sleep_until(deadline) => break,
        }
    }
    let mut st = link.status_watch();
    let _ = st.borrow_and_update();
    link.send(HostMsg::QueryStatus)?;
    let status = tokio::time::timeout(Duration::from_secs(1), async {
        loop {
            if st.changed().await.is_err() {
                return None;
            }
            if let Some(s) = st.borrow_and_update().clone() {
                return Some(s);
            }
        }
    })
    .await
    .ok()
    .flatten()
    .context("no status from the MCU")?;
    if let Some(code) = status.fault {
        return Ok(Some(fault_text(code, None)));
    }
    for &a in axes {
        let Some(s) = status.axes.get(a as usize) else { bail!("status has no axis {a}") };
        if s.moving {
            return Ok(Some(format!("axis {a} still moving after the stage end")));
        }
        if s.position_steps as i64 != expected {
            return Ok(Some(format!("axis {a} at {} steps, expected {expected}", s.position_steps)));
        }
    }
    Ok(None)
}

fn fault_text(code: FaultCode, axis: Option<u8>) -> String {
    match axis {
        Some(a) => format!("{code:?} on axis {a}"),
        None => format!("{code:?}"),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stages_cover_the_range() {
        let o = RateTestOptions { from: 10_000.0, to: 50_000.0, step: 20_000.0, ..Default::default() };
        assert_eq!(stage_rates(&o).unwrap(), [10_000.0, 30_000.0, 50_000.0]);
        assert!(stage_rates(&RateTestOptions { from: 0.0, ..Default::default() }).is_err());
        assert!(stage_rates(&RateTestOptions { from: 2.0, to: 1.0, ..Default::default() }).is_err());
        assert!(stage_rates(&RateTestOptions { from: 1.0, to: 1e9, step: 1.0, ..Default::default() }).is_err());
    }

    #[test]
    fn stage_segments_are_contiguous_and_exact() {
        for rate in [1_000.0f32, 50_000.0, 123_457.0, 250_000.0] {
            for sign in [1.0f32, -1.0] {
                let (segs, total) = stage_segments(3, rate, sign, 1_000_000, Duration::from_millis(50), Duration::from_secs(1));
                assert_eq!(segs.iter().map(|s| s.steps as i64).sum::<i64>(), total);
                assert!(segs.windows(2).all(|w| w[1].start_us == w[0].end_us()));
                assert!(segs.last().unwrap().last && !segs[0].last);
                assert!((segs[2].end_velocity()).abs() < 1.0, "ends at rest");
                for s in &segs {
                    // The MCU's own acceptance rules (firmware-stepper `enqueue`).
                    let t = s.duration_us as f32 * 1e-6;
                    let ideal = s.v0 * t + 0.5 * s.accel * t * t;
                    assert!((ideal - s.steps as f32).abs() <= 1.5 + ideal.abs() * 1e-4, "{rate} {s:?}");
                    assert!(s.v0.abs() <= rate * 1.0001 && s.end_velocity().abs() <= rate * 1.0001);
                    assert_eq!(s.steps.signum() as f32, sign);
                }
            }
        }
    }
}
