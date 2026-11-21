//! Link tests against the real virtual MCU (shared firmware core).

use super::*;
use motioncore_protocol::{AxisConfig, Segment};
use motioncore_simulator::{SimAxis, SimEndstop, VirtualOptions};

fn start_virtual(opts: VirtualOptions) -> (McuLink, mpsc::UnboundedReceiver<McuEvent>, tokio::task::JoinHandle<()>) {
    let (host, mcu) = tokio::io::duplex(4096);
    let h = tokio::spawn(async move {
        let _ = motioncore_simulator::run(mcu, opts).await;
    });
    let (link, ev) = McuLink::start(host);
    (link, ev, h)
}

fn axis_cfg() -> AxisConfig {
    AxisConfig { axis: 0, min_steps: -100_000, max_steps: 100_000, max_rate: 10_000.0, stop_decel: 50_000.0, invert_dir: false, invert_endstop: false }
}

#[tokio::test]
async fn handshake_and_clock_sync() {
    let (link, _ev, _h) = start_virtual(VirtualOptions::default());
    let id = link.identify().await.unwrap();
    assert_eq!(id.target.as_str(), "virtual");
    assert_eq!(id.axes, 6);
    let c = link.sync_clock(8).await.unwrap();
    // The virtual MCU clock starts at 7 s; the link epoch at ~0.
    assert!((c.offset_us - 7_000_000).abs() < 50_000, "{c:?}");
    assert!(link.stats().rx_frames >= 9);
}

#[tokio::test]
async fn nack_is_reported() {
    let (link, _ev, _h) = start_virtual(VirtualOptions::default());
    let err = link.command(HostMsg::Enable { axis: 0 }).await.unwrap_err();
    assert_eq!(err, McuError::Nack(NackReason::InvalidState), "unconfigured axis");
    let err = link.command(HostMsg::Enable { axis: 42 }).await.unwrap_err();
    assert_eq!(err, McuError::Nack(NackReason::UnknownAxis));
}

#[tokio::test]
async fn executes_segments_on_synchronised_clock() {
    let (link, _ev, _h) = start_virtual(VirtualOptions::default());
    link.identify().await.unwrap();
    link.sync_clock(8).await.unwrap();
    link.command(HostMsg::ConfigureAxis(axis_cfg())).await.unwrap();
    link.command(HostMsg::Enable { axis: 0 }).await.unwrap();
    let start = link.mcu_now_us() + 100_000;
    for k in 0..5u64 {
        link.command(HostMsg::QueueSegment(Segment {
            axis: 0,
            start_us: start + k * 20_000,
            duration_us: 20_000,
            steps: 40,
            v0: 2000.0,
            accel: 0.0,
            last: k == 4,
        }))
        .await
        .unwrap();
    }
    tokio::time::sleep(Duration::from_millis(400)).await;
    let st = link.latest_status().unwrap();
    assert_eq!(st.fault, None);
    assert_eq!(st.axes[0].position_steps, 200);
    assert!(!st.axes[0].moving);
}

#[tokio::test]
async fn homing_reports_completion() {
    let opts =
        VirtualOptions { axes: vec![SimAxis { start_steps: 500, endstop: Some(SimEndstop { at_steps: 0, positive_side: false }) }], interlock_open: false };
    let (link, mut ev, _h) = start_virtual(opts);
    link.identify().await.unwrap();
    link.command(HostMsg::ConfigureAxis(axis_cfg())).await.unwrap();
    link.command(HostMsg::Enable { axis: 0 }).await.unwrap();
    link.command(HostMsg::Home(motioncore_protocol::HomeParams { axis: 0, positive: false, speed: 5000.0, max_travel_steps: 2000, home_steps: -123 }))
        .await
        .unwrap();
    let e = tokio::time::timeout(Duration::from_secs(2), ev.recv()).await.unwrap().unwrap();
    assert_eq!(e, McuEvent::HomingDone { axis: 0, position_steps: -123 });
}

#[tokio::test]
async fn link_loss_is_detected_and_commands_fail() {
    let (link, mut ev, h) = start_virtual(VirtualOptions::default());
    link.identify().await.unwrap();
    h.abort(); // "unplug" the MCU
    let e = tokio::time::timeout(Duration::from_secs(3), ev.recv()).await.unwrap().unwrap();
    assert!(matches!(e, McuEvent::LinkLost { .. }), "{e:?}");
    assert!(!link.is_alive());
    assert_eq!(link.command(HostMsg::QueryStatus).await.unwrap_err(), McuError::LinkDown);
}

#[tokio::test]
async fn mcu_watchdog_trips_when_host_goes_silent() {
    // Drive the virtual MCU manually (no heartbeat task) to prove the
    // firmware-side watchdog acts independently of the host.
    use motioncore_protocol::{encode_frame, FrameDecoder, HostFrame, McuFrame};
    let (mut host, mcu) = tokio::io::duplex(4096);
    tokio::spawn(async move {
        let _ = motioncore_simulator::run(mcu, VirtualOptions::default()).await;
    });
    let mut buf = [0u8; MAX_FRAME];
    for (seq, msg) in [(1, HostMsg::SetWatchdog { timeout_ms: 100 }), (2, HostMsg::ConfigureAxis(axis_cfg())), (3, HostMsg::Enable { axis: 0 })] {
        let n = encode_frame(&HostFrame { seq, msg }, &mut buf).unwrap();
        host.write_all(&buf[..n]).await.unwrap();
    }
    let mut dec = FrameDecoder::new();
    let deadline = Instant::now() + Duration::from_secs(2);
    let mut rb = [0u8; 512];
    while Instant::now() < deadline {
        let n = tokio::time::timeout(Duration::from_millis(500), host.read(&mut rb)).await.unwrap().unwrap();
        for &b in &rb[..n] {
            if let Some(Ok(McuFrame { msg: McuMsg::Fault { code, .. }, .. })) = dec.push::<McuFrame>(b) {
                assert_eq!(code, FaultCode::CommunicationLost);
                return;
            }
        }
    }
    panic!("MCU watchdog did not trip");
}

fn seg(k: u64) -> HostMsg {
    HostMsg::QueueSegment(Segment { axis: 0, start_us: 1_000_000 + k * 20_000, duration_us: 20_000, steps: 40, v0: 2000.0, accel: 0.0, last: false })
}

/// A link whose far end is a tiny pipe nobody reads yet, so the writer stalls
/// and the outbound queue saturates.
fn stalled_link() -> (McuLink, tokio::io::DuplexStream) {
    let (host, far) = tokio::io::duplex(64);
    let (link, _ev) = McuLink::start(host);
    (link, far)
}

#[tokio::test]
async fn outbound_queue_is_bounded() {
    let (link, _far) = stalled_link();
    tokio::task::yield_now().await;
    let results: Vec<_> = (0..(OUTBOUND_CAPACITY as u64 + 64)).map(|k| link.submit(seg(k)).map(|_| ())).collect();
    let rejected = results.iter().filter(|r| matches!(r, Err(McuError::Backpressure))).count();
    assert!(rejected > 0, "queue grew past its capacity");
    assert!(results.iter().all(|r| matches!(r, Ok(()) | Err(McuError::Backpressure))));
}

#[tokio::test]
async fn emergency_stop_overtakes_saturated_queue() {
    use motioncore_protocol::{FrameDecoder, HostFrame};
    let (link, mut far) = stalled_link();
    let mut acks = Vec::new();
    for k in 0..200 {
        acks.push(link.submit(seg(k)).unwrap());
    }
    link.send(HostMsg::EmergencyStop).unwrap();

    // Now drain the far end and find where the e-stop landed.
    let mut dec = FrameDecoder::new();
    let mut rb = [0u8; 256];
    let mut segments_before = 0usize;
    loop {
        let n = tokio::time::timeout(Duration::from_secs(1), far.read(&mut rb)).await.unwrap().unwrap();
        assert!(n > 0, "link closed before the e-stop was sent");
        for &b in &rb[..n] {
            match dec.push::<HostFrame>(b) {
                Some(Ok(HostFrame { msg: HostMsg::EmergencyStop, .. })) => {
                    // At most the batch already in flight may precede it.
                    let bound = MAX_BATCH_BYTES / 16 + 1;
                    assert!(segments_before <= bound, "e-stop waited behind {segments_before} segments (bound {bound})");
                    return;
                }
                Some(Ok(HostFrame { msg: HostMsg::QueueSegment(_), .. })) => segments_before += 1,
                _ => {}
            }
        }
    }
}

#[tokio::test]
async fn close_releases_the_transport() {
    let (link, _ev, mcu) = start_virtual(VirtualOptions::default());
    link.identify().await.unwrap();
    // Without close, the heartbeat keeps the link (and the port) open as long
    // as the MCU answers; on Windows a COM port then cannot be reopened.
    link.close();
    tokio::time::timeout(Duration::from_secs(2), mcu).await.expect("the MCU side should see the host close the stream").unwrap();
    assert!(!link.is_alive());
    assert!(matches!(link.command(HostMsg::QueryStatus).await, Err(McuError::LinkDown)));
    link.close(); // idempotent
}
