//! The step-rate sweep against the virtual MCU: the same firmware core and
//! protocol as the boards, so the segments it queues must be accepted and
//! run to the exact step count.

use std::time::Duration;

use motioncore_firmware_builder::bench::{rate_test, RateTestOptions};
use motioncore_mcu_manager::McuLink;
use motioncore_protocol::HostMsg;

fn start_virtual_mcu() -> (McuLink, tokio::sync::mpsc::UnboundedReceiver<motioncore_mcu_manager::McuEvent>) {
    let (host, mcu) = tokio::io::duplex(8192);
    tokio::spawn(async move {
        let _ = motioncore_simulator::run(mcu, Default::default()).await;
    });
    McuLink::start(host)
}

fn opts() -> RateTestOptions {
    RateTestOptions { axes: Some(vec![0, 2, 5]), from: 2_000.0, to: 6_000.0, step: 2_000.0, hold: Duration::from_millis(200), ramp: Duration::from_millis(40) }
}

#[tokio::test]
async fn sweep_runs_every_stage_to_the_exact_step_count() {
    let (link, mut ev) = start_virtual_mcu();
    let mut lines = Vec::new();
    let r = rate_test(&link, &mut ev, &opts(), &mut |l| lines.push(l.to_string())).await.unwrap();
    assert_eq!(r.board, "virtual");
    assert_eq!(r.stages.iter().map(|s| s.rate).collect::<Vec<_>>(), [2_000.0, 4_000.0, 6_000.0]);
    assert!(r.stages.iter().all(|s| s.failure.is_none()), "{:?}", r.stages);
    assert_eq!(r.max_sustained(), 6_000.0);
    assert!(r.limit_not_reached());
    // Drives are left disabled.
    link.send(HostMsg::QueryStatus).unwrap();
    tokio::time::sleep(Duration::from_millis(100)).await;
    let st = link.latest_status().unwrap();
    assert!(st.axes.iter().all(|a| !a.enabled), "{st:?}");
}

#[tokio::test]
async fn a_fault_ends_the_sweep_and_is_reported() {
    let (link, mut ev) = start_virtual_mcu();
    let l2 = link.clone();
    // An E-stop during the second stage stands in for a firmware fault.
    tokio::spawn(async move {
        tokio::time::sleep(Duration::from_millis(550)).await;
        let _ = l2.send(HostMsg::EmergencyStop);
    });
    let r = rate_test(&link, &mut ev, &opts(), &mut |_| {}).await.unwrap();
    assert!(r.stages.len() < 3, "{:?}", r.stages);
    let last = r.stages.last().unwrap();
    assert!(last.failure.as_deref().is_some_and(|f| f.contains("EmergencyStop")), "{:?}", r.stages);
    assert!(!r.limit_not_reached());
    assert_eq!(r.max_sustained(), r.stages.iter().filter(|s| s.failure.is_none()).map(|s| s.rate).fold(0.0, f32::max));
}
