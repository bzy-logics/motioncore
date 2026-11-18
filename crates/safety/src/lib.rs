//! Robot operational state machine and fault model.
//!
//! The [`Supervisor`] is the single source of truth for the robot state. All
//! transitions go through [`Supervisor::apply`], which rejects anything not in
//! the explicit transition table. Two invariants are enforced here:
//!
//! * an emergency stop has precedence over every other event, and
//! * resetting a fault never resumes motion — it always lands in `disabled`
//!   (or `disconnected` if the MCU is gone).

use serde::{Deserialize, Serialize};
use std::fmt;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RobotState {
    Disconnected,
    Initializing,
    Disabled,
    Ready,
    Homing,
    Running,
    Faulted,
    EmergencyStopped,
}

impl RobotState {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Disconnected => "disconnected",
            Self::Initializing => "initializing",
            Self::Disabled => "disabled",
            Self::Ready => "ready",
            Self::Homing => "homing",
            Self::Running => "running",
            Self::Faulted => "faulted",
            Self::EmergencyStopped => "emergency_stopped",
        }
    }

    /// Drives may be energised in this state.
    pub fn drives_enabled(self) -> bool {
        matches!(self, Self::Ready | Self::Homing | Self::Running)
    }

    /// The MCU link is believed to be up.
    pub fn is_connected(self) -> bool {
        !matches!(self, Self::Disconnected | Self::Initializing)
    }
}

impl fmt::Display for RobotState {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FaultCode {
    CommunicationLost,
    QueueUnderrun,
    SegmentLate,
    LimitHit,
    SoftLimit,
    HomingFailed,
    McuFault,
    PluginFailed,
    EmergencyStop,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Fault {
    pub code: FaultCode,
    pub message: String,
    pub axis: Option<String>,
}

impl Fault {
    pub fn new(code: FaultCode, message: impl Into<String>) -> Self {
        Self { code, message: message.into(), axis: None }
    }

    pub fn on_axis(mut self, axis: impl Into<String>) -> Self {
        self.axis = Some(axis.into());
        self
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Event {
    /// Transport opened, handshake starting.
    Connect,
    /// Handshake + configuration succeeded.
    Initialized,
    /// MCU link lost.
    ConnectionLost,
    Enable,
    Disable,
    StartHoming,
    HomingComplete,
    StartMotion,
    /// Motion queue drained and all axes at rest.
    MotionComplete,
    Fault(Fault),
    EmergencyStop,
    /// Operator fault reset. `connected` tells where to land.
    ResetFault {
        connected: bool,
    },
}

#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
#[error("{event} not allowed in state {state}")]
pub struct TransitionError {
    pub state: RobotState,
    pub event: String,
}

/// Robot state authority.
#[derive(Debug, Clone)]
pub struct Supervisor {
    state: RobotState,
    fault: Option<Fault>,
}

impl Default for Supervisor {
    fn default() -> Self {
        Self::new()
    }
}

impl Supervisor {
    pub fn new() -> Self {
        Self { state: RobotState::Disconnected, fault: None }
    }

    pub fn state(&self) -> RobotState {
        self.state
    }

    pub fn fault(&self) -> Option<&Fault> {
        self.fault.as_ref()
    }

    /// Check whether `event` would be accepted, without applying it.
    pub fn check(&self, event: &Event) -> Result<RobotState, TransitionError> {
        next_state(self.state, event).ok_or_else(|| TransitionError { state: self.state, event: event_name(event).to_string() })
    }

    /// Apply an event. Returns the new state.
    pub fn apply(&mut self, event: Event) -> Result<RobotState, TransitionError> {
        let next = self.check(&event)?;
        match &event {
            Event::Fault(f) => {
                // Keep the first fault: it is the root cause.
                if self.fault.is_none() {
                    self.fault = Some(f.clone());
                }
            }
            Event::ConnectionLost if next == RobotState::Faulted => {
                if self.fault.is_none() {
                    self.fault = Some(Fault::new(FaultCode::CommunicationLost, "MCU link lost"));
                }
            }
            Event::EmergencyStop => {
                self.fault = Some(Fault::new(FaultCode::EmergencyStop, "emergency stop activated"));
            }
            Event::ResetFault { .. } => self.fault = None,
            _ => {}
        }
        self.state = next;
        Ok(next)
    }
}

fn event_name(e: &Event) -> &'static str {
    match e {
        Event::Connect => "connect",
        Event::Initialized => "initialized",
        Event::ConnectionLost => "connection_lost",
        Event::Enable => "enable",
        Event::Disable => "disable",
        Event::StartHoming => "home",
        Event::HomingComplete => "homing_complete",
        Event::StartMotion => "motion",
        Event::MotionComplete => "motion_complete",
        Event::Fault(_) => "fault",
        Event::EmergencyStop => "estop",
        Event::ResetFault { .. } => "reset_fault",
    }
}

/// The transition table.
fn next_state(s: RobotState, e: &Event) -> Option<RobotState> {
    use Event as E;
    use RobotState as S;
    Some(match (s, e) {
        // Emergency stop always wins and latches.
        (_, E::EmergencyStop) => S::EmergencyStopped,
        (S::EmergencyStopped, E::ResetFault { connected }) => {
            if *connected {
                S::Disabled
            } else {
                S::Disconnected
            }
        }
        (S::EmergencyStopped, E::ConnectionLost | E::Fault(_) | E::Disable | E::MotionComplete) => S::EmergencyStopped,

        (S::Disconnected, E::Connect) => S::Initializing,
        (S::Initializing, E::Initialized) => S::Disabled,
        (S::Initializing, E::ConnectionLost) => S::Disconnected,
        (S::Initializing, E::Fault(_)) => S::Faulted,

        (S::Disconnected, _) => return None,

        (_, E::ConnectionLost) => S::Faulted,
        (_, E::Fault(_)) => S::Faulted,

        (S::Faulted, E::ResetFault { connected }) => {
            if *connected {
                S::Disabled
            } else {
                S::Disconnected
            }
        }
        (S::Faulted, E::Disable | E::MotionComplete) => S::Faulted,

        (S::Disabled, E::Enable) => S::Ready,
        (S::Disabled, E::Disable) => S::Disabled,
        (S::Ready, E::Enable) => S::Ready,
        (S::Ready | S::Homing | S::Running, E::Disable) => S::Disabled,
        (S::Ready, E::StartHoming) => S::Homing,
        (S::Homing, E::HomingComplete) => S::Ready,
        (S::Ready, E::StartMotion) => S::Running,
        (S::Running, E::MotionComplete) => S::Ready,
        (S::Ready, E::MotionComplete) => S::Ready,
        _ => return None,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use RobotState as S;

    fn ready() -> Supervisor {
        let mut s = Supervisor::new();
        s.apply(Event::Connect).unwrap();
        s.apply(Event::Initialized).unwrap();
        s.apply(Event::Enable).unwrap();
        s
    }

    #[test]
    fn nominal_lifecycle() {
        let mut s = ready();
        assert_eq!(s.state(), S::Ready);
        s.apply(Event::StartHoming).unwrap();
        s.apply(Event::HomingComplete).unwrap();
        s.apply(Event::StartMotion).unwrap();
        assert_eq!(s.state(), S::Running);
        s.apply(Event::MotionComplete).unwrap();
        s.apply(Event::Disable).unwrap();
        assert_eq!(s.state(), S::Disabled);
    }

    #[test]
    fn motion_requires_ready() {
        let mut s = Supervisor::new();
        assert!(s.apply(Event::StartMotion).is_err());
        s.apply(Event::Connect).unwrap();
        s.apply(Event::Initialized).unwrap();
        let err = s.apply(Event::StartMotion).unwrap_err();
        assert_eq!(err.state, S::Disabled);
        assert!(s.apply(Event::StartHoming).is_err());
    }

    #[test]
    fn cannot_start_new_motion_while_running() {
        let mut s = ready();
        s.apply(Event::StartMotion).unwrap();
        assert!(s.apply(Event::StartMotion).is_err());
        assert!(s.apply(Event::StartHoming).is_err());
    }

    #[test]
    fn fault_reset_never_resumes_motion() {
        for setup in [Event::StartMotion, Event::StartHoming] {
            let mut s = ready();
            s.apply(setup).unwrap();
            s.apply(Event::Fault(Fault::new(FaultCode::QueueUnderrun, "x"))).unwrap();
            assert_eq!(s.state(), S::Faulted);
            assert!(s.apply(Event::Enable).is_err(), "must reset first");
            assert_eq!(s.apply(Event::ResetFault { connected: true }).unwrap(), S::Disabled);
            assert!(s.fault().is_none());
        }
    }

    #[test]
    fn first_fault_is_kept() {
        let mut s = ready();
        s.apply(Event::Fault(Fault::new(FaultCode::LimitHit, "a").on_axis("j1"))).unwrap();
        s.apply(Event::Fault(Fault::new(FaultCode::QueueUnderrun, "b"))).unwrap();
        assert_eq!(s.fault().unwrap().code, FaultCode::LimitHit);
        assert_eq!(s.fault().unwrap().axis.as_deref(), Some("j1"));
    }

    #[test]
    fn estop_has_precedence_and_latches() {
        for st in [Event::StartMotion, Event::StartHoming] {
            let mut s = ready();
            s.apply(st).unwrap();
            s.apply(Event::EmergencyStop).unwrap();
            assert_eq!(s.state(), S::EmergencyStopped);
            // Subsequent faults and link loss do not leave the e-stop state.
            s.apply(Event::Fault(Fault::new(FaultCode::McuFault, "x"))).unwrap();
            s.apply(Event::ConnectionLost).unwrap();
            assert_eq!(s.state(), S::EmergencyStopped);
            assert_eq!(s.fault().unwrap().code, FaultCode::EmergencyStop);
            assert!(s.apply(Event::Enable).is_err());
            assert_eq!(s.apply(Event::ResetFault { connected: false }).unwrap(), S::Disconnected);
        }
        let mut s = Supervisor::new();
        assert_eq!(s.apply(Event::EmergencyStop).unwrap(), S::EmergencyStopped);
    }

    #[test]
    fn connection_loss_faults() {
        let mut s = ready();
        s.apply(Event::StartMotion).unwrap();
        s.apply(Event::ConnectionLost).unwrap();
        assert_eq!(s.state(), S::Faulted);
        assert_eq!(s.fault().unwrap().code, FaultCode::CommunicationLost);
        assert_eq!(s.apply(Event::ResetFault { connected: false }).unwrap(), S::Disconnected);
        assert!(!s.state().is_connected());
    }

    #[test]
    fn serialises_snake_case() {
        assert_eq!(serde_json::to_string(&S::EmergencyStopped).unwrap(), "\"emergency_stopped\"");
        assert_eq!(serde_json::to_string(&FaultCode::CommunicationLost).unwrap(), "\"communication_lost\"");
    }
}
