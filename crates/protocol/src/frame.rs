//! Frame encoding/decoding: postcard + CRC-16 + COBS + `0x00` delimiter.

use serde::{de::DeserializeOwned, Serialize};

use crate::{cobs, crc::crc16, MAX_FRAME, MAX_PAYLOAD};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrameError {
    /// Message does not fit into [`MAX_PAYLOAD`] / the receive buffer.
    TooLarge,
    /// COBS structure invalid.
    Cobs,
    /// CRC mismatch: the frame was corrupted in transit.
    Crc,
    /// Frame was intact but the message could not be decoded (e.g. version
    /// mismatch or unknown variant).
    Decode,
    /// Frame shorter than the CRC trailer.
    Truncated,
}

/// Encode `msg` into `out` as a complete delimited frame. Returns the number
/// of bytes written.
pub fn encode_frame<T: Serialize>(msg: &T, out: &mut [u8]) -> Result<usize, FrameError> {
    let mut raw = [0u8; MAX_PAYLOAD];
    let used = postcard::to_slice(msg, &mut raw[..MAX_PAYLOAD - 2])
        .map_err(|_| FrameError::TooLarge)?
        .len();
    let crc = crc16(&raw[..used]);
    raw[used..used + 2].copy_from_slice(&crc.to_le_bytes());
    let n = cobs::encode(&raw[..used + 2], out).map_err(|_| FrameError::TooLarge)?;
    if n >= out.len() {
        return Err(FrameError::TooLarge);
    }
    out[n] = 0;
    Ok(n + 1)
}

/// Streaming frame decoder. Feed it bytes as they arrive; it yields one
/// result per delimiter and resynchronises on garbage or overflow.
pub struct FrameDecoder {
    buf: [u8; MAX_FRAME],
    len: usize,
    overflow: bool,
}

impl Default for FrameDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl FrameDecoder {
    pub const fn new() -> Self {
        Self { buf: [0; MAX_FRAME], len: 0, overflow: false }
    }

    /// Push one byte. Returns `Some` when a delimiter completes a frame.
    pub fn push<T: DeserializeOwned>(&mut self, byte: u8) -> Option<Result<T, FrameError>> {
        if byte != 0 {
            if self.len < self.buf.len() {
                self.buf[self.len] = byte;
                self.len += 1;
            } else {
                self.overflow = true;
            }
            return None;
        }
        let len = core::mem::replace(&mut self.len, 0);
        if core::mem::replace(&mut self.overflow, false) {
            return Some(Err(FrameError::TooLarge));
        }
        if len == 0 {
            return None; // Empty frame: back-to-back delimiters used for resync.
        }
        Some(decode_payload(&self.buf[..len]))
    }
}

fn decode_payload<T: DeserializeOwned>(encoded: &[u8]) -> Result<T, FrameError> {
    let mut raw = [0u8; MAX_FRAME];
    let n = cobs::decode(encoded, &mut raw).map_err(|_| FrameError::Cobs)?;
    if n < 2 {
        return Err(FrameError::Truncated);
    }
    let (body, trailer) = raw[..n].split_at(n - 2);
    let expected = u16::from_le_bytes([trailer[0], trailer[1]]);
    if crc16(body) != expected {
        return Err(FrameError::Crc);
    }
    postcard::from_bytes(body).map_err(|_| FrameError::Decode)
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use crate::messages::*;
    use heapless::Vec;
    use std::vec;

    fn decode_all<T: DeserializeOwned>(bytes: &[u8]) -> std::vec::Vec<Result<T, FrameError>> {
        let mut d = FrameDecoder::new();
        bytes.iter().filter_map(|&b| d.push(b)).collect()
    }

    fn encode<T: Serialize>(msg: &T) -> std::vec::Vec<u8> {
        let mut out = [0u8; MAX_FRAME];
        let n = encode_frame(msg, &mut out).unwrap();
        out[..n].to_vec()
    }

    #[test]
    fn host_frame_roundtrip() {
        let msgs = [
            HostMsg::Identify,
            HostMsg::ClockSync { host_time_us: 123_456_789 },
            HostMsg::QueueSegment(Segment {
                axis: 0,
                start_us: 9_000_000,
                duration_us: 20_000,
                steps: -120,
                v0: -1000.0,
                accel: -2000.0,
                last: false,
            }),
            HostMsg::Home(HomeParams {
                axis: 1,
                positive: false,
                speed: 800.0,
                max_travel_steps: 20_000,
                home_steps: -7555,
            }),
            HostMsg::EmergencyStop,
        ];
        for (i, msg) in msgs.into_iter().enumerate() {
            let frame = HostFrame { seq: i as u32, msg };
            let bytes = encode(&frame);
            assert_eq!(*bytes.last().unwrap(), 0);
            assert!(!bytes[..bytes.len() - 1].contains(&0));
            assert_eq!(decode_all::<HostFrame>(&bytes), vec![Ok(frame)]);
        }
    }

    #[test]
    fn largest_status_fits() {
        let mut axes = Vec::new();
        for _ in 0..crate::MAX_AXES {
            axes.push(AxisStatus {
                position_steps: i32::MIN,
                velocity: -1.0e9,
                enabled: true,
                moving: true,
                homing: true,
                endstop: true,
            })
            .unwrap();
        }
        let frame = McuFrame {
            seq: u32::MAX,
            msg: McuMsg::Status(Status {
                mcu_time_us: u64::MAX,
                fault: Some(FaultCode::StepRateExceeded),
                queue_depth: 255,
                axes,
            }),
        };
        assert_eq!(decode_all::<McuFrame>(&encode(&frame)), vec![Ok(frame)]);
    }

    #[test]
    fn identity_fits() {
        let frame = McuFrame {
            seq: 1,
            msg: McuMsg::Identity(Identity {
                protocol_version: crate::PROTOCOL_VERSION,
                firmware_version: (255, 255, 255),
                target: heapless::String::try_from("0123456789abcdef").unwrap(),
                board: heapless::String::try_from("0123456789abcdef01234567").unwrap(),
                build_hash: u32::MAX,
                capabilities: u32::MAX,
                axes: 8,
                queue_capacity: 255,
            }),
        };
        assert_eq!(decode_all::<McuFrame>(&encode(&frame)), vec![Ok(frame)]);
    }

    #[test]
    fn detects_corruption_and_resyncs() {
        let a = HostFrame { seq: 1, msg: HostMsg::Heartbeat };
        let b = HostFrame { seq: 2, msg: HostMsg::Stop };
        let mut corrupted = encode(&a);
        corrupted[2] ^= 0x40;
        if corrupted[2] == 0 {
            corrupted[2] = 0x7F;
        }
        let mut stream = vec![0x13, 0x37, 0x00]; // line noise + delimiter
        stream.extend_from_slice(&corrupted);
        stream.extend_from_slice(&encode(&b));

        let got = decode_all::<HostFrame>(&stream);
        assert_eq!(got.len(), 3);
        assert!(got[0].is_err(), "noise must not decode");
        assert!(got[1].is_err(), "corrupted frame must be rejected");
        assert_eq!(got[2], Ok(b));
    }

    #[test]
    fn every_single_bit_flip_is_detected() {
        let frame = HostFrame { seq: 77, msg: HostMsg::SetPosition { axis: 2, steps: 4242 } };
        let bytes = encode(&frame);
        for byte in 0..bytes.len() - 1 {
            for bit in 0..8 {
                let mut f = bytes.clone();
                f[byte] ^= 1 << bit;
                for r in decode_all::<HostFrame>(&f) {
                    assert_ne!(r, Ok(frame.clone()), "flip byte {byte} bit {bit} undetected");
                }
            }
        }
    }

    #[test]
    fn overflow_is_reported_then_recovers() {
        let mut d = FrameDecoder::new();
        for _ in 0..MAX_FRAME + 10 {
            assert!(d.push::<HostFrame>(0x55).is_none());
        }
        assert_eq!(d.push::<HostFrame>(0), Some(Err(FrameError::TooLarge)));
        let f = HostFrame { seq: 3, msg: HostMsg::Identify };
        let got: std::vec::Vec<_> =
            encode(&f).iter().filter_map(|&b| d.push::<HostFrame>(b)).collect();
        assert_eq!(got, vec![Ok(f)]);
    }

    #[test]
    fn capability_names_maps_bits() {
        let names: std::vec::Vec<_> =
            capability_names(caps::STEPPER_STEP_DIR | caps::WATCHDOG).collect();
        assert_eq!(names, ["stepper.step_dir", "watchdog"]);
        assert_eq!(caps::from_name("endstop"), Some(caps::ENDSTOP));
    }
}
