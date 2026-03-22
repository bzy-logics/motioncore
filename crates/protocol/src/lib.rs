//! MotionCore host <-> MCU protocol.
//!
//! This crate is `no_std` and allocation-free so the exact same message
//! definitions and framing code run on the MCU firmware, the virtual MCU and
//! the Rust host.
//!
//! Wire format of one frame:
//!
//! ```text
//! COBS( postcard(message) || crc16_le ) || 0x00
//! ```
//!
//! * `postcard` gives a compact, versionable binary encoding.
//! * CRC-16/CCITT-FALSE detects corrupted frames.
//! * COBS removes all zero bytes so `0x00` can delimit frames on a serial
//!   stream and the receiver can resynchronise after garbage.
#![cfg_attr(not(any(feature = "std", test)), no_std)]
#![forbid(unsafe_code)]

pub mod cobs;
pub mod crc;
pub mod frame;
pub mod messages;

pub use frame::{encode_frame, FrameDecoder, FrameError};
pub use messages::*;

/// Protocol version. Bumped on any incompatible change to [`messages`].
pub const PROTOCOL_VERSION: u16 = 1;

/// Maximum size of an *unencoded* frame payload (message + CRC).
pub const MAX_PAYLOAD: usize = 224;

/// Maximum size of an encoded frame including COBS overhead and delimiter.
pub const MAX_FRAME: usize = cobs::max_encoded_len(MAX_PAYLOAD) + 1;

/// Maximum number of axes a single MCU can report.
pub const MAX_AXES: usize = 8;
