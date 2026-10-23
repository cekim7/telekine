#![no_std]
#![forbid(unsafe_op_in_unsafe_fn)]

//! # `telekine-proto`
//!
//! Zero-copy `#[no_std]` 32-byte binary protocol, bounded AI intent definitions,
//! priority/redundancy flags, and application-layer AES-128-GCM encryption with
//! per-device key derivation (`Master Key + Device MAC + Target ID`).
//!
//! ## Wire Frame Layout (32 Bytes Total)
//!
//! ```text
//!  0                   1                   2                   3
//!  0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
//! +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//! |          target_id (u16)      |        seq_nonce (u32)        |  Bytes 0..6 (AAD)
//! +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//! |      seq_nonce (cont.)        | flags_prio(u8)|   opcode (u8) |  Bytes 6..8 (AAD)
//! +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//! |                       exec_at_tick_ms (u32)                   |  Bytes 8..12  (Encrypted)
//! +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//! |       lease_timeout_ms (u16)  |     param_primary_q16 (i32)   |  Bytes 12..16 (Encrypted)
//! +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//! |   param_primary_q16 (cont.)   |       param_limit_q8 (u16)    |  Bytes 16..20 (Encrypted)
//! +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//! |                                                               |
//! +                    aes_gcm_tag (96-bit / 12B)                 +  Bytes 20..32 (Auth Tag)
//! |                                                               |
//! +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//! ```

pub mod crypto;
pub mod packet;

pub use crypto::{
    derive_device_key, Aes128GcmSoftware, CryptoEngine, DeviceSecurityContext, SecurityError,
};
pub use packet::{
    BoundedIntent, EncryptedFrame, IntentOpcode, PacketFlags, Priority, RadioLink, RedundancyMode,
    TelemetryExceptionCode, VerifiedCommand, WirePacket, BROADCAST_TARGET_ID,
    MASTER_CHANNEL_TARGET_ID, WIRE_PACKET_SIZE,
};
