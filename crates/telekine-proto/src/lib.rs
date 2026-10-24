#![no_std]
#![forbid(unsafe_op_in_unsafe_fn)]

//! # `telekine-proto`
//!
//! Zero-copy `#[no_std]` 32-byte binary protocol, bounded AI intent definitions,
//! foundational unit discovery/access-lease protocol (`Servo`, `Stepper`, `DcMotor`),
//! priority/redundancy flags, and application-layer AES-128-GCM encryption with
//! per-device key derivation (`Master Key + Device MAC + Target ID`).

pub mod crypto;
pub mod packet;

pub use crypto::{
    derive_broadcast_key, derive_device_key, open_with_key, seal_with_key, Aes128GcmSoftware,
    CryptoEngine, DeviceSecurityContext, SecurityError,
};
pub use packet::{
    BoundedIntent, EncryptedFrame, IntentOpcode, PacketFlags, Priority, RadioLink, RedundancyMode,
    TelemetryExceptionCode, UnitAccessState, UnitElementKind, VerifiedCommand, WirePacket,
    BROADCAST_TARGET_ID, MASTER_CHANNEL_TARGET_ID, WIRE_PACKET_SIZE,
};
