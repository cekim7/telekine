//! Application-Layer AES-128-GCM encryption, authentication, and per-device key derivation.
//!
//! Because ESP-NOW's hardware CCMP table caps at 17 encrypted peers (and cannot encrypt
//! broadcasts) and Semtech SX1262 Raw LoRa is plaintext by default, Telekine encrypts and
//! authenticates every 32-byte packet in firmware before transmission.
//!
//! - **8-byte Header (AAD):** `target_id (2B) || seq_nonce (4B) || flags (1B) || opcode (1B)`
//! - **12-byte Ciphertext:** `exec_at_tick_ms (4B) || lease_ms (2B) || param_primary (4B) || param_limit (2B)`
//! - **12-byte Auth Tag:** 96-bit AES-128-GCM authentication tag (`Bytes 20..32`).

use crate::packet::{
    EncryptedFrame, PacketFlags, RadioLink, VerifiedCommand, WirePacket, BROADCAST_TARGET_ID,
    MASTER_CHANNEL_TARGET_ID, WIRE_PACKET_SIZE,
};

/// Cryptographic or anti-replay verification failure.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SecurityError {
    /// Packet `target_id` does not match this actuator or a valid broadcast channel.
    TargetIdMismatch { expected: u16, actual: u16 },
    /// Monotonic deduplication / anti-replay check rejected `incoming_seq <= last_executed_seq`.
    StaleOrDuplicateSequence { last_seq: u32, incoming_seq: u32 },
    /// 96-bit AES-128-GCM authentication tag failed verification (tampered or wrong key).
    AuthenticationTagMismatch,
    /// Decrypted opcode byte is invalid.
    MalformedOpcode,
}

/// Hardware or software AES-128 block cipher abstraction.
/// On ESP32-S3 / ESP32-C3, implement this trait using `esp_hal::aes::Aes` peripheral
/// registers for <10 microsecond encryption; in simulation or host builds,
/// `Aes128GcmSoftware` provides a portable constant-structure implementation.
pub trait CryptoEngine {
    /// Encrypt a single 16-byte block with AES-128 using `key`.
    fn aes128_encrypt_block(&self, key: &[u8; 16], block: &[u8; 16]) -> [u8; 16];
}

/// Self-contained `#[no_std]` FIPS-197 AES-128 + NIST SP 800-38D GCM engine.
#[derive(Debug, Default, Clone, Copy)]
pub struct Aes128GcmSoftware;

const SBOX: [u8; 256] = [
    0x63, 0x7c, 0x77, 0x7b, 0xf2, 0x6b, 0x6f, 0xc5, 0x30, 0x01, 0x67, 0x2b, 0xfe, 0xd7, 0xab,
    0x76, 0xca, 0x82, 0xc9, 0x7d, 0xfa, 0x59, 0x47, 0xf0, 0xad, 0xd4, 0xa2, 0xaf, 0x9c, 0xa4,
    0x72, 0xc0, 0xb7, 0xfd, 0x93, 0x26, 0x36, 0x3f, 0xf7, 0xcc, 0x34, 0xa5, 0xe5, 0xf1, 0x71,
    0xd8, 0x31, 0x15, 0x04, 0xc7, 0x23, 0xc3, 0x18, 0x96, 0x05, 0x9a, 0x07, 0x12, 0x80, 0xe2,
    0xeb, 0x27, 0xb2, 0x75, 0x09, 0x83, 0x2c, 0x1a, 0x1b, 0x6e, 0x5a, 0xa0, 0x52, 0x3b, 0xd6,
    0xb3, 0x29, 0xe3, 0x2f, 0x84, 0x53, 0xd1, 0x00, 0xed, 0x20, 0xfc, 0xb1, 0x5b, 0x6a, 0xcb,
    0xbe, 0x39, 0x4a, 0x4c, 0x58, 0xcf, 0xd0, 0xef, 0xaa, 0xfb, 0x43, 0x4d, 0x33, 0x85, 0x45,
    0xf9, 0x02, 0x7f, 0x50, 0x3c, 0x9f, 0xa8, 0x51, 0xa3, 0x40, 0x8f, 0x92, 0x9d, 0x38, 0xf5,
    0xbc, 0xb6, 0xda, 0x21, 0x10, 0xff, 0xf3, 0xd2, 0xcd, 0x0c, 0x13, 0xec, 0x5f, 0x97, 0x44,
    0x17, 0xc4, 0xa7, 0x7e, 0x3d, 0x64, 0x5d, 0x19, 0x73, 0x60, 0x81, 0x4f, 0xdc, 0x22, 0x2a,
    0x90, 0x88, 0x46, 0xee, 0xb8, 0x14, 0xde, 0x5e, 0x0b, 0xdb, 0xe0, 0x32, 0x3a, 0x0a, 0x49,
    0x06, 0x24, 0x5c, 0xc2, 0xd3, 0xac, 0x62, 0x91, 0x95, 0xe4, 0x79, 0xe7, 0xc8, 0x37, 0x6d,
    0x8d, 0xd5, 0x4e, 0xa9, 0x6c, 0x56, 0xf4, 0xea, 0x65, 0x7a, 0xae, 0x08, 0xba, 0x78, 0x25,
    0x2e, 0x1c, 0xa6, 0xb4, 0xc6, 0xe8, 0xdd, 0x74, 0x1f, 0x4b, 0xbd, 0x8b, 0x8a, 0x70, 0x3e,
    0xb5, 0x66, 0x48, 0x03, 0xf6, 0x0e, 0x61, 0x35, 0x57, 0xb9, 0x86, 0xc1, 0x1d, 0x9e, 0xe1,
    0xf8, 0x98, 0x11, 0x69, 0xd9, 0x8e, 0x94, 0x9b, 0x1e, 0x87, 0xe9, 0xce, 0x55, 0x28, 0xdf,
    0x8c, 0xa1, 0x89, 0x0d, 0xbf, 0xe6, 0x42, 0x68, 0x41, 0x99, 0x2d, 0x0f, 0xb0, 0x54, 0xbb,
    0x16,
];

const RCON: [u8; 10] = [0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x1b, 0x36];

#[inline]
const fn xtime(x: u8) -> u8 {
    (x << 1) ^ (((x >> 7) & 1).wrapping_mul(0x1b))
}

impl CryptoEngine for Aes128GcmSoftware {
    fn aes128_encrypt_block(&self, key: &[u8; 16], block: &[u8; 16]) -> [u8; 16] {
        let mut round_keys = [[0u8; 16]; 11];
        round_keys[0] = *key;

        for r in 1..=10 {
            let prev = round_keys[r - 1];
            let mut temp = [
                SBOX[prev[13] as usize] ^ RCON[r - 1],
                SBOX[prev[14] as usize],
                SBOX[prev[15] as usize],
                SBOX[prev[12] as usize],
            ];
            let mut next = [0u8; 16];
            for i in 0..4 {
                for j in 0..4 {
                    next[i * 4 + j] = prev[i * 4 + j] ^ temp[j];
                    temp[j] = next[i * 4 + j];
                }
            }
            round_keys[r] = next;
        }

        let mut state = *block;
        for i in 0..16 {
            state[i] ^= round_keys[0][i];
        }

        for r in 1..=10 {
            // SubBytes
            for b in &mut state {
                *b = SBOX[*b as usize];
            }
            // ShiftRows
            let t1 = state[1];
            state[1] = state[5];
            state[5] = state[9];
            state[9] = state[13];
            state[13] = t1;

            let t2_0 = state[2];
            let t2_1 = state[6];
            state[2] = state[10];
            state[6] = state[14];
            state[10] = t2_0;
            state[14] = t2_1;

            let t3 = state[15];
            state[15] = state[11];
            state[11] = state[7];
            state[7] = state[3];
            state[3] = t3;

            // MixColumns (skip on final round 10)
            if r < 10 {
                for c in 0..4 {
                    let i = c * 4;
                    let a0 = state[i];
                    let a1 = state[i + 1];
                    let a2 = state[i + 2];
                    let a3 = state[i + 3];
                    let t = a0 ^ a1 ^ a2 ^ a3;
                    state[i] ^= t ^ xtime(a0 ^ a1);
                    state[i + 1] ^= t ^ xtime(a1 ^ a2);
                    state[i + 2] ^= t ^ xtime(a2 ^ a3);
                    state[i + 3] ^= t ^ xtime(a3 ^ a0);
                }
            }

            // AddRoundKey
            for i in 0..16 {
                state[i] ^= round_keys[r][i];
            }
        }

        state
    }
}

/// GF(2^128) multiplication for NIST SP 800-38D GHASH.
fn ghash_mul(x: u128, y: u128) -> u128 {
    let mut z: u128 = 0;
    let mut v: u128 = y;
    for i in 0..128 {
        let bit = (x >> (127 - i)) & 1;
        if bit == 1 {
            z ^= v;
        }
        let lsb = v & 1;
        v >>= 1;
        if lsb == 1 {
            v ^= 0xE100_0000_0000_0000_0000_0000_0000_0000u128;
        }
    }
    z
}

/// Compute GHASH over an 8-byte AAD header and 12-byte ciphertext payload.
fn ghash_packet(h_subkey: &[u8; 16], aad_header: &[u8; 8], ciphertext_12: &[u8; 12]) -> [u8; 16] {
    let h = u128::from_be_bytes(*h_subkey);
    let mut y: u128 = 0;

    // Block 1: 8-byte AAD padded with zeros to 16 bytes
    let mut aad_block = [0u8; 16];
    aad_block[..8].copy_from_slice(aad_header);
    y = ghash_mul(y ^ u128::from_be_bytes(aad_block), h);

    // Block 2: 12-byte Ciphertext padded with zeros to 16 bytes
    let mut ct_block = [0u8; 16];
    ct_block[..12].copy_from_slice(ciphertext_12);
    y = ghash_mul(y ^ u128::from_be_bytes(ct_block), h);

    // Block 3: Lengths in bits: len(AAD) = 64 bits, len(C) = 96 bits
    let mut len_block = [0u8; 16];
    len_block[..8].copy_from_slice(&64u64.to_be_bytes());
    len_block[8..16].copy_from_slice(&96u64.to_be_bytes());
    y = ghash_mul(y ^ u128::from_be_bytes(len_block), h);

    y.to_be_bytes()
}

/// Derive a unique per-device 128-bit AES key from the Fleet Master Key, the actuator's
/// 6-byte hardware MAC address, and its 2-byte `target_id`.
///
/// Ensures that physically stealing and extracting flash from one actuator does not expose
/// the Fleet Master Key or any sibling actuator's unicast key.
pub fn derive_device_key(
    master_key: &[u8; 16],
    device_mac: &[u8; 6],
    target_id: u16,
    engine: &impl CryptoEngine,
) -> [u8; 16] {
    let tid = target_id.to_le_bytes();
    let info_block = [
        b'T',
        b'L',
        b'K',
        b'N',
        device_mac[0],
        device_mac[1],
        device_mac[2],
        device_mac[3],
        device_mac[4],
        device_mac[5],
        tid[0],
        tid[1],
        0x01,
        0x02,
        0x03,
        0x04,
    ];
    let intermediate = engine.aes128_encrypt_block(master_key, &info_block);
    let mut tweak = info_block;
    for i in 0..16 {
        tweak[i] ^= intermediate[i] ^ 0x5A;
    }
    engine.aes128_encrypt_block(master_key, &tweak)
}

/// Derive the shared fleet broadcast key from the Fleet Master Key for `0xFFFF` and `0xFFFE` frames.
pub fn derive_broadcast_key(master_key: &[u8; 16], engine: &impl CryptoEngine) -> [u8; 16] {
    derive_device_key(master_key, &[0xFF; 6], BROADCAST_TARGET_ID, engine)
}

/// Construct the deterministic 96-bit (12-byte) AES-GCM nonce from header metadata.
#[inline]
const fn build_gcm_nonce(target_id: u16, seq_nonce: u32, flags: PacketFlags, opcode: u8) -> [u8; 12] {
    let tid = target_id.to_le_bytes();
    let seq = seq_nonce.to_le_bytes();
    [
        b'T',
        b'K',
        tid[0],
        tid[1],
        seq[0],
        seq[1],
        seq[2],
        seq[3],
        flags.0,
        opcode,
        seq[0] ^ tid[0],
        seq[1] ^ tid[1],
    ]
}

/// Per-actuator security and monotonic sequence deduplication context.
#[derive(Debug, Clone)]
pub struct DeviceSecurityContext<E: CryptoEngine = Aes128GcmSoftware> {
    node_id: u16,
    unicast_key: [u8; 16],
    broadcast_key: [u8; 16],
    last_unicast_seq: u32,
    last_broadcast_seq: u32,
    engine: E,
}

impl DeviceSecurityContext<Aes128GcmSoftware> {
    /// Create a new security context using the built-in `Aes128GcmSoftware` engine.
    pub fn new(node_id: u16, master_key: &[u8; 16], device_mac: &[u8; 6]) -> Self {
        let engine = Aes128GcmSoftware;
        let unicast_key = derive_device_key(master_key, device_mac, node_id, &engine);
        let broadcast_key = derive_broadcast_key(master_key, &engine);
        Self {
            node_id,
            unicast_key,
            broadcast_key,
            last_unicast_seq: 0,
            last_broadcast_seq: 0,
            engine,
        }
    }
}

impl<E: CryptoEngine> DeviceSecurityContext<E> {
    /// Construct a security context with a custom hardware crypto engine (e.g. ESP32-S3 AES peripheral).
    pub fn with_engine(
        node_id: u16,
        master_key: &[u8; 16],
        device_mac: &[u8; 6],
        engine: E,
    ) -> Self {
        let unicast_key = derive_device_key(master_key, device_mac, node_id, &engine);
        let broadcast_key = derive_broadcast_key(master_key, &engine);
        Self {
            node_id,
            unicast_key,
            broadcast_key,
            last_unicast_seq: 0,
            last_broadcast_seq: 0,
            engine,
        }
    }

    pub const fn node_id(&self) -> u16 {
        self.node_id
    }

    pub const fn last_unicast_seq(&self) -> u32 {
        self.last_unicast_seq
    }

    pub const fn last_broadcast_seq(&self) -> u32 {
        self.last_broadcast_seq
    }

    /// Seal (`encrypt + authenticate`) a `WirePacket` into a 32-byte `EncryptedFrame`.
    pub fn seal_packet(&self, packet: &WirePacket) -> EncryptedFrame {
        let key = if packet.target_id == BROADCAST_TARGET_ID
            || packet.target_id == MASTER_CHANNEL_TARGET_ID
        {
            &self.broadcast_key
        } else {
            &self.unicast_key
        };
        seal_with_key(&self.engine, key, packet)
    }

    /// **First-Valid-Wins Deduplication + AES-128-GCM Authentication & Decryption.**
    ///
    /// Enforces:
    /// 1. `target_id` matches `self.node_id`, `BROADCAST_TARGET_ID`, or `MASTER_CHANNEL_TARGET_ID`.
    /// 2. `incoming_seq > last_executed_seq` (checked *before* decryption to drop twin-cast
    ///    duplicates and Triple-Burst repeats in nanoseconds, and only committed *after*
    ///    the 96-bit AES-GCM tag succeeds so spoofed packets cannot advance the sequence counter).
    /// 3. Constant-time 96-bit AES-128-GCM tag verification.
    pub fn open_and_verify(
        &mut self,
        frame: &EncryptedFrame,
        received_via: RadioLink,
    ) -> Result<VerifiedCommand, SecurityError> {
        let target_id = frame.peek_target_id();
        let is_broadcast =
            target_id == BROADCAST_TARGET_ID || target_id == MASTER_CHANNEL_TARGET_ID;

        if !is_broadcast && target_id != self.node_id {
            return Err(SecurityError::TargetIdMismatch {
                expected: self.node_id,
                actual: target_id,
            });
        }

        let incoming_seq = frame.peek_seq_nonce();
        let last_seq = if is_broadcast {
            self.last_broadcast_seq
        } else {
            self.last_unicast_seq
        };

        // Monotonic "First-Valid-Wins" check: drop if incoming_seq <= last_seq
        if incoming_seq <= last_seq {
            return Err(SecurityError::StaleOrDuplicateSequence {
                last_seq,
                incoming_seq,
            });
        }

        let key = if is_broadcast {
            &self.broadcast_key
        } else {
            &self.unicast_key
        };

        let packet = open_with_key(&self.engine, key, frame)?;

        // Commit sequence number only after cryptographic authentication succeeds!
        if is_broadcast {
            self.last_broadcast_seq = incoming_seq;
        } else {
            self.last_unicast_seq = incoming_seq;
        }

        Ok(VerifiedCommand {
            packet,
            received_via,
        })
    }
}

/// Seal a `WirePacket` with an explicit 128-bit key into a 32-byte `EncryptedFrame`.
pub fn seal_with_key(
    engine: &impl CryptoEngine,
    key: &[u8; 16],
    packet: &WirePacket,
) -> EncryptedFrame {
    let (header, body) = packet.to_header_and_body();
    let nonce = build_gcm_nonce(
        packet.target_id,
        packet.seq_nonce,
        packet.flags,
        header[7],
    );

    // Hash subkey H = AES_K(0^128)
    let h_subkey = engine.aes128_encrypt_block(key, &[0u8; 16]);

    // J0 = Nonce (96 bits) || 0x00000001
    let mut j0 = [0u8; 16];
    j0[..12].copy_from_slice(&nonce);
    j0[15] = 1;

    // Counter block 1 for payload encryption: inc32(J0) -> counter = 2
    let mut cb1 = j0;
    cb1[15] = 2;
    let keystream = engine.aes128_encrypt_block(key, &cb1);

    let mut ciphertext = [0u8; 12];
    for i in 0..12 {
        ciphertext[i] = body[i] ^ keystream[i];
    }

    // Compute GHASH(H, AAD=header, C=ciphertext)
    let s = ghash_packet(&h_subkey, &header, &ciphertext);
    let e_j0 = engine.aes128_encrypt_block(key, &j0);

    let mut out = [0u8; WIRE_PACKET_SIZE];
    out[..8].copy_from_slice(&header);
    out[8..20].copy_from_slice(&ciphertext);
    for i in 0..12 {
        out[20 + i] = s[i] ^ e_j0[i];
    }

    EncryptedFrame(out)
}

/// Authenticate and decrypt a 32-byte `EncryptedFrame` using an explicit 128-bit key.
pub fn open_with_key(
    engine: &impl CryptoEngine,
    key: &[u8; 16],
    frame: &EncryptedFrame,
) -> Result<WirePacket, SecurityError> {
    let mut header = [0u8; 8];
    header.copy_from_slice(&frame.0[..8]);
    let mut ciphertext = [0u8; 12];
    ciphertext.copy_from_slice(&frame.0[8..20]);
    let received_tag = &frame.0[20..32];

    let target_id = u16::from_le_bytes([header[0], header[1]]);
    let seq_nonce = u32::from_le_bytes([header[2], header[3], header[4], header[5]]);
    let flags = PacketFlags(header[6]);
    let opcode = header[7];

    let nonce = build_gcm_nonce(target_id, seq_nonce, flags, opcode);
    let h_subkey = engine.aes128_encrypt_block(key, &[0u8; 16]);

    let mut j0 = [0u8; 16];
    j0[..12].copy_from_slice(&nonce);
    j0[15] = 1;

    let s = ghash_packet(&h_subkey, &header, &ciphertext);
    let e_j0 = engine.aes128_encrypt_block(key, &j0);

    // Constant-time 96-bit tag comparison
    let mut diff: u8 = 0;
    for i in 0..12 {
        let expected_tag_byte = s[i] ^ e_j0[i];
        diff |= expected_tag_byte ^ received_tag[i];
    }
    if diff != 0 {
        return Err(SecurityError::AuthenticationTagMismatch);
    }

    // Tag verified; decrypt 12-byte body
    let mut cb1 = j0;
    cb1[15] = 2;
    let keystream = engine.aes128_encrypt_block(key, &cb1);
    let mut plaintext_body = [0u8; 12];
    for i in 0..12 {
        plaintext_body[i] = ciphertext[i] ^ keystream[i];
    }

    WirePacket::from_header_and_body(&header, &plaintext_body)
        .ok_or(SecurityError::MalformedOpcode)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::packet::{BoundedIntent, Priority, RedundancyMode};

    #[test]
    fn wire_packet_is_exact_32_bytes_and_roundtrips() {
        let master_key = [0x42u8; 16];
        let mac = [0x24, 0x0A, 0xC4, 0x11, 0x22, 0x33];
        let mut ctx = DeviceSecurityContext::new(101, &master_key, &mac);

        let pkt = WirePacket {
            target_id: 101,
            seq_nonce: 1,
            flags: PacketFlags::new(
                Priority::CriticalVo,
                RedundancyMode::HotHotTwinCast,
                true,
                false,
                false,
            ),
            exec_at_tick_ms: 5000,
            intent: BoundedIntent::move_to_deg(45.0, 20, 50, 250),
        };

        let frame = ctx.seal_packet(&pkt);
        assert_eq!(core::mem::size_of_val(&frame), 32);

        // First packet on ESP-NOW succeeds
        let verified = ctx
            .open_and_verify(&frame, RadioLink::EspNow24Ghz)
            .expect("valid packet should verify");
        assert_eq!(verified.packet, pkt);

        // Twin-cast duplicate arriving 50 ms later on LoRa with same seq_nonce is rejected!
        let dup_err = ctx.open_and_verify(&frame, RadioLink::RawLoRa915Mhz);
        assert_eq!(
            dup_err,
            Err(SecurityError::StaleOrDuplicateSequence {
                last_seq: 1,
                incoming_seq: 1
            })
        );
    }

    #[test]
    fn tampered_packet_is_rejected_without_advancing_sequence_nonce() {
        let master_key = [0x99u8; 16];
        let mac = [0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF];
        let mut ctx = DeviceSecurityContext::new(7, &master_key, &mac);

        let pkt = WirePacket {
            target_id: 7,
            seq_nonce: 10,
            flags: PacketFlags::new(
                Priority::VipEmergency,
                RedundancyMode::HotHotTwinCast,
                false,
                false,
                false,
            ),
            exec_at_tick_ms: 0,
            intent: BoundedIntent::EmergencyStop,
        };

        let mut tampered = ctx.seal_packet(&pkt);
        tampered.0[12] ^= 0x01; // Flip 1 bit in encrypted body

        assert_eq!(
            ctx.open_and_verify(&tampered, RadioLink::EspNow24Ghz),
            Err(SecurityError::AuthenticationTagMismatch)
        );
        // Sequence counter must still be 0 so the real packet #10 can still execute
        assert_eq!(ctx.last_unicast_seq(), 0);

        let valid = ctx.seal_packet(&pkt);
        assert!(ctx.open_and_verify(&valid, RadioLink::RawLoRa915Mhz).is_ok());
        assert_eq!(ctx.last_unicast_seq(), 10);
    }
}
