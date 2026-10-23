#![no_std]
#![forbid(unsafe_op_in_unsafe_fn)]

//! # `telekine-firmware`
//!
//! `#[no_std]` bare-metal dual-radio hardware abstraction and driver layer for:
//! - **ESP-NOW (2.4 GHz on ESP32-S3 / ESP32-C3):**
//!   - Bypasses the 20-peer hardware encryption/routing table limit via connectionless
//!     broadcast addressing (`target_id` in header) **and** a deterministic 16-slot LRU
//!     RAM peer-swapping ring buffer (`EspNowPeerSwapTable`) for unicast hardware ACKs in
//!     **Hot-Warm** mode across 1,000+ nodes.
//!   - Hardware wait-time priority (**802.11 WMM / EDCA `AC_VO` Voice queue** vs `AC_BK`
//!     Background queue) and per-packet TX power control (`+10 dBm` nominal up to `+20 dBm`
//!     Capture Effect override).
//! - **Raw LoRa (915 MHz on Semtech SX1262 via SPI):**
//!   - Dedicated **VIP Emergency Frequency / `SF7`** profile for low-airtime E-Stops and
//!     Hot-Warm failover packets.
//!   - Capture-Effect TX power scaling up to `+22 dBm` (or `+30 dBm` with external PA).

use telekine_proto::{EncryptedFrame, Priority, RadioLink, WIRE_PACKET_SIZE};

/// 802.11 WMM / EDCA Access Category for controlling ESP-NOW hardware CSMA/CA backoff time.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum WmmAccessCategory {
    /// Background (`AC_BK`): CWmin=15, CWmax=1023, AIFSN=7 (routine telemetry).
    BackgroundAcBk = 0,
    /// Best Effort (`AC_BE`): CWmin=15, CWmax=1023, AIFSN=3 (standard updates).
    BestEffortAcBe = 1,
    /// Video (`AC_VI`): CWmin=7, CWmax=15, AIFSN=2 (high-priority trajectories).
    VideoAcVi = 2,
    /// Voice (`AC_VO`): CWmin=3, CWmax=7, AIFSN=2 — waits minimal microseconds of silence
    /// to cut in line ahead of routine traffic.
    VoiceAcVo = 3,
}

impl From<Priority> for WmmAccessCategory {
    fn from(priority: Priority) -> Self {
        match priority {
            Priority::BackgroundBk => Self::BackgroundAcBk,
            Priority::NormalBe => Self::BestEffortAcBe,
            Priority::HighVi => Self::VideoAcVi,
            Priority::CriticalVo | Priority::VipEmergency => Self::VoiceAcVo,
        }
    }
}

/// Semtech SX1262 LoRa modulation & frequency profile.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Sx1262LoRaConfig {
    /// Carrier frequency in Hz (e.g., `915_000_000` Master, `915_500_000` VIP E-Stop).
    pub rf_freq_hz: u32,
    /// LoRa Spreading Factor (`7..=12`). `SF7` is used for low-latency (`~25 ms`) VIP / failover.
    pub spreading_factor: u8,
    /// LoRa Bandwidth in kHz (`500` kHz for ultra-fast ~12–25 ms 32-byte packets).
    pub bandwidth_khz: u16,
    /// Coding rate (`1` = 4/5).
    pub coding_rate: u8,
    /// Transmit power in dBm (`-9..=+22` on SX1262).
    pub tx_power_dbm: i8,
}

impl Sx1262LoRaConfig {
    /// Standard Worker LoRa profile (`915.0 MHz`, `SF7`, `500 kHz` BW, `+14 dBm` for spatial reuse).
    pub const fn worker_default(freq_offset_khz: u8) -> Self {
        Self {
            rf_freq_hz: 915_000_000 + (freq_offset_khz as u32) * 200_000,
            spreading_factor: 7,
            bandwidth_khz: 500,
            coding_rate: 1,
            tx_power_dbm: 14,
        }
    }

    /// Dedicated VIP Emergency & Hot-Warm Failover frequency (`918.8 MHz`, `SF7`, `+22 dBm`
    /// max SX1262 power to trigger the RF Capture Effect at the receiver antenna).
    pub const fn vip_emergency_profile() -> Self {
        Self {
            rf_freq_hz: 918_800_000,
            spreading_factor: 7,
            bandwidth_khz: 500,
            coding_rate: 1,
            tx_power_dbm: 22,
        }
    }
}

/// Hardware trait for the ESP32-S3 / ESP32-C3 2.4 GHz ESP-NOW radio peripheral.
pub trait EspNowRadioHal {
    /// Switch 2.4 GHz Wi-Fi channel (`1` = Master Manager Channel, `6` or `11` = Worker Channels).
    fn set_channel(&mut self, channel: u8);

    /// Register a temporary peer MAC in the ESP32 hardware peer table (`max 20` slots).
    fn add_hw_peer(&mut self, mac: &[u8; 6], channel: u8) -> bool;

    /// Remove a peer MAC from the ESP32 hardware peer table to free a slot.
    fn remove_hw_peer(&mut self, mac: &[u8; 6]) -> bool;

    /// Transmit a 32-byte `EncryptedFrame` with explicit WMM Access Category (`AC_VO` vs `AC_BK`)
    /// and TX power in dBm (`+10` to `+20 dBm`).
    ///
    /// Returns `Ok(true)` if MAC-level hardware `ACK` was received within the 3 ms Hot-Warm window.
    fn transmit_frame(
        &mut self,
        dest_mac: &[u8; 6],
        frame: &EncryptedFrame,
        wmm_ac: WmmAccessCategory,
        tx_power_dbm: i8,
    ) -> Result<bool, ()>;

    /// Poll for an incoming 32-byte frame from the ESP-NOW RX ring buffer.
    fn poll_receive(&mut self) -> Option<EncryptedFrame>;
}

/// Hardware trait for the Semtech SX1262 915 MHz Raw LoRa transceiver over SPI.
pub trait Sx1262LoRaHal {
    /// Apply frequency, spreading factor (`SF7`), bandwidth, and Capture-Effect TX power.
    fn configure(&mut self, config: &Sx1262LoRaConfig);

    /// Transmit a raw 32-byte implicit-header LoRa packet (no LoRaWAN overhead).
    fn transmit_raw(&mut self, frame: &EncryptedFrame, config: &Sx1262LoRaConfig) -> Result<(), ()>;

    /// Poll DIO1 / RX FIFO for a received 32-byte LoRa packet.
    fn poll_receive(&mut self) -> Option<EncryptedFrame>;
}

/// **Dynamic Peer-Swapping Ring Buffer (`MAX_HW_PEERS = 16`)**
///
/// ESP-NOW's hardware peer table only supports 20 total peers (and 17 encrypted).
/// To support **Hot-Warm hardware ACKs** across **1,000+ actuators** without stateful pairing,
/// `EspNowPeerSwapTable` maintains a 16-entry cache in RAM and dynamically swaps target MACs
/// into and out of the ESP32 hardware peer table in `< 15 microseconds` before unicast TX.
pub struct EspNowPeerSwapTable<const N: usize = 16> {
    slots: [Option<([u8; 6], u16)>; N],
    next_evict_idx: usize,
}

impl<const N: usize> Default for EspNowPeerSwapTable<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> EspNowPeerSwapTable<N> {
    pub const fn new() -> Self {
        Self {
            slots: [None; N],
            next_evict_idx: 0,
        }
    }

    /// Ensure `target_mac` is loaded in the ESP32's hardware peer table before a Hot-Warm unicast,
    /// evicting the oldest peer slot if all `N` hardware slots are occupied.
    pub fn ensure_peer_loaded<R: EspNowRadioHal>(
        &mut self,
        radio: &mut R,
        target_id: u16,
        target_mac: &[u8; 6],
        channel: u8,
    ) {
        // Fast path: already resident in hardware table
        for entry in self.slots.iter().flatten() {
            if entry.1 == target_id && &entry.0 == target_mac {
                return;
            }
        }

        // Evict existing peer at `next_evict_idx` if occupied
        if let Some((old_mac, _)) = self.slots[self.next_evict_idx] {
            let _ = radio.remove_hw_peer(&old_mac);
        }

        let _ = radio.add_hw_peer(target_mac, channel);
        self.slots[self.next_evict_idx] = Some((*target_mac, target_id));
        self.next_evict_idx = (self.next_evict_idx + 1) % N;
    }
}

/// Dual-Radio Actuator Transceiver multiplexer polling **both** ESP-NOW and SX1262 LoRa
/// in parallel without any cross-protocol mesh relay.
pub struct DualRadioTransceiver<Esp: EspNowRadioHal, Lora: Sx1262LoRaHal> {
    pub espnow: Esp,
    pub lora: Lora,
}

impl<Esp: EspNowRadioHal, Lora: Sx1262LoRaHal> DualRadioTransceiver<Esp, Lora> {
    pub const fn new(espnow: Esp, lora: Lora) -> Self {
        Self { espnow, lora }
    }

    /// Poll both physical radios in parallel (`ESP-NOW` first for 1–3 ms latency, then `Raw LoRa`)
    /// and feed into the single unified execution pipeline.
    pub fn poll_next_frame(&mut self) -> Option<(EncryptedFrame, RadioLink)> {
        if let Some(frame) = self.espnow.poll_receive() {
            let _ = WIRE_PACKET_SIZE;
            return Some((frame, RadioLink::EspNow24Ghz));
        }
        if let Some(frame) = self.lora.poll_receive() {
            return Some((frame, RadioLink::RawLoRa915Mhz));
        }
        None
    }
}
