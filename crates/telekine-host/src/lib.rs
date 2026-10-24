//! # `telekine-host`
//!
//! Host & AI Hub orchestration engine scaling to **1,000+ actuators** and **100+ hubs**,
//! featuring **Demo 1 Foundational Unit Discovery & Dynamic Access Lease** for
//! **`Servo`**, **`Stepper`**, and **`DcMotor`** units:
//! - **Master Channel 1 Auto-Discovery (`broadcast_discovery_probe` & `ingest_discovery_announce`):**
//!   Discovers nearby compatible `Servo`, `Stepper`, and `DcMotor` units and tracks their
//!   `UnitAccessState` (`IdleAvailable`, `ClaimedByHub`, `Faulted`).
//! - **On-Demand Unit Access Claiming (`claim_unit_access` & `release_unit_access`):**
//!   Claims exclusive control of a discovered unit only when the AI Agent needs it, assigning
//!   its Worker Channel (`6` or `11`) and TDMA slot, and releases it back to `IdleAvailable` when done.
//! - **Hot-Hot (Twin-Cast) & Hot-Warm (3 ms Auto-Failover):** Dual-radio redundancy across
//!   2.4 GHz ESP-NOW and 915 MHz Semtech SX1262 Raw LoRa.

use std::collections::HashMap;
use telekine_firmware::{Sx1262LoRaConfig, WmmAccessCategory};
use telekine_proto::{
    derive_broadcast_key, open_with_key, seal_with_key, Aes128GcmSoftware, BoundedIntent,
    DeviceSecurityContext, EncryptedFrame, PacketFlags, Priority, RadioLink, RedundancyMode,
    SecurityError, TelemetryExceptionCode, UnitAccessState, UnitElementKind, WirePacket,
    BROADCAST_TARGET_ID, MASTER_CHANNEL_TARGET_ID,
};

/// Default hardware ACK timeout in milliseconds before Hot-Warm mode fails over to 915 MHz LoRa.
pub const HOT_WARM_ACK_TIMEOUT_MS: u32 = 3;

/// Outgoing physical radio transmission descriptor scheduled by the Hub.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ScheduledTransmission {
    /// Physical radio link (`EspNow24Ghz` or `RawLoRa915Mhz`).
    pub link: RadioLink,
    /// Destination MAC (`[0xFF; 6]` for connectionless broadcast).
    pub dest_mac: [u8; 6],
    /// 2.4 GHz ESP-NOW channel (`1` = Master Manager, `6`/`11` = Worker) or `0` for LoRa.
    pub espnow_channel: u8,
    /// Hardware WMM / EDCA Access Category (`AC_VO` Voice priority vs `AC_BK`).
    pub wmm_ac: WmmAccessCategory,
    /// Semtech SX1262 LoRa modulation & frequency profile (when `link == RawLoRa915Mhz`).
    pub lora_profile: Sx1262LoRaConfig,
    /// Transmit power in dBm (`+10 dBm` nominal for spatial reuse, `+20..+22 dBm` for Capture Effect).
    pub tx_power_dbm: i8,
    /// 32-byte AES-128-GCM encrypted and authenticated wire frame.
    pub frame: EncryptedFrame,
}

/// Metadata for a Foundational Unit (`Servo`, `Stepper`, or `DcMotor`) discovered by the AI Hub.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DiscoveredUnitInfo {
    pub target_id: u16,
    pub element_kind: UnitElementKind,
    pub access_state: UnitAccessState,
    pub claimed_by_hub_id: u8,
    pub max_travel_deg: f32,
}

/// Registered actuator entry in the Hub's 1,000+ peer directory.
#[derive(Debug, Clone)]
pub struct RegisteredPeer {
    pub target_id: u16,
    pub mac_address: [u8; 6],
    pub worker_channel: u8,
    pub lora_freq_offset_khz: u8,
    pub tdma_slot_index: u16,
    pub next_seq_nonce: u32,
    pub security: DeviceSecurityContext<Aes128GcmSoftware>,
    pub discovered_info: Option<DiscoveredUnitInfo>,
    pub last_reported_exception: Option<(u32, TelemetryExceptionCode, f32)>,
}

/// In-flight ESP-NOW unicast waiting for a 3 ms hardware ACK before failing over to Raw LoRa.
#[derive(Debug, Clone)]
pub struct PendingHotWarmFailover {
    pub target_id: u16,
    pub seq_nonce: u32,
    pub sent_at_ms: u32,
    pub deadline_ms: u32,
    pub fallback_lora_tx: ScheduledTransmission,
}

/// Central AI Swarm Hub Coordinator managing 1,000+ dual-radio actuators and Foundational Units.
pub struct SwarmHubCoordinator {
    hub_id: u8,
    master_key: [u8; 16],
    broadcast_key: [u8; 16],
    broadcast_seq_nonce: u32,
    worker_channel_default: u8,
    peers: HashMap<u16, RegisteredPeer>,
    pending_failovers: Vec<PendingHotWarmFailover>,
}

impl SwarmHubCoordinator {
    /// Create a new `SwarmHubCoordinator` with a 128-bit Fleet Master Key.
    pub fn new(hub_id: u8, master_key: [u8; 16], worker_channel_default: u8) -> Self {
        let broadcast_key = derive_broadcast_key(&master_key, &Aes128GcmSoftware);
        Self {
            hub_id,
            master_key,
            broadcast_key,
            broadcast_seq_nonce: 0,
            worker_channel_default,
            peers: HashMap::with_capacity(1024),
            pending_failovers: Vec::new(),
        }
    }

    pub const fn hub_id(&self) -> u8 {
        self.hub_id
    }

    pub fn peer_count(&self) -> usize {
        self.peers.len()
    }

    /// Register an actuator node (`0..=65533`) and derive its unique per-device AES-128-GCM key.
    pub fn register_actuator(&mut self, target_id: u16, mac_address: [u8; 6]) {
        let worker_channel = if target_id % 2 == 0 {
            self.worker_channel_default
        } else {
            11
        };
        let security = DeviceSecurityContext::new(target_id, &self.master_key, &mac_address);
        self.peers.insert(
            target_id,
            RegisteredPeer {
                target_id,
                mac_address,
                worker_channel,
                lora_freq_offset_khz: (self.hub_id % 8),
                tdma_slot_index: target_id % 1000,
                next_seq_nonce: 0,
                security,
                discovered_info: None,
                last_reported_exception: None,
            },
        );
    }

    /// **Step 1 (Discovery):** Broadcast a `DiscoverUnitsProbe` on Master Manager Channel 1
    /// (`target_id = 0xFFFE`) to discover compatible `Servo`, `Stepper`, or `DcMotor` units.
    /// Pass `UnitElementKind::Unknown` to discover all unit types at once.
    pub fn broadcast_discovery_probe(
        &mut self,
        filter_kind: UnitElementKind,
    ) -> Vec<ScheduledTransmission> {
        self.broadcast_seq_nonce = self.broadcast_seq_nonce.wrapping_add(1);
        let pkt = WirePacket {
            target_id: MASTER_CHANNEL_TARGET_ID,
            seq_nonce: self.broadcast_seq_nonce,
            flags: PacketFlags::new(
                Priority::HighVi,
                RedundancyMode::HotHotTwinCast,
                false,
                true,
                false,
            ),
            exec_at_tick_ms: 0,
            intent: BoundedIntent::DiscoverUnitsProbe {
                hub_id: self.hub_id,
                filter_kind,
            },
        };
        let frame = seal_with_key(&Aes128GcmSoftware, &self.broadcast_key, &pkt);
        vec![
            ScheduledTransmission {
                link: RadioLink::EspNow24Ghz,
                dest_mac: [0xFF; 6],
                espnow_channel: 1, // Master Manager Channel 1
                wmm_ac: WmmAccessCategory::VideoAcVi,
                lora_profile: Sx1262LoRaConfig::worker_default(0),
                tx_power_dbm: 17,
                frame,
            },
            ScheduledTransmission {
                link: RadioLink::RawLoRa915Mhz,
                dest_mac: [0xFF; 6],
                espnow_channel: 0,
                wmm_ac: WmmAccessCategory::VideoAcVi,
                lora_profile: Sx1262LoRaConfig::worker_default(0),
                tx_power_dbm: 18,
                frame,
            },
        ]
    }

    /// Authenticate and ingest a `UnitDiscoveryAnnounce` reply frame sent by a Foundational Unit,
    /// updating the Hub's discovery directory.
    pub fn ingest_discovery_announce(
        &mut self,
        frame: &EncryptedFrame,
    ) -> Result<DiscoveredUnitInfo, SecurityError> {
        let pkt = open_with_key(&Aes128GcmSoftware, &self.broadcast_key, frame)?;
        let node_id = pkt.exec_at_tick_ms as u16;
        match pkt.intent {
            BoundedIntent::UnitDiscoveryAnnounce {
                element_kind,
                access_state,
                claimed_by_hub_id,
                max_travel_q16,
            } => {
                let info = DiscoveredUnitInfo {
                    target_id: node_id,
                    element_kind,
                    access_state,
                    claimed_by_hub_id,
                    max_travel_deg: (max_travel_q16 as f32) / 65536.0,
                };
                if let Some(peer) = self.peers.get_mut(&node_id) {
                    peer.discovered_info = Some(info);
                }
                Ok(info)
            }
            _ => Err(SecurityError::MalformedOpcode),
        }
    }

    /// List all discovered Foundational Units (`Servo`, `Stepper`, `DcMotor`), optionally filtered by kind.
    pub fn list_discovered_units(
        &self,
        filter_kind: Option<UnitElementKind>,
    ) -> Vec<DiscoveredUnitInfo> {
        let mut out: Vec<DiscoveredUnitInfo> = self
            .peers
            .values()
            .filter_map(|p| p.discovered_info)
            .filter(|info| filter_kind.map(|k| info.element_kind == k).unwrap_or(true))
            .collect();
        out.sort_by_key(|i| i.target_id);
        out
    }

    /// **Step 2 (Gain Access When Needed):** Claim exclusive control of a discovered unit
    /// (`Servo`, `Stepper`, or `DcMotor`) for `access_lease_ms` and assign it to this Hub's
    /// Worker Channel and TDMA slot.
    pub fn claim_unit_access(
        &mut self,
        now_ms: u32,
        target_id: u16,
        access_lease_ms: u16,
    ) -> Result<Vec<ScheduledTransmission>, &'static str> {
        let peer = self
            .peers
            .get(&target_id)
            .ok_or("target_id not registered in Hub peer table")?;
        let worker_channel = peer.worker_channel;
        let tdma_slot_index = peer.tdma_slot_index;

        let intent = BoundedIntent::ClaimUnitAccess {
            hub_id: self.hub_id,
            espnow_worker_channel: worker_channel,
            tdma_slot_index,
            access_lease_ms,
        };
        self.dispatch_unicast(
            now_ms,
            target_id,
            intent,
            Priority::CriticalVo,
            RedundancyMode::HotHotTwinCast,
            None,
        )
    }

    /// **Step 4 (Release Access):** Release a claimed unit back to `IdleAvailable` on Master Channel 1.
    pub fn release_unit_access(
        &mut self,
        now_ms: u32,
        target_id: u16,
    ) -> Result<Vec<ScheduledTransmission>, &'static str> {
        let intent = BoundedIntent::ReleaseUnitAccess {
            hub_id: self.hub_id,
        };
        self.dispatch_unicast(
            now_ms,
            target_id,
            intent,
            Priority::CriticalVo,
            RedundancyMode::HotHotTwinCast,
            None,
        )
    }

    /// Dispatch a **Bounded Intent** to a single actuator using either:
    /// - `RedundancyMode::HotHotTwinCast` (simultaneous ESP-NOW + LoRa)
    /// - `RedundancyMode::HotWarmFailover` (ESP-NOW immediately + 3 ms LoRa failover timer)
    pub fn dispatch_unicast(
        &mut self,
        now_ms: u32,
        target_id: u16,
        intent: BoundedIntent,
        priority: Priority,
        redundancy: RedundancyMode,
        exec_at_tick_ms: Option<u32>,
    ) -> Result<Vec<ScheduledTransmission>, &'static str> {
        let peer = self
            .peers
            .get_mut(&target_id)
            .ok_or("target_id not registered in Hub peer table")?;

        peer.next_seq_nonce = peer.next_seq_nonce.wrapping_add(1);
        let seq_nonce = peer.next_seq_nonce;

        let sync_and_fire = exec_at_tick_ms.is_some();
        let pkt = WirePacket {
            target_id,
            seq_nonce,
            flags: PacketFlags::new(priority, redundancy, sync_and_fire, false, false),
            exec_at_tick_ms: exec_at_tick_ms.unwrap_or(0),
            intent,
        };

        let frame = peer.security.seal_packet(&pkt);
        let esp_tx = ScheduledTransmission {
            link: RadioLink::EspNow24Ghz,
            dest_mac: peer.mac_address,
            espnow_channel: peer.worker_channel,
            wmm_ac: WmmAccessCategory::from(priority),
            lora_profile: Sx1262LoRaConfig::worker_default(peer.lora_freq_offset_khz),
            tx_power_dbm: priority.recommended_tx_power_dbm(RadioLink::EspNow24Ghz),
            frame,
        };

        let lora_profile = if priority >= Priority::CriticalVo {
            Sx1262LoRaConfig::vip_emergency_profile()
        } else {
            Sx1262LoRaConfig::worker_default(peer.lora_freq_offset_khz)
        };
        let lora_tx = ScheduledTransmission {
            link: RadioLink::RawLoRa915Mhz,
            dest_mac: [0xFF; 6],
            espnow_channel: 0,
            wmm_ac: WmmAccessCategory::from(priority),
            lora_profile,
            tx_power_dbm: priority.recommended_tx_power_dbm(RadioLink::RawLoRa915Mhz),
            frame,
        };

        match redundancy {
            RedundancyMode::HotHotTwinCast => Ok(vec![esp_tx, lora_tx]),
            RedundancyMode::HotWarmFailover => {
                self.pending_failovers.push(PendingHotWarmFailover {
                    target_id,
                    seq_nonce,
                    sent_at_ms: now_ms,
                    deadline_ms: now_ms.saturating_add(HOT_WARM_ACK_TIMEOUT_MS),
                    fallback_lora_tx: lora_tx,
                });
                Ok(vec![esp_tx])
            }
            RedundancyMode::TripleBurstBroadcast => {
                Ok(vec![esp_tx.clone(), esp_tx.clone(), esp_tx])
            }
        }
    }

    /// Record a hardware `ACK` from ESP-NOW, cancelling the pending 3 ms Hot-Warm LoRa failover.
    pub fn notify_espnow_ack(&mut self, target_id: u16, seq_nonce: u32) -> bool {
        let before = self.pending_failovers.len();
        self.pending_failovers
            .retain(|entry| !(entry.target_id == target_id && entry.seq_nonce == seq_nonce));
        self.pending_failovers.len() < before
    }

    /// Poll the 3 ms Hot-Warm failover timers at `now_ms` and return any Raw LoRa failover
    /// transmissions whose ESP-NOW hardware `ACK` missed the 3 ms deadline.
    pub fn poll_hot_warm_failovers(&mut self, now_ms: u32) -> Vec<ScheduledTransmission> {
        let mut triggered = Vec::new();
        let mut remaining = Vec::new();

        for item in self.pending_failovers.drain(..) {
            if now_ms >= item.deadline_ms {
                triggered.push(item.fallback_lora_tx);
            } else {
                remaining.push(item);
            }
        }
        self.pending_failovers = remaining;
        triggered
    }

    /// **Triple-Burst Swarm Broadcast (`target_id = 0xFFFF`)**
    pub fn dispatch_swarm_broadcast(
        &mut self,
        intent: BoundedIntent,
        priority: Priority,
        exec_at_tick_ms: Option<u32>,
        include_lora_twincast: bool,
    ) -> Vec<ScheduledTransmission> {
        self.broadcast_seq_nonce = self.broadcast_seq_nonce.wrapping_add(1);
        let seq_nonce = self.broadcast_seq_nonce;

        let pkt = WirePacket {
            target_id: BROADCAST_TARGET_ID,
            seq_nonce,
            flags: PacketFlags::new(
                priority,
                RedundancyMode::TripleBurstBroadcast,
                exec_at_tick_ms.is_some(),
                false,
                false,
            ),
            exec_at_tick_ms: exec_at_tick_ms.unwrap_or(0),
            intent,
        };

        let frame = seal_with_key(&Aes128GcmSoftware, &self.broadcast_key, &pkt);
        let esp_burst = ScheduledTransmission {
            link: RadioLink::EspNow24Ghz,
            dest_mac: [0xFF; 6],
            espnow_channel: self.worker_channel_default,
            wmm_ac: WmmAccessCategory::from(priority),
            lora_profile: Sx1262LoRaConfig::worker_default(0),
            tx_power_dbm: priority.recommended_tx_power_dbm(RadioLink::EspNow24Ghz),
            frame,
        };

        let mut txs = vec![esp_burst.clone(), esp_burst.clone(), esp_burst];

        if include_lora_twincast || priority == Priority::VipEmergency {
            txs.push(ScheduledTransmission {
                link: RadioLink::RawLoRa915Mhz,
                dest_mac: [0xFF; 6],
                espnow_channel: 0,
                wmm_ac: WmmAccessCategory::VoiceAcVo,
                lora_profile: Sx1262LoRaConfig::vip_emergency_profile(),
                tx_power_dbm: Priority::VipEmergency
                    .recommended_tx_power_dbm(RadioLink::RawLoRa915Mhz),
                frame,
            });
        }

        txs
    }

    /// Emit a periodic **Master Manager Channel** clock-synchronization beacon on ESP-NOW Channel 1
    /// and LoRa 915.0 MHz so all 1,000+ peers maintain sub-millisecond TDMA alignment.
    pub fn emit_master_clock_beacon(
        &mut self,
        master_tick_ms: u32,
        beacon_interval_ms: u16,
    ) -> Vec<ScheduledTransmission> {
        self.broadcast_seq_nonce = self.broadcast_seq_nonce.wrapping_add(1);
        let pkt = WirePacket {
            target_id: MASTER_CHANNEL_TARGET_ID,
            seq_nonce: self.broadcast_seq_nonce,
            flags: PacketFlags::new(
                Priority::CriticalVo,
                RedundancyMode::HotHotTwinCast,
                false,
                true,
                false,
            ),
            exec_at_tick_ms: master_tick_ms,
            intent: BoundedIntent::SyncClockBeacon {
                master_tick_ms,
                beacon_interval_ms,
            },
        };

        let frame = seal_with_key(&Aes128GcmSoftware, &self.broadcast_key, &pkt);
        vec![
            ScheduledTransmission {
                link: RadioLink::EspNow24Ghz,
                dest_mac: [0xFF; 6],
                espnow_channel: 1,
                wmm_ac: WmmAccessCategory::VoiceAcVo,
                lora_profile: Sx1262LoRaConfig::worker_default(0),
                tx_power_dbm: 20,
                frame,
            },
            ScheduledTransmission {
                link: RadioLink::RawLoRa915Mhz,
                dest_mac: [0xFF; 6],
                espnow_channel: 0,
                wmm_ac: WmmAccessCategory::VoiceAcVo,
                lora_profile: Sx1262LoRaConfig::worker_default(0),
                tx_power_dbm: 20,
                frame,
            },
        ]
    }
}
