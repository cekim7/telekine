//! 32-Byte ultra-compact binary wire protocol, Foundational Unit Discovery & Access Lease,
//! and Bounded AI Intent structures (`Servo`, `Stepper`, `DcMotor`).

/// Fixed wire size of every Telekine command, discovery, or telemetry packet (32 bytes).
/// Keeps ESP-NOW airtime < 1 ms and Raw LoRa SF7 airtime ~ 25 ms.
pub const WIRE_PACKET_SIZE: usize = 32;

/// Reserved target ID for fleet-wide broadcasts (all 1,000+ nodes).
pub const BROADCAST_TARGET_ID: u16 = 0xFFFF;

/// Reserved target ID for Master Manager Channel discovery, access claims & TDMA clock sync.
pub const MASTER_CHANNEL_TARGET_ID: u16 = 0xFFFE;

/// Physical radio link over which a packet was transmitted or received.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RadioLink {
    /// 2.4 GHz ESP-NOW link (1–3 ms latency, 100–250 m range).
    EspNow24Ghz,
    /// 915 MHz Semtech SX1262 Raw LoRa link (30–80 ms latency, 1–10+ km range).
    RawLoRa915Mhz,
}

/// The three foundational hardware unit elements for Demo 1.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum UnitElementKind {
    /// Unconfigured or generic actuator node.
    Unknown = 0,
    /// **RC / Industrial PWM Servo:** Bounded angle (`-90°..+90°` or `0°..180°`), 50–333 Hz pulse width.
    Servo = 1,
    /// **Bipolar Stepper Motor:** Bounded step/angle position or velocity via 2-coil H-bridge / Step-Dir.
    Stepper = 2,
    /// **Reversible Brushed DC Motor / Linear Actuator:** H-bridge PWM duty + stall-current & encoder feedback.
    DcMotor = 3,
}

impl UnitElementKind {
    pub const fn from_u8(val: u8) -> Self {
        match val {
            1 => Self::Servo,
            2 => Self::Stepper,
            3 => Self::DcMotor,
            _ => Self::Unknown,
        }
    }
}

/// Access & reservation state advertised by a Foundational Unit during discovery.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum UnitAccessState {
    /// Listening on Master Manager Channel 1; available to be claimed by any AI Hub.
    IdleAvailable = 0,
    /// Currently leased to a specific AI Hub (`claimed_by_hub_id`); rejects other hubs until released or lease expires.
    ClaimedByHub = 1,
    /// Hardware fault / stall / E-Stop latched; requires inspection or reset.
    Faulted = 2,
}

impl UnitAccessState {
    pub const fn from_u8(val: u8) -> Self {
        match val {
            0 => Self::IdleAvailable,
            1 => Self::ClaimedByHub,
            _ => Self::Faulted,
        }
    }
}

/// Hardware wait-time (WMM / EDCA) and RF power / frequency priority level.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
#[repr(u8)]
pub enum Priority {
    /// Routine telemetry (`AC_BK` on ESP-NOW, low TX power +10 dBm for spatial reuse).
    BackgroundBk = 0,
    /// Standard best-effort state updates (`AC_BE`, moderate TX power +14 dBm).
    NormalBe = 1,
    /// High-priority actuator trajectory updates (`AC_VI`, +17 dBm).
    HighVi = 2,
    /// Critical control commands (`AC_VO` Voice queue on ESP-NOW, +20 dBm).
    CriticalVo = 3,
    /// Global E-Stop / High-priority interrupt (Dedicated LoRa `SF7` VIP channel +
    /// maximum +22 to +30 dBm TX power to exploit the RF Capture Effect).
    VipEmergency = 4,
}

impl Priority {
    pub const fn from_u8(val: u8) -> Self {
        match val & 0x07 {
            0 => Self::BackgroundBk,
            1 => Self::NormalBe,
            2 => Self::HighVi,
            3 => Self::CriticalVo,
            _ => Self::VipEmergency,
        }
    }

    /// Recommended transmit power in dBm to balance spatial reuse across 100+ hubs
    /// against the RF Capture Effect for urgent commands.
    pub const fn recommended_tx_power_dbm(self, link: RadioLink) -> i8 {
        match (self, link) {
            (Self::BackgroundBk, RadioLink::EspNow24Ghz) => 10,
            (Self::NormalBe, RadioLink::EspNow24Ghz) => 14,
            (Self::HighVi, RadioLink::EspNow24Ghz) => 17,
            (Self::CriticalVo, RadioLink::EspNow24Ghz) => 20,
            (Self::VipEmergency, RadioLink::EspNow24Ghz) => 20,
            (Self::BackgroundBk, RadioLink::RawLoRa915Mhz) => 10,
            (Self::NormalBe, RadioLink::RawLoRa915Mhz) => 14,
            (Self::HighVi, RadioLink::RawLoRa915Mhz) => 18,
            (Self::CriticalVo, RadioLink::RawLoRa915Mhz) => 22,
            (Self::VipEmergency, RadioLink::RawLoRa915Mhz) => 22,
        }
    }
}

/// Dual-radio redundancy operating mode selected by the Hub.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum RedundancyMode {
    /// **Hot-Hot (Twin-Cast):** Fired over ESP-NOW and Raw LoRa simultaneously;
    /// the actuator executes whichever packet arrives first (`First-Valid-Wins`).
    HotHotTwinCast = 0,
    /// **Hot-Warm (Auto-Failover):** Fired over ESP-NOW first; if hardware `ACK`
    /// fails within ~3 ms, the Hub immediately fires the packet over LoRa.
    HotWarmFailover = 1,
    /// **Triple-Burst Broadcast:** Fired 3 times in rapid succession with the
    /// exact same `seq_nonce` to drive unacknowledged broadcast loss below 0.1%.
    TripleBurstBroadcast = 2,
}

impl RedundancyMode {
    pub const fn from_u8(val: u8) -> Self {
        match val & 0x03 {
            0 => Self::HotHotTwinCast,
            1 => Self::HotWarmFailover,
            _ => Self::TripleBurstBroadcast,
        }
    }
}

/// Bit-packed 1-byte flags & priority field (Byte 6 of `WirePacket`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PacketFlags(pub u8);

impl PacketFlags {
    pub const fn new(
        priority: Priority,
        redundancy: RedundancyMode,
        sync_and_fire: bool,
        master_channel_mgmt: bool,
        uplink_exception: bool,
    ) -> Self {
        let mut b = (priority as u8) & 0x07;
        b |= ((redundancy as u8) & 0x03) << 3;
        if sync_and_fire {
            b |= 1 << 5;
        }
        if master_channel_mgmt {
            b |= 1 << 6;
        }
        if uplink_exception {
            b |= 1 << 7;
        }
        Self(b)
    }

    pub const fn priority(self) -> Priority {
        Priority::from_u8(self.0 & 0x07)
    }

    pub const fn redundancy_mode(self) -> RedundancyMode {
        RedundancyMode::from_u8((self.0 >> 3) & 0x03)
    }

    pub const fn is_sync_and_fire(self) -> bool {
        (self.0 & (1 << 5)) != 0
    }

    pub const fn is_master_channel_mgmt(self) -> bool {
        (self.0 & (1 << 6)) != 0
    }

    pub const fn is_uplink_exception(self) -> bool {
        (self.0 & (1 << 7)) != 0
    }
}

/// Wire opcode defining the bounded intent, discovery probe, access claim, or telemetry operation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum IntentOpcode {
    /// Immediate hardware E-Stop: zero torque, lock brakes, close valves.
    EmergencyStop = 0x00,
    /// Dual-Link Watchdog lease renewal without modifying current trajectory.
    WatchdogHeartbeat = 0x01,
    /// Bounded position goal: move to target angle/position (Q16.16) within speed/torque limits.
    MoveToBounded = 0x02,
    /// Bounded velocity goal: maintain target velocity (Q16.16) for at most `lease_timeout_ms`.
    SetVelocityBounded = 0x03,
    /// Bounded force/pressure/duty goal: regulate to target (Q16.16) with hardware lease.
    SetActuationEffortBounded = 0x04,
    /// Master Manager Channel beacon: synchronizes swarm millisecond tick clock.
    SyncClockBeacon = 0x10,
    /// Master Manager Channel assignment: assigns worker channel (e.g. Ch 6/11) and TDMA slot.
    AssignChannelAndTdmaSlot = 0x11,
    /// **Discovery Probe:** Broadcast by an AI Hub on Master Channel 1 to discover nearby
    /// compatible `Servo`, `Stepper`, and `DcMotor` units.
    DiscoverUnitsProbe = 0x20,
    /// **Discovery Announce:** Sent by a unit in response to `DiscoverUnitsProbe` (or on boot),
    /// advertising its `UnitElementKind` (`Servo`, `Stepper`, `DcMotor`), `UnitAccessState`,
    /// currently owning `hub_id`, and travel range.
    UnitDiscoveryAnnounce = 0x21,
    /// **Claim Unit Access:** Sent by an AI Hub to gain exclusive control of a discovered unit
    /// for `access_lease_ms` and assign its Worker Channel (`6` or `11`) and TDMA slot.
    ClaimUnitAccess = 0x22,
    /// **Release Unit Access:** Sent by an AI Hub when done controlling a unit so it returns to
    /// safe idle on Master Channel 1 and becomes available for other tasks/hubs.
    ReleaseUnitAccess = 0x23,
    /// Report-by-Exception uplink packet sent by an actuator only on stall/fault/deviation.
    TelemetryExceptionReport = 0xFE,
}

impl IntentOpcode {
    pub const fn from_u8(val: u8) -> Option<Self> {
        match val {
            0x00 => Some(Self::EmergencyStop),
            0x01 => Some(Self::WatchdogHeartbeat),
            0x02 => Some(Self::MoveToBounded),
            0x03 => Some(Self::SetVelocityBounded),
            0x04 => Some(Self::SetActuationEffortBounded),
            0x10 => Some(Self::SyncClockBeacon),
            0x11 => Some(Self::AssignChannelAndTdmaSlot),
            0x20 => Some(Self::DiscoverUnitsProbe),
            0x21 => Some(Self::UnitDiscoveryAnnounce),
            0x22 => Some(Self::ClaimUnitAccess),
            0x23 => Some(Self::ReleaseUnitAccess),
            0xFE => Some(Self::TelemetryExceptionReport),
            _ => None,
        }
    }
}

/// Report-by-Exception error codes transmitted on the uplink only when an actuator deviates.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum TelemetryExceptionCode {
    None = 0,
    MotorStallDetected = 1,
    TrajectoryDeviationExceeded = 2,
    ThermalLimitTriggered = 3,
    WatchdogLeaseExpiredFailsafe = 4,
    EnteredLoRaLimpMode = 5,
    CryptoAuthFailureDetected = 6,
    UnauthorizedHubAccessRejected = 7,
}

impl TelemetryExceptionCode {
    pub const fn from_u8(val: u8) -> Self {
        match val {
            1 => Self::MotorStallDetected,
            2 => Self::TrajectoryDeviationExceeded,
            3 => Self::ThermalLimitTriggered,
            4 => Self::WatchdogLeaseExpiredFailsafe,
            5 => Self::EnteredLoRaLimpMode,
            6 => Self::CryptoAuthFailureDetected,
            7 => Self::UnauthorizedHubAccessRejected,
            _ => Self::None,
        }
    }
}

/// High-level **Bounded Intent** or **Discovery/Access Operation** emitted by the AI Agent or Unit.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BoundedIntent {
    /// Immediate E-Stop (0% torque, brakes locked, valves shut).
    EmergencyStop,
    /// Renew the Dual-Link Dead-Man's Switch lease without altering the goal.
    WatchdogHeartbeat { lease_timeout_ms: u16 },
    /// Move a `Servo`, `Stepper`, or closed-loop `DcMotor` to a bounded target position (Q16.16 degrees/steps/mm).
    MoveToBounded {
        target_pos_q16: i32,
        max_speed_pct: u8,
        max_torque_pct: u8,
        lease_timeout_ms: u16,
    },
    /// Maintain a bounded target velocity on a `Stepper` or `DcMotor` for at most `lease_timeout_ms`.
    SetVelocityBounded {
        target_vel_q16: i32,
        max_accel_pct: u8,
        max_torque_pct: u8,
        lease_timeout_ms: u16,
    },
    /// Regulate effort/duty on a `DcMotor` or `Servo` with a hardware lease.
    SetActuationEffortBounded {
        target_effort_q16: i32,
        max_rate_pct: u8,
        max_effort_pct: u8,
        lease_timeout_ms: u16,
    },
    /// Synchronize the actuator's local clock to the Hub's master millisecond tick.
    SyncClockBeacon {
        master_tick_ms: u32,
        beacon_interval_ms: u16,
    },
    /// Assign an actuator to a Worker Channel and millisecond TDMA transmit slot.
    AssignChannelAndTdmaSlot {
        espnow_worker_channel: u8,
        lora_freq_khz_offset: u8,
        tdma_slot_index: u16,
        tdma_frame_slots: u16,
    },
    /// **AI Hub Discovery Probe:** Asks all compatible units (`or filter_kind`) to announce themselves.
    DiscoverUnitsProbe {
        hub_id: u8,
        filter_kind: UnitElementKind,
    },
    /// **Unit Discovery Announce:** Sent by a `Servo`, `Stepper`, or `DcMotor` unit advertising
    /// its hardware type, current access state, owning Hub ID, and symmetric travel limit in Q16.16.
    UnitDiscoveryAnnounce {
        element_kind: UnitElementKind,
        access_state: UnitAccessState,
        claimed_by_hub_id: u8,
        max_travel_q16: i32,
    },
    /// **Claim Unit Access:** AI Hub claims exclusive control of a unit for `access_lease_ms`
    /// and assigns its operating worker channel.
    ClaimUnitAccess {
        hub_id: u8,
        espnow_worker_channel: u8,
        tdma_slot_index: u16,
        access_lease_ms: u16,
    },
    /// **Release Unit Access:** AI Hub releases a unit back to `IdleAvailable` on Master Channel 1.
    ReleaseUnitAccess { hub_id: u8 },
    /// Report-by-exception uplink alert from an actuator back to the Hub.
    TelemetryExceptionReport {
        exception: TelemetryExceptionCode,
        measured_pos_q16: i32,
        active_links_bitmap: u8,
    },
}

impl BoundedIntent {
    /// Helper to construct a `MoveToBounded` intent from floating-point degrees or steps.
    pub fn move_to_deg(
        angle_deg: f32,
        max_speed_pct: u8,
        max_torque_pct: u8,
        lease_timeout_ms: u16,
    ) -> Self {
        let target_pos_q16 = (angle_deg * 65536.0) as i32;
        Self::MoveToBounded {
            target_pos_q16,
            max_speed_pct: max_speed_pct.clamp(1, 100),
            max_torque_pct: max_torque_pct.clamp(1, 100),
            lease_timeout_ms,
        }
    }

    /// Encode this bounded intent into `(IntentOpcode, lease_timeout_ms, param_primary_q16, param_limit_q8)`.
    pub const fn to_raw_fields(self) -> (IntentOpcode, u16, i32, u16) {
        match self {
            Self::EmergencyStop => (IntentOpcode::EmergencyStop, 0, 0, 0),
            Self::WatchdogHeartbeat { lease_timeout_ms } => {
                (IntentOpcode::WatchdogHeartbeat, lease_timeout_ms, 0, 0)
            }
            Self::MoveToBounded {
                target_pos_q16,
                max_speed_pct,
                max_torque_pct,
                lease_timeout_ms,
            } => {
                let limit = ((max_speed_pct as u16) << 8) | (max_torque_pct as u16);
                (
                    IntentOpcode::MoveToBounded,
                    lease_timeout_ms,
                    target_pos_q16,
                    limit,
                )
            }
            Self::SetVelocityBounded {
                target_vel_q16,
                max_accel_pct,
                max_torque_pct,
                lease_timeout_ms,
            } => {
                let limit = ((max_accel_pct as u16) << 8) | (max_torque_pct as u16);
                (
                    IntentOpcode::SetVelocityBounded,
                    lease_timeout_ms,
                    target_vel_q16,
                    limit,
                )
            }
            Self::SetActuationEffortBounded {
                target_effort_q16,
                max_rate_pct,
                max_effort_pct,
                lease_timeout_ms,
            } => {
                let limit = ((max_rate_pct as u16) << 8) | (max_effort_pct as u16);
                (
                    IntentOpcode::SetActuationEffortBounded,
                    lease_timeout_ms,
                    target_effort_q16,
                    limit,
                )
            }
            Self::SyncClockBeacon {
                master_tick_ms,
                beacon_interval_ms,
            } => (
                IntentOpcode::SyncClockBeacon,
                beacon_interval_ms,
                master_tick_ms as i32,
                0,
            ),
            Self::AssignChannelAndTdmaSlot {
                espnow_worker_channel,
                lora_freq_khz_offset,
                tdma_slot_index,
                tdma_frame_slots,
            } => {
                let chan_packed =
                    ((espnow_worker_channel as u16) << 8) | (lora_freq_khz_offset as u16);
                (
                    IntentOpcode::AssignChannelAndTdmaSlot,
                    tdma_frame_slots,
                    tdma_slot_index as i32,
                    chan_packed,
                )
            }
            Self::DiscoverUnitsProbe {
                hub_id,
                filter_kind,
            } => {
                let packed = ((hub_id as u16) << 8) | (filter_kind as u16);
                (IntentOpcode::DiscoverUnitsProbe, 0, 0, packed)
            }
            Self::UnitDiscoveryAnnounce {
                element_kind,
                access_state,
                claimed_by_hub_id,
                max_travel_q16,
            } => {
                let meta = ((element_kind as u16) << 8) | (access_state as u16);
                (
                    IntentOpcode::UnitDiscoveryAnnounce,
                    claimed_by_hub_id as u16,
                    max_travel_q16,
                    meta,
                )
            }
            Self::ClaimUnitAccess {
                hub_id,
                espnow_worker_channel,
                tdma_slot_index,
                access_lease_ms,
            } => {
                let meta = ((hub_id as u16) << 8) | (espnow_worker_channel as u16);
                (
                    IntentOpcode::ClaimUnitAccess,
                    access_lease_ms,
                    tdma_slot_index as i32,
                    meta,
                )
            }
            Self::ReleaseUnitAccess { hub_id } => (
                IntentOpcode::ReleaseUnitAccess,
                0,
                0,
                (hub_id as u16) << 8,
            ),
            Self::TelemetryExceptionReport {
                exception,
                measured_pos_q16,
                active_links_bitmap,
            } => {
                let meta = ((exception as u16) << 8) | (active_links_bitmap as u16);
                (
                    IntentOpcode::TelemetryExceptionReport,
                    0,
                    measured_pos_q16,
                    meta,
                )
            }
        }
    }

    /// Decode a `BoundedIntent` from raw wire fields.
    pub const fn from_raw_fields(
        opcode: IntentOpcode,
        lease_timeout_ms: u16,
        param_primary_q16: i32,
        param_limit_q8: u16,
    ) -> Self {
        let hi = (param_limit_q8 >> 8) as u8;
        let lo = (param_limit_q8 & 0xFF) as u8;
        match opcode {
            IntentOpcode::EmergencyStop => Self::EmergencyStop,
            IntentOpcode::WatchdogHeartbeat => Self::WatchdogHeartbeat { lease_timeout_ms },
            IntentOpcode::MoveToBounded => Self::MoveToBounded {
                target_pos_q16: param_primary_q16,
                max_speed_pct: hi,
                max_torque_pct: lo,
                lease_timeout_ms,
            },
            IntentOpcode::SetVelocityBounded => Self::SetVelocityBounded {
                target_vel_q16: param_primary_q16,
                max_accel_pct: hi,
                max_torque_pct: lo,
                lease_timeout_ms,
            },
            IntentOpcode::SetActuationEffortBounded => Self::SetActuationEffortBounded {
                target_effort_q16: param_primary_q16,
                max_rate_pct: hi,
                max_effort_pct: lo,
                lease_timeout_ms,
            },
            IntentOpcode::SyncClockBeacon => Self::SyncClockBeacon {
                master_tick_ms: param_primary_q16 as u32,
                beacon_interval_ms: lease_timeout_ms,
            },
            IntentOpcode::AssignChannelAndTdmaSlot => Self::AssignChannelAndTdmaSlot {
                espnow_worker_channel: hi,
                lora_freq_khz_offset: lo,
                tdma_slot_index: param_primary_q16 as u16,
                tdma_frame_slots: lease_timeout_ms,
            },
            IntentOpcode::DiscoverUnitsProbe => Self::DiscoverUnitsProbe {
                hub_id: hi,
                filter_kind: UnitElementKind::from_u8(lo),
            },
            IntentOpcode::UnitDiscoveryAnnounce => Self::UnitDiscoveryAnnounce {
                element_kind: UnitElementKind::from_u8(hi),
                access_state: UnitAccessState::from_u8(lo),
                claimed_by_hub_id: (lease_timeout_ms & 0xFF) as u8,
                max_travel_q16: param_primary_q16,
            },
            IntentOpcode::ClaimUnitAccess => Self::ClaimUnitAccess {
                hub_id: hi,
                espnow_worker_channel: lo,
                tdma_slot_index: param_primary_q16 as u16,
                access_lease_ms: lease_timeout_ms,
            },
            IntentOpcode::ReleaseUnitAccess => Self::ReleaseUnitAccess { hub_id: hi },
            IntentOpcode::TelemetryExceptionReport => Self::TelemetryExceptionReport {
                exception: TelemetryExceptionCode::from_u8(hi),
                measured_pos_q16: param_primary_q16,
                active_links_bitmap: lo,
            },
        }
    }
}

/// Logical representation of the 32-byte Telekine packet before encryption or after decryption.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct WirePacket {
    /// 2-byte connectionless address (`0..=65533` node ID, `0xFFFE` master mgmt, `0xFFFF` broadcast).
    pub target_id: u16,
    /// Monotonic 4-byte sequence counter (`First-Valid-Wins` deduplication + AES-GCM nonce).
    pub seq_nonce: u32,
    /// Priority, redundancy mode, and sync flags.
    pub flags: PacketFlags,
    /// Scheduled execution tick in milliseconds (`0` = execute immediately on arrival).
    pub exec_at_tick_ms: u32,
    /// Decoded bounded AI intent or discovery/access message.
    pub intent: BoundedIntent,
}

impl WirePacket {
    /// Serialize the 8-byte cleartext header (used as Additional Authenticated Data in AES-128-GCM)
    /// and the 12-byte plaintext body to be encrypted.
    pub const fn to_header_and_body(&self) -> ([u8; 8], [u8; 12]) {
        let (opcode, lease_ms, primary_q16, limit_q8) = self.intent.to_raw_fields();
        let tid = self.target_id.to_le_bytes();
        let seq = self.seq_nonce.to_le_bytes();

        let header = [
            tid[0],
            tid[1],
            seq[0],
            seq[1],
            seq[2],
            seq[3],
            self.flags.0,
            opcode as u8,
        ];

        let exec = self.exec_at_tick_ms.to_le_bytes();
        let lease = lease_ms.to_le_bytes();
        let prim = primary_q16.to_le_bytes();
        let lim = limit_q8.to_le_bytes();

        let body = [
            exec[0], exec[1], exec[2], exec[3], lease[0], lease[1], prim[0], prim[1], prim[2],
            prim[3], lim[0], lim[1],
        ];

        (header, body)
    }

    /// Reconstruct a `WirePacket` from its 8-byte authenticated header and 12-byte decrypted body.
    pub const fn from_header_and_body(header: &[u8; 8], body: &[u8; 12]) -> Option<Self> {
        let target_id = u16::from_le_bytes([header[0], header[1]]);
        let seq_nonce = u32::from_le_bytes([header[2], header[3], header[4], header[5]]);
        let flags = PacketFlags(header[6]);
        let opcode = match IntentOpcode::from_u8(header[7]) {
            Some(op) => op,
            None => return None,
        };

        let exec_at_tick_ms = u32::from_le_bytes([body[0], body[1], body[2], body[3]]);
        let lease_timeout_ms = u16::from_le_bytes([body[4], body[5]]);
        let param_primary_q16 = i32::from_le_bytes([body[6], body[7], body[8], body[9]]);
        let param_limit_q8 = u16::from_le_bytes([body[10], body[11]]);

        let intent = BoundedIntent::from_raw_fields(
            opcode,
            lease_timeout_ms,
            param_primary_q16,
            param_limit_q8,
        );

        Some(Self {
            target_id,
            seq_nonce,
            flags,
            exec_at_tick_ms,
            intent,
        })
    }
}

/// Raw 32-byte encrypted wire frame transmitted over ESP-NOW or Semtech SX1262 LoRa.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(C)]
pub struct EncryptedFrame(pub [u8; WIRE_PACKET_SIZE]);

impl EncryptedFrame {
    /// Inspect the unencrypted 2-byte `target_id` in the header for ultra-fast MAC/ID filtering
    /// before spending ~12 microseconds on AES-128-GCM decryption.
    #[inline]
    pub const fn peek_target_id(&self) -> u16 {
        u16::from_le_bytes([self.0[0], self.0[1]])
    }

    /// Inspect the 4-byte `seq_nonce` in the header so duplicates can be dropped in <100 ns
    /// before running AES-128-GCM authentication.
    #[inline]
    pub const fn peek_seq_nonce(&self) -> u32 {
        u32::from_le_bytes([self.0[2], self.0[3], self.0[4], self.0[5]])
    }

    /// Inspect the `PacketFlags` byte in the header.
    #[inline]
    pub const fn peek_flags(&self) -> PacketFlags {
        PacketFlags(self.0[6])
    }
}

/// Typestate-verified command produced ONLY after:
/// 1. Target ID match (`target_id == node_id` or `BROADCAST_TARGET_ID` or `MASTER_CHANNEL_TARGET_ID`)
/// 2. Monotonic sequence check (`incoming_seq > last_executed_seq`)
/// 3. AES-128-GCM 96-bit authentication tag verification & payload decryption.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct VerifiedCommand {
    pub packet: WirePacket,
    pub received_via: RadioLink,
}
