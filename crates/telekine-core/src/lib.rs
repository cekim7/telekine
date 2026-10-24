#![no_std]
#![forbid(unsafe_op_in_unsafe_fn)]

//! # `telekine-core`
//!
//! `#[no_std]` deterministic actuator control, **Foundational Unit Discovery & Access Lease**
//! (`Servo`, `Stepper`, `DcMotor`), and safety state machine shared between bare-metal
//! ESP32-S3/C3 firmware (`telekine-firmware`) and the swarm simulator (`telekine-sim`).
//!
//! ## Demo 1 Lifecycle (Foundational Unit Elements: Servo, Stepper, DC Motor)
//! 1. **Auto-Discovery on Master Channel 1:** Idle units listen on Channel 1 (`UnitAccessState::IdleAvailable`).
//!    When an AI Hub broadcasts `DiscoverUnitsProbe`, each compatible unit replies with a
//!    `UnitDiscoveryAnnounce` packet reporting its `UnitElementKind` (`Servo`, `Stepper`, `DcMotor`),
//!    access state, and travel bounds.
//! 2. **Dynamic Access Claiming (`ClaimUnitAccess`):** When the AI Hub needs a unit, it sends
//!    `ClaimUnitAccess`, locking the unit to `hub_id`, switching it to the Hub's Worker Channel
//!    (`6` or `11`) and TDMA slot, and arming the Dual-Link Dead-Man's Switch lease.
//! 3. **Bounded Closed-Loop Execution & Limp Mode:** Executes `MoveToBounded`, `SetVelocityBounded`,
//!    or `SetActuationEffortBounded` at 1 kHz, degrading speed/torque to 40% if 2.4 GHz ESP-NOW drops.
//! 4. **Graceful Release or Watchdog Fail-Safe:** When the AI Hub sends `ReleaseUnitAccess`
//!    (or if both radios go silent past the lease timeout), the unit locks into `FailSafeLocked`,
//!    resets to `UnitAccessState::IdleAvailable`, and returns to Master Channel 1.

use telekine_proto::{
    BoundedIntent, CryptoEngine, DeviceSecurityContext, EncryptedFrame, PacketFlags, Priority,
    RadioLink, RedundancyMode, SecurityError, TelemetryExceptionCode, UnitAccessState,
    UnitElementKind, VerifiedCommand, WirePacket, MASTER_CHANNEL_TARGET_ID,
};

/// Operating state of the actuator's dual-radio safety supervisor.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ActuatorOperatingState {
    /// Safe idle or watchdog-triggered fail-safe: zero torque, brakes locked, valves shut.
    FailSafeLocked,
    /// Both 2.4 GHz ESP-NOW and 915 MHz LoRa links are healthy; 100% of bounded speed/torque allowed.
    NormalDualLink,
    /// Fast 2.4 GHz ESP-NOW link has gone silent; running solely on 915 MHz LoRa backup with
    /// automatic speed & torque throttling ("Limp Mode").
    LoRaLimpMode,
    /// Immediate hardware E-Stop latched until an explicit new bounded goal clears it.
    EmergencyStopped,
}

/// Hardware actuation output produced every 1 ms by the local 1 kHz control loop.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HardwareOutput {
    /// Current operating mode of the safety supervisor.
    pub state: ActuatorOperatingState,
    /// Foundational unit element type (`Servo`, `Stepper`, `DcMotor`).
    pub element_kind: UnitElementKind,
    /// Commanded motor/servo/stepper effort in signed percent (`-100..=100`, `0` in fail-safe).
    pub effort_pct: i8,
    /// Effective maximum speed cap after Limp Mode scaling (`0..=100`).
    pub effective_max_speed_pct: u8,
    /// Effective maximum torque cap after Limp Mode scaling (`0..=100`).
    pub effective_max_torque_pct: u8,
    /// True if the mechanical brake is locked / power gate is disabled.
    pub brakes_locked: bool,
}

/// TDMA & Channel assignment configuration managed via the Master Manager Channel.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TdmaChannelConfig {
    /// Assigned ESP-NOW Worker Channel (`1` = Master Manager Channel, `6` or `11` = Worker Channels).
    pub espnow_worker_channel: u8,
    /// Assigned LoRa frequency offset in kHz from 915.0 MHz.
    pub lora_freq_khz_offset: u8,
    /// Assigned millisecond TDMA slot index for this node (`0..tdma_frame_slots`).
    pub tdma_slot_index: u16,
    /// Total number of slots in a TDMA cycle (e.g. `1000` for 1,000 peers).
    pub tdma_frame_slots: u16,
}

impl Default for TdmaChannelConfig {
    fn default() -> Self {
        Self {
            espnow_worker_channel: 1, // Starts on Master Manager Channel 1 until claimed by a Hub
            lora_freq_khz_offset: 0,
            tdma_slot_index: 0,
            tdma_frame_slots: 1000,
        }
    }
}

/// Configuration thresholds for the Dual-Link Watchdog and Limp Mode supervisor.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SafetyConfig {
    /// How long (ms) without a 2.4 GHz ESP-NOW packet before degrading to LoRa Limp Mode.
    pub espnow_dropout_threshold_ms: u32,
    /// Speed & torque scaling percentage when operating in `LoRaLimpMode` (e.g. `40` = 40% of normal limit).
    pub limp_mode_scale_pct: u8,
    /// Maximum position tracking error (in Q16.16 degrees) before firing a Report-by-Exception packet.
    pub deviation_threshold_q16: i32,
}

impl Default for SafetyConfig {
    fn default() -> Self {
        Self {
            espnow_dropout_threshold_ms: 150,
            limp_mode_scale_pct: 40,
            deviation_threshold_q16: 15 * 65536, // 15.0 degrees
        }
    }
}

/// Complete `#[no_std]` Foundational Unit Node (`Servo`, `Stepper`, `DcMotor`) combining:
/// - `DeviceSecurityContext` (AES-128-GCM + monotonic `seq_nonce` deduplication)
/// - Master Channel 1 Auto-Discovery & Dynamic Hub Access Claim/Release
/// - Dual-Link Dead-Man's Switch (`lease_expires_at_ms`)
/// - Graceful LoRa Limp Mode speed/torque throttling
/// - Sync-and-Fire pending command slot
/// - Local 1 kHz closed-loop trajectory tracker
pub struct ActuatorNode<E: CryptoEngine> {
    security: DeviceSecurityContext<E>,
    element_kind: UnitElementKind,
    access_state: UnitAccessState,
    claimed_by_hub_id: Option<u8>,
    max_travel_q16: i32,

    safety_cfg: SafetyConfig,
    tdma_cfg: TdmaChannelConfig,
    state: ActuatorOperatingState,

    /// Local millisecond clock (synchronized with Hub via `SyncClockBeacon`).
    local_tick_ms: u32,
    /// Offset added to local hardware timer to match Hub's master clock.
    clock_offset_ms: i64,

    /// Timestamp (in synchronized ms) of last valid ESP-NOW packet.
    last_espnow_rx_ms: Option<u32>,
    /// Timestamp (in synchronized ms) of last valid Raw LoRa packet.
    last_lora_rx_ms: Option<u32>,
    /// Synchronized tick when the active Dual-Link Watchdog Lease expires.
    lease_expires_at_ms: Option<u32>,

    /// Active bounded goal currently being tracked by the 1 kHz loop.
    active_intent: BoundedIntent,
    /// Buffered "Sync-and-Fire" command waiting for `synchronized_tick_ms >= exec_at_tick_ms`.
    pending_sync_command: Option<WirePacket>,
    /// Pending outgoing discovery announcement frame queued in response to a Hub probe/claim.
    pending_discovery_reply: Option<EncryptedFrame>,

    /// Simulated or encoder-measured actuator position in Q16.16 fixed-point degrees/steps.
    measured_pos_q16: i32,
    /// External mechanical stall flag injected by current/encoder sensor.
    stall_detected: bool,
    /// Outgoing Report-by-Exception & Discovery uplink counter.
    uplink_seq_nonce: u32,
}

impl<E: CryptoEngine> ActuatorNode<E> {
    /// Construct a new `ActuatorNode` initialized in `FailSafeLocked` and `IdleAvailable` state.
    pub fn new(
        security: DeviceSecurityContext<E>,
        safety_cfg: SafetyConfig,
        initial_pos_q16: i32,
    ) -> Self {
        Self::new_unit(
            security,
            UnitElementKind::DcMotor,
            180 * 65536,
            safety_cfg,
            initial_pos_q16,
        )
    }

    /// Construct a specific Foundational Unit (`Servo`, `Stepper`, or `DcMotor`) with travel bounds.
    pub fn new_unit(
        security: DeviceSecurityContext<E>,
        element_kind: UnitElementKind,
        max_travel_q16: i32,
        safety_cfg: SafetyConfig,
        initial_pos_q16: i32,
    ) -> Self {
        Self {
            security,
            element_kind,
            access_state: UnitAccessState::IdleAvailable,
            claimed_by_hub_id: None,
            max_travel_q16,
            safety_cfg,
            tdma_cfg: TdmaChannelConfig::default(),
            state: ActuatorOperatingState::FailSafeLocked,
            local_tick_ms: 0,
            clock_offset_ms: 0,
            last_espnow_rx_ms: None,
            last_lora_rx_ms: None,
            lease_expires_at_ms: None,
            active_intent: BoundedIntent::EmergencyStop,
            pending_sync_command: None,
            pending_discovery_reply: None,
            measured_pos_q16: initial_pos_q16,
            stall_detected: false,
            uplink_seq_nonce: 0,
        }
    }

    pub const fn node_id(&self) -> u16 {
        self.security.node_id()
    }

    pub const fn element_kind(&self) -> UnitElementKind {
        self.element_kind
    }

    pub const fn access_state(&self) -> UnitAccessState {
        self.access_state
    }

    pub const fn claimed_by_hub_id(&self) -> Option<u8> {
        self.claimed_by_hub_id
    }

    pub const fn active_intent(&self) -> BoundedIntent {
        self.active_intent
    }

    pub const fn operating_state(&self) -> ActuatorOperatingState {
        self.state
    }

    pub const fn measured_pos_q16(&self) -> i32 {
        self.measured_pos_q16
    }

    pub fn measured_pos_deg(&self) -> f32 {
        (self.measured_pos_q16 as f32) / 65536.0
    }

    pub const fn tdma_config(&self) -> TdmaChannelConfig {
        self.tdma_cfg
    }

    pub const fn synchronized_tick_ms(&self) -> u32 {
        (self.local_tick_ms as i64 + self.clock_offset_ms) as u32
    }

    /// Take any queued `UnitDiscoveryAnnounce` uplink reply frame generated during `receive_frame()`.
    pub fn take_discovery_reply(&mut self) -> Option<EncryptedFrame> {
        self.pending_discovery_reply.take()
    }

    /// Build an encrypted `UnitDiscoveryAnnounce` frame advertising this unit's kind and access state.
    pub fn build_discovery_announce_frame(&mut self) -> EncryptedFrame {
        self.uplink_seq_nonce = self.uplink_seq_nonce.wrapping_add(1);
        let pkt = WirePacket {
            target_id: MASTER_CHANNEL_TARGET_ID,
            seq_nonce: self.uplink_seq_nonce,
            flags: PacketFlags::new(
                Priority::HighVi,
                RedundancyMode::HotHotTwinCast,
                false,
                true,
                false,
            ),
            exec_at_tick_ms: self.node_id() as u32, // Carries originating node_id in exec_at_tick_ms
            intent: BoundedIntent::UnitDiscoveryAnnounce {
                element_kind: self.element_kind,
                access_state: self.access_state,
                claimed_by_hub_id: self.claimed_by_hub_id.unwrap_or(0),
                max_travel_q16: self.max_travel_q16,
            },
        };
        self.security.seal_packet(&pkt)
    }

    /// Receive a 32-byte `EncryptedFrame` from either ESP-NOW (2.4 GHz) or Raw LoRa (915 MHz).
    pub fn receive_frame(
        &mut self,
        frame: &EncryptedFrame,
        link: RadioLink,
    ) -> Result<VerifiedCommand, SecurityError> {
        let verified = self.security.open_and_verify(frame, link)?;
        let now = self.synchronized_tick_ms();

        match link {
            RadioLink::EspNow24Ghz => self.last_espnow_rx_ms = Some(now),
            RadioLink::RawLoRa915Mhz => self.last_lora_rx_ms = Some(now),
        }

        if verified.packet.flags.is_sync_and_fire() && verified.packet.exec_at_tick_ms > now {
            let lease_ms = Self::extract_lease_ms(&verified.packet.intent).unwrap_or(250) as u32;
            self.lease_expires_at_ms =
                Some(verified.packet.exec_at_tick_ms.saturating_add(lease_ms));
            self.pending_sync_command = Some(verified.packet);
        } else {
            self.apply_verified_packet(verified.packet, now);
        }

        self.refresh_link_health_state(now);
        Ok(verified)
    }

    fn extract_lease_ms(intent: &BoundedIntent) -> Option<u16> {
        match *intent {
            BoundedIntent::WatchdogHeartbeat { lease_timeout_ms }
            | BoundedIntent::MoveToBounded {
                lease_timeout_ms, ..
            }
            | BoundedIntent::SetVelocityBounded {
                lease_timeout_ms, ..
            }
            | BoundedIntent::SetActuationEffortBounded {
                lease_timeout_ms, ..
            } => Some(lease_timeout_ms),
            BoundedIntent::ClaimUnitAccess {
                access_lease_ms, ..
            } => Some(access_lease_ms),
            _ => None,
        }
    }

    fn apply_verified_packet(&mut self, pkt: WirePacket, now: u32) {
        match pkt.intent {
            BoundedIntent::EmergencyStop => {
                self.active_intent = BoundedIntent::EmergencyStop;
                self.pending_sync_command = None;
                self.lease_expires_at_ms = None;
                self.state = ActuatorOperatingState::EmergencyStopped;
            }
            BoundedIntent::DiscoverUnitsProbe { filter_kind, .. } => {
                if filter_kind == UnitElementKind::Unknown || filter_kind == self.element_kind {
                    self.pending_discovery_reply = Some(self.build_discovery_announce_frame());
                }
            }
            BoundedIntent::ClaimUnitAccess {
                hub_id,
                espnow_worker_channel,
                tdma_slot_index,
                access_lease_ms,
            } => {
                let can_claim = self.access_state == UnitAccessState::IdleAvailable
                    || self.claimed_by_hub_id == Some(hub_id);
                if can_claim {
                    self.access_state = UnitAccessState::ClaimedByHub;
                    self.claimed_by_hub_id = Some(hub_id);
                    self.tdma_cfg.espnow_worker_channel = espnow_worker_channel;
                    self.tdma_cfg.tdma_slot_index = tdma_slot_index;
                    self.lease_expires_at_ms = Some(now.saturating_add(access_lease_ms as u32));
                    self.pending_discovery_reply = Some(self.build_discovery_announce_frame());
                }
            }
            BoundedIntent::ReleaseUnitAccess { hub_id } => {
                if self.claimed_by_hub_id == Some(hub_id) || self.claimed_by_hub_id.is_none() {
                    self.active_intent = BoundedIntent::EmergencyStop;
                    self.pending_sync_command = None;
                    self.lease_expires_at_ms = None;
                    self.state = ActuatorOperatingState::FailSafeLocked;
                    self.access_state = UnitAccessState::IdleAvailable;
                    self.claimed_by_hub_id = None;
                    self.tdma_cfg.espnow_worker_channel = 1; // Return to Master Manager Channel 1
                    self.pending_discovery_reply = Some(self.build_discovery_announce_frame());
                }
            }
            BoundedIntent::WatchdogHeartbeat { lease_timeout_ms } => {
                self.lease_expires_at_ms = Some(now.saturating_add(lease_timeout_ms as u32));
            }
            BoundedIntent::MoveToBounded {
                lease_timeout_ms, ..
            }
            | BoundedIntent::SetVelocityBounded {
                lease_timeout_ms, ..
            }
            | BoundedIntent::SetActuationEffortBounded {
                lease_timeout_ms, ..
            } => {
                // Auto-claim if directly addressed via unicast key and currently IdleAvailable
                if self.access_state == UnitAccessState::IdleAvailable {
                    self.access_state = UnitAccessState::ClaimedByHub;
                }
                self.active_intent = pkt.intent;
                self.lease_expires_at_ms = Some(now.saturating_add(lease_timeout_ms as u32));
            }
            BoundedIntent::SyncClockBeacon {
                master_tick_ms,
                beacon_interval_ms,
            } => {
                self.clock_offset_ms = (master_tick_ms as i64) - (self.local_tick_ms as i64);
                let synced_now = self.synchronized_tick_ms();
                if beacon_interval_ms > 0 && self.lease_expires_at_ms.is_some() {
                    self.lease_expires_at_ms =
                        Some(synced_now.saturating_add(beacon_interval_ms as u32));
                }
            }
            BoundedIntent::AssignChannelAndTdmaSlot {
                espnow_worker_channel,
                lora_freq_khz_offset,
                tdma_slot_index,
                tdma_frame_slots,
            } => {
                self.tdma_cfg = TdmaChannelConfig {
                    espnow_worker_channel,
                    lora_freq_khz_offset,
                    tdma_slot_index,
                    tdma_frame_slots,
                };
            }
            BoundedIntent::UnitDiscoveryAnnounce { .. }
            | BoundedIntent::TelemetryExceptionReport { .. } => {}
        }
    }

    fn refresh_link_health_state(&mut self, now: u32) {
        if self.state == ActuatorOperatingState::EmergencyStopped {
            if matches!(self.active_intent, BoundedIntent::EmergencyStop) {
                return;
            }
        }

        match self.lease_expires_at_ms {
            Some(deadline) if now < deadline => {
                let espnow_alive = match self.last_espnow_rx_ms {
                    Some(last_esp) => {
                        now.saturating_sub(last_esp) <= self.safety_cfg.espnow_dropout_threshold_ms
                    }
                    None => false,
                };

                if espnow_alive {
                    self.state = ActuatorOperatingState::NormalDualLink;
                } else {
                    self.state = ActuatorOperatingState::LoRaLimpMode;
                }
            }
            _ => {
                // Both radios silent past lease timer -> automatic Fail-Safe State & release claim
                self.state = ActuatorOperatingState::FailSafeLocked;
                self.active_intent = BoundedIntent::EmergencyStop;
                if self.access_state == UnitAccessState::ClaimedByHub {
                    self.access_state = UnitAccessState::IdleAvailable;
                    self.claimed_by_hub_id = None;
                    self.tdma_cfg.espnow_worker_channel = 1;
                }
            }
        }
    }

    /// Inject or clear a hardware motor stall condition (from current-sense or encoder).
    pub fn set_stall_detected(&mut self, stalled: bool) {
        self.stall_detected = stalled;
        if stalled {
            self.access_state = UnitAccessState::Faulted;
        }
    }

    /// Execute one tick (1 ms) of the local 1 kHz hardware control & safety loop.
    pub fn step_1khz(&mut self) -> (HardwareOutput, Option<EncryptedFrame>) {
        self.local_tick_ms = self.local_tick_ms.wrapping_add(1);
        let now = self.synchronized_tick_ms();
        let prev_state = self.state;

        if let Some(pending) = self.pending_sync_command {
            if now >= pending.exec_at_tick_ms {
                self.pending_sync_command = None;
                self.apply_verified_packet(pending, now);
            }
        }

        self.refresh_link_health_state(now);

        let output = match self.state {
            ActuatorOperatingState::FailSafeLocked | ActuatorOperatingState::EmergencyStopped => {
                HardwareOutput {
                    state: self.state,
                    element_kind: self.element_kind,
                    effort_pct: 0,
                    effective_max_speed_pct: 0,
                    effective_max_torque_pct: 0,
                    brakes_locked: true,
                }
            }
            ActuatorOperatingState::NormalDualLink | ActuatorOperatingState::LoRaLimpMode => {
                let scale: u16 = if self.state == ActuatorOperatingState::LoRaLimpMode {
                    self.safety_cfg.limp_mode_scale_pct as u16
                } else {
                    100
                };

                match self.active_intent {
                    BoundedIntent::MoveToBounded {
                        target_pos_q16,
                        max_speed_pct,
                        max_torque_pct,
                        ..
                    } => {
                        let clamped_target =
                            target_pos_q16.clamp(-self.max_travel_q16, self.max_travel_q16);
                        let eff_speed = (((max_speed_pct as u16) * scale) / 100).max(1) as u8;
                        let eff_torque = (((max_torque_pct as u16) * scale) / 100).max(1) as u8;

                        let error_q16 = clamped_target.saturating_sub(self.measured_pos_q16);
                        let max_step_q16 = ((eff_speed as i32) * 16384) / 100;

                        let step_q16 = if self.stall_detected {
                            0
                        } else if error_q16 > max_step_q16 {
                            max_step_q16
                        } else if error_q16 < -max_step_q16 {
                            -max_step_q16
                        } else {
                            error_q16
                        };

                        self.measured_pos_q16 = self.measured_pos_q16.saturating_add(step_q16);

                        let effort_pct = if error_q16 == 0 {
                            0
                        } else if error_q16 > 0 {
                            eff_torque as i8
                        } else {
                            -(eff_torque as i8)
                        };

                        HardwareOutput {
                            state: self.state,
                            element_kind: self.element_kind,
                            effort_pct,
                            effective_max_speed_pct: eff_speed,
                            effective_max_torque_pct: eff_torque,
                            brakes_locked: false,
                        }
                    }
                    BoundedIntent::SetVelocityBounded {
                        target_vel_q16,
                        max_accel_pct,
                        max_torque_pct,
                        ..
                    } => {
                        let eff_speed = (((max_accel_pct as u16) * scale) / 100).max(1) as u8;
                        let eff_torque = (((max_torque_pct as u16) * scale) / 100).max(1) as u8;
                        if !self.stall_detected {
                            let step = ((target_vel_q16 / 1000) * (scale as i32)) / 100;
                            self.measured_pos_q16 = self.measured_pos_q16.saturating_add(step);
                        }
                        let dir_effort = if target_vel_q16 >= 0 {
                            eff_torque as i8
                        } else {
                            -(eff_torque as i8)
                        };
                        HardwareOutput {
                            state: self.state,
                            element_kind: self.element_kind,
                            effort_pct: dir_effort,
                            effective_max_speed_pct: eff_speed,
                            effective_max_torque_pct: eff_torque,
                            brakes_locked: false,
                        }
                    }
                    BoundedIntent::SetActuationEffortBounded {
                        target_effort_q16,
                        max_rate_pct,
                        max_effort_pct,
                        ..
                    } => {
                        let eff_speed = (((max_rate_pct as u16) * scale) / 100).max(1) as u8;
                        let eff_torque = (((max_effort_pct as u16) * scale) / 100).max(1) as u8;
                        let signed_effort =
                            ((target_effort_q16 / 65536) as i32).clamp(-(eff_torque as i32), eff_torque as i32) as i8;
                        HardwareOutput {
                            state: self.state,
                            element_kind: self.element_kind,
                            effort_pct: signed_effort,
                            effective_max_speed_pct: eff_speed,
                            effective_max_torque_pct: eff_torque,
                            brakes_locked: false,
                        }
                    }
                    _ => HardwareOutput {
                        state: self.state,
                        element_kind: self.element_kind,
                        effort_pct: 0,
                        effective_max_speed_pct: 0,
                        effective_max_torque_pct: 0,
                        brakes_locked: false,
                    },
                }
            }
        };

        let exception_code = if self.stall_detected {
            Some(TelemetryExceptionCode::MotorStallDetected)
        } else if prev_state != ActuatorOperatingState::FailSafeLocked
            && self.state == ActuatorOperatingState::FailSafeLocked
        {
            Some(TelemetryExceptionCode::WatchdogLeaseExpiredFailsafe)
        } else if prev_state == ActuatorOperatingState::NormalDualLink
            && self.state == ActuatorOperatingState::LoRaLimpMode
        {
            Some(TelemetryExceptionCode::EnteredLoRaLimpMode)
        } else {
            None
        };

        let exception_frame = exception_code.map(|code| self.build_exception_frame(code, now));
        (output, exception_frame)
    }

    fn build_exception_frame(
        &mut self,
        exception: TelemetryExceptionCode,
        now: u32,
    ) -> EncryptedFrame {
        self.uplink_seq_nonce = self.uplink_seq_nonce.wrapping_add(1);
        let esp_alive = self
            .last_espnow_rx_ms
            .map(|t| now.saturating_sub(t) <= self.safety_cfg.espnow_dropout_threshold_ms)
            .unwrap_or(false);
        let lora_alive = self
            .last_lora_rx_ms
            .map(|t| self.lease_expires_at_ms.map(|d| now < d && t <= now).unwrap_or(false))
            .unwrap_or(false);
        let bitmap = (if esp_alive { 0x01 } else { 0x00 }) | (if lora_alive { 0x02 } else { 0x00 });

        let pkt = WirePacket {
            target_id: self.node_id(),
            seq_nonce: self.uplink_seq_nonce,
            flags: PacketFlags::new(
                Priority::CriticalVo,
                RedundancyMode::HotHotTwinCast,
                false,
                false,
                true,
            ),
            exec_at_tick_ms: now,
            intent: BoundedIntent::TelemetryExceptionReport {
                exception,
                measured_pos_q16: self.measured_pos_q16,
                active_links_bitmap: bitmap,
            },
        };
        self.security.seal_packet(&pkt)
    }
}
