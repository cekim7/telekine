#![no_std]
#![forbid(unsafe_op_in_unsafe_fn)]

//! # `telekine-core`
//!
//! `#[no_std]` deterministic actuator control & safety state machine shared between
//! bare-metal ESP32-S3/C3 firmware (`telekine-firmware`) and the 1,000-node swarm
//! simulator (`telekine-sim`).
//!
//! ## Key Safety Guarantees
//! 1. **Dual-Link Dead-Man's Switch (Watchdog Lease):** Any authenticated packet from
//!    either **ESP-NOW (2.4 GHz)** or **Raw LoRa (915 MHz)** resets the hardware lease
//!    countdown (`250–1,000 ms`). If both links go silent past the lease, the actuator
//!    enters `ActuatorOperatingState::FailSafeLocked` (0% torque, brakes engaged, valves shut).
//! 2. **Graceful Speed Degradation ("Limp Mode"):** If ESP-NOW drops out while LoRa remains
//!    active, the actuator enters `ActuatorOperatingState::LoRaLimpMode` and scales down
//!    maximum speed and torque limits until the fast 2.4 GHz link recovers.
//! 3. **Time-Stamped "Sync-and-Fire" Buffer:** Coordinated swarm commands with
//!    `exec_at_tick_ms > now` are buffered and triggered at the exact synchronized millisecond.
//! 4. **Local 1 kHz Closed-Loop Execution:** The AI only sends bounded goal states;
//!    `ActuatorNode::step_1khz()` closes the 1 ms control loop locally.
//! 5. **Report-by-Exception Telemetry:** The actuator stays silent when movements succeed
//!    and only emits an encrypted uplink exception frame on fault, stall, or state degradation.

use telekine_proto::{
    BoundedIntent, CryptoEngine, DeviceSecurityContext, EncryptedFrame, PacketFlags, Priority,
    RadioLink, RedundancyMode, SecurityError, TelemetryExceptionCode, VerifiedCommand, WirePacket,
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
    /// Commanded motor/valve effort in signed percent (`-100..=100`, `0` in fail-safe).
    pub effort_pct: i8,
    /// Effective maximum speed cap after Limp Mode scaling (`0..=100`).
    pub effective_max_speed_pct: u8,
    /// Effective maximum torque cap after Limp Mode scaling (`0..=100`).
    pub effective_max_torque_pct: u8,
    /// True if the mechanical brake is locked / fail-safe valve is closed.
    pub brakes_locked: bool,
}

/// TDMA & Channel assignment configuration managed via the Master Manager Channel.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TdmaChannelConfig {
    /// Assigned ESP-NOW Worker Channel (e.g., Channel 6 or 11; Channel 1 is Master Manager).
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
            espnow_worker_channel: 1, // Starts on Master Manager Channel 1 until assigned
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

/// Complete `#[no_std]` Actuator Node controller combining:
/// - `DeviceSecurityContext` (AES-128-GCM + monotonic `seq_nonce` deduplication)
/// - Dual-Link Dead-Man's Switch (`lease_expires_at_ms`)
/// - Graceful LoRa Limp Mode speed/torque throttling
/// - Sync-and-Fire pending command slot
/// - Local 1 kHz closed-loop trajectory tracker
/// - Report-by-Exception uplink generator
pub struct ActuatorNode<E: CryptoEngine> {
    security: DeviceSecurityContext<E>,
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

    /// Simulated or encoder-measured actuator position in Q16.16 fixed-point degrees.
    measured_pos_q16: i32,
    /// External mechanical stall flag injected by current/encoder sensor.
    stall_detected: bool,
    /// Outgoing Report-by-Exception telemetry counter.
    uplink_seq_nonce: u32,
}

impl<E: CryptoEngine> ActuatorNode<E> {
    /// Construct a new `ActuatorNode` initialized in `FailSafeLocked` state.
    pub fn new(
        security: DeviceSecurityContext<E>,
        safety_cfg: SafetyConfig,
        initial_pos_q16: i32,
    ) -> Self {
        Self {
            security,
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
            measured_pos_q16: initial_pos_q16,
            stall_detected: false,
            uplink_seq_nonce: 0,
        }
    }

    pub const fn node_id(&self) -> u16 {
        self.security.node_id()
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

    /// Returns `true` if the current synchronized millisecond matches this node's assigned
    /// TDMA uplink transmit slot.
    pub const fn is_my_tdma_slot(&self) -> bool {
        if self.tdma_cfg.tdma_frame_slots == 0 {
            return false;
        }
        let slot = (self.synchronized_tick_ms() % (self.tdma_cfg.tdma_frame_slots as u32)) as u16;
        slot == self.tdma_cfg.tdma_slot_index
    }

    /// Receive a 32-byte `EncryptedFrame` from either ESP-NOW (2.4 GHz) or Raw LoRa (915 MHz).
    ///
    /// - Enforces **First-Valid-Wins** monotonic `seq_nonce` deduplication.
    /// - Verifies the 96-bit AES-128-GCM tag and decrypts the bounded intent.
    /// - Resets the Dual-Link Dead-Man's Switch lease timer.
    /// - Buffers the instruction if `exec_at_tick_ms > now` (Sync-and-Fire), or applies it immediately.
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

        // Check if this is a time-stamped "Sync-and-Fire" command scheduled for the future
        if verified.packet.flags.is_sync_and_fire() && verified.packet.exec_at_tick_ms > now {
            // Renew lease at least until execution time + command lease
            let lease_ms = Self::extract_lease_ms(&verified.packet.intent).unwrap_or(250) as u32;
            self.lease_expires_at_ms = Some(verified.packet.exec_at_tick_ms.saturating_add(lease_ms));
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
            BoundedIntent::TelemetryExceptionReport { .. } => {}
        }
    }

    fn refresh_link_health_state(&mut self, now: u32) {
        if self.state == ActuatorOperatingState::EmergencyStopped {
            if !matches!(self.active_intent, BoundedIntent::EmergencyStop) {
                // Cleared by a new valid bounded intent
            } else {
                return;
            }
        }

        // Check Dual-Link Dead-Man's Switch lease expiration
        match self.lease_expires_at_ms {
            Some(deadline) if now < deadline => {
                // Lease is valid! Determine whether 2.4 GHz ESP-NOW is healthy or if we
                // are running on the 915 MHz Raw LoRa backup link ("Limp Mode").
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
                // Both radios silent past lease timer -> automatic Fail-Safe State
                self.state = ActuatorOperatingState::FailSafeLocked;
                self.active_intent = BoundedIntent::EmergencyStop;
            }
        }
    }

    /// Inject or clear a hardware motor stall condition (from current-sense or encoder).
    pub fn set_stall_detected(&mut self, stalled: bool) {
        self.stall_detected = stalled;
    }

    /// Execute one tick (1 ms) of the local 1 kHz hardware control & safety loop.
    ///
    /// Returns `(HardwareOutput, Option<EncryptedFrame>)` where the second element is
    /// `Some(EncryptedFrame)` **only** when a Report-by-Exception event occurs.
    pub fn step_1khz(&mut self) -> (HardwareOutput, Option<EncryptedFrame>) {
        self.local_tick_ms = self.local_tick_ms.wrapping_add(1);
        let now = self.synchronized_tick_ms();
        let prev_state = self.state;

        // 1. Trigger pending "Sync-and-Fire" command if its execution millisecond has arrived
        if let Some(pending) = self.pending_sync_command {
            if now >= pending.exec_at_tick_ms {
                self.pending_sync_command = None;
                self.apply_verified_packet(pending, now);
            }
        }

        // 2. Evaluate Dual-Link Watchdog Lease and Limp Mode state
        self.refresh_link_health_state(now);

        // 3. Compute closed-loop output if in NormalDualLink or LoRaLimpMode
        let output = match self.state {
            ActuatorOperatingState::FailSafeLocked | ActuatorOperatingState::EmergencyStopped => {
                HardwareOutput {
                    state: self.state,
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
                        let eff_speed = (((max_speed_pct as u16) * scale) / 100).max(1) as u8;
                        let eff_torque = (((max_torque_pct as u16) * scale) / 100).max(1) as u8;

                        let error_q16 = target_pos_q16.saturating_sub(self.measured_pos_q16);
                        // Max step per millisecond at 100% speed = 0.25 deg/ms (250 deg/sec)
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
                        HardwareOutput {
                            state: self.state,
                            effort_pct: eff_torque as i8,
                            effective_max_speed_pct: eff_speed,
                            effective_max_torque_pct: eff_torque,
                            brakes_locked: false,
                        }
                    }
                    _ => HardwareOutput {
                        state: self.state,
                        effort_pct: 0,
                        effective_max_speed_pct: 0,
                        effective_max_torque_pct: 0,
                        brakes_locked: false,
                    },
                }
            }
        };

        // 4. Report-by-Exception check: remain silent unless a fault or state degradation occurred
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

#[cfg(test)]
mod tests {
    use super::*;
    use telekine_proto::Aes128GcmSoftware;

    #[test]
    fn watchdog_lease_expires_into_failsafe_and_emits_exception() {
        let master_key = [0x11u8; 16];
        let mac = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06];
        let hub_sec = DeviceSecurityContext::new(1, &master_key, &mac);
        let node_sec = DeviceSecurityContext::<Aes128GcmSoftware>::new(1, &master_key, &mac);

        let mut node = ActuatorNode::new(node_sec, SafetyConfig::default(), 0);
        assert_eq!(node.operating_state(), ActuatorOperatingState::FailSafeLocked);

        // Send bounded move with 10 ms lease over ESP-NOW
        let cmd = WirePacket {
            target_id: 1,
            seq_nonce: 1,
            flags: PacketFlags::new(
                Priority::HighVi,
                RedundancyMode::HotWarmFailover,
                false,
                false,
                false,
            ),
            exec_at_tick_ms: 0,
            intent: BoundedIntent::move_to_deg(30.0, 50, 80, 10),
        };
        let frame = hub_sec.seal_packet(&cmd);
        node.receive_frame(&frame, RadioLink::EspNow24Ghz).unwrap();
        assert_eq!(node.operating_state(), ActuatorOperatingState::NormalDualLink);

        // Step 9 ms: still active and silent (no exception telemetry)
        for _ in 0..9 {
            let (out, exc) = node.step_1khz();
            assert_eq!(out.state, ActuatorOperatingState::NormalDualLink);
            assert!(!out.brakes_locked);
            assert!(exc.is_none());
        }

        // At 10 ms without a heartbeat on either radio, watchdog triggers FailSafeLocked
        // and emits a single Report-by-Exception frame!
        let (out, exc) = node.step_1khz();
        assert_eq!(out.state, ActuatorOperatingState::FailSafeLocked);
        assert_eq!(out.effort_pct, 0);
        assert!(out.brakes_locked);
        assert!(exc.is_some());
    }

    #[test]
    fn lora_only_fallback_triggers_limp_mode_speed_throttling() {
        let master_key = [0x22u8; 16];
        let mac = [0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F];
        let hub_sec = DeviceSecurityContext::new(5, &master_key, &mac);
        let node_sec = DeviceSecurityContext::new(5, &master_key, &mac);

        let mut node = ActuatorNode::new(node_sec, SafetyConfig::default(), 0);

        // Packet arrives ONLY over Raw LoRa 915 MHz (ESP-NOW blocked by wall/interference)
        let cmd = WirePacket {
            target_id: 5,
            seq_nonce: 1,
            flags: PacketFlags::new(
                Priority::HighVi,
                RedundancyMode::HotWarmFailover,
                false,
                false,
                false,
            ),
            exec_at_tick_ms: 0,
            intent: BoundedIntent::move_to_deg(90.0, 100, 100, 500),
        };
        let frame = hub_sec.seal_packet(&cmd);
        node.receive_frame(&frame, RadioLink::RawLoRa915Mhz).unwrap();

        let (out, _) = node.step_1khz();
        assert_eq!(out.state, ActuatorOperatingState::LoRaLimpMode);
        // 100% requested speed & torque is automatically throttled to 40% in Limp Mode!
        assert_eq!(out.effective_max_speed_pct, 40);
        assert_eq!(out.effective_max_torque_pct, 40);
    }
}
