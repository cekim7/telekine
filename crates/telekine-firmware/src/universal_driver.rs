//! # Universal Hardware Driver Layer (`telekine-firmware::universal_driver`)
//!
//! Enables a single standardized **Telekine Universal Driver Board** (ESP32-S3/C3 + Semtech SX1262)
//! to attach to and control diverse low-intelligence physical hardware components without rewriting
//! core firmware.
//!
//! ## Supported Hardware Personality Profiles
//! 1. **`PwmServoOrEsc`** — 50–400 Hz RC PWM / DShot (hobby servos, drone ESCs, linear servos).
//! 2. **`HBridgeBrushedDc`** — Reversible PWM + `IPROPI` current-sense stall detection (`DRV8876`)
//!    with quadrature/magnetic encoder feedback for dumb brushed DC motors & linear actuators.
//! 3. **`StepperStepDir`** — High-speed `STEP` / `DIR` / `ENABLE` pulse generator (TMC2209 / DM542).
//! 4. **`SolenoidOrRelayBank`** — Proportional PWM hold-current or fail-safe binary valve/relay control.
//! 5. **`CanBusSmartMotor`** — CAN 2.0 / CAN-FD frame translator (ODrive, VESC, MIT Mini-Cheetah actuators).
//! 6. **`Rs485SerialBus`** — Half-duplex RS-485 / TTL UART bridge (Dynamixel, Feetech, Modbus RTU VFDs).
//! 7. **`Analog0To10V`** — DAC / filtered PWM 0–10V industrial process control (valves, dimmers, dampers).

use telekine_core::{ActuatorOperatingState, HardwareOutput};
use telekine_proto::BoundedIntent;

/// Auto-detected or flash-provisioned personality profile of the attached hardware component.
///
/// On the physical Telekine Universal Driver PCB, this is automatically identified at boot
/// via the harness `ID_SENSE` pin (1-Wire EEPROM or calibration resistor) so plugging the
/// universal puck into a new actuator automatically configures the driver stage and safety limits.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum HardwarePersonality {
    /// Unconfigured or missing harness interlock — keeps `SAFE_ENABLE` hardware gate open (disabled).
    UnconfiguredSafe = 0,
    /// 5V level-shifted PWM / DShot output (`1000..=2000 us` pulse width).
    PwmServoOrEsc = 1,
    /// Onboard `DRV8876` H-Bridge (`IN1`/`IN2` + `IPROPI` stall sense + encoder feedback).
    HBridgeBrushedDc = 2,
    /// Stepper driver interface (`STEP` frequency + `DIR` + active-low `EN`).
    StepperStepDir = 3,
    /// Fail-safe solenoid valve or SSR relay with peak-and-hold PWM current saving.
    SolenoidOrRelay = 4,
    /// Industrial/robotics CAN-Bus smart actuator (`TCAN1044V` transceiver).
    CanBusSmartActuator = 5,
    /// RS-485 / half-duplex UART smart servo or Modbus VFD (`SN65HVD72` transceiver).
    Rs485SerialActuator = 6,
    /// 0–10V / 4–20mA industrial analog proportional actuator.
    Analog0To10V = 7,
}

/// Descriptor read from the wiring harness 1-Wire EEPROM (or flash profile) describing
/// the attached hardware's electrical and mechanical limits.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AttachedHardwareDescriptor {
    /// Hardware interface type to activate on the Universal Driver multiplexer.
    pub personality: HardwarePersonality,
    /// Nominal supply voltage in millivolts (e.g., `12_000` for 12V, `24_000` for 24V).
    pub nominal_voltage_mv: u16,
    /// Hardware current trip threshold in milliamps (`IPROPI` comparator stall cutoff).
    pub stall_current_limit_ma: u16,
    /// Encoder counts per full revolution or stroke (0 if open-loop / internal feedback).
    pub encoder_cpr: u32,
    /// Minimum mechanical travel in Q16.16 degrees or mm.
    pub min_travel_q16: i32,
    /// Maximum mechanical travel in Q16.16 degrees or mm.
    pub max_travel_q16: i32,
    /// Fail-safe behavior when `SAFE_ENABLE` drops (`true` = active short-brake, `false` = high-Z coast).
    pub short_brake_on_failsafe: bool,
}

impl AttachedHardwareDescriptor {
    /// Default profile for a 24V Brushed DC gearmotor with a 4096-CPR magnetic encoder.
    pub const fn default_brushed_dc_24v() -> Self {
        Self {
            personality: HardwarePersonality::HBridgeBrushedDc,
            nominal_voltage_mv: 24_000,
            stall_current_limit_ma: 2_500,
            encoder_cpr: 4_096,
            min_travel_q16: -180 * 65536,
            max_travel_q16: 180 * 65536,
            short_brake_on_failsafe: true,
        }
    }

    /// Default profile for a standard 50 Hz RC Servo (`-90..=+90 deg` -> `1000..=2000 us`).
    pub const fn default_rc_servo() -> Self {
        Self {
            personality: HardwarePersonality::PwmServoOrEsc,
            nominal_voltage_mv: 5_000,
            stall_current_limit_ma: 1_500,
            encoder_cpr: 0,
            min_travel_q16: -90 * 65536,
            max_travel_q16: 90 * 65536,
            short_brake_on_failsafe: false,
        }
    }
}

/// Concrete electrical command emitted by the Universal Driver multiplexer on every 1 kHz tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UniversalPinCommand {
    /// Hardware interlock disabled (`SAFE_ENABLE = LOW`, all power stages tri-stated or braked).
    HardwareInterlockDisabled { short_brake: bool },
    /// RC Servo / ESC pulse width in microseconds (`1000..=2000 us`).
    PwmPulseWidthUs { pulse_us: u16 },
    /// H-Bridge duty cycle (`in1_permille: 0..=1000`, `in2_permille: 0..=1000`, `brake: bool`).
    HBridgeDuty {
        in1_permille: u16,
        in2_permille: u16,
        active_brake: bool,
    },
    /// Stepper pulse rate (`step_freq_hz`, `dir_forward`, `enable_driver`).
    StepperPulse {
        step_freq_hz: u32,
        dir_forward: bool,
        enable_driver: bool,
    },
    /// Solenoid / Relay peak-and-hold PWM duty (`duty_permille: 0..=1000`).
    SolenoidHoldDuty { duty_permille: u16 },
    /// Compact 8-byte CAN-Bus frame (`can_id`, `payload`).
    CanBusFrame { can_id: u16, data: [u8; 8] },
    /// Half-duplex RS-485 / Modbus / Smart-Servo register write.
    Rs485WriteRegister {
        slave_id: u8,
        reg_addr: u16,
        value_s16: i16,
    },
    /// 0–10V Analog DAC output in millivolts (`0..=10_000 mV`).
    AnalogVoltageMv { millivolts: u16 },
}

/// Low-level electrical bus trait implemented by the physical Telekine Universal Driver PCB.
pub trait UniversalDriverBusHal {
    /// Read the harness `ID_SENSE` EEPROM / resistor to auto-detect the attached hardware.
    fn detect_attached_hardware(&mut self) -> AttachedHardwareDescriptor;

    /// Pulse the hardware watchdog `WD_PULSE` pin to keep the physical `SAFE_ENABLE` AND-gate closed.
    /// If not called every `< 5 ms`, the hardware RC circuit opens `SAFE_ENABLE` automatically.
    fn pulse_hardware_watchdog_gate(&mut self);

    /// Immediately de-assert `SAFE_ENABLE` in hardware, cutting power to all output stages.
    fn trip_hardware_interlock(&mut self);

    /// Sample motor/actuator phase current in milliamps from the `IPROPI` / INA240 ADC channel.
    fn read_current_ma(&mut self) -> u16;

    /// Read the universal feedback sensor (Quadrature PCNT, AS5600 I2C, or 0–3.3V potentiometer)
    /// normalized to Q16.16 degrees or mm.
    fn read_feedback_pos_q16(&mut self) -> i32;

    /// Apply the translated `UniversalPinCommand` to the physical output stage.
    fn apply_pin_command(&mut self, cmd: UniversalPinCommand);
}

/// Universal Hardware Driver bridge translating `telekine-core`'s 1 kHz `HardwareOutput`
/// and `BoundedIntent` into electrical signals for any attached actuator.
pub struct UniversalDriverController<B: UniversalDriverBusHal> {
    bus: B,
    descriptor: AttachedHardwareDescriptor,
}

impl<B: UniversalDriverBusHal> UniversalDriverController<B> {
    /// Initialize the Universal Driver and auto-detect the attached hardware personality.
    pub fn new(mut bus: B) -> Self {
        let descriptor = bus.detect_attached_hardware();
        Self { bus, descriptor }
    }

    pub const fn descriptor(&self) -> AttachedHardwareDescriptor {
        self.descriptor
    }

    /// Translate the 1 kHz safety loop output (`HardwareOutput`) and current `BoundedIntent`
    /// into a deterministic `UniversalPinCommand` and drive the physical hardware bus.
    ///
    /// Also checks hardware current draw against `descriptor.stall_current_limit_ma`, returning
    /// `true` if an electrical overcurrent/stall was detected so `ActuatorNode` can fire a
    /// Report-by-Exception packet.
    pub fn update_hardware_1khz(
        &mut self,
        core_out: &HardwareOutput,
        active_intent: &BoundedIntent,
    ) -> (UniversalPinCommand, bool) {
        let current_ma = self.bus.read_current_ma();
        let overcurrent_stall = self.descriptor.stall_current_limit_ma > 0
            && current_ma > self.descriptor.stall_current_limit_ma;

        // If in Fail-Safe, E-Stop, or unconfigured, trip the physical hardware interlock gate!
        if matches!(
            core_out.state,
            ActuatorOperatingState::FailSafeLocked | ActuatorOperatingState::EmergencyStopped
        ) || self.descriptor.personality == HardwarePersonality::UnconfiguredSafe
            || overcurrent_stall
        {
            self.bus.trip_hardware_interlock();
            let cmd = UniversalPinCommand::HardwareInterlockDisabled {
                short_brake: self.descriptor.short_brake_on_failsafe,
            };
            self.bus.apply_pin_command(cmd);
            return (cmd, overcurrent_stall);
        }

        // Valid active state: pulse the hardware RC watchdog gate so SAFE_ENABLE stays asserted
        self.bus.pulse_hardware_watchdog_gate();

        let effort_signed = (core_out.effort_pct as i32).clamp(-100, 100);
        let cmd = match self.descriptor.personality {
            HardwarePersonality::UnconfiguredSafe => {
                UniversalPinCommand::HardwareInterlockDisabled { short_brake: true }
            }
            HardwarePersonality::PwmServoOrEsc => {
                // Map target angle (-90..+90 deg) or effort (-100..+100%) to 1000..=2000 us
                let pulse_us = match *active_intent {
                    BoundedIntent::MoveToBounded { target_pos_q16, .. } => {
                        let clamped = target_pos_q16
                            .clamp(self.descriptor.min_travel_q16, self.descriptor.max_travel_q16);
                        let deg = clamped / 65536;
                        (1500 + (deg * 500) / 90).clamp(1000, 2000) as u16
                    }
                    _ => (1500 + (effort_signed * 5)).clamp(1000, 2000) as u16,
                };
                UniversalPinCommand::PwmPulseWidthUs { pulse_us }
            }
            HardwarePersonality::HBridgeBrushedDc => {
                let duty = (effort_signed.unsigned_abs() as u16) * 10; // 0..=1000 permille
                if effort_signed > 0 {
                    UniversalPinCommand::HBridgeDuty {
                        in1_permille: duty,
                        in2_permille: 0,
                        active_brake: false,
                    }
                } else if effort_signed < 0 {
                    UniversalPinCommand::HBridgeDuty {
                        in1_permille: 0,
                        in2_permille: duty,
                        active_brake: false,
                    }
                } else {
                    UniversalPinCommand::HBridgeDuty {
                        in1_permille: 0,
                        in2_permille: 0,
                        active_brake: true,
                    }
                }
            }
            HardwarePersonality::StepperStepDir => {
                let dir_forward = effort_signed >= 0;
                let step_freq_hz = (core_out.effective_max_speed_pct as u32) * 200; // up to 20 kHz
                UniversalPinCommand::StepperPulse {
                    step_freq_hz: if effort_signed == 0 { 0 } else { step_freq_hz },
                    dir_forward,
                    enable_driver: true,
                }
            }
            HardwarePersonality::SolenoidOrRelay => {
                let duty_permille = if effort_signed > 0 {
                    (core_out.effective_max_torque_pct as u16) * 10
                } else {
                    0
                };
                UniversalPinCommand::SolenoidHoldDuty { duty_permille }
            }
            HardwarePersonality::CanBusSmartActuator => {
                let pos_bytes = match *active_intent {
                    BoundedIntent::MoveToBounded { target_pos_q16, .. } => {
                        target_pos_q16.to_le_bytes()
                    }
                    _ => [0u8; 4],
                };
                let data = [
                    pos_bytes[0],
                    pos_bytes[1],
                    pos_bytes[2],
                    pos_bytes[3],
                    core_out.effective_max_speed_pct,
                    core_out.effective_max_torque_pct,
                    core_out.effort_pct as u8,
                    0xA5,
                ];
                UniversalPinCommand::CanBusFrame {
                    can_id: 0x140,
                    data,
                }
            }
            HardwarePersonality::Rs485SerialActuator => UniversalPinCommand::Rs485WriteRegister {
                slave_id: 1,
                reg_addr: 0x0074, // Goal Position / Velocity register
                value_s16: (effort_signed * 10) as i16,
            },
            HardwarePersonality::Analog0To10V => {
                let mv = ((effort_signed.max(0) as u16) * 100).min(10_000);
                UniversalPinCommand::AnalogVoltageMv { millivolts: mv }
            }
        };

        self.bus.apply_pin_command(cmd);
        (cmd, false)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    struct MockUniversalPcb {
        descriptor: AttachedHardwareDescriptor,
        current_ma: u16,
        watchdog_pulsed: bool,
        interlock_tripped: bool,
        last_cmd: Option<UniversalPinCommand>,
    }

    impl UniversalDriverBusHal for MockUniversalPcb {
        fn detect_attached_hardware(&mut self) -> AttachedHardwareDescriptor {
            self.descriptor
        }
        fn pulse_hardware_watchdog_gate(&mut self) {
            self.watchdog_pulsed = true;
            self.interlock_tripped = false;
        }
        fn trip_hardware_interlock(&mut self) {
            self.interlock_tripped = true;
            self.watchdog_pulsed = false;
        }
        fn read_current_ma(&mut self) -> u16 {
            self.current_ma
        }
        fn read_feedback_pos_q16(&mut self) -> i32 {
            0
        }
        fn apply_pin_command(&mut self, cmd: UniversalPinCommand) {
            self.last_cmd = Some(cmd);
        }
    }

    #[test]
    fn universal_driver_translates_hbridge_and_trips_on_overcurrent() {
        let pcb = MockUniversalPcb {
            descriptor: AttachedHardwareDescriptor::default_brushed_dc_24v(),
            current_ma: 800,
            watchdog_pulsed: false,
            interlock_tripped: false,
            last_cmd: None,
        };
        let mut drv = UniversalDriverController::new(pcb);

        let normal_out = HardwareOutput {
            state: ActuatorOperatingState::NormalDualLink,
            effort_pct: 60,
            effective_max_speed_pct: 50,
            effective_max_torque_pct: 60,
            brakes_locked: false,
        };
        let intent = BoundedIntent::move_to_deg(45.0, 50, 60, 250);

        let (cmd, stalled) = drv.update_hardware_1khz(&normal_out, &intent);
        assert!(!stalled);
        assert_eq!(
            cmd,
            UniversalPinCommand::HBridgeDuty {
                in1_permille: 600,
                in2_permille: 0,
                active_brake: false
            }
        );

        // Simulate motor stall current spike (3,200 mA > 2,500 mA limit)
        drv.bus.current_ma = 3_200;
        let (stall_cmd, stalled) = drv.update_hardware_1khz(&normal_out, &intent);
        assert!(stalled);
        assert_eq!(
            stall_cmd,
            UniversalPinCommand::HardwareInterlockDisabled { short_brake: true }
        );
    }
}
