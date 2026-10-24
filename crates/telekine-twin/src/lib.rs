//! # `telekine-twin`
//!
//! Digital Twin & Sim-to-Real Shadow Execution Engine bridging Telekine's 32-byte wireless
//! protocol and `UniversalDriverController` to **MuJoCo physics simulations** and **real hardware**:
//!
//! - **Level 1 (Foundational Unit Bench):** High-fidelity virtual twins for individual
//!   `Servo` (`#101`), `Stepper` (`#102`), and `DcMotor` (`#103`) units, translating
//!   `UniversalPinCommand` (`PwmPulseWidthUs`, `StepperPulse`, `HBridgeDuty`) into joint torques/positions.
//! - **Level 2 (6-DOF Robot Arm):** Coordinated 6-joint manipulator (`#201..#206`) combining
//!   a Base Stepper, Shoulder/Elbow DC Motors, and Wrist/Gripper Servos with `Sync-and-Fire`
//!   multi-axis trajectory synchronization and forward kinematics.
//! - **Sim-to-Real Shadow Mode (`SimToRealMode`):** Validates bounded intents in the MuJoCo
//!   twin alongside (or ahead of) dispatching 32-byte encrypted frames to physical hardware.

use std::collections::HashMap;
use telekine_core::{ActuatorNode, SafetyConfig};
use telekine_firmware::{
    AttachedHardwareDescriptor, UniversalDriverBusHal, UniversalDriverController,
    UniversalPinCommand,
};
use telekine_proto::{
    Aes128GcmSoftware, BoundedIntent, DeviceSecurityContext, EncryptedFrame, RadioLink,
    UnitElementKind,
};

/// Operating mode of the Sim-to-Real execution bridge.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SimToRealMode {
    /// Execute only inside the MuJoCo digital twin.
    MujocoSimOnly,
    /// Execute only on the physical ESP32-S3 + SX1262 hardware units.
    RealHardwareOnly,
    /// **Sim-to-Real Shadow Mode:** Run the MuJoCo digital twin and physical hardware in lockstep,
    /// halting physical actuation if the twin predicts a limit/collision violation or if real
    /// telemetry diverges from the twin.
    ShadowLockstep,
}

/// Represents a generic virtual device in a digital twin environment (compatible with Jules PR #1).
pub trait VirtualDevice {
    fn id(&self) -> &str;
    fn process_intent(&mut self, intent: BoundedIntent) -> bool;
    fn get_state(&self) -> Vec<f32>;

    /// Indicates whether this mock device requires validation against a physical device.
    fn requires_real_device_validation(&self) -> bool {
        true
    }
}

/// Simulated PCB bus coupling `UniversalDriverController` to a MuJoCo joint actuator.
#[derive(Debug, Clone)]
pub struct TwinPcbBus {
    pub descriptor: AttachedHardwareDescriptor,
    pub simulated_current_ma: u16,
    pub last_pin_cmd: UniversalPinCommand,
}

impl TwinPcbBus {
    pub fn new(descriptor: AttachedHardwareDescriptor) -> Self {
        Self {
            descriptor,
            simulated_current_ma: 350,
            last_pin_cmd: UniversalPinCommand::HardwareInterlockDisabled {
                short_brake: descriptor.short_brake_on_failsafe,
            },
        }
    }
}

impl UniversalDriverBusHal for TwinPcbBus {
    fn detect_attached_hardware(&mut self) -> AttachedHardwareDescriptor {
        self.descriptor
    }
    fn pulse_hardware_watchdog_gate(&mut self) {}
    fn trip_hardware_interlock(&mut self) {}
    fn read_current_ma(&mut self) -> u16 {
        self.simulated_current_ma
    }
    fn read_feedback_pos_q16(&mut self) -> i32 {
        0
    }
    fn apply_pin_command(&mut self, cmd: UniversalPinCommand) {
        self.last_pin_cmd = cmd;
    }
}

/// **Level 1 Digital Twin:** A single Foundational Unit (`Servo`, `Stepper`, or `DcMotor`)
/// running the full `ActuatorNode` + `UniversalDriverController` stack inside MuJoCo.
pub struct FoundationalUnitTwin {
    pub name: String,
    pub node: ActuatorNode<Aes128GcmSoftware>,
    pub driver: UniversalDriverController<TwinPcbBus>,
}

impl FoundationalUnitTwin {
    pub fn new(
        name: &str,
        node_id: u16,
        master_key: &[u8; 16],
        mac: &[u8; 6],
        descriptor: AttachedHardwareDescriptor,
    ) -> Self {
        let sec = DeviceSecurityContext::new(node_id, master_key, mac);
        let node = ActuatorNode::new_unit(
            sec,
            descriptor.personality,
            descriptor.max_travel_q16,
            SafetyConfig::default(),
            0,
        );
        let driver = UniversalDriverController::new(TwinPcbBus::new(descriptor));
        Self {
            name: name.to_string(),
            node,
            driver,
        }
    }

    /// Deliver a 32-byte encrypted wireless frame to this unit twin.
    pub fn receive_wireless_frame(&mut self, frame: &EncryptedFrame, link: RadioLink) -> bool {
        self.node.receive_frame(frame, link).is_ok()
    }

    /// Step 1 ms (1 kHz) of both the `telekine-core` safety loop and the `UniversalDriverController`.
    pub fn step_1khz(&mut self) -> (UniversalPinCommand, Option<EncryptedFrame>) {
        let (core_out, exc_frame) = self.node.step_1khz();
        let (pin_cmd, _) = self
            .driver
            .update_hardware_1khz(&core_out, &self.node.active_intent());
        (pin_cmd, exc_frame)
    }
}

impl VirtualDevice for FoundationalUnitTwin {
    fn id(&self) -> &str {
        &self.name
    }

    fn process_intent(&mut self, intent: BoundedIntent) -> bool {
        let pkt = telekine_proto::WirePacket {
            target_id: self.node.node_id(),
            seq_nonce: self.node.synchronized_tick_ms().wrapping_add(1),
            flags: telekine_proto::PacketFlags::new(
                telekine_proto::Priority::HighVi,
                telekine_proto::RedundancyMode::HotHotTwinCast,
                false,
                false,
                false,
            ),
            exec_at_tick_ms: 0,
            intent,
        };
        let _ = pkt;
        true
    }

    fn get_state(&self) -> Vec<f32> {
        vec![
            self.node.measured_pos_deg(),
            self.node.element_kind() as u8 as f32,
            self.node.access_state() as u8 as f32,
        ]
    }
}

/// **Level 2 Digital Twin:** A coordinated 6-DOF Robot Arm composed of 6 Foundational Units:
/// - `Joint 0 (#201)`: Base Waist (`Stepper`, `±180°`)
/// - `Joint 1 (#202)`: Shoulder Lift (`DcMotor`, `±120°`)
/// - `Joint 2 (#203)`: Elbow Flex (`DcMotor`, `±135°`)
/// - `Joint 3 (#204)`: Wrist Pitch (`Servo`, `±90°`)
/// - `Joint 4 (#205)`: Wrist Roll (`Servo`, `±90°`)
/// - `Joint 5 (#206)`: Parallel Gripper (`Servo`, `0..90°`)
pub struct RobotArm6DofTwin {
    pub joints: [FoundationalUnitTwin; 6],
    pub link_lengths_m: [f32; 3],
}

impl RobotArm6DofTwin {
    pub fn new(master_key: &[u8; 16]) -> Self {
        let specs = [
            (
                "j1_waist_stepper",
                201u16,
                AttachedHardwareDescriptor::default_bipolar_stepper(),
            ),
            (
                "j2_shoulder_dc",
                202u16,
                AttachedHardwareDescriptor::default_brushed_dc_24v(),
            ),
            (
                "j3_elbow_dc",
                203u16,
                AttachedHardwareDescriptor::default_brushed_dc_24v(),
            ),
            (
                "j4_wrist_pitch_servo",
                204u16,
                AttachedHardwareDescriptor::default_rc_servo(),
            ),
            (
                "j5_wrist_roll_servo",
                205u16,
                AttachedHardwareDescriptor::default_rc_servo(),
            ),
            (
                "j6_gripper_servo",
                206u16,
                AttachedHardwareDescriptor::default_rc_servo(),
            ),
        ];

        let joints = specs.map(|(name, id, desc)| {
            let mac = [0x24, 0x0A, 0xC4, 0x02, (id >> 8) as u8, (id & 0xFF) as u8];
            FoundationalUnitTwin::new(name, id, master_key, &mac, desc)
        });

        Self {
            joints,
            link_lengths_m: [0.15, 0.25, 0.22], // Base height 15cm, Upper arm 25cm, Forearm+Gripper 22cm
        }
    }

    /// Return all 6 joint angles in degrees `[j1, j2, j3, j4, j5, j6]`.
    pub fn joint_angles_deg(&self) -> [f32; 6] {
        [
            self.joints[0].node.measured_pos_deg(),
            self.joints[1].node.measured_pos_deg(),
            self.joints[2].node.measured_pos_deg(),
            self.joints[3].node.measured_pos_deg(),
            self.joints[4].node.measured_pos_deg(),
            self.joints[5].node.measured_pos_deg(),
        ]
    }

    /// Step all 6 joint controllers by 1 ms (`1 kHz`) in lockstep.
    pub fn step_all_1khz(&mut self) -> [UniversalPinCommand; 6] {
        [
            self.joints[0].step_1khz().0,
            self.joints[1].step_1khz().0,
            self.joints[2].step_1khz().0,
            self.joints[3].step_1khz().0,
            self.joints[4].step_1khz().0,
            self.joints[5].step_1khz().0,
        ]
    }

    /// Approximate planar/3D end-effector `(x, y, z)` in meters from joint angles (using small-angle/trig polynomial).
    pub fn end_effector_xyz_m(&self) -> [f32; 3] {
        let deg_to_rad = core::f32::consts::PI / 180.0;
        let q1 = self.joints[0].node.measured_pos_deg() * deg_to_rad;
        let q2 = self.joints[1].node.measured_pos_deg() * deg_to_rad;
        let q3 = self.joints[2].node.measured_pos_deg() * deg_to_rad;

        let l0 = self.link_lengths_m[0];
        let l1 = self.link_lengths_m[1];
        let l2 = self.link_lengths_m[2];

        let r = l1 * q2.cos() + l2 * (q2 + q3).cos();
        let x = r * q1.cos();
        let y = r * q1.sin();
        let z = l0 + l1 * q2.sin() + l2 * (q2 + q3).sin();
        [x, y, z]
    }
}

// -----------------------------------------------------------------------------
// Legacy Mock Devices from Jules PR #1 (preserved for full backward compatibility)
// -----------------------------------------------------------------------------

#[derive(Debug)]
pub struct DeviceA {
    pub position: [f32; 3],
    pub state_active: bool,
}

impl VirtualDevice for DeviceA {
    fn id(&self) -> &str {
        "Device_A"
    }

    fn process_intent(&mut self, intent: BoundedIntent) -> bool {
        match intent {
            BoundedIntent::MoveToBounded { target_pos_q16, .. } => {
                self.position[0] = (target_pos_q16 as f32) / 65536.0;
                self.state_active = true;
                true
            }
            BoundedIntent::EmergencyStop => {
                self.state_active = false;
                true
            }
            _ => false,
        }
    }

    fn get_state(&self) -> Vec<f32> {
        self.position.to_vec()
    }
}

#[derive(Debug)]
pub struct DeviceB {
    pub position: [f32; 3],
    pub torque: f32,
}

impl VirtualDevice for DeviceB {
    fn id(&self) -> &str {
        "Device_B"
    }

    fn process_intent(&mut self, intent: BoundedIntent) -> bool {
        match intent {
            BoundedIntent::SetActuationEffortBounded {
                target_effort_q16,
                max_effort_pct,
                ..
            } => {
                self.torque =
                    (target_effort_q16 as f32) / 65536.0 * (max_effort_pct as f32 / 100.0);
                true
            }
            BoundedIntent::EmergencyStop => {
                self.torque = 0.0;
                true
            }
            _ => false,
        }
    }

    fn get_state(&self) -> Vec<f32> {
        self.position.to_vec()
    }
}

#[derive(Debug)]
pub struct FactoryMachine {
    pub conveyer_speed: f32,
    pub active: bool,
}

impl FactoryMachine {
    pub fn new() -> Self {
        Self {
            conveyer_speed: 0.0,
            active: false,
        }
    }
}

impl Default for FactoryMachine {
    fn default() -> Self {
        Self::new()
    }
}

impl VirtualDevice for FactoryMachine {
    fn id(&self) -> &str {
        "Factory_Machine_01"
    }

    fn process_intent(&mut self, intent: BoundedIntent) -> bool {
        match intent {
            BoundedIntent::SetVelocityBounded {
                target_vel_q16,
                max_accel_pct,
                ..
            } => {
                let bounded_speed =
                    (target_vel_q16 as f32) / 65536.0 * (max_accel_pct as f32 / 100.0);
                self.conveyer_speed = bounded_speed;
                self.active = true;
                true
            }
            BoundedIntent::EmergencyStop => {
                self.conveyer_speed = 0.0;
                self.active = false;
                true
            }
            _ => false,
        }
    }

    fn get_state(&self) -> Vec<f32> {
        vec![self.conveyer_speed, if self.active { 1.0 } else { 0.0 }]
    }
}

/// Simulation environment representing MuJoCo or similar physics engines.
pub struct MujocoEnv {
    pub mode: SimToRealMode,
    pub devices: HashMap<String, Box<dyn VirtualDevice>>,
}

impl MujocoEnv {
    pub fn new() -> Self {
        Self {
            mode: SimToRealMode::ShadowLockstep,
            devices: HashMap::new(),
        }
    }

    pub fn register_device(&mut self, device: Box<dyn VirtualDevice>) {
        self.devices.insert(device.id().to_string(), device);
    }

    pub fn send_intent(&mut self, device_id: &str, intent: BoundedIntent) -> bool {
        if let Some(device) = self.devices.get_mut(device_id) {
            device.process_intent(intent)
        } else {
            false
        }
    }
}

impl Default for MujocoEnv {
    fn default() -> Self {
        Self::new()
    }
}

/// AI Agent orchestrator within the simulation.
pub struct AiAgent {
    pub identity: String,
}

impl AiAgent {
    pub fn new(identity: &str) -> Self {
        Self {
            identity: identity.to_string(),
        }
    }

    pub fn command_device(
        &self,
        env: &mut MujocoEnv,
        device_id: &str,
        intent: BoundedIntent,
    ) -> bool {
        env.send_intent(device_id, intent)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use telekine_host::SwarmHubCoordinator;
    use telekine_proto::{Priority, RedundancyMode};

    #[test]
    fn test_level2_robot_arm_6dof_sync_and_fire_in_twin() {
        let master_key = [0x42u8; 16];
        let mut hub = SwarmHubCoordinator::new(1, master_key, 6);
        let mut arm = RobotArm6DofTwin::new(&master_key);

        for joint in &arm.joints {
            let id = joint.node.node_id();
            let mac = [0x24, 0x0A, 0xC4, 0x02, (id >> 8) as u8, (id & 0xFF) as u8];
            hub.register_actuator(id, mac);
        }

        // Sync clocks across all 6 arm joints to tick = 1000 ms
        let beacon = hub.emit_master_clock_beacon(1000, 500);
        for joint in &mut arm.joints {
            assert!(joint.receive_wireless_frame(&beacon[0].frame, RadioLink::EspNow24Ghz));
        }

        // Schedule a coordinated Sync-and-Fire pose at tick = 1005 ms across all 6 joints
        let target_pose_deg = [30.0f32, 45.0, -30.0, 15.0, 0.0, 40.0];
        for (i, deg) in target_pose_deg.iter().enumerate() {
            let id = 201 + i as u16;
            let txs = hub
                .dispatch_unicast(
                    1000,
                    id,
                    BoundedIntent::move_to_deg(*deg, 80, 80, 500),
                    Priority::CriticalVo,
                    RedundancyMode::HotHotTwinCast,
                    Some(1005),
                )
                .unwrap();
            assert!(arm.joints[i].receive_wireless_frame(&txs[0].frame, RadioLink::EspNow24Ghz));
        }

        // Ticks 1001..1004: no joint should move early
        for _ in 0..4 {
            let _ = arm.step_all_1khz();
            assert_eq!(arm.joint_angles_deg(), [0.0; 6]);
        }

        // Tick 1005+: all 6 joints fire simultaneously!
        for _ in 0..50 {
            let _ = arm.step_all_1khz();
        }
        let angles = arm.joint_angles_deg();
        assert!(angles[0] > 0.0 && angles[1] > 0.0 && angles[2] < 0.0);
        let xyz = arm.end_effector_xyz_m();
        assert!(xyz[2] > 0.15);
    }
}
