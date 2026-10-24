//! Regression & integration test harness for verifying Demo 1 Foundational Units
//! (`Servo`, `Stepper`, `DcMotor`) and changes built with Google Jules.

use telekine_core::{ActuatorNode, ActuatorOperatingState, SafetyConfig};
use telekine_firmware::{
    AttachedHardwareDescriptor, UniversalDriverBusHal, UniversalDriverController,
    UniversalPinCommand,
};
use telekine_host::SwarmHubCoordinator;
use telekine_proto::{
    Aes128GcmSoftware, BoundedIntent, DeviceSecurityContext, Priority, RadioLink, RedundancyMode,
    UnitAccessState, UnitElementKind, WIRE_PACKET_SIZE,
};

struct DummyBus(AttachedHardwareDescriptor);
impl UniversalDriverBusHal for DummyBus {
    fn detect_attached_hardware(&mut self) -> AttachedHardwareDescriptor {
        self.0
    }
    fn pulse_hardware_watchdog_gate(&mut self) {}
    fn trip_hardware_interlock(&mut self) {}
    fn read_current_ma(&mut self) -> u16 {
        250
    }
    fn read_feedback_pos_q16(&mut self) -> i32 {
        0
    }
    fn apply_pin_command(&mut self, _cmd: UniversalPinCommand) {}
}

#[test]
fn jules_invariant_wire_frame_is_strictly_32_bytes() {
    assert_eq!(WIRE_PACKET_SIZE, 32);
    assert_eq!(core::mem::size_of::<telekine_proto::EncryptedFrame>(), 32);
}

#[test]
fn demo1_discover_claim_control_and_release_servo_stepper_dc() {
    let master_key = [0x5Au8; 16];
    let mut hub = SwarmHubCoordinator::new(1, master_key, 6);

    let mac_servo = [0x24, 0x0A, 0xC4, 0x00, 0x00, 0x01];
    let mac_stepper = [0x24, 0x0A, 0xC4, 0x00, 0x00, 0x02];
    let mac_dc = [0x24, 0x0A, 0xC4, 0x00, 0x00, 0x03];

    hub.register_actuator(1, mac_servo);
    hub.register_actuator(2, mac_stepper);
    hub.register_actuator(3, mac_dc);

    let mut servo = ActuatorNode::new_unit(
        DeviceSecurityContext::<Aes128GcmSoftware>::new(1, &master_key, &mac_servo),
        UnitElementKind::Servo,
        90 * 65536,
        SafetyConfig::default(),
        0,
    );
    let mut stepper = ActuatorNode::new_unit(
        DeviceSecurityContext::<Aes128GcmSoftware>::new(2, &master_key, &mac_stepper),
        UnitElementKind::Stepper,
        3600 * 65536,
        SafetyConfig::default(),
        0,
    );
    let mut dc = ActuatorNode::new_unit(
        DeviceSecurityContext::<Aes128GcmSoftware>::new(3, &master_key, &mac_dc),
        UnitElementKind::DcMotor,
        180 * 65536,
        SafetyConfig::default(),
        0,
    );

    // 1. Discover all 3 foundational units
    let probes = hub.broadcast_discovery_probe(UnitElementKind::Unknown);
    for node in [&mut servo, &mut stepper, &mut dc] {
        node.receive_frame(&probes[0].frame, RadioLink::EspNow24Ghz)
            .unwrap();
        let reply = node.take_discovery_reply().expect("Should emit announce");
        hub.ingest_discovery_announce(&reply).unwrap();
    }
    assert_eq!(hub.list_discovered_units(None).len(), 3);
    assert_eq!(
        hub.list_discovered_units(Some(UnitElementKind::Servo)).len(),
        1
    );

    // 2. Claim Servo (#1), control it to +45 deg, and verify 1750 us PWM pulse
    let claim_tx = hub.claim_unit_access(10, 1, 500).unwrap();
    servo
        .receive_frame(&claim_tx[0].frame, RadioLink::EspNow24Ghz)
        .unwrap();
    assert_eq!(servo.access_state(), UnitAccessState::ClaimedByHub);

    let move_tx = hub
        .dispatch_unicast(
            12,
            1,
            BoundedIntent::move_to_deg(45.0, 50, 80, 300),
            Priority::CriticalVo,
            RedundancyMode::HotHotTwinCast,
            None,
        )
        .unwrap();
    servo
        .receive_frame(&move_tx[0].frame, RadioLink::EspNow24Ghz)
        .unwrap();
    let (out, _) = servo.step_1khz();

    let mut servo_drv =
        UniversalDriverController::new(DummyBus(AttachedHardwareDescriptor::default_rc_servo()));
    let (cmd, stalled) = servo_drv.update_hardware_1khz(&out, &servo.active_intent());
    assert!(!stalled);
    assert_eq!(cmd, UniversalPinCommand::PwmPulseWidthUs { pulse_us: 1750 });

    // 3. Release Servo (#1) and confirm it returns to IdleAvailable on Channel 1
    let rel_tx = hub.release_unit_access(20, 1).unwrap();
    servo
        .receive_frame(&rel_tx[0].frame, RadioLink::EspNow24Ghz)
        .unwrap();
    assert_eq!(servo.access_state(), UnitAccessState::IdleAvailable);
    assert_eq!(servo.tdma_config().espnow_worker_channel, 1);
}
