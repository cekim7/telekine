//! Regression & integration test harness for verifying changes built with Google Jules.
//!
//! Run via:
//!   `cargo test --test jules_regression`

use telekine_core::{ActuatorNode, ActuatorOperatingState, SafetyConfig};
use telekine_host::SwarmHubCoordinator;
use telekine_proto::{
    Aes128GcmSoftware, BoundedIntent, DeviceSecurityContext, Priority, RadioLink, RedundancyMode,
    WIRE_PACKET_SIZE,
};

#[test]
fn jules_invariant_wire_frame_is_strictly_32_bytes() {
    assert_eq!(WIRE_PACKET_SIZE, 32);
    assert_eq!(core::mem::size_of::<telekine_proto::EncryptedFrame>(), 32);
}

#[test]
fn jules_invariant_hot_hot_twincast_first_valid_wins() {
    let master_key = [0x5Au8; 16];
    let mac = [0x24, 0x0A, 0xC4, 0xAA, 0xBB, 0xCC];
    let mut hub = SwarmHubCoordinator::new(1, master_key, 6);
    hub.register_actuator(10, mac);

    let mut node = ActuatorNode::new(
        DeviceSecurityContext::<Aes128GcmSoftware>::new(10, &master_key, &mac),
        SafetyConfig::default(),
        0,
    );

    let txs = hub
        .dispatch_unicast(
            100,
            10,
            BoundedIntent::move_to_deg(60.0, 50, 70, 300),
            Priority::CriticalVo,
            RedundancyMode::HotHotTwinCast,
            None,
        )
        .expect("Twin-cast should succeed");
    assert_eq!(txs.len(), 2);

    // Fast ESP-NOW arrives first (1-3 ms)
    assert!(node.receive_frame(&txs[0].frame, RadioLink::EspNow24Ghz).is_ok());
    // Slow LoRa arrives second (30-80 ms) with identical seq_nonce -> must be dropped
    assert!(node.receive_frame(&txs[1].frame, RadioLink::RawLoRa915Mhz).is_err());
    assert_eq!(node.operating_state(), ActuatorOperatingState::NormalDualLink);
}
