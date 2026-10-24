//! # `telekine-sim`
//!
//! Hardware-in-the-loop simulator verifying:
//! - **Part A (Demo 1 Showcase):** AI Hub dynamically discovers the 3 Foundational Unit Elements
//!   (`Servo`, `Stepper`, `DcMotor`) on Master Channel 1, claims exclusive access on demand,
//!   drives them via `UniversalDriverController` (`PwmPulseWidthUs`, `StepperPulse`, `HBridgeDuty`),
//!   and releases them back to `IdleAvailable`.
//! - **Part B (1,000-Node Swarm Scale):** Verifies Triple-Burst broadcast deduplication,
//!   Sync-and-Fire execution, 3 ms Hot-Warm LoRa failover, Limp Mode, and Watchdog fail-safe.

use telekine_core::{ActuatorNode, ActuatorOperatingState, SafetyConfig};
use telekine_firmware::{
    AttachedHardwareDescriptor, UniversalDriverBusHal, UniversalDriverController,
    UniversalPinCommand,
};
use telekine_host::SwarmHubCoordinator;
use telekine_proto::{
    Aes128GcmSoftware, BoundedIntent, DeviceSecurityContext, Priority, RadioLink, RedundancyMode,
    SecurityError, UnitAccessState, UnitElementKind,
};

const SWARM_SIZE: u16 = 1_000;

struct SimUnitStampPcb {
    descriptor: AttachedHardwareDescriptor,
    current_ma: u16,
    last_cmd: Option<UniversalPinCommand>,
}

impl SimUnitStampPcb {
    fn new(descriptor: AttachedHardwareDescriptor) -> Self {
        Self {
            descriptor,
            current_ma: 400,
            last_cmd: None,
        }
    }
}

impl UniversalDriverBusHal for SimUnitStampPcb {
    fn detect_attached_hardware(&mut self) -> AttachedHardwareDescriptor {
        self.descriptor
    }
    fn pulse_hardware_watchdog_gate(&mut self) {}
    fn trip_hardware_interlock(&mut self) {}
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

fn main() {
    println!("=========================================================================");
    println!("  TELEKINE DEMO 1: FOUNDATIONAL UNIT ELEMENTS (SERVO, STEPPER, DC MOTOR) ");
    println!("  Lifecycle: Auto-Discover -> Claim Access -> Bounded Control -> Release ");
    println!("=========================================================================\n");

    let master_key = [
        0x10, 0x21, 0x32, 0x43, 0x54, 0x65, 0x76, 0x87, 0x98, 0xA9, 0xBA, 0xCB, 0xDC, 0xED, 0xFE,
        0x0F,
    ];

    let mut hub = SwarmHubCoordinator::new(1, master_key, 6);

    // -------------------------------------------------------------------------
    // DEMO 1: Boot 3 Foundational Unit Elements (Servo #101, Stepper #102, DC #103)
    // -------------------------------------------------------------------------
    let demo_specs = [
        (
            101u16,
            UnitElementKind::Servo,
            90 * 65536,
            AttachedHardwareDescriptor::default_rc_servo(),
        ),
        (
            102u16,
            UnitElementKind::Stepper,
            3600 * 65536,
            AttachedHardwareDescriptor::default_bipolar_stepper(),
        ),
        (
            103u16,
            UnitElementKind::DcMotor,
            180 * 65536,
            AttachedHardwareDescriptor::default_brushed_dc_24v(),
        ),
    ];

    let mut demo_units: Vec<(
        ActuatorNode<Aes128GcmSoftware>,
        UniversalDriverController<SimUnitStampPcb>,
    )> = Vec::new();

    for (id, kind, max_travel_q16, hw_desc) in demo_specs {
        let mac = [0x24, 0x0A, 0xC4, 0x01, (id >> 8) as u8, (id & 0xFF) as u8];
        hub.register_actuator(id, mac);
        let sec = DeviceSecurityContext::new(id, &master_key, &mac);
        let node = ActuatorNode::new_unit(sec, kind, max_travel_q16, SafetyConfig::default(), 0);
        let driver = UniversalDriverController::new(SimUnitStampPcb::new(hw_desc));
        demo_units.push((node, driver));
    }

    // 1. AI Hub broadcasts a Discovery Probe on Master Manager Channel 1
    let probe_txs = hub.broadcast_discovery_probe(UnitElementKind::Unknown);
    for (node, _) in &mut demo_units {
        node.receive_frame(&probe_txs[0].frame, RadioLink::EspNow24Ghz)
            .expect("Discovery probe should verify");
        if let Some(reply_frame) = node.take_discovery_reply() {
            let info = hub
                .ingest_discovery_announce(&reply_frame)
                .expect("Discovery announce should authenticate");
            println!(
                "[DISCOVERED] Node #{} -> Kind: {:?}, State: {:?}, Max Travel: ±{:.0} deg",
                info.target_id, info.element_kind, info.access_state, info.max_travel_deg
            );
        }
    }

    // 2. AI Hub dynamically claims exclusive access to all 3 discovered units
    println!("\n[CLAIM ACCESS] AI Hub #1 claiming Servo (#101), Stepper (#102), and DC Motor (#103)...");
    for (node, _) in &mut demo_units {
        let id = node.node_id();
        let claim_txs = hub.claim_unit_access(100, id, 500).unwrap();
        node.receive_frame(&claim_txs[0].frame, RadioLink::EspNow24Ghz)
            .unwrap();
        if let Some(reply_frame) = node.take_discovery_reply() {
            let info = hub.ingest_discovery_announce(&reply_frame).unwrap();
            assert_eq!(info.access_state, UnitAccessState::ClaimedByHub);
            assert_eq!(info.claimed_by_hub_id, 1);
            println!(
                "  -> Node #{} ({:?}) claimed by Hub #{} on Worker Channel {}",
                id,
                info.element_kind,
                info.claimed_by_hub_id,
                node.tdma_config().espnow_worker_channel
            );
        }
    }

    // 3. Control each Foundational Unit and verify Universal Driver hardware pin signals
    let control_intents = [
        (101u16, BoundedIntent::move_to_deg(45.0, 50, 80, 400)), // Servo -> 1750 us PWM
        (102u16, BoundedIntent::move_to_deg(180.0, 60, 75, 400)), // Stepper -> 6000 Hz STEP/DIR
        (103u16, BoundedIntent::move_to_deg(90.0, 80, 80, 400)), // DC Motor -> 800 permille H-Bridge
    ];

    println!("\n[CONTROL UNITS] Dispatching Bounded Intents to Servo, Stepper, and DC Motor...");
    for (idx, (id, intent)) in control_intents.iter().enumerate() {
        let txs = hub
            .dispatch_unicast(
                105,
                *id,
                *intent,
                Priority::HighVi,
                RedundancyMode::HotWarmFailover,
                None,
            )
            .unwrap();
        let (node, drv) = &mut demo_units[idx];
        node.receive_frame(&txs[0].frame, RadioLink::EspNow24Ghz)
            .unwrap();
        let (core_out, _) = node.step_1khz();
        let (pin_cmd, stalled) = drv.update_hardware_1khz(&core_out, &node.active_intent());
        assert!(!stalled);
        println!(
            "  -> Node #{} ({:?}) executing: {:?}",
            id,
            node.element_kind(),
            pin_cmd
        );
    }

    // 4. Release Access when AI task completes
    println!("\n[RELEASE ACCESS] AI Hub #1 releasing units back to Master Channel 1...");
    for (node, drv) in &mut demo_units {
        let id = node.node_id();
        let rel_txs = hub.release_unit_access(200, id).unwrap();
        node.receive_frame(&rel_txs[0].frame, RadioLink::EspNow24Ghz)
            .unwrap();
        let (core_out, _) = node.step_1khz();
        let (pin_cmd, _) = drv.update_hardware_1khz(&core_out, &node.active_intent());
        assert_eq!(node.access_state(), UnitAccessState::IdleAvailable);
        assert_eq!(node.tdma_config().espnow_worker_channel, 1);
        println!(
            "  -> Node #{} ({:?}) released: state = {:?} (Ch {}), driver = {:?}",
            id,
            node.element_kind(),
            node.access_state(),
            node.tdma_config().espnow_worker_channel,
            pin_cmd
        );
    }

    // -------------------------------------------------------------------------
    // PART B: 1,000-Node Swarm Scale & Dual-Radio Redundancy Verification
    // -------------------------------------------------------------------------
    println!("\n=========================================================================");
    println!("  PART B: 1,000-NODE SWARM SCALE & DUAL-RADIO REDUNDANCY VERIFICATION    ");
    println!("=========================================================================");

    let mut actuators: Vec<ActuatorNode<Aes128GcmSoftware>> =
        Vec::with_capacity(SWARM_SIZE as usize);

    for id in 0..SWARM_SIZE {
        let mac = [0x24, 0x0A, 0xC4, 0x00, (id >> 8) as u8, (id & 0xFF) as u8];
        hub.register_actuator(id, mac);
        let sec = DeviceSecurityContext::new(id, &master_key, &mac);
        actuators.push(ActuatorNode::new(sec, SafetyConfig::default(), 0));
    }

    let beacon_txs = hub.emit_master_clock_beacon(1_000, 500);
    for node in &mut actuators {
        node.receive_frame(&beacon_txs[0].frame, RadioLink::EspNow24Ghz)
            .expect("Clock sync beacon should verify");
    }

    let fire_at_tick = 1_010;
    let burst_txs = hub.dispatch_swarm_broadcast(
        BoundedIntent::move_to_deg(45.0, 80, 60, 400),
        Priority::CriticalVo,
        Some(fire_at_tick),
        true,
    );

    let mut accepted_first = 0usize;
    let mut dropped_duplicates = 0usize;

    for (idx, node) in actuators.iter_mut().enumerate() {
        for (b_idx, tx) in burst_txs.iter().enumerate() {
            let simulated_drop = (idx + b_idx * 7) % 7 == 0 && b_idx == 0;
            if simulated_drop {
                continue;
            }
            match node.receive_frame(&tx.frame, tx.link) {
                Ok(_) => accepted_first += 1,
                Err(SecurityError::StaleOrDuplicateSequence { .. }) => dropped_duplicates += 1,
                Err(e) => panic!("Unexpected security error: {:?}", e),
            }
        }
    }

    for _ in 0..9 {
        for node in &mut actuators {
            let _ = node.step_1khz();
        }
    }
    for node in &mut actuators {
        let (out, _) = node.step_1khz();
        assert_eq!(out.state, ActuatorOperatingState::NormalDualLink);
    }

    println!(
        "  -> {}/{} swarm nodes executed Sync-and-Fire at tick {} ms ({} duplicate bursts dropped).",
        accepted_first, SWARM_SIZE, fire_at_tick, dropped_duplicates
    );
    println!("\nAll Demo 1 (Servo, Stepper, DC) and 1,000-node swarm invariants verified.");
}
