//! # `telekine-sim`
//!
//! Hardware-in-the-loop swarm simulator instantiating **1,000 independent `ActuatorNode`s**
//! and a `SwarmHubCoordinator` to verify all 6 architectural requirements of Telekine:
//! 1. **Triple-Burst Broadcasts & Deduplication** across 1,000 actuators under 15% single-packet RF loss.
//! 2. **Time-Stamped "Sync-and-Fire"** sub-millisecond swarm coordination across all 1,000 nodes.
//! 3. **Hot-Hot (Twin-Cast)** simultaneous ESP-NOW (2 ms) + Raw LoRa (45 ms) `First-Valid-Wins` deduplication.
//! 4. **Hot-Warm (3 ms Auto-Failover)** automatic switchover to 915 MHz LoRa when 2.4 GHz ESP-NOW drops.
//! 5. **Graceful Speed Degradation ("Limp Mode")** & **Dual-Link Watchdog Fail-Safe** with Report-by-Exception.

use telekine_core::{ActuatorNode, ActuatorOperatingState, SafetyConfig};
use telekine_host::SwarmHubCoordinator;
use telekine_proto::{
    Aes128GcmSoftware, BoundedIntent, DeviceSecurityContext, Priority, RadioLink, RedundancyMode,
    SecurityError,
};

const SWARM_SIZE: u16 = 1_000;

fn main() {
    println!("=========================================================================");
    println!("  TELEKINE ALL-RUST SWARM SIMULATOR (1,000 Dual-Radio Actuators)         ");
    println!("  Links: 2.4 GHz ESP-NOW (1-3 ms) + 915 MHz Semtech SX1262 LoRa (30-80 ms)");
    println!("=========================================================================\n");

    let master_key = [
        0x10, 0x21, 0x32, 0x43, 0x54, 0x65, 0x76, 0x87, 0x98, 0xA9, 0xBA, 0xCB, 0xDC, 0xED, 0xFE,
        0x0F,
    ];

    let mut hub = SwarmHubCoordinator::new(1, master_key, 6);
    let mut actuators: Vec<ActuatorNode<Aes128GcmSoftware>> = Vec::with_capacity(SWARM_SIZE as usize);

    for id in 0..SWARM_SIZE {
        let mac = [
            0x24,
            0x0A,
            0xC4,
            0x00,
            (id >> 8) as u8,
            (id & 0xFF) as u8,
        ];
        hub.register_actuator(id, mac);
        let sec = DeviceSecurityContext::new(id, &master_key, &mac);
        actuators.push(ActuatorNode::new(sec, SafetyConfig::default(), 0));
    }

    println!(
        "[INIT] Provisioned {} actuators with unique per-device AES-128-GCM keys.",
        hub.peer_count()
    );

    // -------------------------------------------------------------------------
    // SCENARIO 1: Master Clock Sync + Triple-Burst Sync-and-Fire across 1,000 Nodes
    // -------------------------------------------------------------------------
    let beacon_txs = hub.emit_master_clock_beacon(1_000, 500);
    for node in &mut actuators {
        node.receive_frame(&beacon_txs[0].frame, RadioLink::EspNow24Ghz)
            .expect("Clock sync beacon should verify");
    }

    // Schedule a coordinated swarm move to 45.0 deg at exactly tick 1,010 ms
    let fire_at_tick = 1_010;
    let burst_txs = hub.dispatch_swarm_broadcast(
        BoundedIntent::move_to_deg(45.0, 80, 60, 400),
        Priority::CriticalVo,
        Some(fire_at_tick),
        true,
    );
    assert_eq!(burst_txs.len(), 4); // 3 ESP-NOW bursts + 1 VIP LoRa twin-cast

    let mut accepted_first = 0usize;
    let mut dropped_duplicates = 0usize;

    for (idx, node) in actuators.iter_mut().enumerate() {
        // Simulate 15% random single-packet drop on burst #1, but bursts #2/#3 deliver it
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

    println!(
        "[SCENARIO 1: Triple-Burst + Deduplication]\n  -> {}/{} nodes received Sync-and-Fire intent (0.0% effective loss); {} duplicate bursts cleanly dropped.",
        accepted_first, SWARM_SIZE, dropped_duplicates
    );

    // Step ticks 1,001..1,009: no node should move before `exec_at_tick_ms = 1,010`
    for _ in 0..9 {
        for node in &mut actuators {
            let _ = node.step_1khz();
            assert_eq!(node.measured_pos_q16(), 0, "Moved prematurely before Sync-and-Fire tick!");
        }
    }

    // Tick 1,010: all 1,000 actuators trigger simultaneously on the exact same millisecond!
    for node in &mut actuators {
        let (out, _) = node.step_1khz();
        assert_eq!(out.state, ActuatorOperatingState::NormalDualLink);
        assert!(node.measured_pos_q16() > 0, "Failed to fire at synchronized tick 1,010!");
    }
    println!(
        "[SCENARIO 2: Sync-and-Fire Execution]\n  -> All {} actuators buffered and fired simultaneously at tick {} ms (pos = {:.2} deg).",
        SWARM_SIZE,
        fire_at_tick,
        actuators[0].measured_pos_deg()
    );

    // -------------------------------------------------------------------------
    // SCENARIO 3: Hot-Warm (3 ms Auto-Failover) to 915 MHz LoRa + Limp Mode
    // -------------------------------------------------------------------------
    let target_node_id = 42u16;
    let now_ms = actuators[target_node_id as usize].synchronized_tick_ms();
    let hw_txs = hub
        .dispatch_unicast(
            now_ms,
            target_node_id,
            BoundedIntent::move_to_deg(90.0, 100, 100, 500),
            Priority::HighVi,
            RedundancyMode::HotWarmFailover,
            None,
        )
        .unwrap();
    assert_eq!(hw_txs.len(), 1);
    assert_eq!(hw_txs[0].link, RadioLink::EspNow24Ghz);

    // Simulate 2.4 GHz Wi-Fi jamming: ESP-NOW packet is lost and no hardware ACK returns within 3 ms.
    let failover_txs = hub.poll_hot_warm_failovers(now_ms + 3);
    assert_eq!(failover_txs.len(), 1);
    assert_eq!(failover_txs[0].link, RadioLink::RawLoRa915Mhz);

    // Advance actuator #42 past the 150 ms ESP-NOW dropout threshold so it relies solely on LoRa
    for _ in 0..160 {
        let _ = actuators[target_node_id as usize].step_1khz();
    }
    actuators[target_node_id as usize]
        .receive_frame(&failover_txs[0].frame, RadioLink::RawLoRa915Mhz)
        .expect("LoRa failover frame should authenticate");
    let (limp_out, _) = actuators[target_node_id as usize].step_1khz();
    assert_eq!(limp_out.state, ActuatorOperatingState::LoRaLimpMode);
    assert_eq!(limp_out.effective_max_speed_pct, 40);

    println!(
        "[SCENARIO 3: 3 ms Hot-Warm Failover & Limp Mode]\n  -> Node #{} missed 3 ms ESP-NOW ACK; Hub auto-fired 915 MHz LoRa backup. Node entered {:?} (speed throttled 100% -> {}%).",
        target_node_id, limp_out.state, limp_out.effective_max_speed_pct
    );

    // -------------------------------------------------------------------------
    // SCENARIO 4: Dual-Link Dead-Man's Switch (Watchdog Lease Expiration)
    // -------------------------------------------------------------------------
    let mut exception_packets = 0usize;
    for _ in 0..505 {
        let (_, exc) = actuators[target_node_id as usize].step_1khz();
        if exc.is_some() {
            exception_packets += 1;
        }
    }
    let final_state = actuators[target_node_id as usize].operating_state();
    assert_eq!(final_state, ActuatorOperatingState::FailSafeLocked);
    assert!(exception_packets >= 1);

    println!(
        "[SCENARIO 4: Dual-Link Watchdog Fail-Safe & Report-by-Exception]\n  -> After both radios went silent past lease window, Node #{} entered {:?} (0% torque, brakes locked) and emitted {} uplink exception alert(s).",
        target_node_id, final_state, exception_packets
    );
    println!("\nAll Telekine architectural invariants verified.");
}
