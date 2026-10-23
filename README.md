# Telekine

**All-Rust Wireless AI Control Stack for 1,000+ Low-Intelligence Actuators via Dual-Radio Redundancy (ESP-NOW + Raw LoRa)**

Telekine enables an AI agent to deterministically and safely orchestrate swarms of **1,000+ low-intelligence wireless actuators** across **100+ hubs** using parallel physical-layer redundancy:
- **ESP-NOW (2.4 GHz, ESP32-S3 / ESP32-C3):** Ultra-fast local control (`1–3 ms` latency, `100–250 m` range, WMM `AC_VO` priority queuing).
- **Raw LoRa (915 MHz, Semtech SX1262):** Long-range, wall-penetrating backup (`30–80 ms` latency, `1–10+ km` range, `SF7` VIP interrupt frequency + Capture Effect power boost).

---

## Architectural Highlights

| Pillar | Implementation in Telekine |
| :--- | :--- |
| **1. Dual-Radio Redundancy** | **Hot-Hot (Twin-Cast)** fires critical intents & E-Stops on ESP-NOW and LoRa simultaneously (`First-Valid-Wins`). **Hot-Warm (Auto-Failover)** fires on ESP-NOW first and triggers LoRa failover if hardware `ACK` misses the `3 ms` deadline. |
| **2. 1,000+ Peer Scale & Coexistence** | Connectionless addressing via a 2-byte `target_id` (`up to 65,534 nodes` + broadcast `0xFFFF`), dynamic 20-slot ESP-NOW RAM peer-swapping, **Master Manager Channel** (Ch 1 / 915.0 MHz) + **Worker Channels** (Ch 6, 11), **TDMA millisecond time-slotting**, and **Triple-Burst Broadcasts** (`< 0.1%` packet loss). |
| **3. Priority & Capture Effect** | Hardware 802.11 WMM/EDCA **`AC_VO` (Voice)** tagging for urgent ESP-NOW frames, dedicated **LoRa `SF7` VIP Emergency channel**, and dynamic TX power scaling (`+10 dBm` nominal for spatial reuse vs. `+22 to +30 dBm` emergency boost to trigger the **RF Capture Effect**). |
| **4. 32-Byte Wire Protocol & Deduplication** | Zero-copy `#[repr(C, packed)]` 32-byte binary frame (`WirePacket`) with 4-byte monotonic `seq_nonce` (`if incoming_seq <= last_executed_seq { drop() }`) and `exec_at_tick_ms` for sub-millisecond **Sync-and-Fire** swarm coordination. |
| **5. Physical Safety & Bounded Intents** | AI never sends raw open-loop ON/OFF toggles. Every command is a **Bounded Intent** (target goal + max speed % + max torque % + lease timeout) executed by a local **1 kHz hardware loop**, guarded by a **Dual-Link Dead-Man's Switch**, automatic **Limp Mode** when running LoRa-only, and **Report-by-Exception** uplink telemetry. |
| **6. Fleet-Scale Hardware Security** | Application-layer **AES-128-GCM** encryption + 96-bit auth tag inside the 32-byte frame, anti-replay `seq_nonce` verification, and **per-device key derivation** (`KDF(MasterKey, DeviceMAC, TargetID)`) paired with ESP32 Flash Encryption & Secure Boot. |

---

## Repository Structure (All-Rust Workspace + Python Edge SDK)

```text
telekine/
├── Cargo.toml                  # Rust workspace manifest
├── crates/
│   ├── telekine-proto/         # #[no_std] 32-byte wire format, bounded intents, AES-128-GCM & per-device KDF
│   ├── telekine-core/          # #[no_std] Actuator safety engine: deduplicator, watchdog lease, Limp Mode, TDMA, 1kHz controller
│   ├── telekine-firmware/      # #[no_std] ESP32-S3/C3 + SX1262 dual-radio HAL, WMM AC_VO injection, peer-swap ring buffer
│   ├── telekine-host/          # Hub coordinator: 1,000+ node state table, 3ms Hot-Warm failover, Triple-Burst, TDMA beacon
│   ├── telekine-sim/           # Native swarm simulator verifying 1,000 virtual actuators under loss, latency & failover
│   └── telekine-py/            # Python bindings (PyO3) & LLM / MCP bounded-intent tool schemas
└── README.md
```

---

## Quick Start

### 1. Run the Core Protocol & Safety Unit Tests
```bash
cargo test --workspace --exclude telekine-py
```

### 2. Run the 1,000-Actuator Swarm Simulator
Simulates 1,000 dual-radio actuators across 5 scenarios (Hot-Hot Twin-Cast deduplication, 3 ms Hot-Warm LoRa failover, ESP-NOW blackout Limp Mode degradation, Sync-and-Fire coordinated swarm motion, and Dual-Link Dead-Man's Switch fail-safe):
```bash
cargo run -p telekine-sim
```

### 3. Python AI Agent Usage (`telekine-py`)
```python
from telekine import SwarmHub, BoundedIntent, Priority, RedundancyMode

hub = SwarmHub(master_key=bytes.fromhex("000102030405060708090a0b0c0d0e0f"))

# Register an actuator with its unique MAC address (derives per-device AES-128-GCM key)
hub.register_actuator(target_id=42, mac_address=b"\x24\x0A\xC4\x12\x34\x56")

# Dispatch a bounded goal state (never open-loop toggles)
hub.dispatch(
    target_id=42,
    intent=BoundedIntent.move_to(
        position_deg=45.0,
        max_speed_pct=20.0,
        max_torque_pct=50.0,
        lease_timeout_ms=250,
    ),
    mode=RedundancyMode.HOT_WARM,
    priority=Priority.HIGH_VI,
)
```
