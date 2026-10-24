# Telekine

**Rust Wireless Control Stack for Embedded Actuators via Dual-Radio Redundancy (ESP-NOW + LoRa)**

Telekine is designed to help AI agents reliably and safely control distributed wireless actuators using parallel physical-layer redundancy, capable of scaling to large deployments:
- **ESP-NOW (2.4 GHz):** Fast local control (`1–3 ms` latency, local range, priority queuing).
- **LoRa (915 MHz):** Longer-range, reliable backup (`30–80 ms` latency, extended range).

---

## Architectural Highlights

| Pillar | Implementation in Telekine |
| :--- | :--- |
| **1. Dual-Radio Redundancy** | **Hot-Hot (Twin-Cast)** can send critical intents simultaneously. **Hot-Warm (Auto-Failover)** prioritizes ESP-NOW and falls back to LoRa if needed. |
| **2. Peer Scale & Coexistence** | Connectionless addressing, dynamic ESP-NOW RAM peer-swapping, and scheduled time-slotting to coordinate multiple devices. |
| **3. Priority Tagging** | Hardware 802.11 WMM/EDCA tagging for urgent frames, and a dedicated LoRa channel for important messages. |
| **4. Compact Wire Protocol** | Zero-copy `#[repr(C, packed)]` 32-byte binary frame with monotonic sequencing and deduplication. |
| **5. Physical Safety** | Uses **Bounded Intents** (target goal + max limits) instead of raw open-loop toggles. Incorporates local hardware loops and automatic safe states. |
| **6. Security** | Application-layer **AES-128-GCM** encryption with per-device key derivation and anti-replay verification. |

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
