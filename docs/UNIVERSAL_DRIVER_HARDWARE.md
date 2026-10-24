# Telekine Foundational Unit Driver ("Telekine Unit Stamp V1")

**Target Demo 1 Hardware:** Universal AI-Native Driver for **Servo**, **Stepper**, and **Brushed DC Motors**  
**Form Factor:** `28 mm × 28 mm` (Mounts on NEMA-17 backplate, DC motor tail, or inline Servo cable)  
**Target BOM Cost:** `~$8.40 @ 1k qty`  
**Firmware Implementation:** `telekine_firmware::universal_driver::UniversalDriverController`

---

## 1. How One Tiny Board Drives a Servo, a Stepper, or a DC Motor

The **Telekine Unit Stamp V1** uses a **Software-Defined Quad Half-Bridge (`TI DRV8847` / `DRV8908` or dual `DRV8876`)** plus a **5V Level-Shifted Servo PWM Pin** on a single 6-pin spring terminal block (`[VIN+] [GND] [CH1] [CH2] [CH3] [CH4]`).

At power-on, the board **auto-probes coil impedance** across `CH1..CH4` (or reads the mode strap) and wakes up on **Master Manager Channel 1 (`2.4 GHz ESP-NOW + 915 MHz Raw LoRa`)**, ready for any Telekine AI Hub to discover, claim access on demand, and control:

| Foundational Unit | How You Wire It to `CH1..CH4` | Auto-Probe Signature | Local 1 kHz Execution & Fail-Safe |
| :--- | :--- | :--- | :--- |
| **1. `Servo`** (RC / Industrial PWM Servo) | `CH1` = `5V Power`, `CH2` = `PWM Signal (50-333 Hz)`, `GND` = `GND` | High impedance on `CH2` (`>10 kΩ`) + 5V buck rail active on `CH1` | Slew-rate limits angle (`1000..2000 µs`); throttles speed to 40% in LoRa Limp Mode; tri-states PWM when lease expires. |
| **2. `Stepper`** (4-Wire Bipolar Stepper, e.g., NEMA-17) | `CH1+CH2` = `Coil A (A+, A-)`, `CH3+CH4` = `Coil B (B+, B-)` | Two matched inductive coils (`1.5–30 Ω`) on `CH1-CH2` and `CH3-CH4` | Drives microstepped current waveforms (or `STEP/DIR` to external high-power drive); locks holding brake or de-energizes on watchdog expiry. |
| **3. `DcMotor`** (2-Wire Brushed DC Motor / Linear Actuator) | `CH1+CH2` = `Motor M+ / M-` (bridges `CH3+CH4` in parallel for `2x` current up to `4A`) | Single inductive coil (`1–20 Ω`) across `CH1-CH2` | Closes 1 kHz position/velocity loop using onboard magnetic encoder (`AS5600`/`MT6835`) or Back-EMF ripple; trips `MotorStallDetected` on `IPROPI` spike. |

---

## 2. Hardware Block Diagram (Telekine Unit Stamp V1)

```text
          TOP SIDE (Wireless & Terminal Side)              BOTTOM SIDE (Shaft Sensor Side)
    +--------------------------------------------+   +--------------------------------------------+
    | [USB-C]      [Status RGB]     [Teach/Zero] |   |                                            |
    |                                            |   |      +------------------------------+      |
    | +--------------------+   +---------------+ |   |      | Dual DRV8876 / DRV8847       |      |
    | |  ESP32-S3-MINI-1   |   | Semtech SX1262| |   |      | 4-Half-Bridge Power Stage    |      |
    | |  - 2.4GHz ESP-NOW  |   | - 915MHz LoRa | |   |      | (Up to 36V, 3.5A + IPROPI    |      |
    | |  - HW AES-128-GCM  |   | - SF7 VIP Ch  | |   |      |  Analog Stall Current Sense) |      |
    | +---------+----------+   +-------+-------+ |   |      +------------------------------+      |
    |           | WD_PULSE (1kHz)      |         |   |                                            |
    | [TPS3430 Hardware Watchdog] ---> SAFE_EN   |   |           ( ( [AS5600 / MT6835] ) )        |
    |                                            |   |         Center-Mounted On-Axis Hall        |
    | +----------------------------------------+ |   |         Magnetic Rotary Encoder IC         |
    | |  6-Pin Push-Lever Terminal (Wago 2060) | |   |                                            |
    | |  [VIN+] [GND] [CH1] [CH2] [CH3] [CH4]  | |   |           [BMI270 6-Axis IMU]              |
    | +----------------------------------------+ |   |                                            |
    +--------------------------------------------+   +--------------------------------------------+
```

---

## 3. Demo 1 Protocol Flow: Discover $\rightarrow$ Claim Access $\rightarrow$ Control $\rightarrow$ Release

1. **Boot & Self-Probe:**
   Each Telekine Unit Stamp powers up, probes `CH1..CH4` to identify whether it is attached to a **`Servo`**, **`Stepper`**, or **`DcMotor`**, sets its state to `UnitAccessState::IdleAvailable`, and listens on **Master Manager Channel 1**.
2. **AI Hub Discovery (`DiscoverUnitsProbe` $\rightarrow$ `UnitDiscoveryAnnounce`):**
   The AI Hub broadcasts a `DiscoverUnitsProbe` (`opcode = 0x20`). Every unit in range replies with an encrypted `UnitDiscoveryAnnounce` (`opcode = 0x21`) reporting its `node_id`, `UnitElementKind` (`Servo`, `Stepper`, `DcMotor`), `UnitAccessState` (`IdleAvailable` vs `ClaimedByHub`), and calibrated travel range.
3. **On-Demand Access Lease (`ClaimUnitAccess`):**
   When the AI Agent decides it needs `Node #1 (Servo)` and `Node #2 (Stepper)`, the Hub sends `ClaimUnitAccess` (`opcode = 0x22`). Each claimed unit locks itself to that `hub_id`, switches to the Hub's assigned **Worker Channel (`6` or `11`) + TDMA Slot**, and activates its Dual-Link Watchdog timer.
4. **Bounded Control & Release (`MoveToBounded` / `SetVelocityBounded` $\rightarrow$ `ReleaseUnitAccess`):**
   The AI Hub streams bounded intents over dual-redundant ESP-NOW + Raw LoRa. When the task completes, the Hub sends `ReleaseUnitAccess` (`opcode = 0x23`), returning the units to `IdleAvailable` on Master Channel 1.
