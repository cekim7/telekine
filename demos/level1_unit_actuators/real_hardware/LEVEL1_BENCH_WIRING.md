# Level 1 Real-World Hardware Bench Guide: Servo, Stepper & DC Motor

This guide walks through wiring the **3 Foundational Unit Elements** on your desk and running them in lockstep with the MuJoCo digital twin (`demos/level1_unit_actuators/mjcf/unit_actuators_bench.xml`).

---

## 1. Bench Bill of Materials (Level 1 Demo Kit)

| Node ID | Unit Type | Recommended Off-the-Shelf Hardware | Telekine Driver Board | Power Rail |
| :--- | :--- | :--- | :--- | :--- |
| **`Hub #1`** | AI Hub Transceiver | `ESP32-S3-DevKitC-1` + `Ebyte E22-900M22S (SX1262)` over USB-C | Host Mac running `run_level1_demo.py` | 5V USB-C |
| **`Node #101`** | **`Servo`** | `DS3218` or `MG996R` Metal-Gear Standard Servo (`-90°..+90°`) | Telekine Unit Stamp V1 (`#101`) | 5V–6V DC (`2A`) |
| **`Node #102`** | **`Stepper`** | `NEMA-17 Bipolar Stepper` (`17HS4401`, `1.7A`) + 6x2mm diametric shaft magnet | Telekine Unit Stamp V1 (`#102`) bolted to rear cap | 12V–24V DC (`2A`) |
| **`Node #103`** | **`DcMotor`** | `12V/24V JGB37-520 Brushed DC Gearmotor` with built-in Hall encoder | Telekine Unit Stamp V1 (`#103`) | 12V–24V DC (`3A`) |

---

## 2. Terminal Block Wiring (`[VIN+] [GND] [CH1] [CH2] [CH3] [CH4]`)

Every **Telekine Unit Stamp V1** uses the exact same 6-pin Wago 2060 push-lever terminal block:

### A. Node `#101` (`Servo`)
- `VIN+` / `GND`: `6V DC` Bench Supply
- `CH1`: Servo Red Wire (`5V/6V V+`)
- `CH2`: Servo Orange/White Wire (`PWM Signal`, `1000–2000 µs` 5V level-shifted)
- `CH3` / `CH4`: Leave unconnected (Auto-probe detects `>10 kΩ` on `CH2` $\rightarrow$ selects `UnitElementKind::Servo`).

### B. Node `#102` (`Stepper` — 4-Wire Bipolar NEMA-17)
- `VIN+` / `GND`: `24V DC` Bench Supply
- `CH1`: Stepper Coil `A+` (Black)
- `CH2`: Stepper Coil `A-` (Green)
- `CH3`: Stepper Coil `B+` (Red)
- `CH4`: Stepper Coil `B-` (Blue)
- *(Auto-probe detects matched `1.8 Ω` coils on `CH1-CH2` and `CH3-CH4` $\rightarrow$ selects `UnitElementKind::Stepper`).*

### C. Node `#103` (`DcMotor` — 2-Wire Reversible Brushed DC)
- `VIN+` / `GND`: `24V DC` Bench Supply
- `CH1`: Motor Terminal `M+` (Red)
- `CH2`: Motor Terminal `M-` (Black)
- `CH3` / `CH4`: Optional parallel jumper to `CH1`/`CH2` for `3.5A` peak current.
- *(Auto-probe detects single `4.2 Ω` inductive coil on `CH1-CH2` $\rightarrow$ selects `UnitElementKind::DcMotor` with `IPROPI` stall cutoff at `2,500 mA`).*

---

## 3. Running Sim, Shadow, and Real-World Modes

```bash
# 1. Run in Sim-to-Real Shadow Mode (MuJoCo + protocol verification)
python3 demos/level1_unit_actuators/run_level1_demo.py --mode shadow

# 2. Run with Live ESP32-S3 Hub over USB-CDC
python3 demos/level1_unit_actuators/run_level1_demo.py --mode shadow --serial-port /dev/cu.usbmodem14101
```
