# Level 2 Real-World Hardware Guide: 6-DOF Telekine Robot Arm

This guide shows how to assemble a **full 6-DOF robot arm** using **6 identical Telekine Unit Stamp V1 modules (`#201..#206`)**—eliminating the traditional 24-wire slip-ring/harness trunk through the arm joints by running only a **2-wire 24V DC power spine** while every joint receives synchronized `Sync-and-Fire` trajectories over **ESP-NOW (2.4 GHz) + Raw LoRa (915 MHz)**.

---

## 1. Joint-by-Joint Hardware Mapping (`Nodes #201..#206`)

| Joint | Node ID | Foundational Unit Type | Recommended Actuator | Travel Bounds |
| :--- | :--- | :--- | :--- | :--- |
| **J1: Waist Yaw** | `#201` | `UnitElementKind::Stepper` | `NEMA-23 Bipolar Stepper` (`5:1` belt reduction) + `AS5600` encoder | `-180° .. +180°` |
| **J2: Shoulder Lift** | `#202` | `UnitElementKind::DcMotor` | `24V Worm-Gear Brushed DC Motor` (self-locking on fail-safe) + Hall encoder | `-90° .. +90°` |
| **J3: Elbow Flex** | `#203` | `UnitElementKind::DcMotor` | `24V Planetary Brushed DC Gearmotor` + electromagnetic holding brake | `-135° .. +135°` |
| **J4: Wrist Pitch** | `#204` | `UnitElementKind::Servo` | `DS3225 25kg-cm Metal-Gear PWM Servo` (powered via Stamp 5V buck) | `-90° .. +90°` |
| **J5: Wrist Roll** | `#205` | `UnitElementKind::Servo` | `DS3218 20kg-cm PWM Servo` | `-180° .. +180°` |
| **J6: Parallel Gripper** | `#206` | `UnitElementKind::Servo` | `MG996R / Parallel Jaw Servo` (`IPROPI` current-limited grip force) | `0° (open) .. 90° (closed)` |

---

## 2. Why Telekine Simplifies Robot Arm Wiring

Traditional 6-DOF robot arms require routing **24 to 36 flexing signal and power wires** through every moving joint (4 motor wires + 6 encoder wires + 2 limit switch wires per joint).

With **Telekine Unit Stamp V1** mounted directly onto each motor body:
1. **Only 2 Wires Cross the Joints (`+24V DC` and `GND`):** Daisy-chain a single pair of flexible 18 AWG silicone wires (`+24V` and `GND`) from J1 up to J6. Each Telekine Stamp steps `24V` down to `5V/3.3V` locally.
2. **Sub-Millisecond Coordinated Motion (`Sync-and-Fire`):** The AI Hub broadcasts a `SyncClockBeacon` (`0xFFFE`) on Channel 1, then dispatches each joint's next waypoint with `exec_at_tick_ms`. Even though J1..J6 receive their packets sequentially over 2–4 ms, **all 6 joints buffer the instruction and trigger on the exact same millisecond tick**.
3. **Sim-to-Real Shadow Safety Gate:** When running `run_level2_arm_demo.py --mode shadow`, the MuJoCo model (`mjcf/telekine_6dof_arm.xml`) steps every trajectory first and verifies that the end-effector stays above the table surface (`Z > 0.40 m`) before the wireless frames are emitted to the physical arm.

---

## 3. Running the Level 2 Robot Arm Demo

```bash
# Run the 5-phase pick-and-place trajectory in Shadow Mode (MuJoCo + protocol trace)
python3 demos/level2_robot_arm/run_level2_arm_demo.py --mode shadow

# Run with Interactive 3D MuJoCo Viewer (once `pip install mujoco` is installed)
python3 -m mujoco.viewer --mjcf=demos/level2_robot_arm/mjcf/telekine_6dof_arm.xml
```
