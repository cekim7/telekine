# Telekine Universal Hardware Driver ("Telekine Puck") Specification

**Form Factor:** `42 mm × 42 mm` (NEMA-17 backplate mountable + M3 DIN-rail/frame ears)  
**Target BOM Cost:** `~$9.80 @ 1k qty`  
**Firmware Layer:** `telekine_firmware::universal_driver::UniversalDriverController`

---

## 1. Design Philosophy: One Module, Any Low-Intelligence Object

To retrofit **1,000+ diverse, low-intelligence actuators** (brushed DC motors, RC servos, industrial 24V solenoid valves, NEMA steppers, 0–10V dampers, and CAN/RS-485 motor drives) without redesigning PCBs or rewriting safety firmware, the **Telekine Universal Driver** splits the hardware into:

1. **The Core-Stamp (Always Identical):** Dual-radio RF (`ESP32-S3` + `Semtech SX1262`), hardware crypto, **Hardware Dead-Man's Switch Interlock Gate (`SAFE_ENABLE`)**, wide `6V–60V DC` buck power supply, and a multi-protocol output stage.
2. **Smart Harness Auto-ID (`ID_SENSE`):** Each pluggable actuator cable harness carries a `$0.15` 1-Wire EEPROM (`DS2431`) or precision ID resistor. When plugged into any hardware component, the Telekine Puck reads `ID_SENSE` at boot and configures its pin mux, voltage/current limits, and `HardwarePersonality` automatically.

---

## 2. System Block Diagram

```text
                  +---------------------------------------------------------------+
                  |             TELEKINE UNIVERSAL DRIVER (42x42 mm)              |
                  |                                                               |
 [2.4GHz Ant] --->|  +-----------------------+       SPI2      +---------------+  |
                  |  |   ESP32-S3-WROOM-1    |<--------------->| Semtech SX1262|  |<--- [915MHz Ant]
                  |  | - ESP-NOW (WMM AC_VO) |                 | Raw LoRa SF7  |  |
                  |  | - HW AES-128-GCM      |                 +---------------+  |
                  |  | - 1kHz Safety Loop    |                                    |
                  |  +-----------+-----------+                                    |
                  |              | WD_PULSE (1kHz charge-pump watchdog)           |
 [E-STOP LOOP] -->|--------------+---> [HW AND GATE: SN74LVC1G08] ===> SAFE_ENABLE|
                  |                                                       |       |
 [6V-60V DC IN] ->|  [TI LM5164 Buck -> 5V/3.3V]                          v       |
                  |  +---------------------------------------------------------+  |
                  |  |         UNIVERSAL OUTPUT & FEEDBACK MULTIPLEXER         |  |
                  |  |                                                         |  |
                  |  | 1. DRV8876 H-Bridge (36V/3.5A + IPROPI Current Sense)   |==+==> Brushed DC / Solenoid
                  |  | 2. 5V Level-Shifted PWM / STEP-DIR (SN74AHCT125)        |==+==> Servos / ESCs / Steppers
                  |  | 3. CAN-FD (TCAN1044V) + RS-485 (SN65HVD72)              |==+==> ODrive / VESC / Modbus VFD
                  |  | 4. 0-10V Analog Out (Op-Amp) + Isolated SSR Gate        |==+==> Industrial Valves / Relays
                  |  | 5. Universal Feedback (Quad ABZ / I2C AS5600 / 0-3V ADC)|<=+=== Encoder / Pot / Limit SW
                  |  | 6. 1-Wire ID_SENSE (DS2431 EEPROM on Cable Harness)     |<=+=== Auto-Config Personality
                  |  +---------------------------------------------------------+  |
                  +---------------------------------------------------------------+
```

---

## 3. Bill of Materials (Core Subsystems)

| Subsystem | Part Number | Role in Telekine | Approx. Cost (1k) |
| :--- | :--- | :--- | :--- |
| **Primary MCU & 2.4 GHz Radio** | `ESP32-S3-WROOM-1-N8` | Runs 2.4 GHz ESP-NOW (`1–3 ms`), hardware AES-128-GCM, Secure Boot/Flash Encryption, and local 1 kHz PID + Watchdog lease. | $2.20 |
| **915 MHz Raw LoRa Radio** | `Semtech SX1262` (e.g. E22-900M22S) | Long-range wall-penetrating backup (`30–80 ms`, `+22 dBm` Capture Effect, `SF7` VIP Emergency channel). | $3.80 |
| **Wide-Input Buck Regulator** | `TI LM5164` (`6V–60V` to `5V` 1A) + `AP2112K-3.3` | Allows powering the node directly from 12V/24V industrial rails, 3S–12S LiPo packs, or 48V robot buses. | $1.10 |
| **Hardware Safety Interlock** | `TPS3430` Window Watchdog + `SN74LVC1G08` AND Gate | Requires a continuous 1 kHz `WD_PULSE` from `telekine-core`. If both radios drop past the lease or the MCU hangs, `SAFE_ENABLE` drops to `0V` in hardware within `< 5 ms`. | $0.45 |
| **Integrated Power H-Bridge** | `TI DRV8876` (`36V`, `3.5A` peak, `IPROPI` sense) | Directly drives brushed DC motors, linear actuators, pneumatic solenoids, or electromagnetic brakes with hardware stall-current sensing. | $0.95 |
| **Fieldbus Transceivers** | `TCAN1044V` (CAN-FD) + `SN65HVD72` (RS-485) | Directly drives high-power external BLDC drives (ODrive, VESC), industrial VFDs (Modbus RTU), and serial bus servos. | $0.85 |
| **5V PWM / Step-Dir Buffer** | `SN74AHCT125` Quad Tri-State Buffer (`OE` tied to `SAFE_ENABLE`) | Drives 5V RC servos, drone ESCs, and external stepper drivers (`STEP`/`DIR`/`EN`), tri-stated immediately when `SAFE_ENABLE` trips. | $0.20 |
| **Connectors & Protection** | `XT30` Power + `JST-GH 10-pin` Universal I/O + `TVS` Diodes | Reverse-polarity MOSFET protection, ESD diodes on all I/O lines, and u.FL / SMA antenna jacks. | $0.30 |

---

## 4. ESP32-S3 Pinout & Bus Allocation

| Signal Group | ESP32-S3 GPIO | Direction | Function |
| :--- | :--- | :--- | :--- |
| **SX1262 LoRa (SPI2)** | `GPIO10..14` | `SPI + IRQ` | `NSS=10`, `MOSI=11`, `SCK=12`, `MISO=13`, `DIO1=14`, `BUSY=21`, `NRST=47` |
| **Hardware Watchdog Gate** | `GPIO1` (`WD_PULSE`), `GPIO2` (`E_STOP_SENSE`) | `OUT / IN` | 1 kHz charge-pump pulse feeding `TPS3430` $\rightarrow$ `SAFE_ENABLE` hardware gate |
| **H-Bridge & PWM / Step-Dir** | `GPIO4` (`OUT_A / STEP / PWM1`), `GPIO5` (`OUT_B / DIR / PWM2`), `GPIO6` (`nSLEEP / EN`) | `OUT (MCPWM)` | Multiplexed via `HardwarePersonality` between `DRV8876`, `STEP/DIR`, and `RC PWM` |
| **Current Sense (`IPROPI`)** | `GPIO7` (`ADC1_CH6`) | `ANALOG IN` | Reads motor/solenoid current in real time; trips `MotorStallDetected` on spike |
| **CAN-FD & RS-485 Fieldbus** | `GPIO15..18` | `TWAI / UART1` | `CAN_TX=15`, `CAN_RX=16`, `RS485_TX=17`, `RS485_RX=18`, `RS485_DE=8` |
| **Universal Feedback Port** | `GPIO38..42` | `PCNT / I2C` | `ENC_A=38`, `ENC_B=39`, `ENC_Z=40` (or `I2C SDA/SCL` for AS5600 magnetic encoder) |
| **Smart Harness Auto-ID** | `GPIO9` (`ID_SENSE`) | `1-Wire / ADC` | Reads harness EEPROM or resistor divider on boot to select `HardwarePersonality` |

---

## 5. How Diverse Hardware Connects to the Universal 10-Pin Port

| Target Hardware | Harness Personality ID | Pins Used on Universal Port | Local 1 kHz Safety & Fail-Safe Behavior |
| :--- | :--- | :--- | :--- |
| **1. Brushed DC Gearmotor / Linear Actuator** | `HBridgeBrushedDc (0x02)` | `MOT+`, `MOT-` (`DRV8876`), `ENC_A/B` (or Potentiometer ADC) | Closes 1 kHz position loop via encoder; monitors `IPROPI` for stall; active short-brake when watchdog lease expires. |
| **2. RC Servo / Brushless Thruster ESC** | `PwmServoOrEsc (0x01)` | `5V_OUT`, `PWM1` (`1000–2000 µs`), `GND` | Slew-rate limits angle/throttle in software (40% max in LoRa Limp Mode); `SN74AHCT125` tri-states PWM on fail-safe. |
| **3. NEMA-17 / NEMA-23 Stepper Motor** | `StepperStepDir (0x03)` | `STEP`, `DIR`, `EN` (5V buffered), `LIMIT_SW` | Generates deterministic step frequency with acceleration ramp; de-asserts `EN` (or holds brake) on lease timeout. |
| **4. 12V/24V Solenoid Valve or Contactor** | `SolenoidOrRelay (0x04)` | `MOT+`, `MOT-` (`DRV8876` in high-side/low-side PWM mode) | Applies 100% pull-in current for `50 ms`, then drops to 30% PWM hold current; hardware gate forces valve **CLOSED** on lease timeout. |
| **5. High-Power BLDC (ODrive / VESC / MIT Joint)** | `CanBusSmartActuator (0x05)` | `CAN_H`, `CAN_L`, `GND` | Translates `BoundedIntent` into 1 kHz cyclic CAN frames; sends `0 Nm` torque + hardware `E_STOP` on watchdog expiry. |
| **6. Industrial VFD / Smart Bus Servo (Dynamixel)** | `Rs485SerialActuator (0x06)` | `RS485_A`, `RS485_B`, `GND` | Streams bounded velocity/position registers over half-duplex RS-485 with automatic CRC and fail-safe stop packet. |
| **7. HVAC Damper / Analog Process Valve** | `Analog0To10V (0x07)` | `ANALOG_0_10V`, `GND`, `POS_SENSE_ADC` | Drives 0–10V proportional setpoint; drops to `0.0V` fail-safe state when `SAFE_ENABLE` trips. |
