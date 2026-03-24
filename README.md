# Remote-Controlled and Autonomous Sentry Tank

**Authors:** Joon Cho and Yuyang Chen

A dual-Arduino robotic tank with wireless RC control, autonomous obstacle avoidance, and real-time IMU telemetry.

---

## Overview

The system consists of two independent subsystems — a **handheld controller** and a **tracked tank platform** — communicating wirelessly via RFM69HCW RF transceivers at 915 MHz. The controller reads dual joystick inputs and sends speed commands to the tank, which drives two NEMA-17 stepper motors independently for differential steering. An LCD on the controller provides real-time feedback, and an autonomous mode enables basic reactive navigation.

---

## Hardware

### Controller
| Component | Details |
|---|---|
| Microcontroller | Arduino UNO |
| Input | 2× dual-axis joystick modules |
| Display | I2C 1602 LCD |
| Communication | RFM69HCW transceiver (SPI) |
| Power | USB |

The left joystick controls the left track and toggles operating mode (RC/AUTO) via its button. The right joystick controls the right track and requests IMU data via its button. The LCD top row shows average commanded speed and mode; the bottom row shows roll/pitch/yaw when IMU is requested, or a prompt otherwise.

### Tank
| Component | Details |
|---|---|
| Microcontroller | Arduino UNO |
| Motors | 2× NEMA-17 bipolar stepper motors |
| Drivers | 2× TMC2208 (full-step, STEP/DIR mode) |
| Distance sensor | Parallax 28015 ultrasonic sensor |
| IMU | Pololu AltIMU-10 v6 (LSM6 accel/gyro + LIS3MDL magnetometer) |
| Communication | RFM69HCW transceiver (SPI) |
| Power | 11.1 V Li-Po (motors), regulated 5 V (logic) |

---

## Wiring

**Tank:** Left and right stepper drivers connect to Arduino digital pins (STEP/DIR/EN). The ultrasonic sensor (single-wire SIG) connects to D7. The IMU connects via I2C (A4/A5). The RF chip connects via SPI (D10–D13) with RST on D9 and IRQ on D2.

**Controller:** Left and right joystick VRX axes connect to A0/A1; switch pins to D4/D5. The LCD connects via I2C (A4/A5). The RF chip connects via SPI (D10–D13) with RST on D9 and IRQ on D2.

---

## Software

### Controller (`controller.ino`)
- Reads joystick analog values, applies a ±40-count deadband around center (512), and linearly maps displacement to a signed speed command in [−127, +127].
- Transmits a 4-byte packet `[L_speed, R_speed, mode, imu_request]` at ~20 Hz.
- Receives a 6-byte IMU packet `[roll_hi, roll_lo, pitch_hi, pitch_lo, yaw_hi, yaw_lo]` (values scaled ×100, transmitted as 16-bit signed integers) when requested.
- Updates LCD each loop cycle.

### Tank (`tank.ino`)
- Receives command packets and stores target speed values for each track.
- **Acceleration limiting:** ramps current commands toward targets by 2 units every 5 ms (~400 units/s max rate of change) to prevent missed steps.
- **Motor stepping:** generates STEP pulses with variable intervals mapped from command magnitude — speed 1→127 maps to 2500→200 µs between steps.
- **Autonomous mode:** simple two-state machine (FORWARD / TURNING). Samples ultrasonic distance every 150 ms; if obstacle < 15 cm, switches to a fixed left turn until the target heading offset is satisfied.
- **IMU telemetry:** on request, reads accelerometer and magnetometer, computes roll/pitch (from accelerometer) and yaw (from magnetometer), scales by 100, and transmits a 6-byte packet back to the controller.

---

## Operating Modes

| Mode | Behavior |
|---|---|
| **RC** | Left joystick → left track speed, Right joystick → right track speed |
| **AUTO** | Tank drives forward autonomously; turns left when obstacle detected within 15 cm |

Toggle mode with the left joystick button. The LCD displays `RC` or `AUTO` on the top row. Manual speed commands are suppressed in AUTO mode.
