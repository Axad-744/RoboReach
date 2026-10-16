# RoboReach
ESP32-based mobile robot with reach capability.

## Hardware Specification (Authoritative)
- **Platform**: ESP32-WROOM-32 module on a compatible carrier; select **ESP32 Dev Module** in Arduino IDE
- **Arm**: 2-DOF Shoulder/Elbow IK with 60kg digital shoulder servo, 20kg digital elbow servo, and two MG90S servos on the paired gripper
- **Drive**: 2x IBT-2 with 2x 12V 60RPM Johnson motors, one on each side (left motor front-left, right motor rear-right); differential steering
- **Pins**: see the WROOM-32 GPIO map below; each gripper servo has its own signal pin
- **Arm geometry**: L1=140 mm; 390 mm total from shoulder pivot to jaw gripping center; shoulder pivot 100 mm above floor. IK uses effective L2=250 mm.
- **Wi-Fi**: ESP32 SoftAP on UDP port 4210. Replace the SSID and password placeholders in `include/config.h` with your own before flashing; do not commit real credentials.
- **RoboLink keys**: throttle, steer, arm_mode, arm_x, arm_y, shoulder_deg, elbow_deg, gripper_deg, grip_open, all_90
- **Gripper controls**: configure `gripper_deg` as one 0–90° slider for both MG90S servos; 0° is the starter close target and 90° is open/home. The firmware ramps both servos together. The second servo is reversed around 90° by default for a mirrored mount; change `GRIPPER_SECOND_SERVO_REVERSED` in `include/config.h` if the mechanism needs matching directions instead. A momentary `grip_open` button commands both to 90°.
- **All-servos home button**: a momentary `all_90` button commands shoulder, elbow, and both gripper servos to 90°. The firmware holds that pose until an arm or gripper control changes.
- **Arm controls**: set `arm_mode=0` for IK and use `arm_x` (forward X from shoulder pivot, mm) plus `arm_y` (gripping-center height above floor, mm). Set `arm_mode=1` for direct joint control and use `shoulder_deg` and `elbow_deg` (0–180°). Theoretical IK height is 490 mm; reachable targets also depend on the arm's circular workspace and servo travel.

### ESP32-WROOM-32 GPIO Map

| Function | ESP32 GPIO | Connect to |
|---|---:|---|
| Left-front motor RPWM | 25 | Left-front BTS7960 RPWM |
| Left-front motor LPWM | 26 | Left-front BTS7960 LPWM |
| Left-front motor enable | 22 | Left-front BTS7960 R_EN and L_EN tied together |
| Right-rear motor RPWM | 32 | Right-rear BTS7960 RPWM |
| Right-rear motor LPWM | 33 | Right-rear BTS7960 LPWM |
| Right-rear motor enable | 23 | Right-rear BTS7960 R_EN and L_EN tied together |
| Shoulder servo signal | 18 | 60 kg digital servo signal |
| Elbow servo signal | 19 | 20 kg digital servo signal |
| Gripper servo 1 signal | 21 | First MG90S signal |
| Gripper servo 2 signal | 27 | Second MG90S signal |

Power the servos from a suitable external servo supply, not the ESP32 3.3 V pin, and connect the servo-supply ground to ESP32 GND. These GPIO numbers refer to the WROOM-32 module GPIOs, exposed on compatible carrier headers. Avoid GPIOs 6–11 (module flash), 34–39 (input-only), and boot-strapping pins for these outputs. The selected pins are GPIO 18, 19, 21, 22, 23, 25, 26, 27, 32, and 33.

## Documentation & Specifications
- [`docs/FINAL_PROJECT_SPEC.md`](docs/FINAL_PROJECT_SPEC.md): **Authoritative Final Specification** based on the final BOM (`ROBOREACH BOM - Sheet1.pdf`).
- [`docs/SOFTWARE_ARCHITECTURE.md`](docs/SOFTWARE_ARCHITECTURE.md): Software architecture, data flow, and electrical dependencies.
- [`docs/PROTOTYPE_REFERENCE.md`](docs/PROTOTYPE_REFERENCE.md): Prototype test bench notes, architectural patterns, and deprecation details.

## Project Structure
- `main/`: Firmware implementation (drive, arm, inverse kinematics, RoboLink).
- `include/`: Shared configuration, pin assignments, and IK declarations.
- `docs/`: Hardware specifications, architecture, and prototype notes.
- `tests/`: Unit and hardware verification tests:
  - [`tests/robolink_comm_test/`](tests/robolink_comm_test/README.md): RoboLink mobile app communication verification test.
- `platformio.ini`: PlatformIO configuration for an ESP32-WROOM-32 compatible board (`esp32dev`).

## Moving the Repository to Another Laptop

This repository can be transferred without GitHub using a Git bundle:

1. From the repository folder, create a bundle:
   ```bash
   git bundle create RoboReach.bundle --all
   ```
2. Copy `RoboReach.bundle` to the other laptop (USB drive or local file transfer).
3. Install Git there and clone the bundle:
   ```bash
   git clone RoboReach.bundle RoboReach
   ```

The bundle preserves the repository history. Select the ESP32's current serial port in Arduino IDE on the destination laptop; COM port numbers can differ between computers.

## Build, Upload and Test

### Prerequisites
1. **PlatformIO Core**: `pip install platformio`
2. **ESP32 board package**: ESP32 by Espressif Systems, Arduino-ESP32 core 3.x
3. **RoboLink library**: the library providing `RoboLink.h` (not included in this repository)
4. Connect the ESP32-WROOM-32 carrier and select the port shown on that computer; COM numbers can differ.

### Arduino IDE
1. Install the ESP32 board package by Espressif Systems and the RoboLink library that provides `RoboLink.h`.
2. Open `main/main.ino`.
3. Select **ESP32 Dev Module** (FQBN `esp32:esp32:esp32`) and the ESP32's current port in **Tools**.
4. Upload the sketch.

### PlatformIO Setup
1. Install PlatformIO extension/CLI
2. This project uses `platformio.ini` with:
   - Platform: `espressif32` with the Arduino framework
   - Framework: `arduino`
   - Monitor speed: `115200`
   - Port: auto-detected, or set locally in `platformio.ini` if needed
   - Library dependencies: `madhephaestus/ESP32Servo`; the `RoboLink.h` dependency must also be available

### Building
```bash
pio run
```
- Fix any compile errors minimal changes
- The RoboLink library must be available (see lib_deps in platformio.ini)

### Upload
```bash
pio run -t upload
```

### Serial Monitor
```bash
pio device monitor -b 115200
```
- Observe startup messages and control input logging
- Verify: Wi-Fi AP starts, RoboLink connectivity, motor/servo response

### Testing Workflow
1. Upload firmware to ESP32
2. Power cycle or press EN
3. Observe Serial Monitor output (115200 baud)
4. Connect the phone to the Wi-Fi name and password configured in `include/config.h`
5. Open RoboLink app, ensure Wi-Fi/UDP mode
6. Move joysticks/sliders - verify control values appear in Serial Monitor
7. For coordinated end-effector motion, set `arm_mode=0` and move `arm_x` / `arm_y`; firmware solves shoulder and elbow angles with 2-link IK. For separate joint adjustment, set `arm_mode=1` and move `shoulder_deg` / `elbow_deg`. Move `gripper_deg` from 90 (open) toward 0 (close) to drive both MG90S gripper servos together.
   The default IK uses the lower-elbow branch and the measured 140 mm first link, 390 mm total reach, and 100 mm shoulder height.
   Adjust the servo direction/zero constants in `include/config.h` to match the installed horns.
8. Drive with `throttle` forward/backward and `steer` left/right. With throttle centered, positive/right steering drives the left motor while the right stays stopped; negative/left steering drives the right motor while the left stays stopped. When throttle is applied, the firmware uses differential mixing. The other wheels must be able to swivel or skid for the chassis to rotate.
