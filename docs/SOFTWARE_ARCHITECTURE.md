# RoboReach Software Architecture Specification

**Document Version**: 1.0  
**Hardware Authority**: `ROBOREACH BOM - Sheet1.pdf`  
**Target Platform**: ESP32-WROOM-32 module on a compatible carrier; Arduino IDE board target `ESP32 Dev Module` (`esp32:esp32:esp32`)
**Status**: Architecture Complete / Hardware Confirmed

---

## 1. System Overview & Communication Flow

RoboReach is an integrated mobile robotic platform featuring a differential drive base and a 2-DOF articulated robotic arm. The robot communicates wirelessly with the **RoboLink** mobile application over Wi-Fi using the UDP transport protocol.

### End-to-End Data Flow

```
                      +-----------------------------+
                      |   RoboLink Mobile App       |
                      |   (Phone UI: Sticks/Sliders)|
                      +-----------------------------+
                     |
                     | Wi-Fi (UDP port 4210)
                     v
                      +-----------------------------+
                      |        ESP32 (MCU)          |
                      +-----------------------------+
                     |
                    v
             +-----------------------------+
             |     robolink_handler        |
             |  - Non-blocking UDP parse   |
             |  - Key-value extraction     |
             |  - Failsafe timeout monitor |
             +-----------------------------+
                |
                v
       +-----------------+-----------------+
       |                                   |
       v                                   v
   +-----------------------------+     +-----------------------------+
   |       motor_control         |     |       servo_control         |
   |  - Deadband filtering       |     |  - Safe angle clamping      |
   |  - Differential drive mix   |     |  - 2-DOF pose coordination |
   |  - Target duty calculation  |     |  - Park / Home sequence   |
   +-----------------------------+     +-----------------------------+
```

---

## 2. Software Modules Breakdown

The codebase is organized into modular units located in [`src/`](file:///c:/Users/ridhs/OneDrive/Desktop/projects/roboreach/src) and [`include/`](file:///c:/Users/ridhs/OneDrive/Desktop/projects/roboreach/include):

### 2.1 Configuration Header: [`include/config.h`](file:///c:/Users/ridhs/OneDrive/Desktop/projects/roboreach/include/config.h)
- Serves as the single source of truth for software parameters and hardware pinouts.
- Contains confirmed pin assignments (no TBD placeholders for GPIOs):
  - BTS7960 motor driver GPIO pins (Left/Right forward, reverse, and enable lines)
  - Servo signal GPIO pins: SHOULDER=18, ELBOW=19, GRIPPER_1=21, GRIPPER_2=27
  - LEDC timer frequencies and resolution channels
  - Mechanical angle boundaries and safe home/park positions
  - RoboLink network credentials (SSID, password, port) and control key names.
- **Safety Guarantee**: All pins are assigned (no -1 placeholders).

### 2.2 Network & Communication: [`src/robolink_handler.h`](file:///c:/Users/ridhs/OneDrive/Desktop/projects/roboreach/src/robolink_handler.h) & [`src/robolink_handler.cpp`](file:///c:/Users/ridhs/OneDrive/Desktop/projects/roboreach/src/robolink_handler.cpp)
- Encapsulates the official **RoboLink** library (`RoboLink.h`).
- Sets up the ESP32 Wi-Fi SoftAP (SSID and password are configured locally in `include/config.h`, IP `192.168.4.1`, UDP port `4210`).
- Drains UDP packets non-blockingly during every loop cycle.
- Automatically handles timeout detection: if no packets arrive within `ROBOLINK_TIMEOUT_MS` (500 ms), the handler marks the connection as lost and zeros out throttle and steer.
- Exposes clean accessors to retrieve parsed integer control values.

### 2.3 Mobile Base Subsystem: [`src/motor_control.h`](file:///c:/Users/ridhs/OneDrive/Desktop/projects/roboreach/src/motor_control.h) & [`src/motor_control.cpp`](file:///c:/Users/ridhs/OneDrive/Desktop/projects/roboreach/src/motor_control.cpp)
- Implements one-side-pivot steering plus 2-wheel differential mixing:
  - With throttle centered and steer right: `LeftDuty = steer`, `RightDuty = 0`.
  - With throttle centered and steer left: `LeftDuty = 0`, `RightDuty = -steer`.
  - While translating:
  $$\text{LeftDuty} = \text{constrain}(\text{Throttle} + \text{Steer}, -255, 255)$$
  $$\text{RightDuty} = \text{constrain}(\text{Throttle} - \text{Steer}, -255, 255)$$
- Implements deadband filtering (`MOTOR_INPUT_DEADBAND`) to eliminate joystick center jitter.
- Stores logical motor state (`leftTargetDuty`, `rightTargetDuty`, `isStopped`).

### 2.4 2-DOF Articulated Arm Subsystem: [`src/servo_control.h`](file:///c:/Users/ridhs/OneDrive/Desktop/projects/roboreach/src/servo_control.h) & [`src/servo_control.cpp`](file:///c:/Users/ridhs/OneDrive/Desktop/projects/roboreach/src/servo_control.cpp)
- Manages target angular positions for all 4 arm actuators:
  1. **Shoulder**: Primary lift joint (Annimos 60kg)
  2. **Elbow**: Forearm elevation joint (20kg)
  3. **Gripper pair**: Two MG90S servos controlled together by RoboLink's `gripper_deg` slider, from 0° (starter close) to 90° (open/home). The second servo output is reversed around 90° by default for a mirrored mount; configure this after checking the installed linkage.
- Enforces strict software boundary clamping against min/max angle limits defined in `config.h`.
- Provides `servoControlPark()` to restore joints to default neutral resting angles.

### 2.5 Main Application Orchestrator: [`src/main.ino`](file:///c:/Users/ridhs/OneDrive/Desktop/projects/roboreach/src/main.ino)
- Coordinates all subsystems in a strictly non-blocking architecture.
- Dispatches parsed RoboLink inputs to motor and servo handlers every loop iteration.
- Enforces an automated safety failsafe: if connection times out, base motors are commanded to immediate stop.
- Logs a 2-second diagnostic status summary over the Serial Monitor (115200 baud).

---

## 3. Planned Logical Control Mapping

The following mapping links the RoboLink mobile application widgets directly to robot motion:

| Subsystem | RoboLink Key | Input Range | Planned Robot Behavior |
| :--- | :--- | :--- | :--- |
| **Base** | `"throttle"` | -255 to +255 | Forward (+) and Reverse (-) translation |
| **Base** | `"steer"` | -255 to +255 | Left (-) and Right (+) differential turning |
| **Arm target X** | `"arm_x"` | 0 to 390 mm | Forward distance from shoulder pivot |
| **Arm target height** | `"arm_y"` | 0 to 490 mm | Gripping-center height above floor; converted to shoulder-relative Z |
| **Arm mode** | `"arm_mode"` | 0 or 1 | 0 selects Cartesian IK; 1 selects direct joint-angle sliders |
| **Shoulder angle** | `"shoulder_deg"` | 0 to 180° | Direct shoulder target; used when `arm_mode=1` |
| **Elbow angle** | `"elbow_deg"` | 0 to 180° | Direct elbow target; used when `arm_mode=1` |
| **Gripper angle** | `"gripper_deg"` | 0° to 90° | Drives both MG90S servos together; 0° is the starter close endpoint and 90° is open/home |
| **Instant gripper open** | `"grip_open"` | 0 or 1 | Momentary rising-edge command immediately targets both servos to 90°; slider target resumes after it changes |
| **All-servos home** | `"all_90"` | 0 or 1 | Momentary rising-edge command targets shoulder, elbow, and both gripper servos at 90°; normal arm control resumes when an arm/gripper target changes |

The firmware uses the lower-elbow IK branch. Geometry: 140 mm shoulder-to-elbow, 390 mm total shoulder-to-gripping-center reach, and shoulder pivot 100 mm above the floor. Effective elbow-to-gripping-center L2 is 250 mm.
Servo angle mapping constants are in `include/config.h` and must match the installed horns.

---

## 4. Current State: What is Ready vs. What Depends on Electrical Team

### What is Completed & Ready (Software Layer)
1. **Full Software Skeleton**: Modular, clean C++ code in `src/` and `include/` compiling with ESP32 Arduino framework.
2. **Verified Wireless Protocol**: Official RoboLink UDP communication logic, packet parsing, and timeout watchdog implemented.
3. **Drive Steering**: With throttle centered, steering stops one motor and drives the other as a pivot turn. During translation, normalized `throttle ± steer` mixing and deadband filtering are used. The unpowered wheels must be able to swivel or skid for the chassis to turn.
4. **Arm Boundary Protection**: Angle limiting, joint coordinate tracking, and park/home pose management.
5. **Safety Isolation**: Hardcoded pin assignments ensuring hardware outputs are properly initialized.

### What Depends on Electrical Team / Hardware Calibration
1. **ESP32 Module & IDE Target**: ESP32-WROOM-32; Arduino IDE `ESP32 Dev Module`, PlatformIO `esp32dev`.
2. **BTS7960 Driver Pin Connections**: Left RPWM=GPIO25, LPWM=GPIO26, enable=GPIO22; right RPWM=GPIO32, LPWM=GPIO33, enable=GPIO23. Tie each driver's R_EN and L_EN together.
3. **Servo Signal GPIO Allocation**: SHOULDER=18, ELBOW=19, GRIPPER_1=21, GRIPPER_2=27; the second gripper signal pin should be wired and checked before enabling both outputs.
4. **Power Rail Architecture & Buck Converter Topology**: To be confirmed and calibrated on hardware.
5. **Arm Geometry**: L1=140 mm, total reach=390 mm to gripping center, shoulder pivot=100 mm above floor (user-measured; effective L2=250 mm).
6. **Physical Mechanical Calibration**: Servo horn configurations and physical stop limits should be verified on the actual hardware.

---

## 5. Development Roadmap & Constraints

1. **Strict No-Assumptions Rule**: Firmware code uses confirmed pin assignments from `config.h`.
2. **Modular Architecture**:
   - `include/config.h`: Hardware pinouts, constants, and calibration (confirmed values).
   - `lib/drive/`: BTS7960 DC motor driver control using ESP32 LEDC PWM.
   - `lib/arm/`: 2-DOF coordinate / pulse width management using ESP32 LEDC.
   - `lib/comms/`: RoboLink Wi-Fi communication handler.
   - `lib/safety/`: Brownout prevention, watchdog timer, battery voltage monitoring, and emergency stop.
3. **Non-Blocking Operation**: Firmware will adhere to strict non-blocking design (`millis()` timers, asynchronous network callbacks, FreeRTOS tasks where appropriate).
4. **Hardware Calibration Required**:
   - Servo pulse width calibration (verify MG90S and digital servos on actual arm)
   - Arm link endpoint verification (L1=140mm; total shoulder-to-gripping-center reach=390mm)
   - Physical stop limit confirmation
   - Gripper jaw travel measurement
