#ifndef ROBOREACH_CONFIG_H
#define ROBOREACH_CONFIG_H

#include <Arduino.h>

/**
 * ============================================================================
 * RoboReach Firmware Configuration & Hardware Placeholders
 * ============================================================================
 * Authoritative Hardware Source: "ROBOREACH BOM - Sheet1.pdf"
 * 
 * IMPORTANT NOTICE:
 * All pin assignments, electrical parameters, and physical calibrations below
 * are set per the 2-DOF arm specification. 
 * 
 * Pins are assigned - no more TBD placeholders for GPIO outputs.
 * ============================================================================
 */

 /* ----------------------------------------------------------------------------
  * 1. Microcontroller Target
  * ----------------------------------------------------------------------------
  * Microcontroller: ESP32 (Dual-core Tensilica Xtensa LX6)
 * Module:          ESP32-WROOM-32 (Arduino IDE board: ESP32 Dev Module)
 * Note: Avoid strapping pins (GPIO 0, 2, 5, 12, 15), flash pins (6-11), and
 *       input-only pins (34-39) for output controls.
  */
 #define ESP32_BOARD_WROOM_32          1

/* ----------------------------------------------------------------------------
 * 2. Mobile Base Drive System (BTS7960 + 2 x 12V 100RPM Motors)
 * ----------------------------------------------------------------------------
 * Driver: High-Power BTS7960 Dual H-Bridge (43A rating)
 * Motors: 2 x 12V 100RPM Geared DC Motors
 * Wheels: 100mm Diameter
 *
 * BTS7960 Control Pins (Left & Right Channels):
 * Confirmed per task requirements:
 */
 #define PIN_MOTOR_L_RPWM              (25)  // Left-front motor BTS7960 RPWM
 #define PIN_MOTOR_L_LPWM              (26)  // Left-front motor BTS7960 LPWM
 #define PIN_MOTOR_L_EN                (22)  // Left-front BTS7960 R_EN/L_EN tied together

 #define PIN_MOTOR_R_RPWM              (32)  // Right-rear motor BTS7960 RPWM
 #define PIN_MOTOR_R_LPWM              (33)  // Right-rear motor BTS7960 LPWM
 #define PIN_MOTOR_R_EN                (23)  // Right-rear BTS7960 R_EN/L_EN tied together

// Motor PWM Generation Settings (ESP32 LEDC Timer)
 #define MOTOR_PWM_FREQ_HZ             (20000) // Ultrasonic 20kHz default to avoid audible motor hum
 #define MOTOR_PWM_RESOLUTION_BITS     (8)     // 8-bit resolution (0 to 255)
 #define MOTOR_PWM_CHANNEL_L_FWD       (0)     // LEDC Channel 0
 #define MOTOR_PWM_CHANNEL_L_REV       (1)     // LEDC Channel 1
 #define MOTOR_PWM_CHANNEL_R_FWD       (2)     // LEDC Channel 2
 #define MOTOR_PWM_CHANNEL_R_REV       (3)     // LEDC Channel 3

// Base Deadband & Speed Scaling
 #define MOTOR_INPUT_DEADBAND          (10)    // Ignore minor joystick center drift (-10 to +10)
 #define MOTOR_MAX_DUTY                (255)   // Max 8-bit duty cycle

/* ----------------------------------------------------------------------------
 * 3. 2-DOF Robotic Arm (Shoulder + Elbow + dual-servo Gripper)
 * ----------------------------------------------------------------------------
 * Actuator Mapping per task specification:
 * - Shoulder Joint: Annimos 60kg Digital Servo (High torque lift)
 * - Elbow Joint: 20kg Digital Servo
 * - End Effector: two MG90S gripper servos, controlled as one paired gripper
 * - Arm Geometry:
 *   L1 = 140 mm (shoulder to elbow, user-measured)
 *   Shoulder pivot = 100 mm above floor (user-measured)
 *   Total shoulder-to-gripping-center reach = 390 mm (user-measured)
 *   Effective L2 = 250 mm (390 mm total - 140 mm L1)
 *
 * Signal GPIO assignments for the ESP32-WROOM-32 carrier board:
 */
 #define PIN_SERVO_SHOULDER            (18)  // Shoulder lift joint (Annimos 60kg)
 #define PIN_SERVO_ELBOW               (19)  // Elbow joint (20kg servo)
 #define PIN_SERVO_GRIPPER             (21)  // Paired gripper servo 1 (MG90S)
 #define PIN_SERVO_GRIPPER_2           (27)  // Paired gripper servo 2 (MG90S)

// Servo PWM Settings (ESP32 LEDC Channels)
// Channels 4-7 cover the shoulder, elbow, and two gripper servos.
 #define SERVO_PWM_FREQ_HZ             (50)  // Standard 50Hz RC servo refresh rate
 #define SERVO_PWM_RESOLUTION_BITS     (16)  // 16-bit resolution for smooth position control
 #define SERVO_PWM_CHANNEL_SHOULDER    (4)   // LEDC Channel 4
 #define SERVO_PWM_CHANNEL_ELBOW       (5)   // LEDC Channel 5
 #define SERVO_PWM_CHANNEL_GRIPPER     (6)   // LEDC Channel 6
 #define SERVO_PWM_CHANNEL_GRIPPER_2   (7)   // LEDC Channel 7

/* ----------------------------------------------------------------------------
 * 4. Servo Safe Calibration Limits & Park Positions
 * ----------------------------------------------------------------------------
 * All angle boundaries (degrees) and pulse width limits (microseconds)
 * must be verified on the physical mechanical assembly to prevent binding.
 * Values below are standard starting points - calibrate on hardware!
 */
// Pulse width defaults (500us to 2500us standard for 180/270 deg digital servos)
 #define SERVO_PULSE_MIN_US            (500)
 #define SERVO_PULSE_MAX_US            (2500)

// Joint Angle Travel Limits (Degrees)
// Shoulder: 0-180 degrees (Annimos 60kg lifts arm)
// Elbow: 0-180 degrees (20kg arm extension)
// Gripper angle limits and close endpoint are starter values. Verify the
// MG90S jaw travel on the physical assembly before full-range use.
 #define SHOULDER_MIN_DEG              (0)   // Lower physical stop
 #define SHOULDER_MAX_DEG              (180) // Upper physical stop
 #define SHOULDER_HOME_DEG             (90)  // Neutral resting angle

 #define ELBOW_MIN_DEG                 (0)   // Lower physical stop
 #define ELBOW_MAX_DEG                 (180) // Upper physical stop
 #define ELBOW_HOME_DEG                (90)  // Neutral resting angle

 #define GRIPPER_MIN_DEG               (0)
 #define GRIPPER_MAX_DEG               (180)
 #define GRIPPER_HOME_DEG              (90)  // Startup/open position
 #define GRIPPER_CLOSE_DEG             (0)   // Starter close command; calibrate mechanically
 #define GRIPPER_STEP_DEG              (1)   // Gradual motion toward the RoboLink slider target
 #define GRIPPER_STEP_INTERVAL_MS      (20)
 #define GRIPPER_SECOND_SERVO_REVERSED (1)   // Mirrored servo: reverse around 90-degree home

/* ----------------------------------------------------------------------------
 * 5. RoboLink Mobile Application Protocol & Control Keys
 * ----------------------------------------------------------------------------
 * Communication: UDP broadcast over Wi-Fi
 * Default IP:    192.168.4.1 (ESP32 AP Mode)
 * Default Port:  4210
 * Control mapping for 2-DOF arm:
 *   arm_mode -> 0=Cartesian inverse kinematics, 1=direct joint sliders
 *   arm_x -> forward target X from shoulder pivot (0 to 390 mm)
 *   arm_y -> target height above floor (0 to 490 mm)
 *   shoulder_deg / elbow_deg -> direct joint angles in degrees (0 to 180)
 *   gripper_deg -> paired gripper target angle (degrees, drives both MG90S servos)
 *   grip_open -> momentary command to send both gripper servos to open/home
 *   all_90 -> momentary command to set all four servos to 90 degrees
 */
// Set these local values before flashing. Keep real Wi-Fi credentials out of Git.
 #define ROBOLINK_WIFI_SSID            "YOUR_WIFI_SSID"
 #define ROBOLINK_WIFI_PASS            "YOUR_WIFI_PASSWORD" // Replace with a password of at least 8 characters.
 #define ROBOLINK_UDP_PORT             (4210)
 #define ROBOLINK_TIMEOUT_MS           (500)          // Failsafe triggers if no packets for 500ms

// RoboLink values are millimeters. X is forward from the shoulder pivot;
// arm_y is absolute height above the floor and is converted to shoulder-relative Z.
 #define SHOULDER_PIVOT_HEIGHT_MM      (100)
 #define ROBOLINK_ARM_X_MAX_MM         (390)
 #define ROBOLINK_ARM_HEIGHT_MAX_MM    (490) // theoretical reach: pivot height + 390 mm

// Default pose: shoulder horizontal and second link vertical, yielding
// shoulder=90 degrees and elbow=90 degrees with the configured servo mapping.
 #define ROBOLINK_ARM_HOME_X_MM        (140)
 #define ROBOLINK_ARM_HOME_HEIGHT_MM   (350) // 100 mm pivot + 250 mm relative Z

// RoboLink arm control mode. IK remains the default to preserve the existing
// Cartesian behavior; select direct joint mode to use the angle sliders.
 #define ARM_CONTROL_MODE_IK           (0)
 #define ARM_CONTROL_MODE_JOINT        (1)
 #define ARM_CONTROL_MODE_HOME         ARM_CONTROL_MODE_IK

// Initial servo geometry mapping. Adjust for the installed servo horns:
// shoulder horizontal = 90 degrees; positive shoulder angle raises the arm;
// elbow straight = 180 degrees and flexion decreases the servo command.
 #define IK_SHOULDER_HORIZONTAL_SERVO_DEG (90.0f)
 #define IK_SHOULDER_SERVO_DIRECTION      (1.0f)
 #define IK_ELBOW_STRAIGHT_SERVO_DEG      (180.0f)
 #define IK_ELBOW_FLEX_SERVO_DIRECTION    (-1.0f)

// Documented Logical Keys from Mobile App
 #define KEY_BASE_THROTTLE             "throttle"     // Base forward (+255) / backward (-255)
 #define KEY_BASE_STEER                "steer"        // Base left (-255) / right (+255)
 #define KEY_ARM_X                     "arm_x"        // Forward target X (0 to 390 mm)
 #define KEY_ARM_Y                     "arm_y"        // Target height above floor (0 to 490 mm)
 #define KEY_ARM_MODE                  "arm_mode"     // 0=IK, 1=direct joint angle sliders
 #define KEY_ARM_SHOULDER_DEG          "shoulder_deg" // Direct shoulder command (0 to 180 degrees)
 #define KEY_ARM_ELBOW_DEG             "elbow_deg"    // Direct elbow command (0 to 180 degrees)
 #define KEY_ARM_GRIPPER_DEG           "gripper_deg"  // Paired slider: close=0, open/home=90 degrees
 #define KEY_GRIPPER_OPEN              "grip_open"    // Momentary: 1 opens both gripper servos
 #define KEY_ALL_SERVOS_90             "all_90"       // Momentary: 1 sets all four servos to 90 degrees

/* ----------------------------------------------------------------------------
 * 6. Electrical Power Architecture Notes
 * ----------------------------------------------------------------------------
 * - Battery: TBD (Chemistry, voltage, capacity, C-rating) - to calibrate on hardware
 * - Buck Converter: TBD (Exact module, voltage, current capacity) - to calibrate on hardware
 * - Dedicated Servo Rail: TBD (Annimos 60kg high-surge current isolation) - to calibrate on hardware
 */
 #define POWER_BATTERY_CELLS_TBD       0
 #define POWER_SERVO_RAIL_VOLTAGE_TBD  0

#endif // ROBOREACH_CONFIG_H
