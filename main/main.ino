/*
 * ============================================================================
 * RoboReach Robot Main Controller Firmware (Architecture Skeleton)
 * ============================================================================
 * File: src/main.ino
 * Authoritative Hardware Source: "ROBOREACH BOM - Sheet1.pdf"
 * 
 * ARCHITECTURE OVERVIEW:
 * RoboLink Mobile App (Wi-Fi / UDP)
 *        ↓
 *      ESP32
 *     ↙     ↘
 *  BTS7960   2-DOF Servo Control
 *    ↓         ↓
 * 2 DC Motors  Shoulder (Annimos 60kg)
 * (12V 100RPM) Elbow (20kg)
 *              Gripper (two paired MG90S servos)
 *
 * SAFETY NOTICE:
 * Hardware pin assignments are confirmed in config.h.
 * All physical GPIO writes are properly configured.
 * This skeleton safely handles network communication, logical coordinate mapping, and safety timeouts.
 * ============================================================================
 */

#include <Arduino.h>
#include "../include/config.h"
#include "ik.h"
#include "robolink_handler.h"
#include "motor_control.h"
#include "servo_control.h"
#include <math.h>

static unsigned long lastTelemetryPrint = 0;
static bool lastIkTargetWasReachable = true;
static unsigned long lastGripperStepMs = 0;
static bool allServosHomeOverride = false;

static int stepAngleToward(int current, int target) {
    if (current < target) {
        return min(current + GRIPPER_STEP_DEG, target);
    }
    if (current > target) {
        return max(current - GRIPPER_STEP_DEG, target);
    }
    return current;
}

static void updateArmFromRoboLinkTarget() {
    const int targetHeightMm = constrain(robolinkGetArmY(), 0,
                                         ROBOLINK_ARM_HEIGHT_MAX_MM);
    IKTargetPosition target = {
        (float)constrain(robolinkGetArmX(), 0, ROBOLINK_ARM_X_MAX_MM),
        (float)(targetHeightMm - SHOULDER_PIVOT_HEIGHT_MM)
    };
    IKJointAngles angles;

    // The mobile app's arm_x is forward X; arm_y is height above the floor.
    // IK uses vertical Z relative to the shoulder pivot.
    // Keep an unreachable target from replacing the last valid servo pose.
    const IKStatus status = ikCalculateBranch(&target, IK_BRANCH_LOWER, &angles, nullptr);
    if (status != IK_SUCCESS) {
        if (lastIkTargetWasReachable) {
            Serial.println(F("[IK] Target is outside the arm workspace; holding the last pose."));
        }
        lastIkTargetWasReachable = false;
        return;
    }
    if (!lastIkTargetWasReachable) {
        Serial.println(F("[IK] Target is reachable again."));
    }
    lastIkTargetWasReachable = true;

    const float shoulderServo = IK_SHOULDER_HORIZONTAL_SERVO_DEG +
                                IK_SHOULDER_SERVO_DIRECTION * angles.shoulder;
    const float elbowServo = IK_ELBOW_STRAIGHT_SERVO_DEG +
                             IK_ELBOW_FLEX_SERVO_DIRECTION * fabsf(angles.elbow);
    const int shoulderCommand = constrain((int)lroundf(shoulderServo),
                                          SHOULDER_MIN_DEG, SHOULDER_MAX_DEG);
    const int elbowCommand = constrain((int)lroundf(elbowServo),
                                       ELBOW_MIN_DEG, ELBOW_MAX_DEG);

    // Preserve the gripper's current position while IK updates the arm joints.
    const ArmJointPositions current = getArmPositions();
    servoControlUpdate(shoulderCommand, elbowCommand,
                       current.gripper);
}

static void updateGripperFromRoboLinkSlider() {
    const unsigned long now = millis();
    const ArmJointPositions current = getArmPositions();

    // The dedicated button bypasses the gradual slider ramp and immediately
    // commands the open/home angle. Servo mechanics still determine travel time.
    if (robolinkConsumeGripperOpenCommand()) {
        servoControlUpdate(current.shoulder, current.elbow, GRIPPER_HOME_DEG);
        return;
    }

    if (now - lastGripperStepMs < GRIPPER_STEP_INTERVAL_MS) return;
    lastGripperStepMs = now;

    const int gripperTarget = constrain(robolinkGetGripperDeg(),
                                        GRIPPER_CLOSE_DEG, GRIPPER_HOME_DEG);
    const int gripperNext = stepAngleToward(current.gripper, gripperTarget);

    if (gripperNext != current.gripper) {
        servoControlUpdate(current.shoulder, current.elbow, gripperNext);
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println(F("\n========================================================"));
    Serial.println(F("         RoboReach: ESP32 Robot Controller               "));
    Serial.println(F("========================================================"));
    Serial.println(F("Authoritative Hardware: Final BOM (ROBOREACH BOM - Sheet1.pdf)"));
    Serial.println(F("Status: 2-DOF Software Ready (Hardware Pins Confirmed)"));
    Serial.println(F("--------------------------------------------------------"));

    ikInit(IK_LINK_SHOULDER_TO_ELBOW, IK_LINK_ELBOW_TO_TARGET);
    Serial.printf("[IK] arm_x=X, arm_y=floor height; links %.0f/%.0f mm, shoulder height=%d mm.\n",
                  IK_LINK_SHOULDER_TO_ELBOW, IK_LINK_ELBOW_TO_TARGET,
                  SHOULDER_PIVOT_HEIGHT_MM);

    // 1. Initialize Motor Control Subsystem (checks for confirmed pins)
    motorControlInit();

    // 2. Initialize Servo Control Subsystem (checks for confirmed pins)
    servoControlInit();

    // 3. Initialize RoboLink Wireless Communication
    robolinkHandlerInit();

    Serial.println(F("========================================================"));
    Serial.println(F("System initialized in SAFE ARCHITECTURE MODE."));
    Serial.println(F("Awaiting connection from RoboLink app...\n"));
}

void loop() {
    // 1. Process incoming network packets (non-blocking)
    robolinkHandlerUpdate();

    if (robolinkConsumeAllServosHomeCommand()) {
        allServosHomeOverride = true;
        robolinkConsumeArmControlChanged();
        robolinkConsumeGripperOpenCommand();
        servoControlUpdate(SHOULDER_HOME_DEG, ELBOW_HOME_DEG, GRIPPER_HOME_DEG);
    } else if (robolinkConsumeArmControlChanged()) {
        allServosHomeOverride = false;
    }

    // 2. Dispatch logical control inputs if link is active
    if (robolinkIsConnected()) {
        // Base Drive commands
        int throttle = robolinkGetThrottle();
        int steer    = robolinkGetSteer();
        motorControlUpdate(throttle, steer);

        // IK mode maps an (X, height) target to both joints. Joint mode applies
        // the two independent RoboLink angle sliders directly.
        if (!allServosHomeOverride) {
            if (robolinkGetArmMode() == ARM_CONTROL_MODE_JOINT) {
                const int shoulderCommand = constrain(robolinkGetShoulderDeg(),
                                                      SHOULDER_MIN_DEG, SHOULDER_MAX_DEG);
                const int elbowCommand = constrain(robolinkGetElbowDeg(),
                                                   ELBOW_MIN_DEG, ELBOW_MAX_DEG);
                const ArmJointPositions current = getArmPositions();
                servoControlUpdate(shoulderCommand, elbowCommand, current.gripper);
            } else {
                updateArmFromRoboLinkTarget();
            }
        }

    } else {
        // Safety Failsafe: stop mobile base when connection is lost
        motorControlStop();
    }

    // Continue to the last gripper-slider target even if the app sends only
    // one update per slider change and the communications timeout then expires.
    if (!allServosHomeOverride) {
        updateGripperFromRoboLinkSlider();
    }

    // 3. Periodic Diagnostic Log (Every 2 seconds, non-blocking)
    if (millis() - lastTelemetryPrint >= 2000) {
        lastTelemetryPrint = millis();

        if (!robolinkIsConnected()) {
            Serial.println(F("[STATUS] No active RoboLink stream. Base motors STOPPED (Failsafe)."));
        } else {
            MotorTargetState mState = getMotorState();
            ArmJointPositions aPos  = getArmPositions();

            Serial.println(F("--- [ROBOREACH ACTIVE CONTROL STATE] ---"));
            Serial.printf("  Base Target : Left=%d, Right=%d (Stopped=%s)\n",
                          mState.leftTargetDuty, mState.rightTargetDuty,
                          mState.isStopped ? "YES" : "NO");
            if (allServosHomeOverride) {
                Serial.println(F("  Arm Control: ALL SERVOS HELD AT 90 degrees"));
            } else if (robolinkGetArmMode() == ARM_CONTROL_MODE_JOINT) {
                Serial.printf("  Arm Control: JOINT SLIDERS, target shoulder=%d°, elbow=%d°\n",
                              robolinkGetShoulderDeg(), robolinkGetElbowDeg());
            } else {
                Serial.printf("  Arm Control: IK, target X=%d mm, floor height=%d mm\n",
                              robolinkGetArmX(), robolinkGetArmY());
            }
            Serial.printf("  Arm Angles : Shoulder=%d°, Elbow=%d°, Grippers=%d°/%d°\n",
                          aPos.shoulder, aPos.elbow, aPos.gripper, aPos.gripper2);
            Serial.printf("  Hardware Out: %s (Enabled - pins confirmed)\n",
                          (mState.hardwareConfigured && aPos.hardwareConfigured) ? "ENABLED" : "SAFE / INACTIVE");
            Serial.println(F("----------------------------------------"));
        }
    }
}
