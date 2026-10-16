#include "servo_control.h"
#if __has_include(<esp_arduino_version.h>)
#include <esp_arduino_version.h>
#endif

static ArmJointPositions currentPositions = {
    SHOULDER_HOME_DEG,
    ELBOW_HOME_DEG,
    GRIPPER_HOME_DEG,
    GRIPPER_HOME_DEG,
    false
};

static bool attachServoPwm(int pin, int channel) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    return ledcAttach(pin, SERVO_PWM_FREQ_HZ, SERVO_PWM_RESOLUTION_BITS);
#else
    const double configuredFrequency =
        ledcSetup(channel, SERVO_PWM_FREQ_HZ, SERVO_PWM_RESOLUTION_BITS);
    if (configuredFrequency <= 0) return false;
    ledcAttachPin(pin, channel);
    return true;
#endif
}

static bool writeServoPwm(int pin, int channel, int duty) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    return ledcWrite(pin, duty);
#else
    ledcWrite(channel, duty);
    return true;
#endif
}

static int angleToPulseUs(int angleDeg) {
    return map(angleDeg, 0, 180, SERVO_PULSE_MIN_US, SERVO_PULSE_MAX_US);
}

static int pulseUsToDuty(int pulseUs) {
    const int periodUs = 1000000 / SERVO_PWM_FREQ_HZ;
    const int maxDuty = (1 << SERVO_PWM_RESOLUTION_BITS) - 1;
    return (pulseUs * maxDuty) / periodUs;
}

static bool writeServoAngle(int pin, int channel, int angleDeg) {
    int pulseUs = angleToPulseUs(angleDeg);
    int duty = pulseUsToDuty(pulseUs);
    return writeServoPwm(pin, channel, duty);
}

static void writeServoOutputs() {
    if (!currentPositions.hardwareConfigured) return;

    const bool shoulderWritten =
        writeServoAngle(PIN_SERVO_SHOULDER, SERVO_PWM_CHANNEL_SHOULDER, currentPositions.shoulder);
    const bool elbowWritten =
        writeServoAngle(PIN_SERVO_ELBOW, SERVO_PWM_CHANNEL_ELBOW, currentPositions.elbow);
    const bool gripper1Written =
        writeServoAngle(PIN_SERVO_GRIPPER, SERVO_PWM_CHANNEL_GRIPPER, currentPositions.gripper);
    const bool gripper2Written =
        writeServoAngle(PIN_SERVO_GRIPPER_2, SERVO_PWM_CHANNEL_GRIPPER_2, currentPositions.gripper2);
    static bool writeFailureReported = false;
    if ((!shoulderWritten || !elbowWritten || !gripper1Written || !gripper2Written) &&
        !writeFailureReported) {
        writeFailureReported = true;
        Serial.printf("[SERVO] PWM write failed: shoulder=%s elbow=%s gripper1=%s gripper2=%s\n",
                      shoulderWritten ? "OK" : "FAILED",
                      elbowWritten ? "OK" : "FAILED",
                      gripper1Written ? "OK" : "FAILED",
                      gripper2Written ? "OK" : "FAILED");
    }

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    static unsigned long lastReadbackMs = 0;
    if (millis() - lastReadbackMs >= 2000) {
        lastReadbackMs = millis();
        Serial.printf("[SERVO] PWM readback: GPIO %d=%luHz/%lu duty, GPIO %d=%luHz/%lu duty, GPIO %d=%luHz/%lu duty, GPIO %d=%luHz/%lu duty\n",
                      PIN_SERVO_SHOULDER,
                      (unsigned long)ledcReadFreq(PIN_SERVO_SHOULDER),
                      (unsigned long)ledcRead(PIN_SERVO_SHOULDER),
                      PIN_SERVO_ELBOW,
                      (unsigned long)ledcReadFreq(PIN_SERVO_ELBOW),
                      (unsigned long)ledcRead(PIN_SERVO_ELBOW),
                      PIN_SERVO_GRIPPER,
                      (unsigned long)ledcReadFreq(PIN_SERVO_GRIPPER),
                      (unsigned long)ledcRead(PIN_SERVO_GRIPPER),
                      PIN_SERVO_GRIPPER_2,
                      (unsigned long)ledcReadFreq(PIN_SERVO_GRIPPER_2),
                      (unsigned long)ledcRead(PIN_SERVO_GRIPPER_2));
    }
#endif
}

void servoControlInit() {
    Serial.println(F("[SERVO] Initializing 2-DOF Arm and Gripper Servo Subsystem..."));

    // Check if pins have been assigned by electrical team
    if (PIN_SERVO_SHOULDER == -1 ||
        PIN_SERVO_ELBOW == -1 ||
        PIN_SERVO_GRIPPER == -1 ||
        PIN_SERVO_GRIPPER_2 == -1) {
        currentPositions.hardwareConfigured = false;
        Serial.println(F("[SERVO] NOTICE: Servo GPIO pins are marked TBD (-1)."));
        Serial.println(F("[SERVO] Servo PWM generation is INACTIVE until wiring is confirmed."));
        return;
    }

    const bool shoulderAttached =
        attachServoPwm(PIN_SERVO_SHOULDER, SERVO_PWM_CHANNEL_SHOULDER);
    const bool elbowAttached =
        attachServoPwm(PIN_SERVO_ELBOW, SERVO_PWM_CHANNEL_ELBOW);
    const bool gripper1Attached =
        attachServoPwm(PIN_SERVO_GRIPPER, SERVO_PWM_CHANNEL_GRIPPER);
    const bool gripper2Attached =
        attachServoPwm(PIN_SERVO_GRIPPER_2, SERVO_PWM_CHANNEL_GRIPPER_2);
    Serial.printf("[SERVO] GPIO %d PWM attach: %s\n",
                  PIN_SERVO_SHOULDER, shoulderAttached ? "OK" : "FAILED");
    Serial.printf("[SERVO] GPIO %d PWM attach: %s\n",
                  PIN_SERVO_ELBOW, elbowAttached ? "OK" : "FAILED");
    Serial.printf("[SERVO] GPIO %d (gripper 1) PWM attach: %s\n",
                  PIN_SERVO_GRIPPER, gripper1Attached ? "OK" : "FAILED");
    Serial.printf("[SERVO] GPIO %d (gripper 2) PWM attach: %s\n",
                  PIN_SERVO_GRIPPER_2, gripper2Attached ? "OK" : "FAILED");
    if (!shoulderAttached || !elbowAttached || !gripper1Attached || !gripper2Attached) {
        currentPositions.hardwareConfigured = false;
        Serial.println(F("[SERVO] ERROR: PWM setup failed; servo outputs disabled."));
        return;
    }

    currentPositions.hardwareConfigured = true;
    writeServoOutputs();
    Serial.println(F("[SERVO] Servo pins configured."));
}

void servoControlUpdate(int shoulder, int elbow, int gripper) {
    // Clamp each incoming target to safe mechanical boundaries defined in config.h
    currentPositions.shoulder = constrain(shoulder, SHOULDER_MIN_DEG, SHOULDER_MAX_DEG);
    currentPositions.elbow    = constrain(elbow,    ELBOW_MIN_DEG,    ELBOW_MAX_DEG);
    currentPositions.gripper  = constrain(gripper,  GRIPPER_MIN_DEG,  GRIPPER_MAX_DEG);
    const int secondGripperAngle = GRIPPER_SECOND_SERVO_REVERSED
        ? (2 * GRIPPER_HOME_DEG - currentPositions.gripper)
        : currentPositions.gripper;
    currentPositions.gripper2 = constrain(secondGripperAngle,
                                           GRIPPER_MIN_DEG, GRIPPER_MAX_DEG);

    writeServoOutputs();
}

void servoControlPark() {
    servoControlUpdate(
        SHOULDER_HOME_DEG,
        ELBOW_HOME_DEG,
        GRIPPER_HOME_DEG
    );
}

ArmJointPositions getArmPositions() {
    return currentPositions;
}
