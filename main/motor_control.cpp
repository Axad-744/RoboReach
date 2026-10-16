#include "motor_control.h"
#if __has_include(<esp_arduino_version.h>)
#include <esp_arduino_version.h>
#endif

static MotorTargetState currentState = {0, 0, true, false};

static void attachMotorPwm(int pin, int channel) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttach(pin, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_RESOLUTION_BITS);
#else
    ledcSetup(channel, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_RESOLUTION_BITS);
    ledcAttachPin(pin, channel);
#endif
}

static void writeMotorPwm(int pin, int channel, int duty) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(pin, duty);
#else
    ledcWrite(channel, duty);
#endif
}

static void writeMotorSide(int targetDuty, int rpwmPin, int lpwmPin, int rpwmChannel, int lpwmChannel) {
    int duty = constrain(abs(targetDuty), 0, MOTOR_MAX_DUTY);

    if (targetDuty > 0) {
        writeMotorPwm(rpwmPin, rpwmChannel, duty);
        writeMotorPwm(lpwmPin, lpwmChannel, 0);
    } else if (targetDuty < 0) {
        writeMotorPwm(rpwmPin, rpwmChannel, 0);
        writeMotorPwm(lpwmPin, lpwmChannel, duty);
    } else {
        writeMotorPwm(rpwmPin, rpwmChannel, 0);
        writeMotorPwm(lpwmPin, lpwmChannel, 0);
    }
}

static void writeMotorOutputs() {
    if (!currentState.hardwareConfigured) return;

    digitalWrite(PIN_MOTOR_L_EN, HIGH);
    digitalWrite(PIN_MOTOR_R_EN, HIGH);

    writeMotorSide(currentState.leftTargetDuty,
                   PIN_MOTOR_L_RPWM, PIN_MOTOR_L_LPWM,
                   MOTOR_PWM_CHANNEL_L_FWD, MOTOR_PWM_CHANNEL_L_REV);
    writeMotorSide(currentState.rightTargetDuty,
                   PIN_MOTOR_R_RPWM, PIN_MOTOR_R_LPWM,
                   MOTOR_PWM_CHANNEL_R_FWD, MOTOR_PWM_CHANNEL_R_REV);
}

void motorControlInit() {
    Serial.println(F("[MOTOR] Initializing Motor Control Subsystem..."));

    // Check if pins have been assigned by electrical team
    if (PIN_MOTOR_L_RPWM == -1 || PIN_MOTOR_L_LPWM == -1 ||
        PIN_MOTOR_L_EN == -1 ||
        PIN_MOTOR_R_RPWM == -1 || PIN_MOTOR_R_LPWM == -1 ||
        PIN_MOTOR_R_EN == -1) {
        currentState.hardwareConfigured = false;
        Serial.println(F("[MOTOR] NOTICE: BTS7960 GPIO pins are marked TBD (-1)."));
        Serial.println(F("[MOTOR] Hardware GPIO control is INACTIVE until wiring is confirmed."));
        return;
    }

    pinMode(PIN_MOTOR_L_EN, OUTPUT);
    pinMode(PIN_MOTOR_R_EN, OUTPUT);
    digitalWrite(PIN_MOTOR_L_EN, LOW);
    digitalWrite(PIN_MOTOR_R_EN, LOW);

    attachMotorPwm(PIN_MOTOR_L_RPWM, MOTOR_PWM_CHANNEL_L_FWD);
    attachMotorPwm(PIN_MOTOR_L_LPWM, MOTOR_PWM_CHANNEL_L_REV);
    attachMotorPwm(PIN_MOTOR_R_RPWM, MOTOR_PWM_CHANNEL_R_FWD);
    attachMotorPwm(PIN_MOTOR_R_LPWM, MOTOR_PWM_CHANNEL_R_REV);

    writeMotorPwm(PIN_MOTOR_L_RPWM, MOTOR_PWM_CHANNEL_L_FWD, 0);
    writeMotorPwm(PIN_MOTOR_L_LPWM, MOTOR_PWM_CHANNEL_L_REV, 0);
    writeMotorPwm(PIN_MOTOR_R_RPWM, MOTOR_PWM_CHANNEL_R_FWD, 0);
    writeMotorPwm(PIN_MOTOR_R_LPWM, MOTOR_PWM_CHANNEL_R_REV, 0);

    currentState.hardwareConfigured = true;
    writeMotorOutputs();
    Serial.println(F("[MOTOR] Motor pins configured."));
}

void motorControlUpdate(int throttle, int steer) {
    // Keep RoboLink commands within the configured joystick/PWM range.
    throttle = constrain(throttle, -MOTOR_MAX_DUTY, MOTOR_MAX_DUTY);
    steer = constrain(steer, -MOTOR_MAX_DUTY, MOTOR_MAX_DUTY);

    // Apply deadband around center position
    if (abs(throttle) < MOTOR_INPUT_DEADBAND) {
        throttle = 0;
    }
    if (abs(steer) < MOTOR_INPUT_DEADBAND) {
        steer = 0;
    }

    // With the throttle centered, turn around a stationary drive side instead
    // of counter-rotating both motors. Positive steer means right: drive the
    // left motor and stop the right. Negative steer means left: drive the
    // right motor and stop the left. This matches the user's one-motor-per-side
    // chassis and avoids requiring the motors to spin against each other.
    int leftRaw = 0;
    int rightRaw = 0;
    if (throttle == 0 && steer > 0) {
        leftRaw = steer;
    } else if (throttle == 0 && steer < 0) {
        rightRaw = -steer;
    } else {
        // Preserve proportional differential steering while translating.
        leftRaw = throttle + steer;
        rightRaw = throttle - steer;
    }

    // Scale both sides together if either exceeds the PWM range. This preserves
    // the left/right ratio and keeps steering effective at high throttle.
    const int largestOutput = max(abs(leftRaw), abs(rightRaw));
    if (largestOutput > MOTOR_MAX_DUTY) {
        leftRaw = (long)leftRaw * MOTOR_MAX_DUTY / largestOutput;
        rightRaw = (long)rightRaw * MOTOR_MAX_DUTY / largestOutput;
    }

    currentState.leftTargetDuty = leftRaw;
    currentState.rightTargetDuty = rightRaw;

    if (currentState.leftTargetDuty == 0 && currentState.rightTargetDuty == 0) {
        currentState.isStopped = true;
    } else {
        currentState.isStopped = false;
    }

    writeMotorOutputs();
}

void motorControlStop() {
    currentState.leftTargetDuty  = 0;
    currentState.rightTargetDuty = 0;
    currentState.isStopped       = true;

    writeMotorOutputs();
}

MotorTargetState getMotorState() {
    return currentState;
}
