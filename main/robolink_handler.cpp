#include "robolink_handler.h"
#include <RoboLink.h>

static RoboLinkWiFi robolink;
static bool initialized = false;

// Cached control values
static int currentThrottle = 0;
static int currentSteer    = 0;
static int currentArmX     = ROBOLINK_ARM_HOME_X_MM;
static int currentArmY     = ROBOLINK_ARM_HOME_HEIGHT_MM;
static int currentArmMode  = ARM_CONTROL_MODE_HOME;
static int currentShoulderDeg = SHOULDER_HOME_DEG;
static int currentElbowDeg    = ELBOW_HOME_DEG;
static int currentGripperDeg = GRIPPER_HOME_DEG;
static int lastGripperSliderValue = GRIPPER_HOME_DEG;
static bool gripperOpenButtonWasDown = false;
static bool gripperOpenCommandPending = false;
static bool allServosHomeButtonWasDown = false;
static bool allServosHomeCommandPending = false;
static bool armControlChangePending = false;

void robolinkHandlerInit() {
    Serial.println(F("[COMMS] Initializing RoboLink Wi-Fi AP..."));

    if (!robolink.beginAP(ROBOLINK_WIFI_SSID, ROBOLINK_WIFI_PASS, ROBOLINK_UDP_PORT)) {
        Serial.println(F("[COMMS] ERROR: Failed to launch Wi-Fi AP!"));
        initialized = false;
        return;
    }

    robolink.setTimeoutMs(ROBOLINK_TIMEOUT_MS);
    initialized = true;

    Serial.print(F("[COMMS] RoboReach Wi-Fi AP Started: "));
    Serial.println(ROBOLINK_WIFI_SSID);
    Serial.print(F("[COMMS] IP Address : "));
    Serial.println(robolink.localIP());
    Serial.print(F("[COMMS] UDP Port   : "));
    Serial.println(ROBOLINK_UDP_PORT);
}

void robolinkHandlerUpdate() {
    if (!initialized) return;

    robolink.update();

    if (robolinkIsConnected()) {
        // Read mobile base controls
        currentThrottle = robolink.get(KEY_BASE_THROTTLE, 0);
        currentSteer    = robolink.get(KEY_BASE_STEER, 0);

        // Keep the last received slider values when packets omit a key.
        // Cartesian values are millimeters; direct joint values are degrees.
        const int armX = robolink.get(KEY_ARM_X, currentArmX);
        const int armY = robolink.get(KEY_ARM_Y, currentArmY);
        const int armMode = robolink.get(KEY_ARM_MODE, currentArmMode);
        const int shoulderDeg = robolink.get(KEY_ARM_SHOULDER_DEG, currentShoulderDeg);
        const int elbowDeg = robolink.get(KEY_ARM_ELBOW_DEG, currentElbowDeg);
        if (armX != currentArmX || armY != currentArmY ||
            armMode != currentArmMode || shoulderDeg != currentShoulderDeg ||
            elbowDeg != currentElbowDeg) {
            armControlChangePending = true;
        }
        currentArmX = armX;
        currentArmY = armY;
        currentArmMode = armMode;
        currentShoulderDeg = shoulderDeg;
        currentElbowDeg = elbowDeg;
        // Apply slider changes only when its value changes. This lets the
        // instant-open button override a stationary slider position.
        const int sliderValue = robolink.get(KEY_ARM_GRIPPER_DEG,
                                             lastGripperSliderValue);
        if (sliderValue != lastGripperSliderValue) {
            lastGripperSliderValue = sliderValue;
            currentGripperDeg = sliderValue;
            armControlChangePending = true;
        }

        const bool openButtonDown = robolink.get(KEY_GRIPPER_OPEN, 0) != 0;
        if (openButtonDown && !gripperOpenButtonWasDown) {
            currentGripperDeg = GRIPPER_HOME_DEG;
            gripperOpenCommandPending = true;
        }
        gripperOpenButtonWasDown = openButtonDown;

        const bool homeButtonDown = robolink.get(KEY_ALL_SERVOS_90, 0) != 0;
        if (homeButtonDown && !allServosHomeButtonWasDown) {
            allServosHomeCommandPending = true;
            armControlChangePending = false;
        }
        allServosHomeButtonWasDown = homeButtonDown;
    } else {
        // If disconnected or timed out, reset drive throttle/steer to 0
        currentThrottle = 0;
        currentSteer    = 0;
    }
}

bool robolinkIsConnected() {
    if (!initialized) return false;
    return (robolink.hasReceivedData() && !robolink.isTimedOut());
}

int robolinkGetThrottle() { return currentThrottle; }
int robolinkGetSteer()    { return currentSteer; }

int robolinkGetArmX()     { return currentArmX; }
int robolinkGetArmY()     { return currentArmY; }
int robolinkGetArmMode()  { return currentArmMode; }
int robolinkGetShoulderDeg() { return currentShoulderDeg; }
int robolinkGetElbowDeg()    { return currentElbowDeg; }
int robolinkGetGripperDeg() { return currentGripperDeg; }

bool robolinkConsumeGripperOpenCommand() {
    const bool pending = gripperOpenCommandPending;
    gripperOpenCommandPending = false;
    return pending;
}

bool robolinkConsumeAllServosHomeCommand() {
    const bool pending = allServosHomeCommandPending;
    allServosHomeCommandPending = false;
    return pending;
}

bool robolinkConsumeArmControlChanged() {
    const bool changed = armControlChangePending;
    armControlChangePending = false;
    return changed;
}
