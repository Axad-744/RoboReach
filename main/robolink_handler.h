#ifndef ROBOLINK_HANDLER_H
#define ROBOLINK_HANDLER_H

#include <Arduino.h>
#include "../include/config.h"

/**
 * RoboLink Communication Handler Interface
 * 
 * Manages Wi-Fi AP setup and extracts incoming control packets
 * from the RoboLink mobile application.
 */

void robolinkHandlerInit();
void robolinkHandlerUpdate();
bool robolinkIsConnected();

// Base control accessors
int  robolinkGetThrottle();
int  robolinkGetSteer();

// Cartesian 2-DOF arm target accessors
int  robolinkGetArmX();
int  robolinkGetArmY();
int  robolinkGetArmMode();
int  robolinkGetShoulderDeg();
int  robolinkGetElbowDeg();
// Direct gripper-slider target in degrees.
int  robolinkGetGripperDeg();
// Returns true once for each rising edge of the instant-open button.
bool robolinkConsumeGripperOpenCommand();
// All-servos home command and arm-control change notification.
bool robolinkConsumeAllServosHomeCommand();
bool robolinkConsumeArmControlChanged();

#endif // ROBOLINK_HANDLER_H
