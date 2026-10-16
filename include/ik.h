#ifndef ROBOREACH_IK_H
#define ROBOREACH_IK_H

#include <Arduino.h>

// User-measured geometry in millimeters. The total distance from shoulder
// pivot to jaw gripping center is 390 mm, so effective L2 is 390 - 140 = 250.
#define IK_LINK_SHOULDER_TO_ELBOW (140.0f)
#define IK_LINK_ELBOW_TO_TARGET   (250.0f)

struct IKTargetPosition {
    float x; // Forward distance from shoulder, mm
    float z; // Vertical distance from shoulder, mm (may be negative)
};

struct IKJointAngles {
    float shoulder; // Link direction from +X, degrees
    float elbow;    // Signed relative bend: 0 = straight, +/-180 = folded
};

struct IKPose {
    IKTargetPosition elbowPosition;
    IKTargetPosition targetPosition;
    IKJointAngles angles;
};

typedef enum {
    IK_SUCCESS      = 0,
    IK_OUT_OF_REACH = 1,
    IK_INVALID_ARG  = 2
} IKStatus;

typedef enum {
    IK_BRANCH_UPPER = 0,
    IK_BRANCH_LOWER = 1
} IKBranch;

IKStatus ikInit(float shoulderToElbow, float elbowToTarget);
IKStatus ikCalculate(const IKTargetPosition* target, IKJointAngles* angles, IKPose* pose);
IKStatus ikCalculateBranch(const IKTargetPosition* target, IKBranch branch,
                           IKJointAngles* angles, IKPose* pose);
IKStatus ikForwardKinematics(const IKJointAngles* angles,
                             IKTargetPosition* target,
                             IKTargetPosition* elbowPosition);

#endif // ROBOREACH_IK_H
