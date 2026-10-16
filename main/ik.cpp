#include "ik.h"
#include <math.h>

static float linkShoulderToElbow = IK_LINK_SHOULDER_TO_ELBOW;
static float linkElbowToTarget = IK_LINK_ELBOW_TO_TARGET;

static float clampUnit(float value) {
    if (value < -1.0f) return -1.0f;
    if (value >  1.0f) return  1.0f;
    return value;
}

static bool finiteFloat(float value) {
    return isfinite(value);
}

IKStatus ikInit(float shoulderToElbow, float elbowToTarget) {
    if (!finiteFloat(shoulderToElbow) || !finiteFloat(elbowToTarget) ||
        shoulderToElbow <= 0.0f || elbowToTarget <= 0.0f) {
        return IK_INVALID_ARG;
    }

    linkShoulderToElbow = shoulderToElbow;
    linkElbowToTarget = elbowToTarget;
    return IK_SUCCESS;
}

IKStatus ikCalculate(const IKTargetPosition* target, IKJointAngles* angles, IKPose* pose) {
    return ikCalculateBranch(target, IK_BRANCH_LOWER, angles, pose);
}

IKStatus ikCalculateBranch(const IKTargetPosition* target, IKBranch branch,
                           IKJointAngles* angles, IKPose* pose) {
    if (target == NULL || angles == NULL ||
        !finiteFloat(target->x) || !finiteFloat(target->z) ||
        (branch != IK_BRANCH_UPPER && branch != IK_BRANCH_LOWER)) {
        return IK_INVALID_ARG;
    }

    const float l1 = linkShoulderToElbow;
    const float l2 = linkElbowToTarget;
    const float distance = hypotf(target->x, target->z);
    const float minReach = fabsf(l1 - l2);
    const float maxReach = l1 + l2;
    const float tolerance = 1e-3f;

    if (distance < minReach - tolerance || distance > maxReach + tolerance) {
        return IK_OUT_OF_REACH;
    }

    const float distanceSquared = distance * distance;
    const float cosRelative = clampUnit(
        (distanceSquared - l1 * l1 - l2 * l2) / (2.0f * l1 * l2));
    const float bendMagnitude = acosf(cosRelative);
    const float heading = atan2f(target->z, target->x);

    const float relativeA = bendMagnitude;
    const float relativeB = -bendMagnitude;
    const float shoulderA = heading - atan2f(l2 * sinf(relativeA),
                                              l1 + l2 * cosf(relativeA));
    const float shoulderB = heading - atan2f(l2 * sinf(relativeB),
                                              l1 + l2 * cosf(relativeB));
    const float elbowZA = l1 * sinf(shoulderA);
    const float elbowZB = l1 * sinf(shoulderB);

    // Select the solution by the elbow pivot's height in the XZ plane.
    const bool chooseA = (branch == IK_BRANCH_UPPER)
        ? (elbowZA >= elbowZB)
        : (elbowZA < elbowZB);
    const float shoulderRad = chooseA ? shoulderA : shoulderB;
    const float relativeRad = chooseA ? relativeA : relativeB;
    const float radiansToDegrees = 180.0f / PI;

    angles->shoulder = shoulderRad * radiansToDegrees;
    angles->elbow = relativeRad * radiansToDegrees;

    if (pose != NULL) {
        pose->elbowPosition.x = l1 * cosf(shoulderRad);
        pose->elbowPosition.z = l1 * sinf(shoulderRad);
        pose->targetPosition = *target;
        pose->angles = *angles;
    }

    return IK_SUCCESS;
}

IKStatus ikForwardKinematics(const IKJointAngles* angles,
                             IKTargetPosition* target,
                             IKTargetPosition* elbowPosition) {
    if (angles == NULL || target == NULL ||
        !finiteFloat(angles->shoulder) || !finiteFloat(angles->elbow)) {
        return IK_INVALID_ARG;
    }

    const float degreesToRadians = PI / 180.0f;
    const float shoulderRad = angles->shoulder * degreesToRadians;
    const float elbowRad = angles->elbow * degreesToRadians;
    const float elbowX = linkShoulderToElbow * cosf(shoulderRad);
    const float elbowZ = linkShoulderToElbow * sinf(shoulderRad);

    target->x = elbowX + linkElbowToTarget * cosf(shoulderRad + elbowRad);
    target->z = elbowZ + linkElbowToTarget * sinf(shoulderRad + elbowRad);

    if (elbowPosition != NULL) {
        elbowPosition->x = elbowX;
        elbowPosition->z = elbowZ;
    }

    return IK_SUCCESS;
}
