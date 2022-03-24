/*
 * Joint.h
 * @author: Nick Tremaroli
 * Contains the layout and functions regarding a joint
 */

#ifndef JOINT_H_
#define JOINT_H_

#include <math.h>

#include "SSIEncoder.h"

/**
 * Joint
 * Contains all of the data and structures needed by a joint
 * on the Tiva
 */
struct Joint
{
    SSIEncoder encoder;
    float angleRads;
    float angleDegrees;
    int32_t actualRaw;

    int8_t jointReverseFactor;
    uint16_t rawZero;
    uint16_t rawForwardRangeOfMotion;
    uint16_t rawBackwardRangeOfMotion;
};
typedef struct Joint Joint;

// constructs a joint
Joint jointConstruct(uint32_t SSIBase, SSIEncoderBrand encoderBrand, uint16_t sampleRate,
                     int8_t jointReverseFactor, uint16_t rawZero,
                     uint16_t rawForwardRangeOfMotion, uint16_t rawBackwardRangeOfMotion);

// updates the joint angles
void updateJointAngles(Joint* joint);

// converts the raw joint reading to the actual joint angle value
void getRawActualValue(Joint* joint);

#endif /* JOINT_H_ */
