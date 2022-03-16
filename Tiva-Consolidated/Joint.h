/*
 * Joint.h
 *
 *  Created on: Feb 7, 2022
 *      Author: n
 */

#ifndef JOINT_H_
#define JOINT_H_

#include <math.h>

#include "SSIEncoder.h"
/**
 * Joint
 * A structure which holds all of the important
 * variables and information about a Joint for Athena
 */
struct Joint
{
    SSIEncoder encoder;
    float angleRads;
    float angleDegrees;
    int32_t actualRaw;

    // For localization on a joint
    int8_t jointReverseFactor;
    uint16_t rawZero;
    uint16_t rawForwardRangeOfMotion;
    uint16_t rawBackwardRangeOfMotion;
};
typedef struct Joint Joint;

Joint jointConstruct(uint32_t SSIBase, SSIEncoderBrand encoderBrand, uint16_t sample_rate,
                     int8_t jointReverseFactor, uint16_t rawZero,
                     uint16_t rawForwardRangeOfMotion, uint16_t rawBackwardRangeOfMotion);

void updateJointAngles(Joint* joint);
void getRawActualValue(Joint* joint);

#endif /* JOINT_H_ */
