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

    uint32_t upperJointLimitRaw;
    uint32_t lowerJointLimitRaw;
};
typedef struct Joint Joint;

Joint jointConstruct(uint32_t SSIBase, SSIEncoderBrand encoderBrand, uint16_t sample_rate,
                     uint32_t upperLimitRaw, uint32_t lowerLimitRaw);

void updateJointAngles(Joint* joint);

#endif /* JOINT_H_ */
