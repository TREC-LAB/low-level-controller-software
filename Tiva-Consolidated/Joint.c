/*
 * Joint.c
 *
 *  Created on: Feb 7, 2022
 *      Author: n
 */
#include "Joint.h"


Joint jointConstruct(uint32_t SSIBase, SSIEncoderBrand encoderBrand, uint16_t sample_rate,
                     uint32_t upperLimitRaw, uint32_t lowerLimitRaw)
{
    Joint joint;

    joint.encoder = encoderConstruct(SSIBase, sample_rate, encoderBrand);

    joint.upperJointLimitRaw = upperLimitRaw;
    joint.lowerJointLimitRaw = lowerLimitRaw;

    return joint;
}

void updateJointAngles(Joint* joint)
{
    readAbsEncoder(&joint->encoder);
    // Convert raw encoder sensor reading to radians
    if (joint->encoder.encoderBrand == Gurley_Encoder){
        joint->encoder.angleRads = joint->encoder.raw * M_PI / 65535;
    }
    else if (joint->encoder.encoderBrand == Orbis_Encoder){
        joint->encoder.raw &= 16383;
        joint->encoder.angleRads = joint->encoder.raw * ((2 * M_PI) / 16383);
        joint->encoder.angleDegrees = joint->encoder.raw * (360 / 16383);
    }
}
