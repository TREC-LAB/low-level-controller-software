/*
 * Joint.c
 *
 *  Created on: Feb 7, 2022
 *      Author: n
 */
#include "Joint.h"

Joint jointConstruct(uint32_t SSIBase, SSIEncoderBrand encoderBrand, uint16_t sample_rate,
                     int8_t jointReverseFactor, uint16_t rawZero,
                     uint16_t rawForwardRangeOfMotion, uint16_t rawBackwardRangeOfMotion)
{
    Joint joint;

    joint.encoder = encoderConstruct(SSIBase, sample_rate, encoderBrand);

    joint.actualRaw = 0;
    joint.jointReverseFactor = jointReverseFactor;
    joint.rawZero = rawZero;
    joint.rawForwardRangeOfMotion = rawForwardRangeOfMotion;
    joint.rawBackwardRangeOfMotion = rawBackwardRangeOfMotion;

    return joint;
}

void updateJointAngles(Joint* joint)
{
    readAbsEncoder(&joint->encoder);

    if (joint->encoder.encoderBrand == Orbis_Encoder){
        joint->encoder.raw &= 16383;
        if(joint->encoder.raw > 65535)
            joint->encoder.raw -= 65535;
    }
    getRawActualValue(joint);
    if (joint->encoder.encoderBrand == Gurley_Encoder){
        joint->angleRads = joint->actualRaw * M_PI / 65535;
    }
    else if (joint->encoder.encoderBrand == Orbis_Encoder)
    {
        joint->angleRads = joint->actualRaw * ((2 * M_PI) / 16383);
        joint->angleDegrees = joint->actualRaw * (360 / 16383);
    }
    // Convert raw encoder sensor reading to radians

}

// TODO: make more efficient
void getRawActualValue(Joint* joint)
{
    int32_t rawPI = (int32_t) 65535;
//    if(joint->encoder.raw >= rawPI)
//        rawAngleInRange = (int32_t)(joint->encoder.raw - rawPI);
//    else
//        rawAngleInRange = (int32_t)joint->encoder.raw;
//
//    if(joint->rawZero >= rawPI)
//        rawZeroInRange = joint->rawZero - rawPI;
//    else
//        rawZeroInRange = joint->rawZero;

    int32_t rawAngleReversed = joint->encoder.raw * joint->jointReverseFactor;
    int32_t rawZeroReversed = joint->rawZero * joint->jointReverseFactor;

//    printf("encoder raw: %d\n", joint->encoder.raw);
    int32_t rawAngleAltered = 0;
    int32_t rawZeroAltered = 0;
    if(rawAngleReversed < 0)
        rawAngleAltered = rawAngleReversed + rawPI;
    else
        rawAngleAltered = rawAngleReversed;

    if(rawZeroReversed < 0)
        rawZeroAltered = rawZeroReversed + rawPI;
    else
        rawZeroAltered = rawZeroReversed;

    // if there is a jump below zero
    if(rawZeroAltered - joint->rawBackwardRangeOfMotion < 0 &&
       rawAngleAltered > rawZeroAltered + joint->rawForwardRangeOfMotion)
    {
        rawAngleAltered -= rawPI;
    }
    // if there is a jump above zero
    if(rawZeroAltered + joint->rawForwardRangeOfMotion > rawPI &&
       rawAngleAltered < rawZeroAltered - joint->rawBackwardRangeOfMotion)
    {
        rawAngleAltered += rawPI;
    }
    joint->actualRaw = rawAngleAltered - rawZeroAltered;
//    printf("rawAngleAltered: %d\n", rawAngleAltered);
//    printf("rawZeroAltered: %d\n", rawZeroAltered);
//    printf("joint actual Raw: %d\n", joint->actualRaw);
}
