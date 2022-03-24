/*
 * Joint.c
 * @author: Nick Tremaroli
 * Contains all of the low-level code for joint related functions
 */
#include "Joint.h"

/**
 * jointConstruct
 * Constructs a joint given the corresponding input parameters
 *
 * @param SSIBase: The SSI Base for the encoder at the joint
 * @param SSIEncoderBrand: The SSI Encoder Brand of the encoder
 * @param sampleRate: The sample rate of the encoder
 * @param: jointReverseFactor: The jointReverseFactor of the joint
 * @param: rawZero: The raw zero of the joint
 * @param: rawForwardRangeOfMotion: The raw forward range of motion of the joint
 * @param: rawBackwardRangeOfMotion: The raw backward range of motion of the joint
 *
 * @return: an initialized joint structure
 */
Joint jointConstruct(uint32_t SSIBase, SSIEncoderBrand encoderBrand, uint16_t sampleRate,
                     int8_t jointReverseFactor, uint16_t rawZero,
                     uint16_t rawForwardRangeOfMotion, uint16_t rawBackwardRangeOfMotion)
{
    Joint joint;

    joint.encoder = ssiEncoderConstruct(SSIBase, sampleRate, encoderBrand);

    joint.actualRaw = 0;
    joint.jointReverseFactor = jointReverseFactor;
    joint.rawZero = rawZero;
    joint.rawForwardRangeOfMotion = rawForwardRangeOfMotion;
    joint.rawBackwardRangeOfMotion = rawBackwardRangeOfMotion;

    return joint;
}

/**
 * updateJointAngles
 * updates the joint angle of the joint
 *
 * @param joint: a pointer to the joint which will have
 * its angle updated
 */
void updateJointAngles(Joint* joint)
{
    // read the raw encoder value from the joint
    readAbsEncoder(&joint->encoder);


    if (joint->encoder.encoderBrand == Orbis_Encoder){
        joint->encoder.raw %= 8191;
        if(joint->encoder.raw > 65535)
            joint->encoder.raw -= 65535;
    }

    // now get the actual angle of the joint
    getRawActualValue(joint);

    // convert to radians and degrees accordingly
    if (joint->encoder.encoderBrand == Gurley_Encoder){
        joint->angleRads = joint->actualRaw * M_PI / 65535;
    }
    else if (joint->encoder.encoderBrand == Orbis_Encoder)
    {
        joint->angleRads = joint->actualRaw * ((2 * M_PI) / 16383);
        joint->angleDegrees = joint->actualRaw * (360 / 16383);
    }
}

/**
 * getRawActualValue
 * gets the actual angle value of the joint
 *
 * @param joint: a pointer to the joint structure which will
 * have its actual angle calculated
 */
void getRawActualValue(Joint* joint)
{
    int32_t rawPI = 0;
    if (joint->encoder.encoderBrand == Gurley_Encoder)
        rawPI = (int32_t) 65535;
    else if (joint->encoder.encoderBrand == Orbis_Encoder)
        rawPI = (int32_t) 8191;

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
}
