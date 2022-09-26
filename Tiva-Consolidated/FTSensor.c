/**
 * FTSensor.c
 * @author: Nick Tremaroli
 * Contains all of the low-level code for FT sensor related functions
 */

#include "FTSensor.h"

FTSensor ftSensorContruct(void)
{
    FTSensor ftSensor;

    ftSensor.forceX = 0.0;
    ftSensor.forceY = 0.0;
    ftSensor.forceZ = 0.0;
    ftSensor.torqueX = 0.0;
    ftSensor.torqueY = 0.0;
    ftSensor.torqueZ = 0.0;

    ftSensor.bias.forceXBias = 0.0;
    ftSensor.bias.forceYBias = 0.0;
    ftSensor.bias.forceZBias = 0.0;
    ftSensor.bias.torqueXBias = 0.0;
    ftSensor.bias.torqueYBias = 0.0;
    ftSensor.bias.torqueZBias = 0.0;

    return ftSensor;
}

void ftSensorCalibrate(FTSensor* ftSensor)
{
    ReadCAN(&ftSensor->RxData0, &ftSensor->RxData1);
    readForceTorqueData(ftSensor);

    ftSensor->bias.forceXBias = -1.0 * ftSensor->forceX;
    ftSensor->bias.forceYBias = -1.0 * ftSensor->forceY;
    ftSensor->bias.forceZBias = -1.0 * ftSensor->forceZ;

    ftSensor->bias.torqueXBias = -1.0 * ftSensor->torqueX;
    ftSensor->bias.torqueYBias = -1.0 * ftSensor->torqueY;
    ftSensor->bias.torqueZBias = -1.0 * ftSensor->torqueZ;
}

void readForceTorqueData(FTSensor* ftSensor)
{
    float torqueFactor, forceFactor;
    torqueFactor = 1000000.0 / 611.0; // This factor is hardware configuration in the Net FT board, connect the board through ethernet to check the configuration, make this as a fucntion such that it can be configured for left and right sensor
    forceFactor = 1000000.0 / 35402.0;
//    torqueFactor = 100000 / 611;
//    forceFactor = 100000 / 35402;
    ftSensor->forceX = (binTwoCToDec(ftSensor->RxData0.msgRxData[0], ftSensor->RxData0.msgRxData[1])/forceFactor) + ftSensor->bias.forceXBias;
    ftSensor->torqueX = (binTwoCToDec(ftSensor->RxData0.msgRxData[2], ftSensor->RxData0.msgRxData[3])/torqueFactor) + ftSensor->bias.torqueXBias;
    ftSensor->forceY = (binTwoCToDec(ftSensor->RxData0.msgRxData[4], ftSensor->RxData0.msgRxData[5])/forceFactor) + ftSensor->bias.forceYBias;
    ftSensor->torqueY = (binTwoCToDec(ftSensor->RxData0.msgRxData[6], ftSensor->RxData0.msgRxData[7])/torqueFactor) + ftSensor->bias.torqueYBias;
    ftSensor->forceZ = (binTwoCToDec(ftSensor->RxData1.msgRxData[0], ftSensor->RxData1.msgRxData[1])/forceFactor) + ftSensor->bias.forceZBias;
    ftSensor->torqueZ = (binTwoCToDec(ftSensor->RxData1.msgRxData[2], ftSensor->RxData1.msgRxData[3])/torqueFactor) + ftSensor->bias.torqueZBias;
}

