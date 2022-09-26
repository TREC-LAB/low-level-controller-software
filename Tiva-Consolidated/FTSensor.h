/**
 * FTSensor.h
 * @author: Nick Tremaroli
 * Contains the layout and functions regarding an FT Sensor
 */

#ifndef FTSENOR_H
#define FTSENSOR_H

#include "HAL/CAN_TI_TIVA.h"

/**
 * FTSensorBias
 * The values stored after calibrating the FT sensor to
 * account for bias values
 */
struct FTSensorBias
{
    float forceXBias;
    float forceYBias;
    float forceZBias;

    float torqueXBias;
    float torqueYBias;
    float torqueZBias;

};
typedef struct FTSensorBias FTSensorBias;

/**
 * FTSensor
 * Contains all of the data needed by an FT Sensor on the Tiva
 */
struct FTSensor
{
    float forceX;
    float forceY;
    float forceZ;

    float torqueX;
    float torqueY;
    float torqueZ;

    FTSensorBias bias;

    CANBUSRx RxData0;
    CANBUSRx RxData1;
};
typedef struct FTSensor FTSensor;

FTSensor ftSensorConstruct(void);

void ftSensorCalibrate(FTSensor* ftSensor);
void readForceTorqueData(FTSensor* ftSensor);

#endif
