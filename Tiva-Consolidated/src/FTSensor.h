/**
 * FTSensor.h
 * @author: Nick Tremaroli
 * Contains the layout and functions regarding an FT Sensor
 */

#ifndef FTSENSOR_H
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

    CANBUSTx TxData0;
};
typedef struct FTSensor FTSensor;

// constructs an FT sensor
FTSensor ftSensorConstruct(void);

// enalbes the FT sensor
void ftSensorEnable(FTSensor* ftSensor);

// sends a request to the FT sensor to get a responce
void SendFTSensorData(FTSensor* ftSensor);

// calibrate the FT sensor
void ftSensorCalibrate(FTSensor* ftSensor);

// read force and torque data from the FT sensor
void readForceTorqueData(FTSensor* ftSensor);

#endif
