/**
 * ForceSensor.h
 * @author: Nick Tremaroli
 * Contains the layout and functions regarding a force sensor
 */
#ifndef FORCE_SENSOR_H
#define FORCE_SENSOR_H

#include "HAL/ADC_TIVA.h"

/**
 * ForceSensor
 * Contains all of the data needed by a force sensor
 * on the Tiva
 */
struct ForceSensor
{
    uint32_t ADCBase;
    bool enabled;

    float upperLimitNewtons;
    float lowerLimitNewtons;

    float slope;
    float offset;

    uint32_t raw;
    float newtons;
};
typedef struct ForceSensor ForceSensor;

// Constructs a force sensor
ForceSensor forceSensorConstruct(uint32_t ADCBase, float slope, float offset);

// enables a force sensor
void enableForceSensor(ForceSensor* forceSensor);
// TODO: Add a force sensor disable function

// reads the ram value from the force sensor
void readLoadCell(ForceSensor* forceSensor);

// converts the raw force sensor value reading to the actual value
void updateForces(ForceSensor* forceSensor);

#endif /* FORCE_SENSOR_H */
