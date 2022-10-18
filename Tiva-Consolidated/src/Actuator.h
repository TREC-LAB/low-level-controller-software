/*
 * Actuator.h
 * @author: Nick Tremaroli
 * Contains the layout and functions regarding
 * an actuator
 */

#ifndef ACTUATOR_H_
#define ACTUATOR_H_

#include "HAL/PWM_Driver.h"
#include "ForceSensor.h"
#include "QEIEncoder.h"

/**
 * Actuator
 * Contains all of the data and structures needed by
 * an actuator on the Tiva
 */
struct Actuator
{
    QEIEncoder motorEncoder;
    float dutyCycle;
    uint8_t direction;
    uint8_t actuatorNumber;     // 0 or 1 for where the actuator plugs into the sensor board
    ForceSensor forceSensor;
};
typedef struct Actuator Actuator;

// constructs an actuator
Actuator actuatorConstruct(uint8_t actuatorNumber, uint32_t QEIBase, uint16_t QEISampleRate,
                           int32_t QEICountsPerRotation, uint32_t ADCBase,
                           float forceSensorSlope, float forceSensorOffset);

// send a PWM Signal to the actuator
void SendPWMSignal(Actuator* actuator);

// update and read the position from the actuator
void updateActuatorPosition(Actuator* actuator);

#endif /* ACTUATOR_H_ */
