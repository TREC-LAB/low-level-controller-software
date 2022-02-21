/*
 * Actuator.h
 *
 *  Created on: Feb 7, 2022
 *      Author: n
 */

#ifndef ACTUATOR_H_
#define ACTUATOR_H_

#include "HAL/PWM_Driver.h"
#include "ForceSensor.h"
#include "QEIEncoder.h"

struct Actuator
{
    QEIEncoder motorEncoder;
    float dutyCycle;
    uint8_t direction;
    ForceSensor forceSensor;
};
typedef struct Actuator Actuator;

Actuator actuatorConstruct(uint32_t QEIBase, uint16_t QEISampleRate, int32_t QEICountsPerRotation,
                           uint32_t ADCBase, float ForceSensorSlope, float ForceSensorOffset);

void SendPWMSignal(Actuator* actuator);
void updateActuatorPosition(Actuator* actuator);

#endif /* ACTUATOR_H_ */
