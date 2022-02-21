/*
 * Actuator.c
 *
 *  Created on: Feb 7, 2022
 *      Author: n
 */
#include "Actuator.h"

Actuator actuatorConstruct(uint32_t QEIBase, uint16_t QEISampleRate, int32_t QEICountsPerRotation,
                           uint32_t ADCBase, float ForceSensorSlope, float ForceSensorOffset)
{
    Actuator actuator;

    actuator.motorEncoder = qeiEncoderConstruct(QEIBase, QEISampleRate, QEICountsPerRotation);

    actuator.dutyCycle = 0;

    actuator.direction = 0;

    actuator.forceSensor = forceSensorConstruct(ADCBase, ForceSensorSlope, ForceSensorOffset);

    return actuator;
}

void SendPWMSignal(Actuator* actuator)
{
    setPulseWidth(0,20000,actuator->dutyCycle,SysCtlClockGet(),actuator->direction);
}

void updateActuatorPosition(Actuator* actuator)
{
    readQEIEncoderPosition(&actuator->motorEncoder);
}
