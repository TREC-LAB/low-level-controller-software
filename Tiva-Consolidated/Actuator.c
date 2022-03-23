/*
 * Actuator.c
 *
 *  Created on: Feb 7, 2022
 *      Author: n
 */
#include "Actuator.h"

Actuator actuatorConstruct(uint8_t actuatorNumber, uint32_t QEIBase, uint16_t QEISampleRate,
                           int32_t QEICountsPerRotation, uint32_t ADCBase,
                           float ForceSensorSlope, float ForceSensorOffset)
{
    Actuator actuator;

    actuator.actuatorNumber = actuatorNumber;

    actuator.motorEncoder = qeiEncoderConstruct(QEIBase, QEISampleRate, QEICountsPerRotation);

    actuator.dutyCycle = 0;

    actuator.direction = 0;

    actuator.forceSensor = forceSensorConstruct(ADCBase, ForceSensorSlope, ForceSensorOffset);

    return actuator;
}

void SendPWMSignal(Actuator* actuator)
{
    setPulseWidth(actuator->actuatorNumber,20000,actuator->dutyCycle,SysCtlClockGet(),actuator->direction);
//    printf("duty cycle for actuator %d: %f\n", actuator->actuatorNumber, actuator->dutyCycle);
}

void updateActuatorPosition(Actuator* actuator)
{
    readQEIEncoderPosition(&actuator->motorEncoder);
}
