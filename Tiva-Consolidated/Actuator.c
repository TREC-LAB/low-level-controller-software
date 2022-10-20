/*
 * Actuator.c
 * @author: Nick Tremaroli
 * Contains the low-level code for actuator related functions
 */
#include "Actuator.h"

/**
 * actuatorConstruct
 * Constructs an actuator given the corresponding input parameters
 *
 * @param actuatorNumber: The number corresponding to the actuator on the Tiva,
 * actuator 0 is on the 0th side of the sensor board,
 * actuator 1 is on the 1th side of the sensor board
 * @param QEIBase: The QEI Base of the encoder on the actuator
 * @param QEISampleRate: The sample rate of the QEI encoder
 * @param QEICountsPerRotation: The counts per rotation of the QEU Encoder
 * @param ADCBase: The ADC Base of the force sensor on the actuator
 * @param forceSensorSlope: The slope value of the force sensor
 * @param forceSensorOffset: The offset value of the force sensor
 *
 * @return: an initialized actuator structure
 */
Actuator actuatorConstruct(uint8_t actuatorNumber, uint32_t QEIBase, uint16_t QEISampleRate,
                           int32_t QEICountsPerRotation, uint32_t ADCBase,
                           float forceSensorSlope, float forceSensorOffset)
{
    Actuator actuator;

    actuator.actuatorNumber = actuatorNumber;

    actuator.motorEncoder = qeiEncoderConstruct(QEIBase, QEISampleRate, QEICountsPerRotation);

    actuator.pwmGenerator = PWMGeneratorConstruct(20000);

    actuator.forceSensor = forceSensorConstruct(ADCBase, forceSensorSlope, forceSensorOffset);

    return actuator;
}

/**
 * SendPWMSignal
 * Sends the PWM Signal to the actuator
 *
 * @param actuator: a pointer to the actuator which a PWM signal
 * will be applied too
 */
void SendPWMSignal(Actuator* actuator)
{
    setPulseWidth(actuator->actuatorNumber, &actuator->pwmGenerator, SysCtlClockGet());
}

/**
 * updateActuatorPosition
 * Updates the position of the actuator
 *
 * @param actuator: a pointer to the actuator which the position
 * will be updated
 */
void updateActuatorPosition(Actuator* actuator)
{
    readQEIEncoderPosition(&actuator->motorEncoder);
}
