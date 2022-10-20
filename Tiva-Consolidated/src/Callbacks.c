/*
 * Callbacks.c
 *
 *  Created on: Oct 5, 2022
 *      Author: Max Stelmack
 */
#include "Callbacks.h"

PandoraLowLevel pandora;

volatile bool runTimer1 = true;
volatile bool runTimer3 = true;

bool EngageVirtualEStop(PandoraLowLevel* pandora);

void lowLevelStartup(void)
{
    // Populate pandora object
    pandora = pandoraConstruct();

    // Initialize tiva

    tivaInitEtherCAT(&pandora);

    while(!pandora.initialized)
    {
        GetAndSendDataToMaster(&pandora.easyCAT);
        pandora.prevProcessIdFromMaster = pandora.processIdFromMaster;
        pandora.processIdFromMaster = pandora.easyCAT.etherCATInputFrames.rawBytes[PROCESS_ID_INDEX];

        if(pandora.processIdFromMaster != pandora.prevProcessIdFromMaster)
        {
            storeDataFromMaster(&pandora);
            processDataFromMaster(&pandora);
            loadDataForMaster(&pandora);
        }
    }

    tivaInit(&pandora);

    // Enable processor interrupts
    IntMasterEnable();
}

/**
 * For testing without Master
 */
void lowLevelStartup_noMaster(void)
{
    // Populate pandora object
    pandora = pandoraConstruct();

    uint8_t actuatorNumber = 0;

    uint8_t rawQEIBase = 0;
    uint32_t actuator0_QEIBase = QEI0_BASE + ((1 << 12) * rawQEIBase);

    uint16_t actuator0_QEISampleRate = 1000;
    uint32_t actuator0_QEICountsPerRotation = 1000;

    uint8_t rawADCBase = 0;
    uint32_t actuator0_ADCBase = ADC0_BASE + ((1 << 12) * rawADCBase);

    float actuator0_ForceSensorSlope = -1993.222361;
    float actuator0_ForceSensorOffset = 0.973456123;
    pandora.actuator0 = actuatorConstruct(actuatorNumber, actuator0_QEIBase,
                                          actuator0_QEISampleRate, actuator0_QEICountsPerRotation,
                                          actuator0_ADCBase, actuator0_ForceSensorSlope,
                                          actuator0_ForceSensorOffset);

    tivaInit(&pandora);
}

void checkEstop(void)
{
    if (runTimer1)
    {
        if (EngageVirtualEStop(&pandora))
        {
            // Stop timer1
            runTimer3 = false;
            pandora.signalToMaster = HALT_SIGNAL_TM;

            // Stop motor
            pandora.actuator0.pwmGenerator.dutyCycle = 0;
            pandora.actuator1.pwmGenerator.dutyCycle = 0;
            SendPWMSignal(&pandora.actuator0);
            SendPWMSignal(&pandora.actuator1);

            // Send shutdown signal to master
            haltLEDS();
            GetAndSendDataToMaster(&pandora.easyCAT);
        }
        else
        {
            runTimer3 = true;
            pandora.signalToMaster = NORMAL_OPERATION;
        }
    }
    else
    {
        // Stop motors if not running estop interrupt
        pandora.actuator0.pwmGenerator.dutyCycle = 0;
        pandora.actuator1.pwmGenerator.dutyCycle = 0;
        SendPWMSignal(&pandora.actuator0);
        SendPWMSignal(&pandora.actuator1);
    }
}

/*
 * EngageVirtualEStop
 * Checks conditions to decide whether to engage Virtual EStop
 * based on Force, Abs Encoder Angle Ranges.
 *
 * @param pandora: a pointer to the pandora struct which contains all of the
 * values to check
 * @return: if the virtual estop should be triggered
 */
bool EngageVirtualEStop(PandoraLowLevel* pandora)
{
    return (((pandora->joint0.actualRaw < -1 * pandora->joint0.rawBackwardRangeOfMotion ||
            pandora->joint0.actualRaw > pandora->joint0.rawForwardRangeOfMotion)) ||
            (pandora->joint1.actualRaw < -1 * pandora->joint1.rawBackwardRangeOfMotion ||
                    pandora->joint1.actualRaw > pandora->joint1.rawForwardRangeOfMotion) ||
                    pandora->actuator0.forceSensor.newtons > pandora->actuator0.forceSensor.upperLimitNewtons ||
                    pandora->actuator0.forceSensor.newtons < pandora->actuator0.forceSensor.lowerLimitNewtons ||
                    pandora->actuator1.forceSensor.newtons > pandora->actuator1.forceSensor.upperLimitNewtons ||
                    pandora->actuator1.forceSensor.newtons < pandora->actuator1.forceSensor.lowerLimitNewtons) &&
            pandora->signalFromMaster == CONTROL_SIGNAL && pandora->settings.softwareEStopEnable;
}

void readSensors(void)
{
    GetAndSendDataToMaster(&pandora.easyCAT);
    pandora.signalFromMaster = pandora.easyCAT.etherCATInputFrames.rawBytes[SIGNAL_INDEX];
    if (pandora.signalFromMaster == CONTROL_SIGNAL && pandora.initialized)
    {
        readForceSensors();
        readJointEncoders();
        readMotorPositions();
        readMotorVelocities();
        if(pandora.imu.enabled)
            readSensorData(&pandora.imu);
    }

    // Send TivaToMaster and receive MasterToTiva
    if (runTimer3 || pandora.signalFromMaster != CONTROL_SIGNAL)
    {

        pandora.prevProcessIdFromMaster = pandora.processIdFromMaster;
        pandora.processIdFromMaster = pandora.easyCAT.etherCATInputFrames.rawBytes[PROCESS_ID_INDEX];

        if(pandora.processIdFromMaster != pandora.prevProcessIdFromMaster)
        {
            // Read desired Tiva status, duty cycles, and directions from MasterToTiva
            storeDataFromMaster(&pandora);

            // Act according Tiva status
            // Turns off estop timer if necessary
            runTimer1 = processDataFromMaster(&pandora);
            // Populate TivaToMaster data frame
            loadDataForMaster(&pandora);
        }
        else
        {
            runTimer1 = processDataFromMaster(&pandora);
            loadDataForMaster(&pandora);
        }
    }
}

void readForceSensors(void){
    updateForces(&pandora.actuator0.forceSensor);
    //updateForces(&pandora.actuator1.forceSensor);
}

void readJointEncoders(void){
    updateJointAngles(&pandora.joint0);
    updateJointAngles(&pandora.joint1);
}

void readMotorPositions(void){
    readQEIEncoderPosition(&pandora.actuator0.motorEncoder);
    readQEIEncoderPosition(&pandora.actuator1.motorEncoder);
}

void readMotorVelocities(void){
    readQEIEncoderVelocity(&pandora.actuator0.motorEncoder);
    readQEIEncoderVelocity(&pandora.actuator1.motorEncoder);
}
