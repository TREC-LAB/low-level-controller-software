/**
 * main.c
 * @author: Nick Tremaroli
 * Contains the main function which is used for initialization and all of the interrupts
 * used for processing. After the main functions is done with initialization, it enables the
 * interrupts which run routinely based on a timer. These interrupts are responsible for communicating
 * with the master computer and checking the virtual Estop conditions.
 */

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>

// HAL
#include "HAL/LAN9252_TI_TIVA.h"
#include "HAL/Timer_TIVA.h"
#include "HAL/UART_TIVA.h"
#include "HAL/CAN_TI_TIVA.h"

// Implementation
#include "EtherCAT_FrameData.h"
#include "PandoraLowLevel.h"

// the main pandora structure
PandoraLowLevel pandora;

// the variables associated with running
// certain parts of the timer
volatile bool runTimer1 = true;
volatile bool runTimer3 = true;

// the rates at which to run the interrupts
uint16_t sample_rate = 1000; // Hz
uint16_t estop_rate = 1000;  // Hz

// puts the Tiva in a halt state if the Estop is reached
bool EngageVirtualEStop(PandoraLowLevel* pandora);

/**
 * main
 * This is where the program starts. Main initializes the Tiva and
 * then enables the interrupts to tak over the processing after initialization
 */
int main(void)
{
    //Set the system clock to 80Mhz
    SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);

    // Populate pandora object
    pandora = pandoraConstruct();

    // Initialize EtherCAT on the Tiva so the Tiva can start receiving data from the master
    tivaInitEtherCAT(&pandora);

    // stores the master's initialization data
    while(!pandora.initialized)
    {
        // get and send new data to the master
        GetAndSendDataToMaster(&pandora.easyCAT);

        // update the process ID variables
        pandora.prevProcessIdFromMaster = pandora.processIdFromMaster;
        pandora.processIdFromMaster = pandora.easyCAT.etherCATInputFrames.rawBytes[PROCESS_ID_INDEX];

        // only process this data if the master process ID has updated
        if(pandora.processIdFromMaster != pandora.prevProcessIdFromMaster)
        {
            storeDataFromMaster(&pandora);
            processDataFromMaster(&pandora);
            loadDataForMaster(&pandora);
        }
    }

    // initialize pandora
    tivaInit(&pandora);

    // Enable processor interrupts
    IntMasterEnable();

    // Start virtual estop timer
    startTimer1(estop_rate);

    // start controller timer
    startTimer3(sample_rate);

    // loop forever, let the interrupts handle the tasks
    while(1);
}


/*
 * UART0 interrupt to handle communication between Tiva MCU and the computer
 */
void UART0IntHandler(void) {}

/*
 * UART1 interrupt handler triggers if Tiva receives information from motor controller
 */
void UART1IntHandler(void) {}

/**
 * Timer1A interrupt handler
 * Checks for software estop conditions.
 * If any are met, turns off motors and sends shutdown signal to master.
 */
void Timer1AIntHandler(void)
{
    // if the estop should be checked
    if (runTimer1)
    {
        // check the estop
        if (EngageVirtualEStop(&pandora))
        {
            // Stop timer3
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
        // if the estop is not hit
        else
        {
            runTimer3 = true;
            pandora.signalToMaster = NORMAL_OPERATION;
        }
    }
    // if the estop is not needed to be checked
    else
    {
        // stop motors because the estop is not needed
        pandora.actuator0.pwmGenerator.dutyCycle = 0;
        pandora.actuator1.pwmGenerator.dutyCycle = 0;
        SendPWMSignal(&pandora.actuator0);
        SendPWMSignal(&pandora.actuator1);
    }

    // clear the interrupt
    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
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

/*
 * Timer2A interrupt handler
 * Used for data logging to serial port
 */
void Timer2AIntHandler(void) {}

/*
 * Timer3A interrupt handler
 * Sends TivaToMaster frame to master
 * Receives MasterToTiva frame from master
 */
void Timer3AIntHandler(void)
{
    // get and send data to the master
    GetAndSendDataToMaster(&pandora.easyCAT);

    // get the signal from master
    pandora.signalFromMaster = pandora.easyCAT.etherCATInputFrames.rawBytes[SIGNAL_INDEX];

    // update sensor values if the signal is a control signal and pandora is initialized
    if (pandora.signalFromMaster == CONTROL_SIGNAL && pandora.initialized)
    {
        // Send a request to read FT sensor data (will be read from last)
        SendFTSensorData(&pandora.ftSensor);

        // get the updated values from the force sensors
        updateForces(&pandora.actuator0.forceSensor);
        updateForces(&pandora.actuator1.forceSensor);

        // get the updated values for the joint angles
        updateJointAngles(&pandora.joint0);
        updateJointAngles(&pandora.joint1);

        // get the updated positions of the QEI encoders
        readQEIEncoderPosition(&pandora.actuator0.motorEncoder);
        readQEIEncoderPosition(&pandora.actuator1.motorEncoder);
        readQEIEncoderVelocity(&pandora.actuator0.motorEncoder);
        readQEIEncoderVelocity(&pandora.actuator1.motorEncoder);

        // If the IMU is enabled, read from it
        if(pandora.imu.enabled)
            readSensorData(&pandora.imu);

        // read the FT sensor data
        readForceTorqueData(&pandora.ftSensor);
    }

    // the data from the master computer should be processed
    if (runTimer3 || pandora.signalFromMaster != CONTROL_SIGNAL)
    {
        // store the current and previous master process IDs
        pandora.prevProcessIdFromMaster = pandora.processIdFromMaster;
        pandora.processIdFromMaster = pandora.easyCAT.etherCATInputFrames.rawBytes[PROCESS_ID_INDEX];

        // if the master process ID has updated
        if(pandora.processIdFromMaster != pandora.prevProcessIdFromMaster)
        {
            // store the data from the master computer
            storeDataFromMaster(&pandora);

            // process the data from the master computer and
            // turn off estop timer if necessary
            runTimer1 = processDataFromMaster(&pandora);

            // load the data to send to the master computer
            loadDataForMaster(&pandora);
        }
        // if the master process ID has not been updated
        else
        {
            // process the old data
            runTimer1 = processDataFromMaster(&pandora);

            // load the data to send to the master computer
            loadDataForMaster(&pandora);
        }
    }

    // clear the interrupt
    TimerIntClear(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
}

