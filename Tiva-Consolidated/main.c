#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <time.h>
#include <string.h>
#include <stdlib.h>

// TivaWare
#include "inc/hw_ints.h"
#include "inc/hw_ssi.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/timer.h"
#include "driverlib/qei.h"
#include "driverlib/ssi.h"
#include "driverlib/adc.h"
#include "driverlib/pwm.h"
#include "utils/uartstdio.h"

// HAL
#include "HAL/LAN9252_TI_TIVA.h"
#include "HAL/Timer_TIVA.h"
#include "HAL/UART_TIVA.h"
#include "HAL/CAN_TI_TIVA.h"

// Implementation
#include "EtherCAT_FrameData.h"
#include "PandoraLowLevel.h"

PandoraLowLevel pandora;

volatile bool runTimer1 = true;
volatile bool runTimer3 = true;

uint16_t sample_rate = 1000; // Hz
uint16_t estop_rate = 1000;  // Hz


bool EngageVirtualEStop(PandoraLowLevel* pandora);


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
        GetAndSendDataToMaster(&pandora);
        pandora.prevProcessIdFromMaster = pandora.processIdFromMaster;
        pandora.processIdFromMaster = pandora.etherCATInputFrames.rawBytes[PROCESS_ID_INDEX];

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

    startTimer1(estop_rate); // Start vstop timer
    startTimer3(sample_rate); // Start motor timer

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
    if (runTimer1)
    {
        if (EngageVirtualEStop(&pandora))
        {
            // Stop timer1
            runTimer3 = false;
            pandora.signalToMaster = HALT_SIGNAL_TM;

            // Stop motor
            pandora.actuator0.dutyCycle = 0;
            pandora.actuator1.dutyCycle = 0;
            SendPWMSignal(&pandora.actuator0);
            SendPWMSignal(&pandora.actuator1);

            // Send shutdown signal to master
            haltLEDS();
            GetAndSendDataToMaster(&pandora);
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
        pandora.actuator0.dutyCycle = 0;
        pandora.actuator1.dutyCycle = 0;
        SendPWMSignal(&pandora.actuator0);
        SendPWMSignal(&pandora.actuator1);
    }
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
    GetAndSendDataToMaster(&pandora);
    pandora.signalFromMaster = pandora.etherCATInputFrames.rawBytes[SIGNAL_INDEX];
    if (pandora.signalFromMaster == CONTROL_SIGNAL && pandora.initialized)
    {
        SendFTSensorData(&pandora.ftSensor);
        updateForces(&pandora.actuator0.forceSensor);
        updateForces(&pandora.actuator1.forceSensor);
        updateJointAngles(&pandora.joint0);
        updateJointAngles(&pandora.joint1);
        readQEIEncoderPosition(&pandora.actuator0.motorEncoder);
        readQEIEncoderPosition(&pandora.actuator1.motorEncoder);
        readQEIEncoderVelocity(&pandora.actuator0.motorEncoder);
        readQEIEncoderVelocity(&pandora.actuator1.motorEncoder);

        if(pandora.imu.enabled)
            readSensorData(&pandora.imu);

        readForceTorqueData(&pandora.ftSensor);
    }

    // Send TivaToMaster and receive MasterToTiva
    if (runTimer3 || pandora.signalFromMaster != CONTROL_SIGNAL)
    {

        pandora.prevProcessIdFromMaster = pandora.processIdFromMaster;
        pandora.processIdFromMaster = pandora.etherCATInputFrames.rawBytes[PROCESS_ID_INDEX];

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
    TimerIntClear(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
}

