/**
 * main.c
 * @author: Nick Tremaroli
 * Contains the main function which is used for initialization and all of the interrupts
 * used for processing. After the main functions is done with initialization, it enables the
 * interrupts which run routinely based on a timer. These interrupts are responsible for communicating
 * with the master computer and checking the virtual Estop conditions.
 */

#define NO_RTOS 1


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
#include "Callbacks.h"

// the main pandora structure
PandoraLowLevel pandora;

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

    SysCtlDelay(2000);

    lowLevelStartup();

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
    checkEstop();
    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
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
    readSensors();
    TimerIntClear(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
}
