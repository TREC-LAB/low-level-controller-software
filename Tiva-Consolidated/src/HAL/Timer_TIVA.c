#include "Timer_TIVA.h"

/**
 * Timer 1A is for sending command to motor controller periodically
 */
void timer1A_Config()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
    //The timer configured below (Timer 1A) is for sending command to mc periodically.
    TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);                    //Set as a periodic timer in so that the timer we reset to the max value after running down to 0.

    //either one is fine since the main clock is running at 16 Mhz
    TimerClockSourceSet(TIMER1_BASE, TIMER_CLOCK_SYSTEM);             //Setting the system clock source for the Timer.
    //TimerClockSourceSet(TIMER1_BASE, TIMER_CLOCK_PIOSC);               //Setting the PIO clock source for the Timer.
    IntEnable(INT_TIMER1A);

    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);    //Clear roll-over interrupt due to counting down to 0.
    TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);   //Enable a Timer interrupt to occur when the timer runs down to 0.
}

/**
 * Timer 2A is for logging data
 */
void timer2A_Config()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
    TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC);
    TimerClockSourceSet(TIMER2_BASE, TIMER_CLOCK_SYSTEM);
    IntEnable(INT_TIMER2A);
    TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);    //Clear roll-over interrupt due to counting down to 0.
    TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);   //Enable a Timer interrupt to occur when the timer runs down to 0.
}

/**
 * Timer 3A is for sampling periodically
 */
void timer3A_Config()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);
    TimerConfigure(TIMER3_BASE, TIMER_CFG_PERIODIC);
    TimerClockSourceSet(TIMER3_BASE, TIMER_CLOCK_SYSTEM);
    IntEnable(INT_TIMER3A);
    TimerIntClear(TIMER3_BASE, TIMER_TIMA_TIMEOUT);    //Clear roll-over interrupt due to counting down to 0.
    TimerIntEnable(TIMER3_BASE, TIMER_TIMA_TIMEOUT);   //Enable a Timer interrupt to occur when the timer runs down to 0.
}

/**
 * Start estop timer
 */
void startTimer1(uint16_t rate)
{
    TimerEnable(TIMER1_BASE, TIMER_A); // E-stop
    TimerLoadSet(TIMER1_BASE, TIMER_A, SysCtlClockGet() / rate);
}

/**
 * Start logging timer
 */
void startTimer2(uint16_t rate)
{
    TimerEnable(TIMER2_BASE, TIMER_A); // Logging
    TimerLoadSet(TIMER2_BASE, TIMER_A, SysCtlClockGet() / rate);
}

/**
 * Start EtherCAT timer
 */
void startTimer3(uint16_t rate)
{
    TimerEnable(TIMER3_BASE, TIMER_A); // Communication
    TimerLoadSet(TIMER3_BASE, TIMER_A, SysCtlClockGet() / rate);
}
