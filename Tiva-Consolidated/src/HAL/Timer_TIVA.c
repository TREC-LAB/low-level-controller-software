/**
 * Timer_TIVA.c
 * Contains all of the low-level code for
 * timer related functions of the Tiva
 */

#include "Timer_TIVA.h"

/**
 * timer1A_Config
 * Configures timer 1A on the Tiva
 */
void timer1A_Config()
{
    // Enable the peripheral timer 1
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);

    // Set as a periodic timer in so that the timer we reset to the max value after running down to 0.
    TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);

    // Setting the system clock source for the Timer.
    TimerClockSourceSet(TIMER1_BASE, TIMER_CLOCK_SYSTEM);

    // Enable a timer 1A interrupt
    IntEnable(INT_TIMER1A);

    // Clear roll-over interrupt due to counting down to 0.
    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

    // Enable a Timer interrupt to occur when the timer runs down to 0.
    TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
}

/**
 * timer2A_Config
 * Configures timer 2A on the Tiva
 */
void timer2A_Config()
{
    // Enable the peripheral timer 2
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);

    // Set as a periodic timer in so that the timer we reset to the max value after running down to 0.
    TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC);

    // Setting the system clock source for the Timer.
    TimerClockSourceSet(TIMER2_BASE, TIMER_CLOCK_SYSTEM);

    // Enable a timer 2A interrupt
    IntEnable(INT_TIMER2A);

    // Clear roll-over interrupt due to counting down to 0.
    TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);

    // Enable a Timer interrupt to occur when the timer runs down to 0.
    TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
}

/**
 * timer3A_Config
 * Configures timer 3A on the Tiva
 */
void timer3A_Config()
{
    // Enable the peripheral timer 2
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);

    // Set as a periodic timer in so that the timer we reset to the max value after running down to 0.
    TimerConfigure(TIMER3_BASE, TIMER_CFG_PERIODIC);

    // Setting the system clock source for the Timer.
    TimerClockSourceSet(TIMER3_BASE, TIMER_CLOCK_SYSTEM);

    // Enable timer 3A interrupt
    IntEnable(INT_TIMER3A);

    // Clear roll-over interrupt due to counting down to 0.
    TimerIntClear(TIMER3_BASE, TIMER_TIMA_TIMEOUT);

    // Enable a Timer interrupt to occur when the timer runs down to 0.
    TimerIntEnable(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
}

/**
 * startTimer1
 * Starts timer 1 given a rate in Hz
 * @param rate: the rate in Hz to run the timer
 */
void startTimer1(uint16_t rate)
{
    TimerEnable(TIMER1_BASE, TIMER_A);
    TimerLoadSet(TIMER1_BASE, TIMER_A, SysCtlClockGet() / rate);
}

/**
 * startTimer2
 * Starts timer 2 given a rate in Hz
 * @param rate: the rate in Hz to run the timer
 */
void startTimer2(uint16_t rate)
{
    TimerEnable(TIMER2_BASE, TIMER_A);
    TimerLoadSet(TIMER2_BASE, TIMER_A, SysCtlClockGet() / rate);
}

/**
 * startTimer3
 * Starts timer 3 given a rate in Hz
 * @param rate: the rate in Hz to run the timer
 */
void startTimer3(uint16_t rate)
{
    TimerEnable(TIMER3_BASE, TIMER_A);
    TimerLoadSet(TIMER3_BASE, TIMER_A, SysCtlClockGet() / rate);
}
