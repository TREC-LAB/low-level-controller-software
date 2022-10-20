/**
 * Timer_TIVA.h
 * @author: Nick Tremaroli
 * Contains the layout and functions regarding timer
 * related functions of the Tiva
 */

#ifndef TIMER_TIVA_H_
#define TIMER_TIVA_H_

// Standard library
#include <stdint.h>
#include <stdbool.h>

// TivaWare
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"

// Configure timer 1
void timer1A_Config();

// Configure timer 2
void timer2A_Config();

// Configure timer 3
void timer3A_Config();

// starts timer 1
void startTimer1(uint16_t rate);

// starts timer 2
void startTimer2(uint16_t rate);

// starts timer 3
void startTimer3(uint16_t rate);

#endif /* TIMER_TIVA_H_ */
