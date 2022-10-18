/*
 * myTimer.h
 *
 *  Created on: Aug 27, 2018
 *      Author: Zhoubao Pang
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

void timer1A_Config();
void timer2A_Config();
void timer3A_Config();

void startTimer1(uint16_t rate);
void startTimer2(uint16_t rate);
void startTimer3(uint16_t rate);

#endif /* TIMER_TIVA_H_ */
