/*
 * myADC.h
 *
 *  Created on: Aug 27, 2018
 *      Author: Zhoubao Pang
 *  Modified by: Nick Tremaroli and Sam Schoedel
 */

#ifndef ADC_TIVA_H
#define ADC_TIVA_H

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/ssi.h"
#include "driverlib/adc.h"

void ADCConfig0(void);
void ADCConfig1(void);

#endif /* ADC_TIVA_H */
