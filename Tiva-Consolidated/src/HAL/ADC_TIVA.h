/**
 * ADC_TIVA.h
 * @author: Nick Tremaroli
 * Contains the layout and functions regarding the Tiva ADCs
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

// Configure ADC0
void ADCConfig0(void);

// Configure ADC1
void ADCConfig1(void);

#endif /* ADC_TIVA_H */
