/**
 * PWM_Driver.h
 * @author: Nick Tremaroli
 * Contains the layout and functions regarding the Tiva's
 * generation of a PWM
 */

#ifndef PWM_DRIVER_H
#define PWM_DRIVER_H

#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/pwm.h"

/**
 * PWMGenerator
 * a struct which contains all of the data
 * needed to generate a PWM signal
 */
struct PWMGenerator
{
    float dutyCycle;
    uint16_t pwmFrequency;
    uint8_t direction;
};
typedef struct PWMGenerator PWMGenerator;

// Configure the PWM
void PWMConfig(void);

// Construct a PWM generator object
PWMGenerator PWMGeneratorConstruct(uint16_t pwmFrequency);

//Provide a desired integer PWM Frequency, followed by an integer duty cycle.
void setPulseWidth(uint8_t actuator, PWMGenerator* pwmGenerator, uint32_t sysClock);

//Get the current duty cycle of the PWM signal.
uint8_t getDutyCycle(void);

//Get the current frequency of the PWM signal.
uint32_t getPWMFrequency(void);

#endif	/* PWM_DRIVER_H_ */
