/*
*This is the header file for sendind the PWM signals to the BLDC motors for motor control.
*
*Code Created by
*   Brandyn Greczek, Date: February 2, 2018.
*/


#ifndef PWM_DRIVER_H
#define PWM_DRIVER_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/pwm.h"

extern volatile uint32_t loop_count;
extern volatile float f_m_inv;
extern float K1;
extern float K2;
extern float Fg;
extern float x_com;
extern float y_com;
extern volatile uint32_t des_force_p; //High level controller output value (for printing)
extern volatile float q1_des;
extern volatile float dq1_des;
extern volatile float tau_des;
//extern volatile float des_force;

//Function Prototypes
extern void PWMConfig(void);
extern void setPulseWidth(uint8_t actuator, uint16_t pwmFrequency, float dc, uint32_t SysClock, uint8_t dir);            //Provide a desired integer PWM Frequency, followed by an integer duty cycle.
uint8_t getDutyCycle(void);                                             //Get the current duty cycle of the PWM signal.
uint32_t getPWMFrequency(void);                                         //Get the current frequency of the PWM signal.

#endif	/* PWM_DRIVER_H_ */
