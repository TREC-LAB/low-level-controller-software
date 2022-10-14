/*
*This is PWM library which writes the desired PWM frequency to the motor for motor control.
*
*Code Created by
*   Brandyn Greczek, Date: February 2, 2018.
*
*/

#include "PWM_Driver.h"

void PWMConfig()
{
    //Configure Pin for PWM
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE); //actuator 1
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    GPIOPinConfigure(GPIO_PE4_M0PWM4);
    GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_4);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);//ankle left
    HWREG(GPIO_PORTB_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY; //Use Hardware Reg to unlock and change function of GPIO pin, then lock them again
    HWREG(GPIO_PORTB_BASE + GPIO_O_CR) |= 0x01;
    HWREG(GPIO_PORTB_BASE + GPIO_O_LOCK) = 0;
    GPIOPinConfigure(GPIO_PB6_M0PWM0);
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6);
    PWMDeadBandDisable(PWM0_BASE, PWM_GEN_0); //Deadband set to disable delay


    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_2); //actuator 1

    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_1); //actuator 2

}

void setPulseWidth(uint8_t actuator,uint16_t pwmFrequency, float dc, uint32_t SysClock, uint8_t dir)
{
    uint32_t pwmClockSpeed = SysClock/2;                                                                        //get current processor clock speed
//    actuator = act;
    if (dc < 0.1)
    {
        dc = 0;
    }
    else if(dc >= 100.0)
    {
        dc = 100.0;
    }

    //Added SysCtlPeripheralEnable for GPIO and PWM, Modified the PWM generator in the Configure and PeriodSet to generator 0
    SysCtlPWMClockSet(SYSCTL_PWMDIV_2);

    switch(actuator)
    {
    case 0: //Actuator 0

        //set pwm clock to 20MHz, a quarter of system clock. This should give the resolution needed to set duty cycle in increments of 0.1%
        //Pin PE4
        PWMGenConfigure(PWM0_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);                                    //Configure the PWM generator for count down mode with immediate updates to the parameters.
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, (int)((float)pwmClockSpeed/(float)pwmFrequency));                             //Set the period of the pwm signal using the pwm clock frequency and the desired signal frequency
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, (int)(((float)pwmClockSpeed/(float)pwmFrequency)*((float)dc/100)));  //Set the pulse width of PWM2 with duty cycle. PWM_OUT_2 refers to the second PWM2.
        PWMGenEnable(PWM0_BASE, PWM_GEN_2);                                                                                 //Start the timers in generator 2 which controls PWM2.

        //output duty cycle and direction pin
        if (dc == 0)
        {
           PWMOutputState(PWM0_BASE, (PWM_OUT_4_BIT), false);
        }
        else
        {
            PWMOutputState(PWM0_BASE, (PWM_OUT_4_BIT), true);
        }
        if (dir == 0) // Direction is Clockwise
        {
            GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2, 0);
        }
        else // Direction is Counter-Clockwise
        {
            GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2, 0xff);
        }
        break;

    case 1: //Actuator 1
        //set pwm clock to 20MHz, a quarter of system clock. This should give the resolution needed to set duty cycle in increments of 0.1%
        //PB4
        PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);                                    //Configure the PWM generator for count down mode with immediate updates to the parameters.
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, (int)((float)pwmClockSpeed/(float)pwmFrequency));                             //Set the period of the pwm signal using the pwm clock frequency and the desired signal frequency
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, (int)(((float)pwmClockSpeed/(float)pwmFrequency)*((float)dc/100)));  //Set the pulse width of PWM2 with duty cycle. PWM_OUT_2 refers to the second PWM2.
        PWMGenEnable(PWM0_BASE, PWM_GEN_0);
        //output duty cycle and direction pin
        if (dc == 0)
        {
            PWMOutputState(PWM0_BASE, (PWM_OUT_0_BIT), false);
        }
        else
        {
            PWMOutputState(PWM0_BASE, (PWM_OUT_0_BIT), true);
        }
        if (dir == 0) // Direction is Clockwise
        {
            GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, 0);
        }
        else // Direction is Counter-Clockwise
        {
            GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, 0xff);
        }
        break;
      }

}
