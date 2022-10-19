/**
 * PWM_Driver.c
 * Contains all of the low-level code for a Tiva PWM
 */
#include "PWM_Driver.h"

/**
 * PWMConfig
 * Configures the PWMs which the Tiva is currently using on Pandora
 */
void PWMConfig()
{
    // Enable GPIO Port E for actuator 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    // Enable Pin E4 for actuator 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    GPIOPinConfigure(GPIO_PE4_M0PWM4);
    GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_4);

    // Enable GPIO Port B for actuator 1
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    // Use Hardware Reg to unlock and change function of GPIO pin, then lock them again
    HWREG(GPIO_PORTB_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTB_BASE + GPIO_O_CR) |= 0x01;
    HWREG(GPIO_PORTB_BASE + GPIO_O_LOCK) = 0;

    // Enable Pin B6 for actuator 1
    GPIOPinConfigure(GPIO_PB6_M0PWM0);
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6);

    // Deadband set to disable delay
    PWMDeadBandDisable(PWM0_BASE, PWM_GEN_0);

    // Configure the actuator 0 direction pin
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_2);

    // Configure the actuator 1 direction pin
    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_1);

}

/**
 * PWMGeneratorConstruct
 * Generators and initializes a PWMGenerator struct
 * @param pwmFrequency: The frequency the PWM will run at
 */
PWMGenerator PWMGeneratorConstruct(uint16_t pwmFrequency)
{
    // create the PWM generator
    PWMGenerator pwmGenerator;

    // initialize direction and duty cycle to zero
    pwmGenerator.direction = 0;
    pwmGenerator.dutyCycle = 0.0;

    // set the pwm Frequency accordingly
    pwmGenerator.pwmFrequency = pwmFrequency;

    // return the newly constructed PWM generator
    return pwmGenerator;
}

/**
 * SetPulseWidth
 * Sets the pulse width of a PWM pin given the appropriate input parameters
 * @param actuator: a number corresponding to which side of the sensor board to
 * send a pwm signal (actuator 0 or actuator 1)
 * @param pwmGenerator: a pointer to the pwmGenerator which contains all of the settings
 * on how to command the pwm
 * @param sysClock: The input clock speed of the Tiva
 */
void setPulseWidth(uint8_t actuator, PWMGenerator* pwmGenerator, uint32_t sysClock)
{
    uint32_t pwmClockSpeed = sysClock/2;                                                                        //get current processor clock speed
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
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, (int)((float)pwmClockSpeed/(float)pwmGenerator->pwmFrequency));                             //Set the period of the pwm signal using the pwm clock frequency and the desired signal frequency
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, (int)(((float)pwmClockSpeed/(float)pwmGenerator->pwmFrequency)*((float)pwmGenerator->dutyCycle/100)));  //Set the pulse width of PWM2 with duty cycle. PWM_OUT_2 refers to the second PWM2.
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
