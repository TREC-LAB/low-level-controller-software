/*
*This is PWM library which writes the desired PWM frequency to the motor for motor control.
*
*Code Created by
*   Brandyn Greczek, Date: February 2, 2018.
*
*/

#include "PWM_Driver.h"

//Variables
//static float dc;
//static uint16_t pwmFrequency;
//static uint8_t dir; // direction pin

volatile uint32_t loop_count = 10;
volatile float f_m_inv;
volatile float tau_des;
float K1 = 2; //Desired Stiffness (Nm / rad)
float K2 = 10; //Desired Damping (Nm*s / rad)
float dc_max = 100.0;

float Fg = 105.6047; //Knee Link Weight (N, from CAD)

//Knee Link Center of Mass Coordinates (m, from CAD)
float x_com = 0.0074;
float y_com = 0.3838;

volatile uint32_t des_force_p; //Value for printing

//Desired joint angle and velocity
volatile float q1_des = -2;
volatile float dq1_des = 0;

volatile float tau_des;
//volatile float des_force;

/*void PWMConfig(){
    //Configure Pin for PWM
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE); //actuator 1
        SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
        GPIOPinConfigure(GPIO_PE4_M0PWM4);
        GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_4);
//
////        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE); //actuator 2
////        SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
////        GPIOPinConfigure(GPIO_PE5_M0PWM5);
////        GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_5);
////        PWMDeadBandDisable(PWM0_BASE, PWM_GEN_2); //Deadband set to disable delay between PE4 and PE5 because they are on the same generator
//
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);//ankle left
        HWREG(GPIO_PORTB_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY; //Use Hardware Reg to unlock and change function of GPIO pin, then lock them again
        HWREG(GPIO_PORTB_BASE + GPIO_O_CR) |= 0x01;
        HWREG(GPIO_PORTB_BASE + GPIO_O_LOCK) = 0;
        GPIOPinConfigure(GPIO_PB6_M0PWM0);
        GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6);
        PWMDeadBandDisable(PWM0_BASE, PWM_GEN_0); //Deadband set to disable delay

//        //Configure Pin for Direction, uses same peripheral Letter as PWM
  //      SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
//        HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY; //Use Hardware Reg to unlock and change function of GPIO pin, then lock them again
//        HWREG(GPIO_PORTD_BASE + GPIO_O_CR) &= ~0x11;
//        HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;
        GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_2); //actuator 1
////        GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_3); //actuator 2
//        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
        GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_1); //actuator 2
}*/

void PWMConfig()
{
    //Configure Pin for PWM
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE); //actuator 1
        SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
        GPIOPinConfigure(GPIO_PE4_M0PWM4);
        GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_4);
//
////        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE); //actuator 2
////        SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
////        GPIOPinConfigure(GPIO_PE5_M0PWM5);
////        GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_5);
////        PWMDeadBandDisable(PWM0_BASE, PWM_GEN_2); //Deadband set to disable delay between PE4 and PE5 because they are on the same generator
//

        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);//ankle left
        HWREG(GPIO_PORTB_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY; //Use Hardware Reg to unlock and change function of GPIO pin, then lock them again
        HWREG(GPIO_PORTB_BASE + GPIO_O_CR) |= 0x01;
        HWREG(GPIO_PORTB_BASE + GPIO_O_LOCK) = 0;
        GPIOPinConfigure(GPIO_PB6_M0PWM0);
        GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6);
        PWMDeadBandDisable(PWM0_BASE, PWM_GEN_0); //Deadband set to disable delay

//        //Configure Pin for Direction, uses same peripheral Letter as PWM
  //      SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
//        HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY; //Use Hardware Reg to unlock and change function of GPIO pin, then lock them again
//        HWREG(GPIO_PORTD_BASE + GPIO_O_CR) &= ~0x11;
//        HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;
        GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_2); //actuator 1
////        GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_3); //actuator 2
//        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
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
    else if (dc > dc_max)
    {
        dc = dc_max;
    }
//    printf("Actuator: %d, DC: %f\n", actuator, dc);

    //Added SysCtlPeripheralEnable for GPIO and PWM, Modified the PWM generator in the Configure and PeriodSet to generator 0
    SysCtlPWMClockSet(SYSCTL_PWMDIV_2);

    switch(actuator){
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
}
//    case 2: //Ankle Left Actuator
//        }
//    //set pwm clock to 20MHz, a quarter of system clock. This should give the resolution needed to set duty cycle in increments of 0.1%
//            //Pin PE5
//            PWMGenConfigure(PWM0_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);                                    //Configure the PWM generator for count down mode with immediate updates to the parameters.
//            PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, (int)((float)pwmClockSpeed/(float)pwmFrequency));                             //Set the period of the pwm signal using the pwm clock frequency and the desired signal frequency
//            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, (int)(((float)pwmClockSpeed/(float)pwmFrequency)*((float)dc/100)));  //Set the pulse width of PWM2 with duty cycle. PWM_OUT_2 refers to the second PWM2.
//            PWMGenEnable(PWM0_BASE, PWM_GEN_2);
//
//            //output duty cycle and direction pin
//            if (dc == 0)
//            {
//                PWMOutputState(PWM0_BASE, (PWM_OUT_5_BIT), false);
//            }
//            else
//            {
//                PWMOutputState(PWM0_BASE, (PWM_OUT_5_BIT), true);
//            }
//            if (dir == 0) // Direction is Clockwise
//            {
//                GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, 0);
//            }
//            else // Direction is Counter-Clockwise
//            {
//                GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, 0xff);
//            }
//            break;
//    }

//uint8_t getDutyCycle(void)
//{
//    return dc;
//}
//
//uint32_t getPWMFrequency(void)
//{
//    return pwmFrequency;
//}
