/**
 * QEI_TIVA.c
 * Contains all of the low-level code for QEI communication
 * on the Tiva
 */
#include "QEI_TIVA.h"

/**
 * QEIConfig0
 * Configures QEI BASE 0 on the Tiva
 */
void QEIConfig0()
{
    // Enable GPIO Port D because the QEI Encoder used PD6 and PD7
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

    // Use Hardware Reg to unlock and change function of GPIO pin, then lock them again
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 0x80;
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;

    // Configure PD6 and PD7 for the QEI encoder
    GPIOPinConfigure(GPIO_PD6_PHA0);
    GPIOPinConfigure(GPIO_PD7_PHB0);
    GPIOPinTypeQEI(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);

    // Wait for the QEI0 module to be ready.
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_QEI0)){}

    // Disable the QEI Base to configure it
    QEIDisable(QEI0_BASE);

    // swap channel A and B to make the quad encoder to have the same direction as the abs encoder
    QEIConfigure(QEI0_BASE, (QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_NO_RESET| QEI_CONFIG_QUADRATURE | QEI_CONFIG_SWAP), 4E5);

    // Configure and Enable the QEI Velocity Capture Module on the QEI.
    // Specify a velocity divider of 1 for the input Quadrature signal, and set the number of clock ticks over which to measure the velocity.
    QEIVelocityConfigure(QEI0_BASE, QEI_VELDIV_1, SysCtlClockGet() / 1000);

    // Enable the velocity capture capabilities of the QEI module.
    QEIVelocityEnable(QEI0_BASE);

    // Enable the QEI Base
    QEIEnable(QEI0_BASE);

    // Set the Position
    QEIPositionSet(QEI0_BASE, 2E5);
}

/**
 * QEIConfig1
 * Configures QEI BASE 0 on the Tiva
 */
void QEIConfig1()
{
    // Enable GPIO Port C because the QEI Encoder used PC5 and PC6
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);

    // Configure PC5 and PC6 for the QEI encoder
    GPIOPinConfigure(GPIO_PC5_PHA1);
    GPIOPinConfigure(GPIO_PC6_PHB1);
    GPIOPinTypeQEI(GPIO_PORTC_BASE, GPIO_PIN_5 |  GPIO_PIN_6);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI1);

    // Wait for the QEI1 module to be ready.
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_QEI1)){}

    // Disable the QEI Base to configure it
    QEIDisable(QEI1_BASE);

    // Swap channel A and B to make the quad encoder to have the same direction as the abs encoder
    QEIConfigure(QEI1_BASE, (QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_NO_RESET| QEI_CONFIG_QUADRATURE | QEI_CONFIG_SWAP), 4E5);

    // Configure and Enable the QEI Velocity Capture Module on the QEI.
    // Specify a velocity divider of 1 for the input Quadrature signal, and set the number of clock ticks over which to measure the velocity.
    QEIVelocityConfigure(QEI1_BASE, QEI_VELDIV_1, SysCtlClockGet() / 1000);

    // Enable the velocity capture capabilities of the QEI module.
    QEIVelocityEnable(QEI1_BASE);

    // Enable the QEI Base
    QEIEnable(QEI1_BASE);

    // Set the Position
    QEIPositionSet(QEI1_BASE, 2E5);
}

//extern void ConfigQEI(void)
//{
//    // Enable QEI Peripherals
//    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
//    SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);
//
//    //Unlock GPIOD7 - Like PF0 its used for NMI - Without this step it doesn't work
//    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY; //In Tiva include this is the same as "_DD" in older versions (0x4C4F434B)
//    HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 0x80;
//    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;
//
//    //Set Pins to be PHA0 and PHB0
//    GPIOPinConfigure(GPIO_PD6_PHA0);
//    GPIOPinConfigure(GPIO_PD7_PHB0);
//
//    //Set GPIO pins for QEI. PhA0 -> PD6, PhB0 ->PD7. I believe this sets the pull up and makes them inputs
//    GPIOPinTypeQEI(GPIO_PORTD_BASE, GPIO_PIN_6 |  GPIO_PIN_7);
//
//    //DISable peripheral and int before configuration
//    QEIDisable(QEI0_BASE);
//    QEIIntDisable(QEI0_BASE,QEI_INTERROR | QEI_INTDIR | QEI_INTTIMER | QEI_INTINDEX);
//
//    // Configure quadrature encoder, use an arbitrary top limit
//    QEIConfigure(QEI0_BASE, (QEI_CONFIG_CAPTURE_A_B  | QEI_CONFIG_NO_RESET  | QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP), 50000000);
//
//    // Enable the quadrature encoder.
//    QEIEnable(QEI0_BASE);
//
//    //Set position to a middle value so we can see if things are working
//    QEIPositionSet(QEI0_BASE, 50000000/2);
//
//    //Configure and Enable the QEI Velocity Capture Module on the QEI
//    QEIVelocityConfigure(QEI0_BASE, QEI_VELDIV_1, SysCtlClockGet() / 100); //Specify a velocity divider of 1 for the input Quadrature signal, and set the number of clock ticks over which to measure the velocity.
//    QEIVelocityEnable(QEI0_BASE); //Enable the velocity capture capabilities of the QEI module.
//}

