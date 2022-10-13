#include "QEI_TIVA.h"

void QEIConfig0()
{
    //Setup Ports C and D for incremental encoders
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY; //Use Hardware Reg to unlock and change function of GPIO pin, then lock them again
    HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 0x80;
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;

    //Incremental Encoder PHA0 and PHB0
    GPIOPinConfigure(GPIO_PD6_PHA0); //green wire
    GPIOPinConfigure(GPIO_PD7_PHB0); //blue wire
    GPIOPinTypeQEI(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);

    //Wait for the QEI1 module to be ready.
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_QEI0)){}
    QEIDisable(QEI0_BASE);
    QEIConfigure(QEI0_BASE, (QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_NO_RESET| QEI_CONFIG_QUADRATURE | QEI_CONFIG_SWAP), 4E5);//swap channel A and B to make the quad encoder to have the same direction as the abs encoder

    //Configure and Enable the QEI Velocity Capture Module on the QEI.
    QEIVelocityConfigure(QEI0_BASE, QEI_VELDIV_1, SysCtlClockGet() / 1000);                            //Specify a velocity divider of 1 for the input Quadrature signal, and set the number of clock ticks over which to measure the velocity.
    QEIVelocityEnable(QEI0_BASE);                                                                      //Enable the velocity capture capabilities of the QEI module.

    QEIEnable(QEI0_BASE);
    QEIPositionSet(QEI0_BASE, 2E5);
}

void QEIConfig1()
{
    //Setup Ports C and D for incremental encoders
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);

    //Incremental Encoder PHA1 and PHB1
    GPIOPinConfigure(GPIO_PC5_PHA1); //green wire
    GPIOPinConfigure(GPIO_PC6_PHB1); //blue wire
    GPIOPinTypeQEI(GPIO_PORTC_BASE, GPIO_PIN_5 |  GPIO_PIN_6);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI1);

    //Wait for the QEI1 module to be ready.
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_QEI1)){}
    QEIDisable(QEI1_BASE);
    QEIConfigure(QEI1_BASE, (QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_NO_RESET| QEI_CONFIG_QUADRATURE | QEI_CONFIG_SWAP), 4E5);//swap channel A and B to make the quad encoder to have the same direction as the abs encoder

    //Configure and Enable the QEI Velocity Capture Module on the QEI.
    QEIVelocityConfigure(QEI1_BASE, QEI_VELDIV_1, SysCtlClockGet() / 1000);                            //Specify a velocity divider of 1 for the input Quadrature signal, and set the number of clock ticks over which to measure the velocity.
    QEIVelocityEnable(QEI1_BASE);                                                                      //Enable the velocity capture capabilities of the QEI module.

    QEIEnable(QEI1_BASE);
    QEIPositionSet(QEI1_BASE, 2E5);
}

extern void ConfigQEI(void)
{
    // Enable QEI Peripherals
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);

    //Unlock GPIOD7 - Like PF0 its used for NMI - Without this step it doesn't work
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY; //In Tiva include this is the same as "_DD" in older versions (0x4C4F434B)
    HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 0x80;
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;

    //Set Pins to be PHA0 and PHB0
    GPIOPinConfigure(GPIO_PD6_PHA0);
    GPIOPinConfigure(GPIO_PD7_PHB0);

    //Set GPIO pins for QEI. PhA0 -> PD6, PhB0 ->PD7. I believe this sets the pull up and makes them inputs
    GPIOPinTypeQEI(GPIO_PORTD_BASE, GPIO_PIN_6 |  GPIO_PIN_7);

    //DISable peripheral and int before configuration
    QEIDisable(QEI0_BASE);
    QEIIntDisable(QEI0_BASE,QEI_INTERROR | QEI_INTDIR | QEI_INTTIMER | QEI_INTINDEX);

    // Configure quadrature encoder, use an arbitrary top limit
    QEIConfigure(QEI0_BASE, (QEI_CONFIG_CAPTURE_A_B  | QEI_CONFIG_NO_RESET  | QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP), 50000000);

    // Enable the quadrature encoder.
    QEIEnable(QEI0_BASE);

    //Set position to a middle value so we can see if things are working
    QEIPositionSet(QEI0_BASE, 50000000/2);

    //Configure and Enable the QEI Velocity Capture Module on the QEI
    QEIVelocityConfigure(QEI0_BASE, QEI_VELDIV_1, SysCtlClockGet() / 100); //Specify a velocity divider of 1 for the input Quadrature signal, and set the number of clock ticks over which to measure the velocity.
    QEIVelocityEnable(QEI0_BASE); //Enable the velocity capture capabilities of the QEI module.
}

