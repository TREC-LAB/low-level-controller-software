#include "SSI_TIVA.h"

void SSI0_Gurley_Config()
{
    // The SSI0 peripheral must be enabled for use.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //--------------------------------------------------------------
    //SSI pin configuration
    //      PA5 - SSI0Tx
    //      PA4 - SSI0Rx
    //      PA3 - SSI0Fss
    //      PA2 - SSI0CLK
    GPIOPinConfigure(GPIO_PA2_SSI0CLK); // green
    GPIOPinConfigure(GPIO_PA3_SSI0FSS);
    GPIOPinConfigure(GPIO_PA4_SSI0RX);  // blue
    GPIOPinConfigure(GPIO_PA5_SSI0TX);

    GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_3 |
                   GPIO_PIN_2);

    SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_TI,
                       SSI_MODE_MASTER, 500000, 16);    //16 bit

    SSIEnable(SSI0_BASE);
}

void SSI0_Orbis_Config()
{
    // The SSI0 peripheral must be enabled for use.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //--------------------------------------------------------------
    //SSI pin configuration
    //      PA5 - SSI0Tx
    //      PA4 - SSI0Rx
    //      PA3 - SSI0Fss
    //      PA2 - SSI0CLK
    GPIOPinConfigure(GPIO_PA2_SSI0CLK); // green
    GPIOPinConfigure(GPIO_PA3_SSI0FSS);
    GPIOPinConfigure(GPIO_PA4_SSI0RX);  // blue
    GPIOPinConfigure(GPIO_PA5_SSI0TX);

    GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_3 |
                   GPIO_PIN_2);

    SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
                       SSI_MODE_MASTER, 500000, 16);    //16 bit

    SSIEnable(SSI0_BASE);
}

void SSI0_Disable()
{
    SSIDisable(SSI0_BASE);
    SysCtlPeripheralDisable(SYSCTL_PERIPH_SSI0);
}

void SSI1_Gurley_Config()
{
    // The SSI1 peripheral must be enabled for use.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    HWREG(GPIO_PORTF_BASE+GPIO_O_LOCK) = GPIO_LOCK_KEY; //Unlock
    HWREG(GPIO_PORTF_BASE+GPIO_O_CR) |= 0X01;           // Enable PF0 AFS
    HWREG(GPIO_PORTF_BASE+GPIO_O_LOCK) = 0;             // Relock

    //--------------------------------------------------------------
    //SSI pin configuration
    //      PF2 - SSI0Tx
    //      PF3 - SSI0Rx
    //      PF0 - SSI0Fss
    //      PF1 - SSI0CLK
    GPIOPinConfigure(GPIO_PF2_SSI1CLK); //green
    GPIOPinConfigure(GPIO_PF3_SSI1FSS);
    GPIOPinConfigure(GPIO_PF0_SSI1RX); //blue
    GPIOPinConfigure(GPIO_PF1_SSI1TX);

    GPIOPinTypeSSI(GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_0 |
                       GPIO_PIN_1);

    SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_TI,
                       SSI_MODE_MASTER, 500000, 16);//16 bit

    SSIEnable(SSI1_BASE);
}
void SSI1_Orbis_Config()
{
    // The SSI1 peripheral must be enabled for use.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    HWREG(GPIO_PORTF_BASE+GPIO_O_LOCK) = GPIO_LOCK_KEY; //Unlock
    HWREG(GPIO_PORTF_BASE+GPIO_O_CR) |= 0X01;           // Enable PF0 AFS
    HWREG(GPIO_PORTF_BASE+GPIO_O_LOCK) = 0;             // Relock

    //--------------------------------------------------------------
    //SSI pin configuration
    //      PF2 - SSI0Tx
    //      PF3 - SSI0Rx
    //      PF0 - SSI0Fss
    //      PF1 - SSI0CLK
    GPIOPinConfigure(GPIO_PF2_SSI1CLK); //green
    GPIOPinConfigure(GPIO_PF3_SSI1FSS);
    GPIOPinConfigure(GPIO_PF0_SSI1RX); //blue
    GPIOPinConfigure(GPIO_PF1_SSI1TX);

    GPIOPinTypeSSI(GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_0 |
                   GPIO_PIN_1);

    SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
                       SSI_MODE_MASTER, 500000, 16);//16 bit

    SSIEnable(SSI1_BASE);
}

void SSI1_Disable()
{
    // The SSI0 peripheral must be enabled for use.
    SSIDisable(SSI1_BASE);
    SysCtlPeripheralDisable(SYSCTL_PERIPH_SSI1);
    HWREG(GPIO_PORTF_BASE+GPIO_O_LOCK) = GPIO_LOCK_KEY; //Unlock
    HWREG(GPIO_PORTF_BASE+GPIO_O_CR) &= 0XFE;           // Enable PF0 AFS
    HWREG(GPIO_PORTF_BASE+GPIO_O_LOCK) = 0;             // Relock
}

void SSI2_Config(){
//    //--------------------------------------------------------------
//    //SSI pin configuration
//    //      PB7 - SSI2Tx
//    //      PB6 - SSI2Rx
//    //      PB5 - SSI2Fss
//    //      PB4 - SSI2CLK

    // Configure SSI2 pins
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI2);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    GPIOPinConfigure(GPIO_PB4_SSI2CLK);
    GPIOPinConfigure(GPIO_PB6_SSI2RX);
    GPIOPinConfigure(GPIO_PB7_SSI2TX);

    // Initialize SSI2 pins
    GPIOPinTypeSSI(GPIO_PORTB_BASE, GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_7);

    // Initialize chip select pin separately
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_5);
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, GPIO_PIN_5);

    // Configure SSI clock
    SSIConfigSetExpClk(SSI2_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
                           SSI_MODE_MASTER, 500000, 8);
    SSIEnable(SSI2_BASE);
}

void SSI2_Disable()
{
    SSIDisable(SSI2_BASE);
    SysCtlPeripheralDisable(SYSCTL_PERIPH_SSI2);
}

void SSI3_Config_SPI()
{
    // Configure SSI3 pins
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI3);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    GPIOPinConfigure(GPIO_PD0_SSI3CLK);
    GPIOPinConfigure(GPIO_PD2_SSI3RX);
    GPIOPinConfigure(GPIO_PD3_SSI3TX);

    // Initialize SSI3 pins
    GPIOPinTypeSSI(GPIO_PORTD_BASE, GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_3);

    // Initialize chip select pin separately
    // Using port B pin 3 for chip select since PWM_Driver uses port D pin 1.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_3);
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, GPIO_PIN_3);

    // Configure SSI clock
    SSIConfigSetExpClk(SSI3_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
                           SSI_MODE_MASTER, 500000, 8);
    SSIEnable(SSI3_BASE);
}
