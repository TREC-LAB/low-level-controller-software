/**
 * SSI_TIVA.c
 * Contains all of the low-level code for SSI communication
 * on the Tiva
 */
#include "SSI_TIVA.h"

/**
 * SSI0_Gurley_Config
 * Configures a Gurley encoder to work on SSI0
 */
void SSI0_Gurley_Config()
{
    // The SSI0 peripheral must be enabled for use.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);

    // Enable GPIO Port A as SSI0 used PA2, PA3, PA4, and PA5
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    // Pin A2 is the SSI0 Clock
    GPIOPinConfigure(GPIO_PA2_SSI0CLK);

    // Pin A3 is the SSI0 slave select
    GPIOPinConfigure(GPIO_PA3_SSI0FSS);

    // Pin PA4 is the SSI0 receive pin
    GPIOPinConfigure(GPIO_PA4_SSI0RX);

    // Pin A5 is the SSI0 transmit pin
    GPIOPinConfigure(GPIO_PA5_SSI0TX);

    // Configure these pins to be used for SSI
    GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_3 | GPIO_PIN_2);

    // Configure SSI0 with the following settings
    SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_TI,
                       SSI_MODE_MASTER, 500000, 16);    //16 bit

    // Enable the SSI0 Base
    SSIEnable(SSI0_BASE);
}

/**
 * SSI0_Orbis_Config
 * Configures a Orbis encoder to work on SSI0
 */
void SSI0_Orbis_Config()
{
    // The SSI0 peripheral must be enabled for use.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);

    // Enable GPIO Port A as SSI0 used PA2, PA3, PA4, and PA5
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    // Pin A2 is the SSI0 Clock
    GPIOPinConfigure(GPIO_PA2_SSI0CLK);

    // Pin A3 is the SSI0 slave select
    GPIOPinConfigure(GPIO_PA3_SSI0FSS);

    // Pin PA4 is the SSI0 receive pin
    GPIOPinConfigure(GPIO_PA4_SSI0RX);

    // Pin A5 is the SSI0 transmit pin
    GPIOPinConfigure(GPIO_PA5_SSI0TX);

    // Configure these pins to be used for SSI
    GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_3 | GPIO_PIN_2);

    // Configure the SSI0 clock with the following settings
    SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
                       SSI_MODE_MASTER, 500000, 16);    //16 bit

    // Enable the SSI0 Base
    SSIEnable(SSI0_BASE);
}

/**
 * SSI0_Disable
 * Disables SSI0
 */
void SSI0_Disable()
{
    SSIDisable(SSI0_BASE);
    SysCtlPeripheralDisable(SYSCTL_PERIPH_SSI0);
}

/**
 * SSI1_Gurley_Config
 * Configures a Gurley encoder to work on SSI1
 */
void SSI1_Gurley_Config()
{
    // The SSI1 peripheral must be enabled for use.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI1);

    // Enable GPIO Port F as SSI0 used PF0, PF1, PF2, and PF3
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    // Edit the hardware registry to enable these pins
    HWREG(GPIO_PORTF_BASE+GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTF_BASE+GPIO_O_CR) |= 0X01;
    HWREG(GPIO_PORTF_BASE+GPIO_O_LOCK) = 0;

    // Pin F2 is the SSI1 clock
    GPIOPinConfigure(GPIO_PF2_SSI1CLK);

    // Pin F3 is the SSI1 slave select
    GPIOPinConfigure(GPIO_PF3_SSI1FSS);

    // Pin F0 is the SSI1 receive pin
    GPIOPinConfigure(GPIO_PF0_SSI1RX);

    // Pin F1 is the SSI1 transmit pin
    GPIOPinConfigure(GPIO_PF1_SSI1TX);

    // Configure these pins to be used for SSI1
    GPIOPinTypeSSI(GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_0 | GPIO_PIN_1);

    // Configure the SSI1 clock with the following settings
    SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_TI,
                       SSI_MODE_MASTER, 500000, 16);//16 bit

    // Enable the SSI1 Base
    SSIEnable(SSI1_BASE);
}

/**
 * SSI1_Orbis_Config
 * Configures a Orbis encoder to work on SSI1
 */
void SSI1_Orbis_Config()
{
    // The SSI1 peripheral must be enabled for use.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI1);

    // Enable GPIO Port F as SSI0 used PF0, PF1, PF2, and PF3
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    // Edit the hardware registry to enable these pins
    HWREG(GPIO_PORTF_BASE+GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTF_BASE+GPIO_O_CR) |= 0X01;
    HWREG(GPIO_PORTF_BASE+GPIO_O_LOCK) = 0;

    // Pin F2 is the SSI1 clock
    GPIOPinConfigure(GPIO_PF2_SSI1CLK);

    // Pin F3 is the SSI1 slave select
    GPIOPinConfigure(GPIO_PF3_SSI1FSS);

    // Pin F0 is the SSI1 receive pin
    GPIOPinConfigure(GPIO_PF0_SSI1RX);

    // Pin F1 is the SSI1 transmit pin
    GPIOPinConfigure(GPIO_PF1_SSI1TX);

    // Configure these pins to be used for SSI1
    GPIOPinTypeSSI(GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_0 | GPIO_PIN_1);

    // Configure the SSI1 clock with the following settings
    SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
                       SSI_MODE_MASTER, 500000, 16);//16 bit

    // Enable the SSI1 Base
    SSIEnable(SSI1_BASE);
}
/**
 * SSI0_Disable
 * Disables SSI1
 */
void SSI1_Disable()
{
    // The SSI0 peripheral must be enabled for use.
    SSIDisable(SSI1_BASE);
    SysCtlPeripheralDisable(SYSCTL_PERIPH_SSI1);
//    HWREG(GPIO_PORTF_BASE+GPIO_O_LOCK) = GPIO_LOCK_KEY; //Unlock
//    HWREG(GPIO_PORTF_BASE+GPIO_O_CR) &= 0XFE;           // Enable PF0 AFS
//    HWREG(GPIO_PORTF_BASE+GPIO_O_LOCK) = 0;             // Relock
}

void SSI2_Config()
{
    //--------------------------------------------------------------
    //SSI pin configuration
    //      PB7 - SSI2Tx
    //      PB6 - SSI2Rx
    //      PB5 - SSI2Fss
    //      PB4 - SSI2CLK

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
//
void SSI2_Disable()
{
    SSIDisable(SSI2_BASE);
    SysCtlPeripheralDisable(SYSCTL_PERIPH_SSI2);
}

/**
 * SSI3_Config_SPI
 * Configures SPI on SSI3 pins on the Tiva
 * This SSI3 Base is primarily reserved for EtherCAT
 */
void SSI3_Config_SPI()
{
    // Configure SSI3 pins
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI3);

    // Enable GPIO Port D as SSI3 used PD0, PD2, and PFD3
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

    // Configure SSI clock with the following settings
    SSIConfigSetExpClk(SSI3_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
                           SSI_MODE_MASTER, 20000000, 8);

    // Enable the SSI3 Base
    SSIEnable(SSI3_BASE);
}
