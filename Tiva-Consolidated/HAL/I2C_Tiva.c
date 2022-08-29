#include "I2C_Tiva.h"


void InitI2C0(void)
{
    //enable I2C module 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);

    //reset module
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);

    //enable GPIO peripheral that contains I2C 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    // Configure the pin muxing for I2C0 functions on port B2 and B3.
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);

    // Select the I2C function for these pins.
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

    // Enable and initialize the I2C0 master module.  Use the system clock for
    // the I2C0 module.  The last parameter sets the I2C data transfer rate.
    // If false the data rate is set to 100kbps and if true the data rate will
    // be set to 400kbps.
    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), false);

    //clear I2C FIFOs
    HWREG(I2C0_BASE + I2C_O_FIFOCTL) = 80008000;
}

void I2C2_Config(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C2);
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C2);

    // Enable I2C communication interface, SCL, SDA lines

    GPIOPinConfigure(GPIO_PE5_I2C2SDA);
    GPIOPinConfigure(GPIO_PE4_I2C2SCL);
    GPIOPinTypeI2CSCL(GPIO_PORTE_BASE, GPIO_PIN_4);
    GPIOPinTypeI2C(GPIO_PORTE_BASE, GPIO_PIN_5);

    //  Enable I2C master interface
    I2CMasterEnable(I2C2_BASE);

    // Run I2C bus in high-speed mode, 400kHz speed (May not be at 400 kHz atm)
    I2CMasterInitExpClk(I2C2_BASE, SysCtlClockGet(), true);

    //  Configure power-switch pin
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_5);
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, 0x00);

    //enable I2C module 0
/*    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);

    //reset module
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);

    //enable GPIO peripheral that contains I2C 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    // Configure the pin muxing for I2C0 functions on port B2 and B3.
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);

    // Select the I2C function for these pins.
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

    // Enable and initialize the I2C0 master module.  Use the system clock for
    // the I2C0 module.  The last parameter sets the I2C data transfer rate.
    // If false the data rate is set to 100kbps and if true the data rate will
    // be set to 400kbps.
    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), false);

    //clear I2C FIFOs
    HWREG(I2C0_BASE + I2C_O_FIFOCTL) = 80008000;
*/
}

/**
 * Write one byte of data to I2C bus and wait until transmission is over (blocking)
 * @param I2Caddress 7-bit address of I2C device (8. bit is for R/W)
 * @param regAddress Address of register in I2C device to write into
 * @param data Data to write into the register of I2C device
 */
void I2C_WriteByte(uint32_t IMUBase, uint8_t I2Caddress, uint8_t regAddress, uint8_t data)
{
    //  Set I2C address of MPU, writing mode
    I2CMasterSlaveAddrSet(IMUBase, I2Caddress, false);
    //  Push register address into a sending buffer
    I2CMasterDataPut(IMUBase, regAddress);
    //  Send start sequence and address, followed by register address
    I2CMasterControl(IMUBase, I2C_MASTER_CMD_BURST_SEND_START);
//    HAL_DelayUS(4);
    SysCtlDelay(40);
    while(I2CMasterBusy(IMUBase));

    //  Send register data to write and stop sequence
    I2CMasterDataPut(IMUBase, data);
    I2CMasterControl(IMUBase, I2C_MASTER_CMD_BURST_SEND_FINISH);
//    HAL_DelayUS(4);
    SysCtlDelay(40);
    while(I2CMasterBusy(IMUBase));

}

uint8_t I2C_ReadByte(uint32_t IMUBase, uint8_t I2Caddress, uint8_t regAddress)
{
    uint32_t data;

    //  Set I2C address of MPU, reading mode (incorrect)
    //  I'm not sure why is the sending condition requires address in reading,
    //  bute there are problems if it's not done this way :/
    I2CMasterSlaveAddrSet(IMUBase, I2Caddress, false);
    //  Push register address into a sending buffer
    I2CMasterDataPut(IMUBase, regAddress);
    //  Send start sequence and address, followed by register address
    I2CMasterControl(IMUBase, I2C_MASTER_CMD_BURST_SEND_START);
    //    HAL_DelayUS(4);
    while(I2CMasterBusy(IMUBase));

    //  Perform s single receive from I2C bus
    I2CMasterSlaveAddrSet(IMUBase, I2Caddress, true);
    I2CMasterControl(IMUBase, I2C_MASTER_CMD_SINGLE_RECEIVE);
    //    HAL_DelayUS(4);
    while(I2CMasterBusy(IMUBase));
    //  Read a byte from receiving buffer
    data = I2CMasterDataGet(IMUBase);

    //  We're dealing with 8-bit data so return only lower 8 bits
    return (data & 0xFF);
}
/**
 * Read several bytes from I2C device
 * @param I2Caddress 7-bit address of I2C device (8. bit is for R/W)
 * @param regAddress address of register in I2C device to write into
 * @param count number of bytes to red
 * @param dest pointer to data buffer in which data is saved after reading
 */
uint8_t I2C_ReadBytes(uint32_t IMUBase, uint8_t I2Caddress, uint8_t regAddress, uint16_t length, uint8_t* data)
{
    uint16_t i;

    //  Set I2C address of MPU, reading mode (incorrect)
    //  I'm not sure why is the sending condition requires address in reading,
    //  but there are problems if it's not done this way :/
    I2CMasterSlaveAddrSet(IMUBase, I2Caddress, false);
    //  Push register address into a sending buffer
    I2CMasterDataPut(IMUBase, regAddress);
    //  Send start sequence and address, followed by register address. Use burst
    //  mode as we're reading more than 1 byte
    // This sends controls byte and register address byte to slave device (MPU 9250)
    I2CMasterControl(IMUBase, I2C_MASTER_CMD_BURST_SEND_START);

//    HAL_DelayUS(4);
//    SysCtlDelay(40);      // not sure if this delay was necessary

    // Wait for MCU to finish transaction
    while(I2CMasterBusy(IMUBase));

    //  Change address to reading mode
    I2CMasterSlaveAddrSet(IMUBase, I2Caddress, true);

    //  Check how many bytes we need to receive
    if (length == 1)
        I2CMasterControl(IMUBase, I2C_MASTER_CMD_SINGLE_RECEIVE);
    else
    {
        I2CMasterControl(IMUBase, I2C_MASTER_CMD_BURST_RECEIVE_START);
//        HAL_DelayUS(4);
//        SysCtlDelay(10);
        while(I2CMasterBusy(IMUBase));
        data[0] = (uint8_t)(I2CMasterDataGet(IMUBase) & 0xFF);

        for (i = 1; i < (length-1); i++)
        {
            I2CMasterControl(IMUBase, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
//            HAL_DelayUS(4);
//            SysCtlDelay(10);
            while(I2CMasterBusy(IMUBase));
            data[i] = (uint8_t)(I2CMasterDataGet(IMUBase) & 0xFF);
        }
        I2CMasterControl(IMUBase, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
    }

//    HAL_DelayUS(4);
//    SyswCtlDelay(10);
    while(I2CMasterBusy(IMUBase));
    data[length-1] = (uint8_t)(I2CMasterDataGet(IMUBase) & 0xFF);

    return 0;
}
