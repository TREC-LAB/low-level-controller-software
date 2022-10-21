/**
 * I2C_Tiva.c
 * @author: Nick Tremaroli
 * Contains all of the low-level code for Tiva I2C communication
 */
#include "I2C_Tiva.h"

/**
 * I2C1_Config
 * Enables I2C1 to be working on Pin A6 and Pin A7
 */
void I2C1_Config()
{
    // Enable and Resets I2C1
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1);
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C1);

    // Enable Port A because I2C1 uses A6 and A7
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    // Enable I2C communication interface, SCL, SDA lines
    GPIOPinConfigure(GPIO_PA7_I2C1SDA);
    GPIOPinConfigure(GPIO_PA6_I2C1SCL);
    GPIOPinTypeI2CSCL(GPIO_PORTA_BASE, GPIO_PIN_6);
    GPIOPinTypeI2C(GPIO_PORTA_BASE, GPIO_PIN_7);

    //  Enable I2C master interface
    I2CMasterEnable(I2C1_BASE);

    // Run I2C bus in high-speed mode
    I2CMasterInitExpClk(I2C1_BASE, SysCtlClockGet(), true);
}

/**
 * I2C2_Config
 * Enables I2C2 to be working on Pin E4 and Pin E5
 */
void I2C2_Config(void)
{
    // Enable and Resets I2C2
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C2);
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C2);

    // Enable Port E because I2C2 uses E4 and E5
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    // Enable I2C communication interface, SCL, SDA lines
    GPIOPinConfigure(GPIO_PE5_I2C2SDA);
    GPIOPinConfigure(GPIO_PE4_I2C2SCL);
    GPIOPinTypeI2CSCL(GPIO_PORTE_BASE, GPIO_PIN_4);
    GPIOPinTypeI2C(GPIO_PORTE_BASE, GPIO_PIN_5);

    // Enable I2C master interface
    I2CMasterEnable(I2C2_BASE);

    // Run I2C bus in high-speed mode
    I2CMasterInitExpClk(I2C2_BASE, SysCtlClockGet(), true);
}

/**
 * I2C_WriteByte
 * Write one byte of data to I2C bus and wait until transmission is over (blocking)
 * @param I2CBase: The Base used for the I2C communication (I2C1_BASE, I2C2_BASE, etc.)
 * @param I2Caddress: 7-bit address of I2C device (8. bit is for R/W)
 * @param regAddress: Address of register in I2C device to write into
 * @param data: Data to write into the register of I2C device
 */
void I2C_WriteByte(uint32_t I2CBase, uint8_t I2Caddress, uint8_t regAddress, uint8_t data)
{
    // Set I2C address to write too
    I2CMasterSlaveAddrSet(I2CBase, I2Caddress, false);

    // Put register address into a sending buffer
    I2CMasterDataPut(I2CBase, regAddress);

    //  Send start sequence and address, followed by register address
    I2CMasterControl(I2CBase, I2C_MASTER_CMD_BURST_SEND_START);

    SysCtlDelay(40);
    while(I2CMasterBusy(I2CBase));

    // Send register data to write and stop sequence
    I2CMasterDataPut(I2CBase, data);
    I2CMasterControl(I2CBase, I2C_MASTER_CMD_BURST_SEND_FINISH);

    SysCtlDelay(40);
    while(I2CMasterBusy(I2CBase));

}

/**
 * I2C_ReadByte
 * Reads a single byte of data from the I2C device
 * @param I2CBase: The Base used for the I2C communication (I2C1_BASE, I2C2_BASE, etc.)
 * @param I2Caddress: 7-bit address of I2C device (8. bit is for R/W)
 * @param regAddress: Address of register in I2C device to write into
 * @return: The byte of data which was read
 */
uint8_t I2C_ReadByte(uint32_t I2CBase, uint8_t I2Caddress, uint8_t regAddress)
{
    // Set I2C address to read from
    I2CMasterSlaveAddrSet(I2CBase, I2Caddress, false);

    // Put register address into a sending buffer
    I2CMasterDataPut(I2CBase, regAddress);

    // Send start sequence and address, followed by register address
    I2CMasterControl(I2CBase, I2C_MASTER_CMD_BURST_SEND_START);

    while(I2CMasterBusy(I2CBase));

    // Change address to reading mode
    I2CMasterSlaveAddrSet(I2CBase, I2Caddress, true);

    // Perform a single receive from I2C bus
    I2CMasterControl(I2CBase, I2C_MASTER_CMD_SINGLE_RECEIVE);


    while(I2CMasterBusy(I2CBase));

    // Read a byte from receiving buffer
    uint32_t data = I2CMasterDataGet(I2CBase);

    // We're dealing with 8-bit data so return only lower 8 bits
    return (data & 0xFF);
}
/**
 * I2C_ReadBytes
 * Read several bytes from I2C device
 * @param I2CBase: The Base used for the I2C communication (I2C1_BASE, I2C2_BASE, etc.)
 * @param I2Caddress: 7-bit address of I2C device (8. bit is for R/W)
 * @param regAddress: address of register in I2C device to write into
 * @param length: number of bytes to red
 * @param data: a pointer to data buffer in which data is saved after reading
 */
void I2C_ReadBytes(uint32_t I2CBase, uint8_t I2Caddress, uint8_t regAddress, uint16_t length, uint8_t* data)
{
    uint16_t i;

    // Set I2C address to read from
    I2CMasterSlaveAddrSet(I2CBase, I2Caddress, false);

    // Put register address into a sending buffer
    I2CMasterDataPut(I2CBase, regAddress);

    // Send start sequence and address, followed by register address
    I2CMasterControl(I2CBase, I2C_MASTER_CMD_BURST_SEND_START);

    while(I2CMasterBusy(I2CBase));

    // Change address to reading mode
    I2CMasterSlaveAddrSet(I2CBase, I2Caddress, true);

    // Check how many bytes we need to receive
    if (length == 1)
        // if only one perform single receive
        I2CMasterControl(I2CBase, I2C_MASTER_CMD_SINGLE_RECEIVE);
    else
    {
        // if more than one perform burst receive
        I2CMasterControl(I2CBase, I2C_MASTER_CMD_BURST_RECEIVE_START);

        while(I2CMasterBusy(I2CBase));

        // get the first byte
        data[0] = (uint8_t)(I2CMasterDataGet(I2CBase) & 0xFF);

        // loop to finish the rest of the bytes
        for (i = 1; i < (length-1); i++)
        {
            // receive another byte
            I2CMasterControl(I2CBase, I2C_MASTER_CMD_BURST_RECEIVE_CONT);

            while(I2CMasterBusy(I2CBase));

            // store the byte from the buffer
            data[i] = (uint8_t)(I2CMasterDataGet(I2CBase) & 0xFF);
        }

        // finish the receive
        I2CMasterControl(I2CBase, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
    }

    while(I2CMasterBusy(I2CBase));

    // store the last byte
    data[length-1] = (uint8_t)(I2CMasterDataGet(I2CBase) & 0xFF);
}
