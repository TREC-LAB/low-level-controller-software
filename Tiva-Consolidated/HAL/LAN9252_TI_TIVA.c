/**
 * LAN9252_TI_TIVA.c
 * @author: Nick Tremaroli
 * Contains the layout and functions regarding the LAN9252
 */

#include "LAN9252_TI_TIVA.h"

/**
 * EtherCAT_MainTask
 * Sends and receives new data to and from the master computer respectivly
 * @param etherCATOutputFrames: the output data frame structure to send to the master
 * @param etherCATInputFrames: the input data frame structure which is populated by the master
 * @return: the status of the EasyCAT
 */
uint16_t EtherCAT_MainTask(EtherCATFrames_OUT *etherCATOutputFrames, EtherCATFrames_IN *etherCATInputFrames)
{
    // setup and initialize variables used
    uint16_t WatchDog = 0;
    unsigned Operational = 0;
    uint16_t i;
    ULONG TempLong;
    uint16_t Status;

    // Read the watchdog status
    TempLong.Long = LAN9252ReadRegisterIndirectOneByte(WDOG_STATUS);

    // set/reset the watchdog based on the corresponding flag
    if ((TempLong.Word[0] & 0x01) == 0x01)
        WatchDog = 0;
    else
        WatchDog = 1;

    // Read the EtherCAT State Machine status
    TempLong.Long = LAN9252ReadRegisterIndirectOneByte(AL_STATUS);
    Status = TempLong.Word[0] & 0x0F;

    // set/reset the operational state based on the corresponding flag
    if (Status == ESM_OP)
        Operational = 1;
    else
        Operational = 0;

    // Reset the output buffer if watchdog is active or we are not in operational state,
    if (WatchDog == 1 | Operational == 0)
    {
        for (i = 0; i < TOT_BYTE_NUM_OUT; i++)
            etherCATInputFrames->rawBytes[i] = 0;
    }
    // Otherwise transfer process data from the EtherCAT core to the output buffer
    else
        LAN9252ReadProcRamFifo(etherCATInputFrames);

    // Transfer process data from the output buffer to the master
    LAN9252WriteProcRamFifo(etherCATOutputFrames);

    // Return the status of the State Machine and of the watchdog
    if (WatchDog)
        Status |= 0x80;
    return Status;
}

/**
 * EtherCAT_Init
 * Initializes the EasyCAT board
 * @param etherCATOutputFrames: a pointer to the output data frame structure
 * @return: 1 on success. 2 on reset time-out failure. 3 on byte order timeout failure.
 * 4 on ready flag timeout failure
 */
uint16_t EtherCAT_Init(EtherCATFrames_OUT *etherCATOutputFrames)
{
    ULONG TempLong;
    uint16_t i;

    // Reset the LAN9252
    LAN9252WriteRegisterDirect(RESET_CTL, DIGITAL_RST);

    // Wait for the reset to complete
    // have a time-out feature to check for failure
    i = 0;
    do
    {
        i++;
        TempLong.Long = LAN9252ReadRegisterDirect(RESET_CTL, 4);
    }
    while (((TempLong.Word[0] & 0x01) != 0x00) && (i != ETHERCAT_INIT_TIMEOUT));

    // If the time out expired, then initialization failed on the reset
    if (i == ETHERCAT_INIT_TIMEOUT)
        return 2;

    // Check the Byte Order Test Register
    // have a time-out feature to check for failure
    i = 0;
    do
    {
        i++;
        TempLong.Long = LAN9252ReadRegisterDirect(BYTE_TEST, 4);
    }
    while ((TempLong.Long != 0x87654321) && (i != ETHERCAT_INIT_TIMEOUT));

    // if the time out expired, then initialization failed checking the byte order
    if (i == ETHERCAT_INIT_TIMEOUT)
        return 3;

    // Check the Ready flag
    // have a time-out feature to check for failure
    i = 0;
    do
    {
        i++;
        TempLong.Long = LAN9252ReadRegisterDirect(HW_CFG, 4);
    }
    while (((TempLong.Word[1] & READY) == 0) && (i != ETHERCAT_INIT_TIMEOUT));

    // if the time out expired, then initialization failed for not seeing the ready flag
    if (i == ETHERCAT_INIT_TIMEOUT)
        return 4;

    // Clear the output buffer to master
    for (i = 0; i < TOT_BYTE_NUM_IN; i++)
        etherCATOutputFrames->rawBytes[i] = 0;

    // Initialization completed successfully
    return 1;
}

/**
 * LAN9252ReadRegisterDirect
 * Reads the register of the LAN9252 given its address
 * @param address: the address of the register to read from
 * @param length: the number of bytes to read from the register address
 * @return: The value from the register
 */
uint32_t LAN9252ReadRegisterDirect(uint16_t address, uint16_t length)
{
    ULONG Result;
    uint16_t i;

    // SPI chip select enable
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, 0);

    // SPI read command
    SPI_Transfer(COMM_SPI_READ);

    // First byte of the register address
    SPI_Transfer(address >> 8);

    // Seond byte of the address
    SPI_Transfer(address);

    // Read the requested number of bytes
    for (i = 0; i < length; i++)
        Result.Byte[i] = SPI_Transfer(DUMMY_BYTE);

    // SPI chip select disable
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, GPIO_PIN_3);

    // return the result
    return Result.Long;
}

/**
 * LAN9252WriteRegisterDirect
 * Writes data directly to a register address on the LAN9252
 * @param address: the address of the register to write too
 * @param dataToSend: the data to send to the register
 */
void LAN9252WriteRegisterDirect(uint16_t address, uint32_t dataToSend)
{
    ULONG Data;
    Data.Long = dataToSend;

    // SPI chip select enable
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, 0);

    // SPI write command
    SPI_Transfer(COMM_SPI_WRITE);

    // First byte of the register address
    SPI_Transfer(address >> 8);

    // The second byte of the register address
    SPI_Transfer(address);

    // After the address is send, then send the data
    // to store in that register address
    SPI_Transfer(Data.Byte[0]);
    SPI_Transfer(Data.Byte[1]);
    SPI_Transfer(Data.Byte[2]);
    SPI_Transfer(Data.Byte[3]);

    // SPI chip select enable
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, GPIO_PIN_3);
}

/**
 * LAN9252ReadRegisterIndirect
 * Indirectly reads a register of the LAN9252 given an address.
 * This allows an operation to be completed silently until it is completed in which
 * a signal gets sent
 * @param address: the register address to read from
 * @param length: the number of bytes to read from the memory address
 */
uint32_t LAN9252ReadRegisterIndirect(uint16_t address, uint16_t length)
{
    ULONG TempLong;

    // Build the indirect command
    TempLong.Word[0] = address;
    TempLong.Word[1] = (length | (ESC_READ << 8));

    // Write the command
    LAN9252WriteRegisterDirect(ECAT_CSR_CMD, TempLong.Long);

    // Wait for command execution
    do
    {
        TempLong.Long = LAN9252ReadRegisterDirect(ECAT_CSR_CMD, 4);
    }
    // Run this loop until the completion signal is received
    while (TempLong.Word[1] & ECAT_CSR_BUSY);

    // Read the requested register
    TempLong.Long = LAN9252ReadRegisterDirect(ECAT_CSR_DATA, (length / 2));

    // Return the data
    return TempLong.Long;
}

/**
 * LAN9252WriteRegisterIndirect
 * Indirectly writes a register to an address on the LAN9252
 * This allows an operation to be completed silently until it is completed in which
 * a signal gets sent
 * @param address: the register address to write too
 * @param dataToSend: The data to send to the register
 * @param length: The length of the data to send
 */
void LAN9252WriteRegisterIndirect(uint16_t address, uint32_t dataToSend, uint16_t length)
{
    ULONG TempLong;

    // Write the data
    LAN9252WriteRegisterDirect(ECAT_CSR_DATA, dataToSend);

    // Build the indirect command
    TempLong.Word[0] = address;
    TempLong.Word[1] = (length | (ESC_WRITE << 8));

    // Write the command
    LAN9252WriteRegisterDirect(ECAT_CSR_CMD, TempLong.Long);

    // Wait for command execution
    do
    {
        TempLong.Long = LAN9252ReadRegisterDirect(ECAT_CSR_CMD, 2);
    }
    // Run this loop until the completion signal is received
    while (TempLong.Word[1] & ECAT_CSR_BUSY);
}

/**
 * LAN9252ReadProcRamFifo
 * Reads data from the output process RAM through the FIFO
 * This data is what is received from the etherCAT master
 * @param: etherCATInputFrames: a pointer to the union of possible
 * etherCAT input frames defined which is used to store the data
 * from the LAN9252
 */
void LAN9252ReadProcRamFifo(EtherCATFrames_IN *etherCATInputFrames)
{
    ULONG TempLong;
    unsigned char i;

#if TOT_BYTE_NUM_IN > 0

    // stop any possible pending transfers
    LAN9252WriteRegisterDirect(ECAT_PRAM_RD_CMD, PRAM_ABORT);

    // the high word is the number of bytes to read
    // the low word is the output process ram offset 0x----1000
    LAN9252WriteRegisterDirect(ECAT_PRAM_RD_ADDR_LEN,
                               (0x00001000 | (((uint32_t) TOT_BYTE_NUM_OUT) << 16)));

    // start command
    LAN9252WriteRegisterDirect(ECAT_PRAM_RD_CMD, 0x80000000);

    // Wait for the data to be transferred from the output process ram to the read FIFO.
    do
    {
        TempLong.Long = LAN9252ReadRegisterDirect(ECAT_PRAM_RD_CMD, 4);
    }
    while ((TempLong.Word[0] >> 8) != FST_BYTE_NUM_ROUND_IN / 4);

    // Enable SPI chip select
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, 0);

    // SPI read command
    SPI_Transfer(COMM_SPI_READ);

    // First byte of the read FIFO address
    SPI_Transfer(0x00);

    // Second most significant byte
    SPI_Transfer(0x00);

    // Receive data from EasyCat
    for (i = 0; i < FST_BYTE_NUM_ROUND_IN; i++)
        etherCATInputFrames->rawBytes[i] = SPI_Transfer(DUMMY_BYTE);

    // SPI chip select disable
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, GPIO_PIN_3);

#endif // for TOT_BYTE_NUM_IN > 0

// if there is more than 64 bytes to transfer
// then we must transfer the remaining bytes
#if SEC_BYTE_NUM_IN > 0

    // Enable SPI chip select
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, 0);

    // Wait for the data to be transferred from the output process ram to the read FIFO.
    do
    {
        TempLong.Long = LAN9252ReadRegisterDirect(ECAT_PRAM_RD_CMD, 4);
    }
    while ((TempLong.Word[0] >> 8) != SEC_BYTE_NUM_ROUND_IN / 4);

    // Enable SPI chip select
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, 0);

    // SPI read command
    SPI_Transfer(COMM_SPI_READ);

    // First byte of the read FIFO address
    SPI_Transfer(0x00);

    // Second most significant byte
    SPI_Transfer(0x00);

    // get the remaining bytes
    for (i = 0; i < (SEC_BYTE_NUM_ROUND_IN); i++)
        etherCATInputFrames->rawBytes[i + 64] = SPI_Transfer(DUMMY_BYTE);

    // SPI chip select disable
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, GPIO_PIN_3);

#endif  // endif for SEC_BYTE_NUM_IN
}

/**
 * LAN9252WriteProcRamFifo
 * Writes data to the input process ram through the fifo.
 * This is the data which will be send to the etherCAT mater
 * @param etherCATOutputFrames: a pointer to the union of possible
 * etherCAT output frames which will be sent to the etherCAT master
 */
void LAN9252WriteProcRamFifo(EtherCATFrames_OUT *etherCATOutputFrames)
{
    ULONG TempLong;
    unsigned char i;

#if TOT_BYTE_NUM_OUT > 0

    // stop any possible pending transfer
    LAN9252WriteRegisterDirect(ECAT_PRAM_WR_CMD, PRAM_ABORT);

    // the high word is the number of bytes
    // the low word is the input process ram offset
    LAN9252WriteRegisterDirect(ECAT_PRAM_WR_ADDR_LEN,
                               (0x00001200 | (((uint32_t) TOT_BYTE_NUM_IN) << 16)));

    // start command
    LAN9252WriteRegisterDirect(ECAT_PRAM_WR_CMD, 0x80000000);

    // Check that the fifo has enough free space
    do
    {
        TempLong.Long = LAN9252ReadRegisterDirect(ECAT_PRAM_WR_CMD, 4);
    }
    while ((TempLong.Word[0] >> 8) < (FST_BYTE_NUM_ROUND_OUT / 4));

    // Enable SPI chip select
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, 0);

    // Send SPI write command
    SPI_Transfer(COMM_SPI_WRITE);

    // First byte of the write FIFO address
    SPI_Transfer(0x00);

    // Second most significant byte
    SPI_Transfer(0x20);

    // Transfer data to the LAN9252
    for (i = 0; i < (FST_BYTE_NUM_ROUND_OUT - 1); i++)
        SPI_Transfer(etherCATOutputFrames->rawBytes[i]);

    // Transfer last byte
    SPI_Transfer(etherCATOutputFrames->rawBytes[i]);

    // Disable SPI chip select
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, GPIO_PIN_3);

#endif  // endif for TOT_BYTE_NUM_OUT

// if there are more than 64 bytes to transfer
// then we must transfer the remaining bytes
#if SEC_BYTE_NUM_OUT > 0

    // Enable SPI chip select
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, 0);

    // check that the fifo has enough free space
    do
    {
        TempLong.Long = LAN9252ReadRegisterDirect(ECAT_PRAM_WR_CMD, 4);
    }
    while ((TempLong.Word[0] >> 8) < (SEC_BYTE_NUM_ROUND_OUT / 4));

    // Enable SPI chip select
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, 0);

    // SPI write command
    SPI_Transfer(COMM_SPI_WRITE);

    // Address of the write FIFO
    SPI_Transfer(0x00);

    // Second most significant byte
    SPI_Transfer(0x20);

    // transfer the remaining bytes
    for (i = 0; i < (SEC_BYTE_NUM_ROUND_OUT - 1); i++)
        SPI_Transfer(etherCATOutputFrames->rawBytes[i + 64]);

    // send the last byte
    SPI_Transfer(etherCATOutputFrames->rawBytes[i + 64]);

    // SPI chip disable
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, GPIO_PIN_3);

#endif  // endif for SEC_BYTE_NUM_OUT

}

/**
 * LAN9252ReadRegisterIndirectOneByte
 * Indirectly reads one byte from the LAN9252 given an address.
 * This allows an operation to be completed silently until it is completed in which
 * a signal gets sent
 * @param address: the register address to read from
 */
uint32_t LAN9252ReadRegisterIndirectOneByte(uint16_t address)
{
    ULONG TempLong;
    ULONG Result;

    // Build the command
    TempLong.Word[0] = address;
    TempLong.Word[1] = (1 | (ESC_READ << 8));

    // Write the command
    LAN9252WriteRegisterDirect(ECAT_CSR_CMD, TempLong.Long);

    // Wait for command execution
    do
    {
        TempLong.Long = LAN9252ReadRegisterDirect(ECAT_CSR_CMD, 4);
    }
    while (TempLong.Word[1] & ECAT_CSR_BUSY);

    // SPI chip select enable
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, 0);

    // SPI read command
    SPI_Transfer(COMM_SPI_READ);

    // First byte of the register address
    SPI_Transfer(ECAT_CSR_DATA >> 8);

    // Second byte of the register address
    SPI_Transfer(ECAT_CSR_DATA);

    // Receive LsByte first
    uint16_t data = SPI_Transfer(DUMMY_BYTE);
    Result.Word[0] = data;

    // SPI chip select disable
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, GPIO_PIN_3);

    // return the result
    return Result.Long;

}

/**
 * SPI_Transfer
 * Transfer data to and from an SPI device
 * @param value: the value to send to the SPI device
 * @return: the value the SPI device returns as a result
 * of the send
 */
uint16_t SPI_Transfer(uint16_t value)
{
    // Send the data
    SSIDataPut(SSI3_BASE, value);

    // Get the result
    uint32_t rData;
    SSIDataGet(SSI3_BASE, &rData);

    // return the result
    return rData;
}
