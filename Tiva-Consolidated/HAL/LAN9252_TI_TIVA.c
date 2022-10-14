//Author:       Hongxu Guo
//Organization: Virginia Tech
//Description:  This library is a modified version of the EasyCAT library
//              written by AB&T. This library is written for 32 bit
//              microcontrollers, especially for TI C2000 microcontrollers.


// Version:      2.0 - TIVA support
// Last modified 2/24/2021

// Modified by: Sam Schoedel and Nick Tremaroli


#include "LAN9252_TI_TIVA.h"

//initalize EtherCAT
//Errors are 2,3,4
//pass is 1

uint16_t EtherCAT_MainTask(EtherCATFrames_OUT* etherCATOutputFrames, EtherCATFrames_IN* etherCATInputFrames)
{
  uint16_t WatchDog = 0;
  unsigned Operational = 0;
  uint16_t i;
  ULONG TempLong;
  uint16_t Status;

  // Read the watchdog status
  TempLong.Long = LAN9252ReadRegisterIndirectOneByte (WDOG_STATUS);

  // set/reset the corrisponding flag
  if ((TempLong.Word[0] & 0x01) == 0x01)
    WatchDog = 0;
  else
    WatchDog = 1;

  // Read the EtherCAT State Machine status to see if we are in operational state
  TempLong.Long = LAN9252ReadRegisterIndirectOneByte (AL_STATUS);
  Status = TempLong.Word[0] & 0x0F;

  // set/reset the corresponding flag
  if (Status == ESM_OP)
    Operational = 1;
  else
    Operational = 0;

  // Reset the output buffer if watchdog is active or we are not in operational state,
  if (WatchDog == 1 | Operational == 0)
  {
    for (i=0; i < TOT_BYTE_NUM_OUT ; i++)
    {
      etherCATInputFrames->rawBytes[i] = 0;
    }
  }
  else
  {
    // Otherwise transfer process data from the EtherCAT core to the output buffer
    LAN9252ReadProcRamFifo(etherCATInputFrames);
  }

  // We always transfer process data from the input buffer to the EtherCAT core
  LAN9252WriteProcRamFifo(etherCATOutputFrames);

  // Return the status of the State Machine and of the watchdog
  if (WatchDog)
  {
    Status |= 0x80;
  }

  return Status;
}

uint16_t EtherCAT_Init(EtherCATFrames_OUT* etherCATOutputFrames)
{
  ULONG TempLong;
  uint16_t i;

  // LAN9252 reset
  LAN9252WriteRegisterDirect (RESET_CTL, DIGITAL_RST);

  // Reset timeout
  // Wait for reset to complete
  i = 0;
  do
  {
    i++;
    TempLong.Long = LAN9252ReadRegisterDirect (RESET_CTL, 4);
  } while (((TempLong.Word[0] & 0x01) != 0x00) && (i != ETHERCAT_INIT_TIMEOUT));

  // Time out expired
  if (i == ETHERCAT_INIT_TIMEOUT)
  {
    // Initialization failed
    return 2;
  }

  // Reset timeout
  // Check the Byte Order Test Register
  i = 0;
  do
  {
    i++;
    TempLong.Long = LAN9252ReadRegisterDirect (BYTE_TEST, 4);
  } while ((TempLong.Long != 0x87654321) && (i != ETHERCAT_INIT_TIMEOUT));

  // Time out expired
  if (i == ETHERCAT_INIT_TIMEOUT)
  {
    // Initialization failed
    return 3;
  }

  // Reset timeout
  // Check the Ready flag
  i = 0;
  do
  {
    i++;
    TempLong.Long = LAN9252ReadRegisterDirect (HW_CFG, 4);
  } while (((TempLong.Word[1] & READY) == 0) && (i != ETHERCAT_INIT_TIMEOUT));

  // Time out expired
  if (i == ETHERCAT_INIT_TIMEOUT)
  {
    // initialization failed
    return 4;
  }

  // Clear the input buffer
  for (i=0; i < TOT_BYTE_NUM_IN ; i++)
  {
    etherCATOutputFrames->rawBytes[i] = 0;
  }

  // Initalization completed
  return 1;
}

// Address = register to read
// Len = number of bytes to read (1,2,3,4)
//
// A long is returned but only the requested bytes are meaningful, starting from LsByte.
uint32_t LAN9252ReadRegisterDirect (uint16_t Address, uint16_t Len)
{
  ULONG Result;
  uint16_t i;

  // SPI chip select enable
  GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, 0);

  // SPI read command
  SPI_Transfer(COMM_SPI_READ);
  SPI_Transfer(Address>>8);    // First byte of the register addres
  SPI_Transfer(Address);       // Second (most significant) byte of the address

  // Read the requested number of bytes
  for (i=0; i<Len; i++)
  {
      Result.Byte[i] = SPI_Transfer(DUMMY_BYTE);
  }

  // SPI chip select disable
  GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, GPIO_PIN_3);

  return Result.Long;
}


//---- write a directly addressable registers  ----------------------------------------------------

// Address = register to write
// DataOut = data to write
void LAN9252WriteRegisterDirect (uint16_t Address, uint32_t DataOut)
{
  ULONG Data;
  Data.Long = DataOut;

  // SPI chip select enable
  GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, 0);

  // SPI write command
  SPI_Transfer(COMM_SPI_WRITE);
  SPI_Transfer(Address>>8);     // First byte of the register address
  SPI_Transfer(Address);        // Second (most significant) byte of the address
  // Write least significant byte first
  SPI_Transfer(Data.Byte[0]);
  SPI_Transfer(Data.Byte[1]);
  SPI_Transfer(Data.Byte[2]);
  SPI_Transfer(Data.Byte[3]);

  // SPI chip select enable
  GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, GPIO_PIN_3);
}

// Address = register to read
// Len = number of bytes to read (1,2,3,4)
//
// A long is returned but only the requested bytes are meaningful, starting from LsByte.
uint32_t LAN9252ReadRegisterIndirect (uint16_t Address, uint16_t Len)
{
  ULONG TempLong;

  // Compose the command
  TempLong.Word[0] = Address;
  TempLong.Word[1] = (Len | (ESC_READ << 8));

  // Write the command
  LAN9252WriteRegisterDirect (ECAT_CSR_CMD, TempLong.Long);

  // Wait for command execution
  do
  {
    TempLong.Long = LAN9252ReadRegisterDirect(ECAT_CSR_CMD,4);
  }
  while(TempLong.Word[1] & ECAT_CSR_BUSY);


  // read the requested register
  TempLong.Long = LAN9252ReadRegisterDirect(ECAT_CSR_DATA,(Len/2));
  return TempLong.Long;
}

// Address = register to write
// DataOut = data to write
void LAN9252WriteRegisterIndirect (uint32_t DataOut, uint16_t Address, uint16_t Len)
{
  ULONG TempLong;

  // Write the data
  LAN9252WriteRegisterDirect (ECAT_CSR_DATA, DataOut);

  // Compose the command
  TempLong.Word[0] = Address;
  TempLong.Word[1] = (Len | (ESC_WRITE << 8));

  // Write the command
  LAN9252WriteRegisterDirect (ECAT_CSR_CMD, TempLong.Long);

  // Wait for command execution
  do
  {
    TempLong.Long = LAN9252ReadRegisterDirect (ECAT_CSR_CMD, 2);
  }
  while (TempLong.Word[1] & ECAT_CSR_BUSY);
}

//---- read from process ram fifo ----------------------------------------------------------------
// Read data from the output process ram, through the FIFO.
// These are the bytes received from the EtherCAT master and
// that will be use by our application to write the outputs.
void LAN9252ReadProcRamFifo(EtherCATFrames_IN* etherCATInputFrames)
{
  ULONG TempLong;
  unsigned char i;


  #if TOT_BYTE_NUM_OUT > 0

    LAN9252WriteRegisterDirect (ECAT_PRAM_RD_CMD, PRAM_ABORT);        // abort any possible pending transfer

    LAN9252WriteRegisterDirect (ECAT_PRAM_RD_ADDR_LEN, (0x00001000 | (((uint32_t)TOT_BYTE_NUM_OUT) << 16)));
                                                                  // the high word is the num of bytes
                                                                  // to read 0xTOT_BYTE_NUM_OUT----
                                                                  // the low word is the output process
                                                                  // ram offset 0x----1000
    LAN9252WriteRegisterDirect (ECAT_PRAM_RD_CMD, 0x80000000);        // start command

                                                //------- one round is enough if we have ----
                                                //------- to transfer up to 64 bytes --------

    // Wait for the data to be transferred from the output process ram to the read FIFO.
    do
    {
      TempLong.Long = LAN9252ReadRegisterDirect (ECAT_PRAM_RD_CMD,4);
    }
    while ((TempLong.Word[0]>>8) != FST_BYTE_NUM_ROUND_OUT/4);

    // Enable SPI chip select
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, 0);

    // SPI read command
    SPI_Transfer(COMM_SPI_READ);
    SPI_Transfer(0x00);         // First byte of the read FIFO address
    SPI_Transfer(0x00);         // Second (most significant) byte

    // Receive data from EasyCat
    for (i=0; i< FST_BYTE_NUM_ROUND_OUT; i++)
    {
      etherCATInputFrames->rawBytes[i] = SPI_Transfer(DUMMY_BYTE);
    }

    // SPI chip select disable
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, GPIO_PIN_3);
  #endif


  #if SEC_BYTE_NUM_OUT > 0                    //-- if we have to transfer more then 64 bytes --
                                              //-- we must do another round -------------------
                                              //-- to transfer the remainig bytes -------------


    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, 0);
    do                                                          // wait for the data to be
    {                                                           // transferred from the output
      TempLong.Long = LAN9252ReadRegisterDirect(ECAT_PRAM_RD_CMD,4);// process ram to the read fifo
    }                                                           //
    while ((TempLong.Word[0]>>8) != SEC_BYTE_NUM_ROUND_OUT/4);       // *CCC*

    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, 0);

    SPI_Transfer(COMM_SPI_READ);                              // SPI read command
    SPI_Transfer(0x00);                                       // address of the read
    SPI_Transfer(0x00);                                   // fifo MsByte first

    for (i=0; i< (SEC_BYTE_NUM_ROUND_OUT); i++)                 // transfer loop for the remaining
    {                                                           // bytes
      etherCATInputFrames->rawBytes[i+64] = SPI_Transfer(DUMMY_BYTE);        // we transfer the second part of
    }                                                           // the buffer, so offset by 64

    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, GPIO_PIN_3);
  #endif
}

//---- write to the process ram fifo --------------------------------------------------------------

// Write data to the input process ram through the fifo.
// These are the bytes that we have read from the inputs of our
// application and that will be sent to the EtherCAT master.
void LAN9252WriteProcRamFifo(EtherCATFrames_OUT* etherCATOutputFrames)
{
  ULONG TempLong;
  unsigned char i;

  #if TOT_BYTE_NUM_IN > 0

    // abort any possible pending transfer
    LAN9252WriteRegisterDirect (ECAT_PRAM_WR_CMD, PRAM_ABORT);

    LAN9252WriteRegisterDirect (ECAT_PRAM_WR_ADDR_LEN, (0x00001200 | (((uint32_t)TOT_BYTE_NUM_IN) << 16)));
                                                                  // the high word is the num of bytes
                                                                  // to write 0xTOT_BYTE_NUM_IN----
                                                                  // the low word is the input process
                                                                  // ram offset  0x----1200

    LAN9252WriteRegisterDirect (ECAT_PRAM_WR_CMD, 0x80000000);        // start command

                                                //------- one round is enough if we have ----
                                                //------- to transfer up to 64 bytes --------

    // Check that the fifo has enough free space
    do
    {
      TempLong.Long = LAN9252ReadRegisterDirect (ECAT_PRAM_WR_CMD,4);
    }
    while ((TempLong.Word[0]>>8) <   (FST_BYTE_NUM_ROUND_IN/4));

    // Enable SPI chip select
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, 0);

    // Send SPI write command
    SPI_Transfer(COMM_SPI_WRITE);
    SPI_Transfer(0x00);           // First byte of the write FIFO address
    SPI_Transfer(0x20);           // Second (most significant) byte

    // Transfer data to EasyCat
    for (i=0; i< (FST_BYTE_NUM_ROUND_IN - 1 ); i++)
    {
        SPI_Transfer (etherCATOutputFrames->rawBytes[i]);
    }

    // Transfer last byte
    SPI_Transfer (etherCATOutputFrames->rawBytes[i]);

    // Disable SPI chip select
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, GPIO_PIN_3);
  #endif


  #if SEC_BYTE_NUM_IN > 0                     //-- if we have to transfer more then 64 bytes --
                                              //-- we must do another round -------------------
                                              //-- to transfer the remainig bytes -------------

    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, 0);
    do                                                          // check that the fifo has
    {                                                           // enough free space
      TempLong.Long = LAN9252ReadRegisterDirect(ECAT_PRAM_WR_CMD,4);//
    }                                                           //
    while ((TempLong.Word[0]>>8) < (SEC_BYTE_NUM_ROUND_IN/4));       //   *CCC*

    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, 0);

    SPI_Transfer(COMM_SPI_WRITE);                             // SPI write command
    SPI_Transfer(0x00);                                       // address of the write fifo
    SPI_Transfer(0x20);                                       // MsByte first

    for (i=0; i< (SEC_BYTE_NUM_ROUND_IN - 1); i++)              // transfer loop for the remaining
    {                                                           // bytes
      SPI_Transfer (etherCATOutputFrames->rawBytes[i+64]);                     // we transfer the second part of
    }                                                           // the buffer, so offset by 64
                                                                //
    SPI_Transfer (etherCATOutputFrames->rawBytes[i+64]);                   // one last byte

    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, GPIO_PIN_3);
  #endif
}

// Address = register to read
// Len = number of bytes to read (1,2,3,4)
//
// A long is returned but only the requested bytes are meaningful, starting from LsByte
uint32_t LAN9252ReadRegisterIndirectOneByte (uint16_t Address)
{
  ULONG TempLong;
  ULONG Result;

  // Compose the command
  TempLong.Word[0] = Address;
  TempLong.Word[1] = (1 | (ESC_READ << 8));

  // Write the command
  LAN9252WriteRegisterDirect (ECAT_CSR_CMD, TempLong.Long);

  // Wait for command execution
  do
  {
    TempLong.Long = LAN9252ReadRegisterDirect(ECAT_CSR_CMD,4);
  }
  while(TempLong.Word[1] & ECAT_CSR_BUSY);

  // SPI chip select enable
  GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, 0);

  // SPI read command
  SPI_Transfer(COMM_SPI_READ);
  SPI_Transfer(ECAT_CSR_DATA>>8); // First byte of register address
  SPI_Transfer(ECAT_CSR_DATA);    // Second byte of register address

  // Receive LsByte first
  uint16_t data = SPI_Transfer(DUMMY_BYTE);
  Result.Word[0] = data;

  // SPI chip select disable
  GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, GPIO_PIN_3);

  return Result.Long;

}

uint16_t SPI_Transfer(uint16_t Value)
{
    SSIDataPut(SSI3_BASE, Value);
    uint32_t rData;
    SSIDataGet(SSI3_BASE, &rData);
    return rData;
}
