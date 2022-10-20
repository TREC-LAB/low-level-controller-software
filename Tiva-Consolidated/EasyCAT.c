/**
 * EasyCAT.c
 * @author: Nick Tremaroli
 * Contains the low-level code for EasyCAT
 * related functions
 */

#include "EasyCAT.h"

/**
 * EasyCAT_Init
 * Initializes the EasyCAT board
 * @param etherCATOutputFrames: a pointer to the output data frame structure
 * @return: 1 on success. 2 on reset time-out failure. 3 on byte order timeout failure.
 * 4 on ready flag timeout failure
 */
uint16_t EasyCAT_Init(EtherCATFrames_OUT *etherCATOutputFrames)
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
 * EasyCAT_MainTask
 * Sends and receives new raw data to and from the master computer respectively
 * @param etherCATOutputFrames: the output data frame structure to send to the master
 * @param etherCATInputFrames: the input data frame structure which is populated by the master
 * @return: the status of the EasyCAT
 */
uint16_t EasyCAT_MainTask(EtherCATFrames_OUT *etherCATOutputFrames, EtherCATFrames_IN *etherCATInputFrames)
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
 * GetAndSendDataToMaster
 * Gets the latest data from the master computer and
 * sends the latest data to the master computer
 * @param easyCAT: a pointer to the easyCAT which is used
 * to exchange data to and from the master
 */
void GetAndSendDataToMaster(EasyCAT* easyCAT)
{
    // call the easyCAT's main task
    EasyCAT_MainTask(&easyCAT->etherCATOutputFrames, &easyCAT->etherCATInputFrames);
}



