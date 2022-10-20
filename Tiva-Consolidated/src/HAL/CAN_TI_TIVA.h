/**
 * CAN_TI_TIVA.h
 * @author: Nick Tremaroli
 * Contains the layout and functions regarding CAN communication
 * on the Tiva
 */

#ifndef MYCAN_H_
#define MYCAN_H_

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_can.h"
#include "inc/hw_ints.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/pin_map.h"
#include "driverlib/can.h"
#include "utils/uartstdio.h"

/**
 * CANBUSTx
 * A struct containing all of the data necessary to
 * send values via CAN
 */
struct CANBUSTx
{
    tCANMsgObject msgTx;
    unsigned char msgTxData;
    unsigned char* msgTxDataPtr;
    uint32_t canObjId;
};
typedef struct CANBUSTx CANBUSTx;

/**
 * CANBUSRx
 * A struct containing all of the data necessary to
 * receive values via CAN
 */
struct CANBUSRx
{
    tCANMsgObject msgRx;
    unsigned char msgRxData[8];
    uint32_t canObjId;
};
typedef struct CANBUSRx CANBUSRx;

// reads the data through the respective CAN Rx
void ReadCAN(CANBUSRx* canMsgRx);

// Initialize CAN using the canRate provided
void CANInitial(uint32_t canRate);

// Initialize Rx0 of the CAN BUS
void CANRx0Initial(CANBUSRx* msgRx0, uint16_t canID, uint16_t canObj);

// Initialize Rx1 of the CAN BUS
void CANRx1Initial(CANBUSRx* msgRx1, uint16_t canID, uint16_t canObj);

// Initialize the tranfer  of the CAN BUS
void CANTxInitial(CANBUSTx* msgTx0, uint16_t canID, uint16_t canObj);

// send data to the CAN BUS
void CANSend(CANBUSTx* msgTx0);

// convert the binary data from the CAN bus to decimal data
float BinaryToDecimal(unsigned char com0, unsigned char com1);

#endif /* MYCAN_H_ */
