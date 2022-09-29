/*
 * myCAN.h
 *
 *  Created on: May 11, 2021
 *      Author: Chi
 */

#ifndef MYCAN_H_
#define MYCAN_H_

#include <stdbool.h>
#include <stdint.h>
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

struct CANBUSTx
{
    tCANMsgObject msgTx;
    unsigned char msgTxData;
    unsigned char* msgTxDataPtr;
};
typedef struct CANBUSTx CANBUSTx;

struct CANBUSRx
{
    tCANMsgObject msgRx;
    unsigned char msgRxData[8];
};
typedef struct CANBUSRx CANBUSRx;

//extern void CANIntHandler(void);
//UART & GPIO Initialization for CAN BUS
extern void CANuartInitial(void);
void ReadCAN(CANBUSRx* msgRx0, CANBUSRx* msgRx1);
extern void CANInitial(uint32_t canRate);//rate has to be 500,000(500kbps) to match ATI board, unlikely to change this
extern void CANRx0Initial(CANBUSRx* msgRx0, uint16_t canID, uint16_t canObj); // this set your Tiva to Receive, specify ID and Object number
extern void CANRx1Initial(CANBUSRx* msgRx1, uint16_t canID, uint16_t canObj); // this set your Tiva to Receive, specify ID and Object number
extern void CANTxInitial(CANBUSTx* msgTx0, uint16_t canID, uint16_t canObj);
extern unsigned char * myCANRx(uint16_t canObj);
extern void CANSend(CANBUSTx* msgTx0, uint16_t canID, uint16_t canObj);
extern void tare(void);
//extern void readFTSensor(FTSensor* ftSensor);
void storeForce(void);

float binTwoCToDec(unsigned char com0, unsigned char com1);


//FTSensor FTSensorConfig(void);

#endif /* MYCAN_H_ */
