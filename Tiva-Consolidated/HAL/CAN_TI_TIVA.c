/*
 * myCAN.c
 *
 *  Created on: May 11, 2021
 *      Author: Chi
 */


#include "CAN_TI_TIVA.h"


void ReadCAN(CANBUSRx* msgRx0, CANBUSRx* msgRx1)
{
    uint32_t status = CANStatusGet(CAN0_BASE, CAN_STS_CONTROL);

    CANMessageGet(CAN0_BASE, 2, &msgRx0->msgRx, 1);
    CANMessageGet(CAN0_BASE, 3, &msgRx1->msgRx, 1);
}

float binTwoCToDec(unsigned char com0, unsigned char com1)
{
    //this is the real money
    int output;
    int conct = (com1 << 8 | com0);
    const int negative = ( conct & (1 << 15)) != 0;
    if (negative)
        output = conct | ~((1 << 15) - 1);
    else
        output = conct;

    return (float) output;
}


void CANInitial(uint32_t canRate)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE); // enable GPIOE peripheral for CAN0
    GPIOPinConfigure(GPIO_PE4_CAN0RX);
    GPIOPinConfigure(GPIO_PE5_CAN0TX);
    GPIOPinTypeCAN(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_CAN0)){
    }
    CANInit(CAN0_BASE);
    CANBitRateSet(CAN0_BASE, SysCtlClockGet(), canRate); // set CAN communication rate to 500kbps, same as ATI CAN rate
//    CANIntRegister(CAN0_BASE, CANIntHandler); // use dynamic vector table allocation
//    CANIntEnable(CAN0_BASE, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);
//    IntEnable(INT_CAN0);
    CANEnable(CAN0_BASE);
}

void CANRx0Initial(CANBUSRx* msgRx0, uint16_t canID, uint16_t canObj){
    // Set up msg object 2
    msgRx0->msgRx.ui32MsgID = 0x1b5; //ID of the board
    msgRx0->msgRx.ui32MsgIDMask = 0x1b5;
    msgRx0->msgRx.ui32Flags = MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER; //flag this object as Rx and use Mask
    msgRx0->msgRx.ui32MsgLen = 8;
    CANMessageSet(CAN0_BASE, canObj, &msgRx0->msgRx, MSG_OBJ_TYPE_RX);
    msgRx0->msgRx.pui8MsgData = msgRx0->msgRxData;

}

void CANRx1Initial(CANBUSRx* msgRx1, uint16_t canID, uint16_t canObj)
{
    // Set up msg object 3
    msgRx1->msgRx.ui32MsgID = 0x1b6; //ID of the board
    msgRx1->msgRx.ui32MsgIDMask = 0x1b6;
    msgRx1->msgRx.ui32Flags = MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER; //flag this object as Rx and use Mask
    msgRx1->msgRx.ui32MsgLen = 8;
    CANMessageSet(CAN0_BASE, canObj, &msgRx1->msgRx, MSG_OBJ_TYPE_RX);
    msgRx1->msgRx.pui8MsgData = msgRx1->msgRxData;
}

void CANRxInitial(uint32_t msgId, uint32_t msgIdMas)
{

}

void CANTxInitial(CANBUSTx* msgTx0, uint16_t canID, uint16_t canObj){
    // Set up msg object
//    msgTxData = 0;
    msgTx0->msgTxDataPtr = (unsigned char *)msgTx0->msgTxData;
    msgTx0->msgTx.ui32MsgID = canID;
    msgTx0->msgTx.ui32MsgIDMask = canObj;
    msgTx0->msgTx.ui32Flags = MSG_OBJ_TX_INT_ENABLE;
    msgTx0->msgTx.ui32MsgLen = sizeof(msgTx0->msgTxDataPtr);
    msgTx0->msgTx.pui8MsgData = msgTx0->msgTxDataPtr;
    msgTx0->msgTxDataPtr[0] = 0x02; //This configures a frame for asking force data from NET FT board.
    msgTx0->msgTxDataPtr[1] = 0x00;
    msgTx0->msgTxDataPtr[2] = 0x00;
    msgTx0->msgTxDataPtr[3] = 0x00;
}

void CANSend(CANBUSTx* msgTx0, uint16_t canID, uint16_t canObj)
{
    CANMessageSet(CAN0_BASE, canObj, &msgTx0->msgTx, MSG_OBJ_TYPE_TX);
}
