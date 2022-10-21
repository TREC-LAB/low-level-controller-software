/**
 * CAN_TI_TIVA.c
 * @author: Nick Tremaroli
 * Contains all of the low-level code for CAN communication
 * on the TIVA
 */

#include "CAN_TI_TIVA.h"

/**
 * ReadCAN
 * read the can data from the CAN Bus given the Rx
 * @param canMsgRx: a pointer to the CANBUSRx data structure
 * to which the CAN data to be read will be stored
 */
void ReadCAN(CANBUSRx* canMsgRx)
{
    // data gets stored in canMgRx->msgRx
    CANMessageGet(CAN0_BASE, canMsgRx->canObjId, &canMsgRx->msgRx, 1);
}

/**
 * BinaryTwoDecimal
 * Converts two unsigned bytes to a decimal number.
 * This decimal number is then casted to a float
 * @param com0: the Least Significant Byte (LSB) of data to convert
 * @param com1: the Most Significant Byte (MSB) of data to convert
 * @return: the decimal value after conversion casted to a float
 */
float BinaryToDecimal(unsigned char com0, unsigned char com1)
{
    int output = 0;
    // combine the LSB and the MSB
    int conct = (com1 << 8 | com0);

    // account for the negative case
    const int negative = ( conct & (1 << 15)) != 0;
    if (negative)
        output = conct | ~((1 << 15) - 1);
    else
        output = conct;

    // return by casting the output to a float
    return (float) output;
}

/**
 * CANInitial
 * Initialize CAN Base 0 for communication using
 * Pins B4 and B5
 * @param canRate: The rate at which CAN will operate
 */
void CANInitial(uint32_t canRate)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    GPIOPinConfigure(GPIO_PB4_CAN0RX);
    GPIOPinConfigure(GPIO_PB5_CAN0TX);
    GPIOPinTypeCAN(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_5);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN0);
    CANInit(CAN0_BASE);
    CANBitRateSet(CAN0_BASE, SysCtlClockGet(), canRate); // set CAN communication rate to 500kbps, same as ATI CAN rate
    CANEnable(CAN0_BASE);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_CAN0));
}

/**
 * CANRx0Initial
 * Initializes the receive object number 0 for the CAN bus
 * @param msgRx0: a pointer to the CANBUSRx object which stores all
 * of the receive settings
 * @param canID: The CAN message identifier
 * @param canObj: The CAN object ID
 */
void CANRx0Initial(CANBUSRx* msgRx0, uint16_t canID, uint16_t canObj)
{
    // setup this CAN Object
    msgRx0->canObjId = canObj;

    // the ID of the board
    msgRx0->msgRx.ui32MsgID = canID;
    msgRx0->msgRx.ui32MsgIDMask = canID;

    //flag this object as Rx and use Mask
    msgRx0->msgRx.ui32Flags = MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER;

    msgRx0->msgRx.ui32MsgLen = 8;
    CANMessageSet(CAN0_BASE, msgRx0->canObjId, &msgRx0->msgRx, MSG_OBJ_TYPE_RX);
    msgRx0->msgRx.pui8MsgData = msgRx0->msgRxData;
}

/**
 * CANRx1Initial
 * Initializes the receive object number 1 for the CAN bus
 * @param msgRx1: a pointer to the CANBUSRx object which stores all
 * of the receive settings
 * @param canID: The CAN message identifier
 * @param canObj: The CAN object ID
 */
void CANRx1Initial(CANBUSRx* msgRx1, uint16_t canID, uint16_t canObj)
{
    // setup this CAN Object
    msgRx1->canObjId = canObj;

    // the ID of the board
    msgRx1->msgRx.ui32MsgID = canID;
    msgRx1->msgRx.ui32MsgIDMask = canID;

    //flag this object as Rx and use Mask
    msgRx1->msgRx.ui32Flags = MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER;

    msgRx1->msgRx.ui32MsgLen = 8;
    CANMessageSet(CAN0_BASE, canObj, &msgRx1->msgRx, MSG_OBJ_TYPE_RX);
    msgRx1->msgRx.pui8MsgData = msgRx1->msgRxData;
}

/**
 * CANTxInitial
 * Initializes the transer object for the CAN bus
 * @param msgTx0: a pointer to the CANBUSTx object which stores all
 * of the receive settings
 * @param canID: The CAN message identifier
 * @param canObj: The CAN object ID
 */
void CANTxInitial(CANBUSTx* msgTx0, uint16_t canID, uint16_t canObj)
{
    // setup this CAN Object
    msgTx0->canObjId = canObj;

    // setup the data pointer
    msgTx0->msgTxDataPtr = (unsigned char *)msgTx0->msgTxData;

    // the ID of the board
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

/**
 * CANSend
 * Sends CAN data to the tx object
 * @param msgTx0: a pointer to the Tx object to send data too
 */
void CANSend(CANBUSTx* msgTx0)
{
    CANMessageSet(CAN0_BASE, msgTx0->canObjId, &msgTx0->msgTx, MSG_OBJ_TYPE_TX);
}
