/*
 * myCAN.c
 *
 *  Created on: May 11, 2021
 *      Author: Chi
 */


#include "CAN_TI_TIVA.h"

/*tCANMsgObject msgTx; // the CAN message object << is this accessible by main c?
tCANMsgObject msgRx0;
tCANMsgObject msgRx1;
unsigned char msgRx0Data[8]; // 8 byte buffer for rx message data
unsigned char msgRx1Data[8]; // 8 byte buffer for rx message data
unsigned char msgTxData; //create a buffer for Tx msg
unsigned char *msgTxDataPtr = (unsigned char *)&msgTxData; */
float fx,fy,fz,tx,ty,tz; // buffer for FT
float tarefx,tarefy,tarefz,taretx,tarety,taretz;
bool TareFlag = 0;
/*
void CANIntHandler(void)
{
    //UARTprintf("counter is %d",counter);
    unsigned long cause;
    unsigned long status = CANIntStatus(CAN0_BASE, CAN_INT_STS_CAUSE); // read interrupt status
    if(status == CAN_INT_INTID_STATUS)
    {
        cause = CANStatusGet(CAN0_BASE, CAN_STS_CONTROL); // read interrupt cause
        if((cause & CAN_STATUS_RXOK)==CAN_STATUS_RXOK)
        {
            CANMessageGet(CAN0_BASE, 2, &msgRx0, 1);
            CANMessageGet(CAN0_BASE, 3, &msgRx1, 1);
            if((msgRx0.ui32Flags & MSG_OBJ_NEW_DATA)==MSG_OBJ_NEW_DATA)
            {
                           //UARTprintf("Received data from ID:0x%x, Data: ,\tLen: %d,\tMessage:0x%X,0x%X,0x%X,0x%X,0x%X,0x%X,0x%X,0x%X\n", msgRx0.ui32MsgID, msgRx0.ui32MsgLen, msgRx0Data[0],msgRx0Data[1],msgRx0Data[2],msgRx0Data[3],msgRx0Data[4],msgRx0Data[5],msgRx0Data[6],msgRx0Data[7]);
            }
            else if((msgRx0.ui32Flags & MSG_OBJ_DATA_LOST)==MSG_OBJ_DATA_LOST)
            {
                //UARTprintf("object 2 data lost");
                CANMessageClear(CAN0_BASE, 2);
            }
            if((msgRx1.ui32Flags & MSG_OBJ_NEW_DATA)==MSG_OBJ_NEW_DATA)
            {
              //UARTprintf("Received data from ID:0x%x, Data: ,\tLen: %d,\tMessage:0x%X,0x%X,0x%X,0x%X,0x%X,0x%X,0x%X,0x%X\n", msgRx1.ui32MsgID, msgRx1.ui32MsgLen, msgRx1Data[0],msgRx1Data[1],msgRx1Data[2],msgRx1Data[3],msgRx1Data[4],msgRx1Data[5],msgRx1Data[6],msgRx1Data[7]);
              //storeForce();
            }
            else if((msgRx1.ui32Flags & MSG_OBJ_DATA_LOST)==MSG_OBJ_DATA_LOST)
            {
                //UARTprintf("object 3 data lost");
                CANMessageClear(CAN0_BASE, 3);
            }
        }
        else if(cause == CAN_STATUS_TXOK)
        {
           //UARTprintf("TxOK Message sent!\n");
           CANIntClear(CAN0_BASE,1);
        }
        else if(cause == CAN_STATUS_LEC_STUFF)
        {
//            printf("BIT STUFF\n");
        }
        else if(cause == CAN_STATUS_LEC_ACK)
        {
//            printf("Connection Lost\n");
        }
    }
    else
    {
        CANIntClear(CAN0_BASE, 1);
        CANIntClear(CAN0_BASE, 2);
        CANIntClear(CAN0_BASE, 3);
        CANMessageClear(CAN0_BASE, 2);
        CANMessageClear(CAN0_BASE, 3);
    }
    storeForce();
}
*/

void ReadCAN(CANBUSRx* msgRx0, CANBUSRx* msgRx1)
{
    uint32_t status = CANStatusGet(CAN0_BASE, CAN_STS_CONTROL);
//    CANMessageClear(CAN0_BASE, 2);
//    CANMessageClear(CAN0_BASE, 3);
    CANMessageGet(CAN0_BASE, 2, &msgRx0->msgRx, 1);
    CANMessageGet(CAN0_BASE, 3, &msgRx1->msgRx, 1);
    return;
    if((status & CAN_STATUS_RXOK) == CAN_STATUS_RXOK)
    {
        CANMessageGet(CAN0_BASE, 2, &msgRx0->msgRx, 1);
        CANMessageGet(CAN0_BASE, 3, &msgRx1->msgRx, 1);
        if((msgRx0->msgRx.ui32Flags & MSG_OBJ_DATA_LOST)==MSG_OBJ_DATA_LOST)
        {
            printf("Clearing message\n");
            CANMessageClear(CAN0_BASE, 2);
        }
        else if((msgRx1->msgRx.ui32Flags & MSG_OBJ_DATA_LOST)==MSG_OBJ_DATA_LOST)
        {
            printf("clearing message 3\n");
            CANMessageClear(CAN0_BASE, 3);
        }
    }
    else if(status == CAN_STATUS_TXOK)
    {
       CANIntClear(CAN0_BASE,1);
    }
    else
    {
        CANMessageClear(CAN0_BASE, 2);
        CANMessageClear(CAN0_BASE, 3);
    }
}

/*
FTSensor FTSensorConfig(void){

    FTSensor ftSensor;

    ftSensor.forceX = 0.0;
    ftSensor.forceY = 0.0;
    ftSensor.forceZ = 0.0;
    ftSensor.torqueX = 0.0;
    ftSensor.torqueY = 0.0;
    ftSensor.torqueZ = 0.0;

    return ftSensor;
}
*/
/*
void tare()
{
    tarefx = -1.0 * fx;
    tarefy = -1.0 * fy;
    tarefz = -1.0 * fz;
    taretx = -1.0 * tx;
    tarety = -1.0 * ty;
    taretz = -1.0 * tz;
    TareFlag = 1;
}
*/
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

/*
void storeForce(void)
{
    float torqueFactor, forceFactor;
    torqueFactor = 1000000 / 611; // This factor is hardware configuration in the Net FT board, connect the board through ethernet to check the configuration, make this as a fucntion such that it can be configured for left and right sensor
    forceFactor = 1000000 / 35402;
//    torqueFactor = 100000 / 611;
//    forceFactor = 100000 / 35402;
    fx = binTwoCToDec(msgRx0Data[0], msgRx0Data[1])/forceFactor;
    tx = binTwoCToDec(msgRx0Data[2], msgRx0Data[3])/torqueFactor;
    fy = binTwoCToDec(msgRx0Data[4], msgRx0Data[5])/forceFactor;
    ty = binTwoCToDec(msgRx0Data[6], msgRx0Data[7])/torqueFactor;
    fz = binTwoCToDec(msgRx1Data[0], msgRx1Data[1])/forceFactor;
    tz = binTwoCToDec(msgRx1Data[2], msgRx1Data[3])/torqueFactor;

    printf("0: %d\n", msgRxData[0]);
    printf("1: %d\n", msgRxData[1]);

    if((fz != 0) && (TareFlag != 1))
    {
        tare();
    }
}
*/
/*void readFTSensor(FTSensor* ftSensor)
{
    ftSensor->forceX = fx + tarefx;
    ftSensor->forceY = fy + tarefy;
    ftSensor->forceZ = fz + tarefz;
    ftSensor->torqueX = tx + taretx;
    ftSensor->torqueY = ty + tarety;
    ftSensor->torqueZ = tz + taretz;
}*/


//void myCANuartInitial(void){ // This function shouldn't be called
//    //SysCtlClockSet(SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ|SYSCTL_SYSDIV_2_5);
//    //SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ); //80Mhz
//    SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN); //80Mhz
//    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA); // enable UART0 GPIO peripheral
//    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
//    GPIOPinConfigure(GPIO_PA0_U0RX);
//    GPIOPinConfigure(GPIO_PA1_U0TX);
//    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
//    UARTStdioConfig(0, 115200, SysCtlClockGet()); // 115200 baud
//    //sysTickInitial();
//}


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
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_CAN0)){
    }
//    CANIntRegister(CAN0_BASE, CANIntHandler); // use dynamic vector table allocation
//    CANIntEnable(CAN0_BASE, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);
//    IntEnable(INT_CAN0);
}

void CANRx0Initial(CANBUSRx* msgRx0, uint16_t canID, uint16_t canObj){
    // Set up msg object 2
    msgRx0->msgRx.ui32MsgID = 0x1b5; //ID of the board
    msgRx0->msgRx.ui32MsgIDMask = 0x1b5;
    msgRx0->msgRx.ui32Flags = MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER; //flag this object as Rx and use Mask
    msgRx0->msgRx.ui32MsgLen = 8;
    CANMessageSet(CAN0_BASE, canObj, &msgRx0->msgRx, MSG_OBJ_TYPE_RX);
    msgRx0->msgRx.pui8MsgData = msgRx0->msgRxData;

//    msgRx0.ui32MsgID =0; //ID of the board
//    msgRx0.ui32MsgIDMask = 0;
//    msgRx0.ui32Flags = MSG_OBJ_RX_INT_ENABLE; //flag this object as Rx and use Mask
//    msgRx0.ui32MsgLen = 8;
//    CANMessageSet(CAN0_BASE, canObj, &msgRx0, MSG_OBJ_TYPE_RX);
//    msgRx0.pui8MsgData = msgRx0Data;

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
