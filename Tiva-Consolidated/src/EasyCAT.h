/**
 * EasyCAT.h
 * @author: Nick Tremaroli
 * Contains the layout and functions related to
 * the EasyCAT board
 */

#ifndef EASYCAT_H_
#define EASYCAT_H_

#include "HAL/LAN9252_TI_TIVA.h"

/**
 * EasyCAT
 * Contains all of the data and structures needed
 * by the EasyCAT board
 */
struct EasyCAT
{
    // for communication with the master computer
    EtherCATFrames_IN etherCATInputFrames;
    EtherCATFrames_OUT etherCATOutputFrames;
};
typedef struct EasyCAT EasyCAT;

// Initialize the EasyCAT board
uint16_t EasyCAT_Init(EtherCATFrames_OUT *etherCATOutputFrames);

// run the data transfer to and from the master computer
uint16_t EasyCAT_MainTask(EtherCATFrames_OUT *etherCATOutputFrames, EtherCATFrames_IN *etherCATInputFrames);

// data transfer to the master computer
void GetAndSendDataToMaster(EasyCAT* easyCAT);

#endif
