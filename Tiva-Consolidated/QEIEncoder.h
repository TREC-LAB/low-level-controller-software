/*
 * QEIEncoder.h
 *
 *  Created on: Feb 11, 2022
 *      Author: n
 */

#ifndef QEIENCODER_H_
#define QEIENCODER_H_

#include "HAL/QEI_TIVA.h"

struct QEIEncoder
{
    uint32_t QEIBase;   // QEI0_BASE, QEI1_Base, etc.

    bool enabled;

    uint16_t sampleRate;

    uint32_t raw;
    int32_t speed;
    int32_t direction;
    int32_t rawVel;

    int32_t countsPerRotation;
};
typedef struct QEIEncoder QEIEncoder;

QEIEncoder qeiEncoderConstruct(uint32_t QEIBase, uint16_t sampleRate, int32_t countsPerRotation);
void enableQEIEncoder(QEIEncoder* encoder);
void disableQEIEncoder(QEIEncoder* encoder);
void readQEIEncoderPosition(QEIEncoder* encoder);
void readQEIEncoderVelocity(QEIEncoder* encoder);


#endif /* QEIENCODER_H_ */
