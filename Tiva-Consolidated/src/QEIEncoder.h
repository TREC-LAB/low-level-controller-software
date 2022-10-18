/*
 * QEIEncoder.h
 * @author: Nick Tremaroli
 * Contains the layout and functions regarding a QEI Encoder
 */

#ifndef QEIENCODER_H_
#define QEIENCODER_H_

#include "HAL/QEI_TIVA.h"

/**
 * QEIEncoder
 * Contains all of the data needed by a QEI encoder
 * on the Tiva
 */
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

// construct the QEI encoder
QEIEncoder qeiEncoderConstruct(uint32_t QEIBase, uint16_t sampleRate, int32_t countsPerRotation);

// enable the QEI encoder
void enableQEIEncoder(QEIEncoder* encoder);

// get the position of the QEI encoder
void readQEIEncoderPosition(QEIEncoder* encoder);

// get the velocity of the QEI encoder
void readQEIEncoderVelocity(QEIEncoder* encoder);

#endif /* QEIENCODER_H_ */
