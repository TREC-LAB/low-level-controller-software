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
   // QEI0_BASE, QEI1_Base, etc.
    uint32_t QEIBase;

    // a flag to determine if the
    // encoder is enabled
    bool enabled;

    // the sample rate of the QEI encoder
    uint16_t sampleRate;

    // the raw value of the encoder
    uint32_t raw;

    // the speed of the encoder
    int32_t speed;

    // the direction of the encoder
    int32_t direction;

    // the raw velocity of the encoder
    int32_t rawVel;

    // the counts per rotation of the encoder
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
