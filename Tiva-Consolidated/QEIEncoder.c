/**
 * QEIEncoder.c
 * @author: Nick Tremaroli
 * Contains all of the low-level code for QEI encoder related functions
 */
#include "QEIEncoder.h"

/**
 * qeiEncoderConstruct
 * Construct a QEI encoder given the corresponding input parameters
 *
 * @param QEIBase: the QEI Base of the encoder
 * @param sampleRate: the sample rate of the QEI encoder
 * @param countsPerRotation: the counts per rotation of the QEI encoder
 *
 * @return: an initialized QEI encoder structure
 */
QEIEncoder qeiEncoderConstruct(uint32_t QEIBase, uint16_t sampleRate, int32_t countsPerRotation)
{
    QEIEncoder encoder;

    // set the QEI Base accordingly
    encoder.QEIBase = QEIBase;

    // set the sample rate accordingly
    encoder.sampleRate = sampleRate;

    // set the counts per rotation accordingly
    encoder.countsPerRotation = countsPerRotation;

    // by default the encoder is disabled
    encoder.enabled = false;

    // initialize the rest of the values to 0
    encoder.raw = 0;
    encoder.speed = 0;
    encoder.direction = 0;
    encoder.rawVel = 0;

    // return the new encoder
    return encoder;
}

/**
 * enableQEIEncoder
 * enables the QEI Encoder
 *
 * @param encoder: a pointer to the QEI encoder to enable
 */
void enableQEIEncoder(QEIEncoder* encoder)
{
    // configure the encoder based on what QEI
    // Base it is on
    if(encoder->QEIBase == QEI0_BASE)
        QEIConfig0();
    else if(encoder->QEIBase == QEI1_BASE)
        QEIConfig1();

    // set the enabled flag to true
    encoder->enabled = true;
}

/**
 * readQEIEncoderPosition
 * reads the raw encoder position
 *
 * @param encoder: a pointer to the QEI encoder to read its position
 */
void readQEIEncoderPosition(QEIEncoder* encoder)
{
    // store the raw value of the encoder
    encoder->raw = QEIPositionGet(encoder->QEIBase);
}

/**
 * readQEIEncoderVelocity
 * reads the velocity of the QEI Encoder
 *
 * @param encoder: a pointer to the QEI encoder to read its velocity
 */
void readQEIEncoderVelocity(QEIEncoder* encoder)
{
    // get the speed of the encoder
    encoder->speed = QEIVelocityGet(encoder->QEIBase);

    // get the direction of the encoder
    encoder->direction = QEIDirectionGet(encoder->QEIBase);

    // get the raw velocity of the encoder
    encoder->rawVel = encoder->direction*encoder->speed*encoder->sampleRate/(encoder->countsPerRotation * 4);
}
