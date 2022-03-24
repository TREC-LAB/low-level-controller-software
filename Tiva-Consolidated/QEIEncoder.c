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

    encoder.QEIBase = QEIBase;
    encoder.sampleRate = sampleRate;

    encoder.enabled = false;

    encoder.raw = 0;
    encoder.speed = 0;
    encoder.direction = 0;
    encoder.rawVel = 0;

    encoder.countsPerRotation = countsPerRotation;

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
    if(encoder->QEIBase == QEI0_BASE)
        QEIConfig0();
    else if(encoder->QEIBase == QEI1_BASE)
        QEIConfig1();

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
    encoder->speed = QEIVelocityGet(encoder->QEIBase);

    encoder->direction = QEIDirectionGet(encoder->QEIBase);
    encoder->rawVel = encoder->direction*encoder->speed*encoder->sampleRate/(encoder->countsPerRotation * 4);
}
