#include "QEIEncoder.h"

QEIEncoder qeiEncoderConstruct(uint32_t QEIBase, uint16_t sample_rate, int32_t countsPerRotation)
{
    QEIEncoder encoder;

    encoder.QEIBase = QEIBase;
    encoder.sampleRate = sample_rate;

    encoder.enabled = false;

    encoder.raw = 0;
    encoder.speed = 0;
    encoder.direction = 0;
    encoder.rawVel = 0;

    encoder.countsPerRotation = countsPerRotation;

    return encoder;
}

void enableQEIEncoder(QEIEncoder* encoder)
{
    if(encoder->QEIBase == QEI0_BASE)
        QEIConfig0();
    else if(encoder->QEIBase == QEI1_BASE)
        QEIConfig1();

    encoder->enabled = true;
}

void readQEIEncoderPosition(QEIEncoder* encoder)
{
    encoder->raw = QEIPositionGet(encoder->QEIBase);
}

void readQEIEncoderVelocity(QEIEncoder* encoder)
{
    encoder->speed = QEIVelocityGet(encoder->QEIBase);

    encoder->direction = QEIDirectionGet(encoder->QEIBase);
    encoder->rawVel = encoder->direction*encoder->speed*encoder->sampleRate/(encoder->countsPerRotation * 4);
}
