/**
 * Encoder.h
 * @author: Nick Tremaroli and Sam Schoedel
 * Contains all of the structures and functions needed
 * to use an encoder on the Tiva
 */
#ifndef ENCODER_H
#define ENCODER_H

#include "HAL/SSI_TIVA.h"

// The different types of encoders
enum EncoderType
{
    SSI_Encoder,
    QEI_Encoder
};
typedef enum EncoderType EncoderType;

enum SSIEncoderBrand
{
    Gurley_Encoder,
    Orbis_Encoder
};
typedef enum SSIEncoderBrand SSIEncoderBrand;

/**
 * Encoder
 * Contains all of the data and features of
 * an encoder connected to the Tiva
 */
struct SSIEncoder
{
    enum SSIEncoderBrand encoderBrand;
    uint32_t SSIBase;   // SSI0_Base, SSI1_Base, etc.
    uint16_t sampleRate;
    bool enabled;

    // Data received from reading from the encoder
    uint32_t raw;
    uint32_t rawPrev;
    int32_t rawVelF;
    int32_t rawVelPrev;
    int32_t rawVel;

    float angleRads;
    float angleDegrees;

    // For encoder localization on a joint
    float jointReverse;
    float rawMax;
    float rawMin;
    float rawZero;
};
typedef struct SSIEncoder SSIEncoder;

// Constructs an encoder object
SSIEncoder encoderConstruct(uint32_t SSIBase, uint16_t sampleRate, SSIEncoderBrand encoderBrand);
// Enable or Disable the encoder
void enableSSIEncoder(SSIEncoder* encoder);
void disableSSIEncoder(SSIEncoder* encoder);
// Read data from the abs/motor encoder
void readAbsEncoder(SSIEncoder* encoder);
void readAbsEncoderVelocity(SSIEncoder* encoder);

#endif /* ENCODER_H */
