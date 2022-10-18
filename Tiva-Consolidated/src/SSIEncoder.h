/**
 * SSIEncoder.h
 * @author: Nick Tremaroli
 * Contains the layout and functions regarding a SSI Encoder
 */
#ifndef ENCODER_H
#define ENCODER_H

#include "HAL/SSI_TIVA.h"

/**
 * SSIEncoderBrand
 * The different SSI encoder brands which are
 * used on the Tivas
 */
enum SSIEncoderBrand
{
    Gurley_Encoder,
    Orbis_Encoder
};
typedef enum SSIEncoderBrand SSIEncoderBrand;

/**
 * SSIEncoder
 * Contains all of the data needed by a SSI encoder
 * on the Tiva
 */
struct SSIEncoder
{
    enum SSIEncoderBrand encoderBrand;
    uint32_t SSIBase;   // SSI0_Base, SSI1_Base, etc.
    uint16_t sampleRate;
    bool enabled;

    uint32_t raw;
    uint32_t rawPrev;
    int32_t rawVelF;
    int32_t rawVelPrev;
    int32_t rawVel;

};
typedef struct SSIEncoder SSIEncoder;

// Constructs the SSI Encoder
SSIEncoder ssiEncoderConstruct(uint32_t SSIBase, uint16_t sampleRate, SSIEncoderBrand encoderBrand);

// Enable the SSI encoder
void enableSSIEncoder(SSIEncoder* encoder);

// Disable the SSI encoder
void disableSSIEncoder(SSIEncoder* encoder);

// Read the raw position from the SSI encoder
void readAbsEncoder(SSIEncoder* encoder);

// Read the raw velocity from the SSI encoder
void readAbsEncoderVelocity(SSIEncoder* encoder);

#endif /* ENCODER_H */
