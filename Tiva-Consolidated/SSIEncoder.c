/**
 * SSIEncoder.c
 * @author: Nick Tremaroli
 * Contains all of the low-level code for SSI encoder related functions
 */
#include "SSIEncoder.h"

const float alpha = 0.99;

/**
 * ssiEncoderConstruct
 * Constructs a SSI encoder given the corresponding input parameters
 * @param SSIBase: the SSI base of the encoder
 * @param sampleRate: the sample rate of the encoder
 * @param encoderBrand: the encoder brand of the encoder
 *
 * @return: an initialized SSI encoder structure
 */
SSIEncoder ssiEncoderConstruct(uint32_t SSIBase, uint16_t sampleRate, SSIEncoderBrand encoderBrand)
{
    SSIEncoder encoder;

    encoder.encoderBrand = encoderBrand;
    encoder.SSIBase = SSIBase;
    encoder.raw = 0;
    encoder.rawPrev = 0;
    encoder.rawVelF = 0;
    encoder.rawVelPrev = 0;
    encoder.rawVel = 0;

    encoder.sampleRate = sampleRate;

    encoder.enabled = false;

    return encoder;
}

/**
 * encoderSSIEncoder
 * enables the SSI encoder on the Tiva
 * @param encoder: a pointer to the SSI encoder to enable
 */
void enableSSIEncoder(SSIEncoder* encoder)
{
    // TODO: Add error checking if the SSIBase is not correct
    if (encoder->SSIBase == SSI0_BASE)
    {
        if (encoder->encoderBrand == Gurley_Encoder)
            SSI0_Gurley_Config();
        else if (encoder->encoderBrand == Orbis_Encoder)
            SSI0_Orbis_Config();
    }
    else if (encoder->SSIBase == SSI1_BASE)
    {
        if (encoder->encoderBrand == Gurley_Encoder)
            SSI1_Gurley_Config();
        else if (encoder->encoderBrand == Orbis_Encoder)
            SSI1_Orbis_Config();
    }
    else if (encoder->SSIBase == SSI2_BASE)
        SSI2_Config();
    else
        return;

    encoder->enabled = true;

}

/**
 * disableSSIEncoder
 * disables the SSI Encoder
 * @param encoder: A pointer to the SSI encoder to disable
 */
void disableSSIEncoder(SSIEncoder* encoder)
{
    // TODO: See if when a encoder if disabled if all of the members of the
    // encoder struct should be set to 0 (with the exception of the SSI Base)
    if(encoder->SSIBase == SSI0_BASE)
        SSI0_Disable();
    else if(encoder->SSIBase == SSI1_BASE)
        SSI1_Disable();
    else if(encoder->SSIBase == SSI2_BASE)
        SSI2_Disable();
    else
        return;

    encoder->enabled = false;
}

/**
 * readAbsEncoder
 * reads the raw position value
 *
 * @param encoder: a pointer to the SSI encoder to read from
 */
void readAbsEncoder(SSIEncoder* encoder)
{
    SSIDataPut(encoder->SSIBase, 0);
    while(SSIBusy(encoder->SSIBase)) {}
    // read and store it in the encoder structure
    SSIDataGet(encoder->SSIBase, &encoder->raw);
}

/**
 * readAbsEncoderVelocity
 * reds the velocity of the SSI encoder
 * @param encoder: a pointer to the SSI encoder to read the velocity from
 */
void readAbsEncoderVelocity(SSIEncoder* encoder)
{
    if((int32_t) (encoder->raw - encoder->rawPrev) < 60000
    && (int32_t) (encoder->raw - encoder->rawPrev) > -60000)
    {
        encoder->rawVel = (int32_t)(encoder->raw - encoder->rawPrev) * encoder->sampleRate;
        // Filter Velocity
        encoder->rawVelF = (int32_t) (alpha * encoder->rawVelPrev + (1 - alpha) * encoder->rawVel);

        encoder->rawVelPrev = encoder->rawVelF;
    }
    encoder->rawPrev = encoder->raw;  // Update the previous angle
}

