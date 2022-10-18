/**
 * ForceSensor.c
 * @author: Nick Tremaroli
 * Contains all of the low-level code for force sensor related functions
 */
#include "ForceSensor.h"

/**
 * forceSensorConstruct
 * Constructs a force sensor given the corresponding input parameters
 *
 * @param ADCBase: The ADC base of the force sensor
 * @param slope: The slope value of the force sensor
 * @param offset: The offset value of the force sensor
 *
 * @return: an initialized force sensor structure
 */
ForceSensor forceSensorConstruct(uint32_t ADCBase, float slope, float offset)
{
    ForceSensor forceSensor;

    forceSensor.ADCBase = ADCBase;
    forceSensor.enabled = true;

    forceSensor.upperLimitNewtons = 1800;
    forceSensor.lowerLimitNewtons = -1800;

    forceSensor.slope = slope;
    forceSensor.offset = offset;

    forceSensor.raw = 0;
    forceSensor.newtons = 0;

    return forceSensor;
}

/**
 * enableForceSensor
 * Enables the force sensor
 *
 * @param forceSensor: a pointer to the force sensor to enable
 */
void enableForceSensor(ForceSensor* forceSensor)
{
    if(forceSensor->ADCBase == ADC0_BASE)
        ADCConfig0();
    else if(forceSensor->ADCBase == ADC1_BASE)
        ADCConfig1();
    else
        return;

    forceSensor->enabled = true;
}

/**
 * readLoadCell
 * reads the raw value from the force sensor
 *
 * @param forceSensor: a pointer to the force sensor to read
 */
void readLoadCell(ForceSensor* forceSensor)
{
    ADCProcessorTrigger(forceSensor->ADCBase, 0);                       // Causes a processor trigger for a sample sequence, trigger the sample sequence.
    while(!ADCIntStatusEx(forceSensor->ADCBase, false)){}               // Wait until the sample sequence has completed.
    ADCIntClear(forceSensor->ADCBase, 0);                               // Clear the ADC interrupt flag generated upon ADC completion.
    ADCSoftwareOversampleDataGet(forceSensor->ADCBase, 0, &forceSensor->raw, 1);   // Obtain single software averaged ADC value.
}

/**
 * updateForces
 * updates the force reading to the actual value
 *
 * @param forceSensor: a pointer to the force sensor which
 * will update its value
 */
void updateForces(ForceSensor* forceSensor)
{
    readLoadCell(forceSensor);
    forceSensor->newtons = forceSensor->raw * forceSensor->slope + forceSensor->offset;
}
