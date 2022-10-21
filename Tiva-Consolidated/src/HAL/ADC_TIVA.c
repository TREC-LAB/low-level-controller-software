/**
 * ADC_TIVA.c
 * @author: Nick Tremaroli
 * Contains all of the low-level code for ADC related functions
 */
#include "ADC_TIVA.h"

/**
 * ADCConfig0
 * Enables ADC0 to be working on Pin E5 on the Tiva
 */
void ADCConfig0()
{
    // Enable ADC 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

    // Enable GPIO Port E because ADC0 uses pin E5
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    // wait for the ADC module to be ready
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0));

    // Disable the ADC before configuring
    ADCSequenceDisable(ADC0_BASE, 0);

    // Enable software over-sampling for software averaging of ADC values.
    ADCSoftwareOversampleConfigure(ADC0_BASE, 0, 8);
    ADCSoftwareOversampleStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_IE | ADC_CTL_END | ADC_CTL_CH8);

    // Enable the ADC sequence
    ADCSequenceEnable(ADC0_BASE, 0);

    // Enable interrupt on ADC0 sequencer 1
    ADCIntEnableEx(ADC0_BASE, ADC_INT_SS0);

    // Enable GPIO Pin E5 for the ADC
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_5);
}

/**
 * ADCConfig1
 * Enables ADC1 to be working on Pin E2 on the Tiva
 */
void ADCConfig1()
{
    // Enable ADC 1
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);

    // Enable GPIO Port E because ADC1 uses pin E2
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    // Wait for the ADC module to be ready
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC1));

    // Disable ADC1 before configuring
    ADCSequenceDisable(ADC1_BASE, 0);

    // Enable software over-sampling for software averaging of ADC values.
    ADCSoftwareOversampleConfigure(ADC1_BASE, 0, 8);
    ADCSoftwareOversampleStepConfigure(ADC1_BASE, 0, 0, ADC_CTL_IE | ADC_CTL_END | ADC_CTL_CH1); // Originally channel 1

    // Enable the ADC sequence
    ADCSequenceEnable(ADC1_BASE, 0);

    // Enable interrupt on ADC1 sequencer 1
    ADCIntEnableEx(ADC1_BASE, ADC_INT_SS0);

    // Enable GPIO Pin E2 for the ADC
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_2);
}
