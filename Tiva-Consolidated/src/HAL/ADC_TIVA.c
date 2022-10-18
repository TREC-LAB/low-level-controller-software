#include "ADC_TIVA.h"

void ADCConfig0()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);            //Port E setup for ADC conversion.
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0)){}     //Wait for the ADC module to be ready.

//For Software Averaging.  Can add in hardware averaging later if needed.
    ADCSequenceDisable(ADC0_BASE, 0);                    //Disable ADC0 before configuring.
    ADCSoftwareOversampleConfigure(ADC0_BASE, 0, 8);     //Enable software over-sampling for software averaging of ADC values.
    ADCSoftwareOversampleStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_IE | ADC_CTL_END | ADC_CTL_CH8); //Originally channel 8
    ADCSequenceEnable(ADC0_BASE, 0);                     //Enable the ADC sequence.

    ADCIntEnableEx(ADC0_BASE, ADC_INT_SS0);              //Enable interrupt on ADC0 sequencer 1.
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_5); // Originally pin 5
}

void ADCConfig1()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);            //Port E setup for ADC conversion.
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC1)){}     //Wait for the ADC module to be ready.

//For Software Averaging.  Can add in hardware averaging later if needed.
    ADCSequenceDisable(ADC1_BASE, 0);                    //Disable ADC1 before configuring.
    ADCSoftwareOversampleConfigure(ADC1_BASE, 0, 8);
    ADCSoftwareOversampleStepConfigure(ADC1_BASE, 0, 0, ADC_CTL_IE | ADC_CTL_END | ADC_CTL_CH1); // Originally channel 1
    ADCSequenceEnable(ADC1_BASE, 0);                     //Enable the ADC sequence.

    ADCIntEnableEx(ADC1_BASE, ADC_INT_SS0);
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_2); // Originally pin 2
}
