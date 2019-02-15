/****************************************************************************
;*	Microchip Technology Inc. 2007                                      *
;*	Assembler version:  XC8 v1.34                                       *
;*	Filename:                                                           *
;*		adc.c                                                       *
;*	Dependents:                                                         *
;*                                                                          *
;*	April 13, 2015                                                      *
;*      PIC10(L)32X Developmental Board                                     *
;*      Demonstration program:                                              *
;*          After applying power to the PIC10(L)F32X Development Board,     *
;*          LED (D1) will automatically turn on.  Turn POT1 clockwise       *
;*          to increase the brightness of LED (D2).  Press switch (SW1)     *
;*          to turn both LEDs D1 and D2 off, release switch (SW1) and       *
;*          LEDs D1 and D2 will turn on.                                    *
;*                                                                          *
;****************************************************************************/

//Section: Included Files

#include <xc.h>
#include "adc.h"

//Section: ADC Module APIs

void ADC_Initialize(void)
{
    ADCON = 0x89;         // ADON enabled; CHS AN2;

    TRISAbits.TRISA2 = 1; // POT1 pin (RA2) as input
    ANSELAbits.ANSA2 = 1; // configure (RA2) as analog input

}


adc_result_t ADC_GetConversion(adc_channel_t channel)
{
    // Select the A/D channel
    ADCONbits.CHS = channel;

    // Turn on the ADC module
    ADCONbits.ADON = 1;

    // Start the conversion
    ADCONbits.GO_nDONE = 1;

    // Wait for the conversion to finish
    while (ADCONbits.GO_nDONE);

    // Conversion finished, return the result
    return ADRES;
}

// End of File


