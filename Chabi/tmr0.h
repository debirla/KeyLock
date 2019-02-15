/****************************************************************************
;*	Microchip Technology Inc. 2007                                      *
;*	Assembler version:  XC8 v1.34                                       *
;*	Filename:                                                           *
;*		tmr0.h                                                      *
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

#ifndef _TMR0_H
#define _TMR0_H

/**
  Section: Included Files
*/

#include <stdint.h>
#include <stdbool.h>

/**
  Section: Macro Declarations
*/

/**
  Section: TMR0 APIs
*/



void TMR0_Initialize(void);

/* 250 kHz = 250000 Hz 
   Timer 0 tick = 250000/4 Hz = 62500 Hz 
   Pre-scalar = 2 => 62500/2 Hz = 31250 Hz */
#define TMR0_Load() (TMR0 = 0xFF - 19) // To get 1 ms interrupt

#endif // _TMR0_H

//End of File