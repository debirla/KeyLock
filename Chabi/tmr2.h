/****************************************************************************
 File Name   :- tmr2.h
 Author      :- Debojyoti Lahiri
 Date        :- 03 November 2017
 Processor   :- Microchip PIC10F322
 IDE         :- Microchip MPLAB X IDE v3.65
 Compiler    :- Microchip XC8 (v1.44) Free version   
 Description :- C Header file for timer 2 functions
*****************************************************************************/
/*
  Copyright (c) 2017 confidential
*/

#ifndef _TMR2_H
#define _TMR2_H

/* Section: Included Files */
#include <stdint.h>
#include <stdbool.h>
#include "device_initialize.h"

/**
  Section: Macro Declarations
*/

/**
  Section: TMR2 APIs
*/


void TMR2_Initialize(void);

void TMR2_StartTimer(void);

#endif /* _TMR2_H */

/* End of File */
