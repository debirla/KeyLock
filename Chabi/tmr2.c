/****************************************************************************
 File Name   :- tmr2.c
 Author      :- Debojyoti Lahiri
 Date        :- 03 November 2017
 Processor   :- Microchip PIC10F322
 IDE         :- Microchip MPLAB X IDE v3.65
 Compiler    :- Microchip XC8 (v1.44) Free version   
 Description :- C source file for timer 2 functions
*****************************************************************************/
/*
  Copyright (c) 2017 confidential
*/

/* Section: Included Files */

#include "device_initialize.h"

/* Section: Functions */

void TMR2_Initialize(void)
{
  /* T2CON: TIMER2 CONTROL REGISTER
     bit 7 Unimplemented: Read as ?0?
     bit 6-3 TOUTPS<3:0>: Timer2 Output Postscaler Select bits
         1111 = 1:16 Postscaler
         1110 = 1:15 Postscaler
         1101 = 1:14 Postscaler
         1100 = 1:13 Postscaler
         1011 = 1:12 Postscaler
         1010 = 1:11 Postscaler
         1001 = 1:10 Postscaler
         1000 = 1:9 Postscaler
         0111 = 1:8 Postscaler
         0110 = 1:7 Postscaler
         0101 = 1:6 Postscaler
         0100 = 1:5 Postscaler
         0011 = 1:4 Postscaler
         0010 = 1:3 Postscaler
         0001 = 1:2 Postscaler
         0000 = 1:1 Postscaler
     bit 2 TMR2ON: Timer2 On bit
         1 = Timer2 is on
         0 = Timer2 is off
     bit 1-0 T2CKPS<1:0>: Timer2 Clock Prescale Select bits
         11 = Prescaler is 64
         10 = Prescaler is 16
         01 = Prescaler is 4
         00 = Prescaler is 1 
  */
  #ifdef INTOSC_FREQ_31K
    T2CON = 0x00; /* 0 0000 0 00 */ /* 1:1 Postscalar, T2 off, Prescalar is 1 */
    PR2 = 8; /* 31000 / 4 = 7750; 7750 / (1*1*8) = 968.75 Hz => 1.032 ms */
  #endif
  #ifdef INTOSC_FREQ_250K
    T2CON = 0x00; /* 0 0000 0 00 */ /* 1:1 Postscalar, T2 off, Prescalar is 1 */
    PR2 = 63; /* 250000 / 4 = 62500; 62500 / (1*1*63) = 992.06 Hz => 1.008 ms */
  #endif
  #ifdef INTOSC_FREQ_8M
    T2CON = 0x48; /* 0 1001 0 00 */ /* 1:10 Postscalar, T2 off, Prescalar is 1 */
    PR2 = 200; /* 8000000 / 4 = 2000000; 2000000 / (1*10*200) = 1000 Hz => 1 ms */
  #endif 

    
  /*  TMR2 0x00; */
  TMR2 = 0x00;

  /* Clearing IF flag. */
  PIR1bits.TMR2IF = 0;
}

void TMR2_StartTimer(void)
{
    /* Start the Timer by writing to TMRxON bit */
    T2CONbits.TMR2ON = 1;
}

/* End of File */

