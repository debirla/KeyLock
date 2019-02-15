/****************************************************************************
 File Name   :- device_initilize.h
 Author      :- Debojyoti Lahiri
 Date        :- 24 September 2017
 Processor   :- Microchip PIC10F322
 IDE         :- Microchip MPLAB X IDE v3.65
 Compiler    :- Microchip XC8 (v1.44) Free version   
 Description :- Device configuration header file for Chabi  
*****************************************************************************/
/*
  Copyright (c) 2018 confidential
*/

#ifndef DEVICE_INITIALIZE_H
#define	DEVICE_INITIALIZE_H
#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include "tmr2.h"
#include "txmit.h"

/* Use any one */
#define INTOSC_FREQ_31K /* To save power, but sacrificing time accuracy */
//#define INTOSC_FREQ_250K
//#define INTOSC_FREQ_8M /* More time accurate but consumes power */

/* OSCCON: OSCILLATOR CONTROL REGISTER
     bit 7 Unimplemented: Read as ?0?
     bit 6-4 IRCF<2:0>: INTOSC (FOSC) Frequency Select bits
       111 = 16 MHz
       110 = 8 MHz (default value)
       101 = 4 MHz
       100 = 2 MHz
       011 = 1 MHz
       010 = 500 kHz
       001 = 250 kHz
       000 = 31 kHz (LFINTOSC)
     bit 3 HFIOFR: High-Frequency Internal Oscillator Ready bit
       1 = 16 MHz Internal Oscillator (HFINTOSC) is ready
       0 = 16 MHz Internal Oscillator (HFINTOSC) is not ready
     bit 2 Unimplemented: Read as ?0?
     bit 1 LFIOFR: Low-Frequency Internal Oscillator Ready bit
       1 = 31 kHz Internal Oscillator (LFINTOSC) is ready
       0 = 31 kHz Internal Oscillator (LFINTOSC) is not ready
     bit 0 HFIOFS: High-Frequency Internal Oscillator Stable bit
       1 = 16 MHz Internal Oscillator (HFINTOSC) is stable
       0 = 16 MHz Internal Oscillator (HFINTOSC) is not stable
*/
#ifdef INTOSC_FREQ_31K
  #define OSCILLATOR_Initialize() (OSCCON = 0x00) /* 31 kHz (LFINTOSC) */
  #define _XTAL_FREQ (31000u)
  #define ONE_MS_DELAY() (_delay(8u))
#endif
#ifdef INTOSC_FREQ_250K
  #define OSCILLATOR_Initialize() (OSCCON = 0x10) /* 250 kHz */
  #define _XTAL_FREQ (250000u)
  #define ONE_MS_DELAY() (_delay(63u))
#endif
#ifdef INTOSC_FREQ_8M
  #define OSCILLATOR_Initialize() (OSCCON = 0x60) /* 8 MHz */
  #define _XTAL_FREQ (8000000u)
  #define ONE_MS_DELAY() (_delay(2000u))
#endif 

/* INTCON: INTERRUPT CONTROL REGISTER
     bit 7 GIE: Global Interrupt Enable bit
       1 = Enables all active interrupts
       0 = Disables all interrupts
     bit 6 PEIE: Peripheral Interrupt Enable bit
       1 = Enables all active peripheral interrupts
       0 = Disables all peripheral interrupts
     bit 5 TMR0IE: Timer0 Overflow Interrupt Enable bit
       1 = Enables the Timer0 interrupt
       0 = Disables the Timer0 interrupt
     bit 4 INTE: INT External Interrupt Enable bit
       1 = Enables the INT external interrupt
       0 = Disables the INT external interrupt
     bit 3 IOCIE: Interrupt-on-Change Interrupt Enable bit
       1 = Enables the interrupt-on-change interrupt
       0 = Disables the interrupt-on-change interrupt
     bit 2 TMR0IF: Timer0 Overflow Interrupt Flag bit
       1 = TMR0 register has overflowed
       0 = TMR0 register did not overflow
     bit 1 INTF: INT External Interrupt Flag bit
       1 = The INT external interrupt occurred
       0 = The INT external interrupt did not occur
     bit 0 IOCIF: Interrupt-on-Change Interrupt Flag bit(1)
       1 = When at least one of the interrupt-on-change pins changed state
       0 = None of the interrupt-on-change pins have changed state
*/
#define GLOBAL_INTERRUPT_Disable() (INTCONbits.GIE = 0) /* GIE disable */

/* WDTCON: WATCHDOG TIMER CONTROL REGISTER
     bit 7-6 Unimplemented: Read as ?0?
     bit 5-1 WDTPS<4:0>: Watchdog Timer Period Select bits(1)
       Bit Value = Prescale Rate
       11111 = Reserved. Results in minimum interval (1:32)
       ?
       ?
       ?
       10011 = Reserved. Results in minimum interval (1:32)
       10010 = 1:8388608 (223) (Interval 256s nominal)
       10001 = 1:4194304 (222) (Interval 128s nominal)
       10000 = 1:2097152 (221) (Interval 64s nominal)
       01111 = 1:1048576 (220) (Interval 32s nominal)
       01110 = 1:524288 (219) (Interval 16s nominal)
       01101 = 1:262144 (218) (Interval 8s nominal)
       01100 = 1:131072 (217) (Interval 4s nominal)
       01011 = 1:65536 (Interval 2s nominal) (Reset value)
       01010 = 1:32768 (Interval 1s nominal)
       01001 = 1:16384 (Interval 512 ms nominal)
       01000 = 1:8192 (Interval 256 ms nominal)
       00111 = 1:4096 (Interval 128 ms nominal)
       00110 = 1:2048 (Interval 64 ms nominal)
       00101 = 1:1024 (Interval 32 ms nominal)
       00100 = 1:512 (Interval 16 ms nominal)
       00011 = 1:256 (Interval 8 ms nominal)
       00010 = 1:128 (Interval 4 ms nominal)
       00001 = 1:64 (Interval 2 ms nominal)
       00000 = 1:32 (Interval 1 ms nominal)
    bit 0 SWDTEN: Software Enable/Disable for Watchdog Timer bit
      If WDTE<1:0> = 00:
        This bit is ignored.
      If WDTE<1:0> = 01:
        1 = WDT is turned on
        0 = WDT is turned off
      If WDTE<1:0> = 1x:
        This bit is ignored. 
*/
/* 00 01011 1 binary = 0x17 and 00 01011 0 binary = 0x16  */
/* #define WATCHDOG_EnableAndInit() (WDTCON = 0x17) */ /* Interval 2s nominal */
/* #define WATCHDOG_Disable() (WDTCON = 0x16) */ /* Reset for Interval 2s nominal set */
/* 00 01101 1 binary = 0x1B  and 00 01101 0 binary = 0x1A */
#define WATCHDOG_EnableAndInit() (WDTCON = 0x1B) /* Interval 8s nominal */

#endif	/* DEVICE_INITIALIZE_H */

/* End of File */

