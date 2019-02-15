/****************************************************************************
 File Name   :- device_initilize.c
 Author      :- Debojyoti Lahiri
 Date        :- 24 September 2017
 Processor   :- Microchip PIC10F322
 IDE         :- Microchip MPLAB X IDE v3.65
 Compiler    :- Microchip XC8 (v1.44) Free version   
 Description :- Device configuration source file for Chabi  
*****************************************************************************/
/*
  Copyright (c) 2018 confidential
*/

/* Configuration Bits */
#pragma config FOSC = INTOSC    /* Oscillator Selection bits (INTOSC oscillator: CLKIN function disabled) */
#pragma config BOREN = OFF      /* Brown-out Reset Enable (Brown-out Reset disabled) */
/* #pragma config WDTE = OFF */     /* Watchdog Timer Enable (WDT disabled) */
/* #pragma config WDTE = 0x3 */     /* Watchdog Timer Enable (WDT enabled) */
/* #pragma config WDTE = 0x2 */     /* Watchdog Timer Enable (Awake => Active, Sleep => Disabled) */
#pragma config WDTE = 0x1       /* Watchdog Timer Enable (WDT controlled by the SWDTEN bit in the WDTCON register) */
#pragma config PWRTE = OFF      /* Power-up Timer Enable bit (PWRT disabled) */
#pragma config MCLRE = ON       /* MCLR Pin Function Select bit (MCLR pin function is MCLR) */
#pragma config CP = OFF         /* Code Protection bit (Program memory code protection is disabled) */
#pragma config LVP = ON         /* Low-Voltage Programming Enable (Low-voltage programming enabled) */
#pragma config LPBOR = OFF      /* Brown-out Reset Selection bits (BOR disabled) */
#pragma config BORV = LO        /* Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.) */
#pragma config WRT = OFF        /* Flash Memory Self-Write Protection (Write protection off) */

#include "device_initialize.h"


/* End of File */
