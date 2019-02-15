/****************************************************************************
 File Name   :- txmit.h
 Author      :- Debojyoti Lahiri
 Date        :- 24 September 2017
 Processor   :- Microchip PIC10F322
 IDE         :- Microchip MPLAB X IDE v3.65
 Compiler    :- Microchip XC8 (v1.44) Free version   
 Description :- C Header file for transmit functions
*****************************************************************************/
/*
  Copyright (c) 2018 confidential
*/

#ifndef _TXMIT_H
#define _TXMIT_H

/**
  Section: Included Files
*/

#include <stdint.h>
#include <stdbool.h>
#include "device_initialize.h"

/*
  Section: Macro Declarations
*/
/* Key code bytes */

/*
Key code bytes :- 87 65 43  [remainder of 0x876543 divided by 0xa7 => 0x48]
CRC8 value for polynomial (a7) = 2f

Key code bytes :- ab cd ef  [remainder of 0xabcdef divided by 0xa7 => 0x44]
CRC8 value for polynomial (a7) = 40

Key code bytes :- 90 0 21  [remainder of 0x900021 divided by 0xa7 => 0x2f]
CRC8 value for polynomial (a7) = f3

Key code bytes :- aa aa aa  [remainder of 0xaaaaaa divided by 0xa7 => 0x98]
CRC8 value for polynomial (a7) = 82

Key code bytes :- d5 55 55  [remainder of 0xd55555 divided by 0xa7 => 0x6b]
CRC8 value for polynomial (a7) = b0

Key code bytes :- ff ff ff  [remainder of 0xffffff divided by 0xa7 => 0x3d]
CRC8 value for polynomial (a7) = c3

Key code bytes :- 80 0 0  [remainder of 0x800000 divided by 0xa7 => 0x1f]
CRC8 value for polynomial (a7) = f1

Key code bytes :- a8 64 de  [remainder of 0xa864de divided by 0xa7 => 0x9]
CRC8 value for polynomial (a7) = ff

Key code bytes :- 0 0 0  [remainder of 0x0 divided by 0xa7 => 0x0]
CRC8 value for polynomial (a7) = 0
*/

#if 0
#define KEY_CODE_BYTE_1 (0x00) /* Keep LSB as 1 - put value greater than 0x80 */
#define KEY_CODE_BYTE_2 (0X00)
#define KEY_CODE_BYTE_3 (0x00)
#define KEY_CRC_BYTE_4  (0x00) /* Pre Calculated CRC with polynomial 0xA7 */
#endif 
#if 0
#define KEY_CODE_BYTE_1 (0xFF) /* Keep LSB as 1 - put value greater than 0x80 */
#define KEY_CODE_BYTE_2 (0XFF)
#define KEY_CODE_BYTE_3 (0xFF)
#define KEY_CRC_BYTE_4  (0xC3) /* Pre Calculated CRC with polynomial 0xA7 */
#endif
#if 0
#define KEY_CODE_BYTE_1 (0xAB) /* Keep LSB as 1 - put value greater than 0x80 */
#define KEY_CODE_BYTE_2 (0XCD)
#define KEY_CODE_BYTE_3 (0xEF)
#define KEY_CRC_BYTE_4  (0x40) /* Pre Calculated CRC with polynomial 0xA7 */
#endif
#if 0
#define KEY_CODE_BYTE_1 (0x87) /* Keep LSB as 1 - put value greater than 0x80 */
#define KEY_CODE_BYTE_2 (0X65)
#define KEY_CODE_BYTE_3 (0x43)
#define KEY_CRC_BYTE_4  (0x2F) /* Pre Calculated CRC with polynomial 0xA7 */
#endif
#if 0
#define KEY_CODE_BYTE_1 (0x90) /* Keep LSB as 1 - put value greater than 0x80 */
#define KEY_CODE_BYTE_2 (0X00)
#define KEY_CODE_BYTE_3 (0x21)
#define KEY_CRC_BYTE_4  (0xF3) /* Pre Calculated CRC with polynomial 0xA7 */
#endif
#if 0
#define KEY_CODE_BYTE_1 (0xAA) /* Keep LSB as 1 - put value greater than 0x80 */
#define KEY_CODE_BYTE_2 (0XAA)
#define KEY_CODE_BYTE_3 (0xAA)
#define KEY_CRC_BYTE_4  (0x82) /* Pre Calculated CRC with polynomial 0xA7 */
#endif
#if 0
#define KEY_CODE_BYTE_1 (0xD5) /* Keep LSB as 1 - put value greater than 0x80 */
#define KEY_CODE_BYTE_2 (0X55)
#define KEY_CODE_BYTE_3 (0x55)
#define KEY_CRC_BYTE_4  (0xB0) /* Pre Calculated CRC with polynomial 0xA7 */
#endif
#if 0
#define KEY_CODE_BYTE_1 (0xA8) /* Keep LSB as 1 - put value greater than 0x80 */
#define KEY_CODE_BYTE_2 (0X64)
#define KEY_CODE_BYTE_3 (0xDE)
#define KEY_CRC_BYTE_4  (0xFF) /* Pre Calculated CRC with polynomial 0xA7 */
#endif
#if 0
#define KEY_CODE_BYTE_1 (0x80) /* Keep LSB as 1 - put value greater than 0x80 */
#define KEY_CODE_BYTE_2 (0X00)
#define KEY_CODE_BYTE_3 (0x00)
#define KEY_CRC_BYTE_4  (0xF1) /* Pre Calculated CRC with polynomial 0xA7 */
#endif
#if 0
#define KEY_CODE_BYTE_1 (0x80) /* Keep LSB as 1 - put value greater than 0x80 */
#define KEY_CODE_BYTE_2 (0X00)
#define KEY_CODE_BYTE_3 (0x01)
#define KEY_CRC_BYTE_4  (0x56) /* Pre Calculated CRC with polynomial 0xA7 */
#endif
#if 0
#define KEY_CODE_BYTE_1 (0x90) /* Keep LSB as 1 - put value greater than 0x80 */
#define KEY_CODE_BYTE_2 (0X00)
#define KEY_CODE_BYTE_3 (0x00)
#define KEY_CRC_BYTE_4  (0xB2) /* Pre Calculated CRC with polynomial 0xA7 */
#endif
#if 1
#define KEY_CODE_BYTE_1 (0x81) /* Keep LSB as 1 - put value greater than 0x80 */
#define KEY_CODE_BYTE_2 (0X00)
#define KEY_CODE_BYTE_3 (0x00)
#define KEY_CRC_BYTE_4  (0x55) /* Pre Calculated CRC with polynomial 0xA7 */
#endif



#define NUMBER_OF_TX_KEYCODE_BYTES (4u)
/* Key code bits */
uint8_t gKeyCode[NUMBER_OF_TX_KEYCODE_BYTES];
#define NUMBER_OF_TX_KEYMANCODE_BYTES (NUMBER_OF_TX_KEYCODE_BYTES*(2u))
uint8_t gKeyCodeManchester[NUMBER_OF_TX_KEYMANCODE_BYTES];

/*
  Section: _TXMIT_H APIs
*/
/* Init the preamble and key code */
void Txmt_Initialize(void);
/* Construct Transmit byte function before transmission */
void constructTxBytesOfManchesterPulses(void);
/* Transmit Idle */
#define Txmt_Idle()               (LATAbits.LATA1 = 0)
/* Transmit Break between bytes */
void Txmt_Break(void);



#endif /* _TXMIT_H */

/* End of File */
