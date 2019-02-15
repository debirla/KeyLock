/****************************************************************************
 File Name   :- txmit.c
 Author      :- Debojyoti Lahiri
 Date        :- 24 September 2018
 Processor   :- Microchip PIC10F322
 IDE         :- Microchip MPLAB X IDE v3.65
 Compiler    :- Microchip XC8 (v1.44) Free version   
 Description :- C source file for transmit functions
*****************************************************************************/
/*
  Copyright (c) 2018 confidential
*/

/* Section: Included Files */

#include "device_initialize.h"

/* Section: Functions */

/* Initialize the preamble and key code */
void Txmt_Initialize(void)
{
  /* Later Read code bytes from flash D.Lahiri 24Sept2018 */
  gKeyCode[0]  = KEY_CODE_BYTE_1;  /* Key code byte # 01 - Read from Flash */
  gKeyCode[1]  = KEY_CODE_BYTE_2;  /* Key code byte # 02 - Read from Flash */
  gKeyCode[2]  = KEY_CODE_BYTE_3;  /* Key code byte # 03 - Read from Flash */
  /* Put more bytes here - if required D.Lahiri 24Sept2018 */
  gKeyCode[NUMBER_OF_TX_KEYCODE_BYTES-(1u)]  = KEY_CRC_BYTE_4;  /* Key code byte # 04 - Read from Flash */

  /* Make the highest significant bits as '1' D.Lahiri 24Sept2018 */
  /* This is done to transmit 1 at start - so effective bits of code word is 23 bits */
  //gKeyCode[0] |= 0x80;
}

/* Construct Transmit Manchester Pulse bytes function */
void constructTxBytesOfManchesterPulses(void)
{
  unsigned char codeBitByteIndex, bitIndex;
  for(codeBitByteIndex = 0; codeBitByteIndex <= NUMBER_OF_TX_KEYCODE_BYTES; codeBitByteIndex++)
  {
     /* one of two Manchester byte */
     gKeyCodeManchester[(2u)*codeBitByteIndex] = 0x00;

     /* Higher nibble from highest to lowest */
     for(bitIndex = 4; bitIndex <= 7; bitIndex++)
     {
       /* Place for 2 Manchester bits */
       gKeyCodeManchester[(2u)*codeBitByteIndex] <<= 2;
       /* Check if bit = '0' */
       if(((gKeyCode[codeBitByteIndex] >> bitIndex) & 0x01) == 0)
       {
         gKeyCodeManchester[(2u)*codeBitByteIndex] |= 0x02;
       }
       else /* Else Bit = '1' */
       {
         gKeyCodeManchester[(2u)*codeBitByteIndex] |= 0x01;
       }
     }

     /* two of two Manchester byte */
     gKeyCodeManchester[(2u)*codeBitByteIndex + 1] = 0x00;

     /* Lower nibble from highest to lowest */
     for(bitIndex = 0; bitIndex <= 3; bitIndex++)
     {
       /* Place for 2 Manchester bits */
       gKeyCodeManchester[(2u)*codeBitByteIndex + 1] <<= 2;
       /* Check if bit = '0' */
       if(((gKeyCode[codeBitByteIndex] >> bitIndex) & 0x01) == 0)
       {
         gKeyCodeManchester[(2u)*codeBitByteIndex + 1] |= 0x02;
       }
       else /* Else Bit = '1' */
       {
         gKeyCodeManchester[(2u)*codeBitByteIndex + 1] |= 0x01;
       }
     }
  }
}

/* Transmit Break between bytes */
void Txmt_Break(void)
{
  //__delay_ms(2); 
  /* Transmit 0 */  
  LATAbits.LATA1 = 0x00;
  /* Wait 15 ms */
  __delay_ms(15);
    /* Transmit 1 */  
  LATAbits.LATA1 = 0x01;
  /* Wait 3 ms */
  __delay_ms(3);
}


/*  End of File */
