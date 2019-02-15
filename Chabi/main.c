/****************************************************************************
 File Name   :- main.c
 Author      :- Debojyoti Lahiri
 Date        :- 24 September 2018
 Processor   :- Microchip PIC10F322
 IDE         :- Microchip MPLAB X IDE v3.65
 Compiler    :- Microchip XC8 (v1.44) Free version   
 Description :- main source file for Chabi  
*****************************************************************************/
/*
  Copyright (c) 2018 confidential
*/

#include "device_initialize.h"

/* Main function */
void main(void)
{
  /* Tx byte index */
  uint8_t txmtManCodeBitIndex, txmtManCodeByteIndex, tempManByte[NUMBER_OF_TX_KEYMANCODE_BYTES];
  
  /* Disable interrupts */
  GLOBAL_INTERRUPT_Disable();
  
  /* Oscillator */
  OSCILLATOR_Initialize();
   
  /* Low current requirements */
  ANSELA = 0x00; /* Set ports as digital I/O, not analog input */
  WPUA = 0x00;   /* WEAK PULL-UP disable */
  FVRCON = 0x00; /* FIXED VOLTAGE REFERENCE */
  ADCON = 0x00;  /* Shut off the A/D Converter */
  
  /* Init and Set Transmit pin to Idle */
  Txmt_Initialize();
  /* port pins */  
  TRISAbits.TRISA0 = 0;   /* Set Channel RA0 as output */
  TRISAbits.TRISA1 = 0;   /* Set Channel RA1 as output */
  TRISAbits.TRISA2 = 0;   /* Set Channel RA1 as output */
  LATAbits.LATA0 = 0;     /* Set RA0 (LED Red in dev board) low */
  Txmt_Idle();     /* Set RA1 - Txmt pin low (LED Green in dev board)*/
  LATAbits.LATA2 = 0;     /* Set RA2 low */
  /* Construct Transmit Manchester Pulse bytes to transmit */
  constructTxBytesOfManchesterPulses();
  
  /* Watchdog timer set and start */
  WATCHDOG_EnableAndInit();

  /* Wait 50 ms before Transmit the preamble bits */
  __delay_ms(50);
  
  /* Clear watchdog */
  CLRWDT();
  
  /* No transmission - load modulation off */
  Txmt_Idle();
  
  /* Wait 100 ms before Transmit the key code bits */
  __delay_ms(100);

  /* Forever loop */
  for(;;)
  {
    /* Clear watchdog */
    CLRWDT();
        
    /* Wait 100 ms before Transmit the key code bits */
    __delay_ms(100);
    
    /* Copy for transmission */
    for(txmtManCodeByteIndex = 0; txmtManCodeByteIndex < NUMBER_OF_TX_KEYMANCODE_BYTES; txmtManCodeByteIndex++)
    {
      tempManByte[txmtManCodeByteIndex] = gKeyCodeManchester[txmtManCodeByteIndex];   
    }

    /* Transmit 1 for slicer threshold stabilization */
    LATAbits.LATA1 = 0x01;
    /* Wait 30 ms */
    __delay_ms(30);
    Txmt_Idle();
    __delay_ms(3);
    
    /* Transmit 16 bits with 1, that is 0xFFFF to set the threshold of Tala */
    for(txmtManCodeBitIndex = 0; txmtManCodeBitIndex <= 3; txmtManCodeBitIndex++)
    {
      LATAbits.LATA1 = 0x01;
      __delay_ms(7);
      LATAbits.LATA1 = 0x00;
      __delay_ms(4);
    }
    /* Transmit Break */
    Txmt_Break();
    /* Transmit the Key Manchester Code bytes */ 
    /* unrolling the 8 byte external "for loop" to get more time accuracy  */
    /* Only using bit "for loop" to save space DLahiri 24Sept2018 */
    /* Manchester Byte # 0 */ /* Higher Nibble of Key Code byte # 0 */
    for(txmtManCodeBitIndex = 0; txmtManCodeBitIndex <= 7; txmtManCodeBitIndex++)
    { 
        /* Set transmit pulse */
        LATAbits.LATA1 =  (uint8_t)(tempManByte[0] & 0x01);
        tempManByte[0] >>= 1;
        ONE_MS_DELAY();
    }
    /* Manchester Byte # 1 */ /* Lower Nibble of Key Code byte # 0 */
    for(txmtManCodeBitIndex = 0; txmtManCodeBitIndex <= 7; txmtManCodeBitIndex++)
    { 
        /* Set transmit pulse */
        LATAbits.LATA1 =  (uint8_t)(tempManByte[1] & 0x01);
        tempManByte[1] >>= 1;
        ONE_MS_DELAY();
    }
    /* Transmit Break */
    Txmt_Break();
    /* Manchester Byte # 2 */ /* Higher Nibble of Key Code byte # 1 */
    for(txmtManCodeBitIndex = 0; txmtManCodeBitIndex <= 7; txmtManCodeBitIndex++)
    { 
        /* Set transmit pulse */
        LATAbits.LATA1 =  (uint8_t)(tempManByte[2] & 0x01);
        tempManByte[2] >>= 1;
        ONE_MS_DELAY();
    }   
    /* Manchester Byte # 3 */ /* Lower Nibble of Key Code byte # 1 */
    for(txmtManCodeBitIndex = 0; txmtManCodeBitIndex <= 7; txmtManCodeBitIndex++)
    { 
        /* Set transmit pulse */
        LATAbits.LATA1 =  (uint8_t)(tempManByte[3] & 0x01);
        tempManByte[3] >>= 1;
        ONE_MS_DELAY();
    } 
    /* Transmit Break */
    Txmt_Break();
    /* Manchester Byte # 4 */ /* Higher Nibble of Key Code byte # 2 */
    for(txmtManCodeBitIndex = 0; txmtManCodeBitIndex <= 7; txmtManCodeBitIndex++)
    { 
        /* Set transmit pulse */
        LATAbits.LATA1 =  (uint8_t)(tempManByte[4] & 0x01);
        tempManByte[4] >>= 1;
        ONE_MS_DELAY();
    }   
    /* Manchester Byte # 5 */ /* Lower Nibble of Key Code byte # 2 */
    for(txmtManCodeBitIndex = 0; txmtManCodeBitIndex <= 7; txmtManCodeBitIndex++)
    { 
        /* Set transmit pulse */
        LATAbits.LATA1 =  (uint8_t)(tempManByte[5] & 0x01);
        tempManByte[5] >>= 1;
        ONE_MS_DELAY();
    }  
    /* Transmit Break */
    Txmt_Break();
    /* Manchester Byte # 6 */ /* Higher nibble of CRC byte */
    for(txmtManCodeBitIndex = 0; txmtManCodeBitIndex <= 7; txmtManCodeBitIndex++)
    { 
        /* Set transmit pulse */
        LATAbits.LATA1 =  (uint8_t)(tempManByte[6] & 0x01);
        tempManByte[6] >>= 1;
        ONE_MS_DELAY();
    }   
    /* Manchester Byte # 7 i.e. (NUMBER_OF_TX_KEYMANCODE_BYTES - 1) */ 
    /* Lower nibble of CRC byte */
    for(txmtManCodeBitIndex = 0; txmtManCodeBitIndex <= 7; txmtManCodeBitIndex++)
    { 
        /* Set transmit pulse */
        LATAbits.LATA1 =  (uint8_t)(tempManByte[NUMBER_OF_TX_KEYMANCODE_BYTES - 1] & 0x01);
        tempManByte[NUMBER_OF_TX_KEYMANCODE_BYTES - 1] >>= 1;
        ONE_MS_DELAY();
    }

    /* No transmission - load modulation off */
    Txmt_Idle();
    /* Wait 15 ms */
    __delay_ms(15);

  }
  
  /* Testing the blinking LEDs */
  #if 0
  for(;;)
  {
    CLRWDT();
    __delay_ms(500);
    LATAbits.LATA0 = 0;
    LATAbits.LATA1 = 1;
    CLRWDT();
    __delay_ms(500);
    LATAbits.LATA0 = 1;
    LATAbits.LATA1 = 0;
  }
  #endif /* #if 0 */
}

/* End of File */

