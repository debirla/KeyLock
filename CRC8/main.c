#include <stdio.h>
#include <stdlib.h>

/*
 * The width of the CRC calculation and result.
 * Modify the typedef for a 16 or 32-bit CRC standard.
 */
typedef unsigned char crc;
#define POLYNOMIAL 0xA7  /* 10100111 */
#define WIDTH  (8 * sizeof(crc))
#define TOPBIT (1 << (WIDTH - 1))

/* CRC Lookup table */
crc crcTable[256];

/* Compute the CRC Table */
void crcLutCreate(void)
{
  crc remainder;
  /* Compute the remainder of each possible dividend. */
  for (int dividend = 0; dividend < 256; ++dividend)
  {
    /* Start with the dividend followed by zeros.*/
    remainder = dividend << (WIDTH - 8);
    /* Perform modulo-2 division, a bit at a time. */
    for (unsigned char bit = 8; bit > 0; --bit)
    {
      /* Try to divide the current data bit.*/
      if (remainder & TOPBIT)
      {
        remainder = (remainder << 1) ^ POLYNOMIAL;
      }
      else
      {
        remainder = (remainder << 1);
      }
    }
    /* Store the result into the table.*/
    crcTable[dividend] = remainder;
  }
}/* crcLutCreate() */

/* Compute the CRC from Lookup table */
crc crcFromLut(unsigned char const message[], int nBytes)
{
  unsigned char data;
  crc remainder = 0;
  /* Divide the message by the polynomial, a byte at a time.*/
  for (int byte = 0; byte < nBytes; ++byte)
  {
    data = message[byte] ^ (remainder >> (WIDTH - 8));
    remainder = crcTable[data] ^ (remainder << 8);
  }
  /* The final remainder is the CRC.*/
  return (remainder);
} /* crcFromLut() */

/* Key code for calculating CRC */
#define NUM_KEY_CODE_BYTES (3u)
unsigned char gKeyCode1[] = {0x87, 0x65, 0x43}; /* Write the Key Code bytes here */
unsigned char gKeyCode2[] = {0xAB, 0xCD, 0xEF}; /* Write the Key Code bytes here */
unsigned char gKeyCode3[] = {0x90, 0x00, 0x21}; /* Write the Key Code bytes here */
unsigned char gKeyCode4[] = {0xAA, 0xAA, 0xAA}; /* Write the Key Code bytes here */
unsigned char gKeyCode5[] = {0xD5, 0x55, 0x55}; /* Write the Key Code bytes here */
unsigned char gKeyCode6[] = {0xFF, 0xFF, 0xFF}; /* Write the Key Code bytes here */
unsigned char gKeyCode7[] = {0x80, 0x00, 0x00}; /* Write the Key Code bytes here */
unsigned char gKeyCode8[] = {0x80, 0x00, 0x01}; /* Write the Key Code bytes here */
unsigned char gKeyCode9[] = {0x90, 0x00, 0x00}; /* Write the Key Code bytes here */
unsigned char gKeyCode10[] = {0x81, 0x00, 0x00}; /* Write the Key Code bytes here */
unsigned char gKeyCode11[] = {0xA8, 0x64, 0xDE}; /* Write the Key Code bytes here */
unsigned char gKeyCode12[] = {0x00, 0x00, 0x00}; /* Write the Key Code bytes here */

/* Generate CRC by calling crcLutCreate() followed by crcFromLut() */
void printCRC8(unsigned char *gKeyCode)
{
  crc crcValue;
  unsigned int dataValue;
  dataValue = gKeyCode[0];
  dataValue <<= (8u);
  dataValue += gKeyCode[1];
  dataValue <<= (8u);
  dataValue += gKeyCode[2];

  printf("Key code bytes :- ");
  for (int byteIndex = 0; byteIndex < NUM_KEY_CODE_BYTES; byteIndex++)
  {
    printf("%x ", gKeyCode[byteIndex]);
  }
  printf(" [remainder of 0x%x divided by 0x%x => 0x%x]", dataValue, POLYNOMIAL, dataValue%POLYNOMIAL);
  printf("\n");
  crcLutCreate();
  crcValue = crcFromLut(gKeyCode, NUM_KEY_CODE_BYTES);
  printf("CRC8 value for polynomial (%x) = %x\n\n", POLYNOMIAL, crcValue);
}

int main()
{
  /* Create crc look up table */
  crcLutCreate();
  printCRC8(gKeyCode1);
  printCRC8(gKeyCode2);
  printCRC8(gKeyCode3);
  printCRC8(gKeyCode4);
  printCRC8(gKeyCode5);
  printCRC8(gKeyCode6);
  printCRC8(gKeyCode7);
  printCRC8(gKeyCode8);
  printCRC8(gKeyCode9);
  printCRC8(gKeyCode10);
  printCRC8(gKeyCode11);
  printCRC8(gKeyCode12);
  return 0;
}
