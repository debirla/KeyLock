/****************************************************************************
 Timer 0 functions - D.Lahiri 08Oct2017
;*                                                                          *
;****************************************************************************/

// Section: Included Files

#include <xc.h>
#include "tmr0.h"

void TMR0_Initialize(void)
{
  /* OPTION_REG 
     bit 7 WPUEN: Weak Pull-up Enable bit(1)
           1 = Weak pull-ups are disabled
           0 = Weak pull-ups are enabled by individual PORT latch values
     bit 6 INTEDG: Interrupt Edge Select bit
           1 = Interrupt on rising edge of INT pin
           0 = Interrupt on falling edge of INT pin
     bit 5 T0CS: TMR0 Clock Source Select bit
           1 = Transition on T0CKI pin
           0 = Internal instruction cycle clock (FOSC/4)
     bit 4 T0SE: TMR0 Source Edge Select bit
           1 = Increment on high-to-low transition on T0CKI pin
           0 = Increment on low-to-high transition on T0CKI pin
     bit 3 PSA: Prescaler Assignment bit
           1 = Prescaler is inactive and has no effect on the Timer 0 module
           0 = Prescaler is assigned to the Timer0 module
     bit 2-0 PS<2:0>: Prescaler Rate Select bits
           000  1 : 2
           001  1 : 4
           010  1 : 8
           011  1 : 16
           100  1 : 32
           101  1 : 64
           110  1 : 128
           111  1 : 256
  */
  OPTION_REG = 0x00;  // Pre-scaler = 2
  
  TMR0_Load();

  // Clearing IF flag.
  // Done in INTCON
}


// End of File

