#include "msp430x22x4.h"

/* RC5 Protocol Receiver algorithm
   using Hardware-Interrupt based on Capture/Compare Mode
   using MSP430. Algorithm-Only - Test Code 
*/

/* Prototypes */
void sysInit(void);
void setupIR(void);
void setupTimer(void);

/* GPIO Constants */
#define LED0 0x01
#define LED1 0x02
#define IRIN 0x04

#define BIT_TIME_3BY4 17

int IRData=0;
int BitCount=0;

void main(void)
{
	__disable_interrupt();
	sysInit();
	setupIR();
	setupTimer();
	__enable_interrupt();
	
  __bis_SR_register(LPM3_bits + GIE);
}

void sysInit(void) {
  WDTCTL = WDTPW + WDTHOLD;  
  BCSCTL3 |= LFXT1S_2;	
}

void setupIR(void) {
P1DIR |= LED0 + LED1;
P1OUT |= LED1;
P1OUT &= ~LED0;
P1SEL |= IRIN;
P1DIR &= ~IRIN;
}

void setupTimer(void) {
  
  TACCTL1 = CM_2 + CCIS_0 + SCS + CAP + CCIE;
  TACTL = TASSEL_1 + MC_2 + TAIE;
}

#pragma vector=TIMERA1_VECTOR
__interrupt void ReceiveIR(void)
{
  switch (TAIV)
  {
    case  2:  {
    	P1OUT |= LED0;
    	if (BitCount > 12) { 
    		BitCount =0;
    		TAR=0;
    		TACCTL1 = CM_2 + CCIS_0 + SCS + CAP + CCIE;
    		TACCTL1 &= ~CCIFG;
    		P1OUT &= ~LED0;
    		break;
    	}
    	if (TACCTL1 & CAP) {
    	TACCTL1 &= ~CAP;
    	TACCR1 += BIT_TIME_3BY4;
    	} else {
    	IRData <<= 1;
    	if (TACCTL1 & SCCI) { IRData |= 0x01;}
    	TAR =0;
    	TACCTL1 = CM_3 + CCIS_0 + SCS + CAP + CCIE;
    	BitCount++;	
    	}
    	break;
    }
    case  4:  break;
    case 10:  break;
  }
}
