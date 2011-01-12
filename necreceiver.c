#include "necreceiver.h"

void main(void)
{
	__disable_interrupt();
	sysInit();
	setupTimer();
	__enable_interrupt();
	
	while (!(UCA0TXIFG));
	UCA0TXBUF = 0xAA;								// Send serial code to PC to identify as a Serial PS/2 Input Device
	InputMode.AlphaMode = False;
	
	// Turn off CPU and switch to Low Power Mode 3
	__bis_SR_register(LPM3_bits + GIE);
	
	
	while (1) {
		
		/* When a valid IR Command for the specific remote address
		 * is available decode the command and send it through
		 * UART and turn CPU off and switch to Low power mode again. 
		 */
		 if (!CodeRepeat) {
		 	decodeCommand();
		 	encodePackets();
		 }
		 if (InputMode.ModeType !=0) sendPackets();
		 __bis_SR_register(LPM3_bits);
	}
}

void sysInit(void) {
	// Initialize System Clocks
	WDTCTL = WDTPW + WDTHOLD;							// Stop Watchdog Timer
	BCSCTL1 = CALBC1_1MHZ;								// Set Basic Clock -> 1Mhz
    DCOCTL = CALDCO_1MHZ;  								// Set DCO -> 1Mhz
	BCSCTL3 |= LFXT1S_2;								// Set ACLK -> VLOCLK = 12Khz
	
	// Initialize GPIO
	P1DIR |= LED0 + LED1;
	P1OUT |= LED1;
	P1OUT &= ~LED0;
	P1SEL |= IRIN;										// IR Detector - Capture Mode - TA1
	P1DIR &= ~IRIN;
	P3SEL = 0x30;										// UART - P3.4,5 = USCI_A0 TXD/RXD
	
	// Initialize UART
	UCA0CTL1 |= UCSSEL_2;                   			// SMCLK
	UCA0BR0 = 104;                          			// 1MHz 9600
	UCA0BR1 = 0;                            			// 1MHz 9600
	UCA0MCTL = UCBRS0;                      			// Modulation UCBRSx = 1
	UCA0CTL1 &= ~UCSWRST;               			    // Initialize USCI state machine
	IE2 |= UCA0RXIE;									// Enable UCA0 Receive Interrupt
}


void setupTimer(void) {
	// Initialize TimerA
	TACCTL1 = CM_2 + CCIS_0 + SCS + CAP + CCIE;			// Capture Mode P1.2
	TACCR2 = 1000;
	TACCTL2 = CM_2 + CCIE;
	TACTL = TASSEL_1 + MC_2 + TAIE;						// Source -> ACLK, Continuous Mode
}


// P1.2 Capture Interrupt - Receive IR Codes - NEC Protocol 
#pragma vector=TIMERA1_VECTOR
__interrupt void ISR_receiveIR(void)
{
  switch (TAIV)
  {
  	// TACCR1 Interrupt
    case  2:  {
    	++EdgeCount;
    	if (BitCount == 16 && IRAddress != REMOTE_ADDRESS) {
    	resetDetection();
    	break;	
    	}   	 
    	if (EdgeCount !=1) {
    		TimeInterval = TACCR1 - BitStartTime;
    		
    		/* If EdgeCount is 2, i.e, Complete First Pulse Received, Check for
    	 	* validity of the NEC Protocol and reset detection if invalid.
    	 	* And for any other edges, i.e, NEC pulse timing is satisfied already, 
    	 	* proceed with the decoding of the NEC Protocol.
    	 	*/
    	 	
    		if (EdgeCount ==2) {
    			if (TimeInterval < BIT_TIME_REPEAT || TimeInterval >BIT_TIME_STARTEND) {
    				// Non-NEC IR Data -> Discard
    				TACCTL1 &= ~CCIFG;
    				resetDetection(); 
    				break;	
    			}
    			if (TimeInterval < BIT_TIME_START) {
    				// NEC Code Repeat Signal Detected
						resetDetection();
						CodeRepeat=1;
						__bic_SR_register_on_exit(LPM3_bits);
    					break;
    			}
    		} else {
    			if (TimeInterval >BIT_TIME_MAX || TimeInterval < BIT_TIME_MIN) { resetDetection(); break; }
    			// Decoding NEC Protocol
    			P1OUT |= LED0; 								// Turn on LED0 to indicate the IR reception
    			CodeRepeat=0;								// Receiving new signal
    			// First 16bits of NEC -> Address
    			if (TimeInterval > BIT_TIME_HIGH) {
    				if (BitCount < 16) {
    					IRAddress <<=1; IRAddress |= 0x01;
    				} else {
    					IRCommand <<=1;IRCommand |= 0x01;	
    				}
    				// Last 16 bits of NEC -> Command
    			} else {
    				if (BitCount < 16) {
    					IRAddress <<=1;
    				} else {
    					IRCommand <<=1;		
    				}
    			}
    			BitCount++;
    		}
    			
    	}
    	BitStartTime = TACCR1;
    	TACCTL1 &= ~CCIFG;
    	if (BitCount > 31) { 
			resetDetection();
			__bic_SR_register_on_exit(LPM3_bits);
    	}
    	break;
    }
    //TACCR2 Interrupt
    case  4:  if (EdgeCount==1) EdgeCount=0; break; 			// Edge Fault Correction
    case 10: break; 											// Timer Overflow Interrupt - Not used
  }
}

void resetDetection(void) {
    // Reset Timer
    TACTL = TACLR;
    TACTL = TASSEL_1 + MC_2 + TAIE;
    // Clear IR-Pulse Train Specific Variables
	BitCount =0;
    EdgeCount =0;
    CodeRepeat =0;
    P1OUT &= ~LED0;												// Turn off LED0

}

void decodeCommand(void) {

	DecoderInput = IRCommand;
	InputMode.ModeType=KeyBoard;
	InputMode.RepeatEnabled=0;
	switch (DecoderInput) {
		case KEYCODE_0: DecodeRegister=0x45; break;
		case KEYCODE_1: DecodeRegister=0x16; break;
		case KEYCODE_2: DecodeRegister=0x1E; break;
		case KEYCODE_3: DecodeRegister=0x26; break;
		case KEYCODE_4: DecodeRegister=0x25; break;
		case KEYCODE_5: DecodeRegister=0x2E; break;
		case KEYCODE_6: DecodeRegister=0x36; break;
		case KEYCODE_7: DecodeRegister=0x3D; break;
		case KEYCODE_8: DecodeRegister=0x3E; break;
		case KEYCODE_9: DecodeRegister=0x46; break;
		case KEYCODE_OK: DecodeRegister=0x5A; break;
		case KEYCODE_CHUP: 
			DecodeRegister=0x6B; 
			InputMode.ModeType=KeyBoardExtended;
			InputMode.RepeatEnabled=1;
			break;
		case KEYCODE_CHDOWN: 
			DecodeRegister=0x74; 
			InputMode.ModeType=KeyBoardExtended;
			InputMode.RepeatEnabled=1;  
			break;
		case KEYCODE_VOLUP: 
			DecodeRegister=0x75; 
			InputMode.ModeType=KeyBoardExtended;
			InputMode.RepeatEnabled=1;  
			break;
		case KEYCODE_VOLDOWN: 
			DecodeRegister=0x72; 
			InputMode.ModeType=KeyBoardExtended;
			InputMode.RepeatEnabled=1; 
			break;
		case KEYCODE_UP: 
			DecodeRegister='0'; 
			InputMode.ModeType=Mouse;
			InputMode.RepeatEnabled=1;
			break;
		case KEYCODE_DOWN: 
			DecodeRegister='1';  
			InputMode.ModeType=Mouse;
			InputMode.RepeatEnabled=1;
			break;
		case KEYCODE_RIGHT:
			DecodeRegister='2'; 
			InputMode.ModeType=Mouse;
			InputMode.RepeatEnabled=1;
			break;
		case KEYCODE_LEFT: 
			DecodeRegister='3'; 
			InputMode.ModeType=Mouse;
			InputMode.RepeatEnabled=1;
			break;
		case KEYCODE_LBUTTON: 
			DecodeRegister='4';
			InputMode.ModeType=Mouse;
			break;
		case KEYCODE_RBUTTON: 
			DecodeRegister='5';
			InputMode.ModeType=Mouse;
			break;
		case KEYCODE_Mode: 
			DecodeRegister=0xFF; 
			InputMode.ModeType=MCUCommand; 
			if (InputMode.AlphaMode == True) InputMode.AlphaMode = False;
			else InputMode.AlphaMode == True;
			break;
		default: 
			DecodeRegister='\0'; 
			InputMode.ModeType=MCUCommand;
			InputMode.RepeatEnabled=0;
	}
	if (InputMode.AlphaMode == True) DecodeRegister=alphaDecode();

}


void encodePackets() {
	switch(InputMode.ModeType) {
		case KeyBoard:
			PS2Packet[0] = DecodeRegister;
			PS2Packet[1] = 0xF0;
			PS2Packet[2] = DecodeRegister;
			break;
		case KeyBoardExtended:
			PS2Packet[0] = 0xE0;
			PS2Packet[1] = DecodeRegister;
			PS2Packet[2] = 0xF0;
			PS2Packet[3] = 0xE0;
			PS2Packet[4] = DecodeRegister;
			break;
		case Mouse:
			PS2Packet[0] = DecodeRegister;
			PS2Packet[1] = 0xF0;
			PS2Packet[2] = DecodeRegister;
			break;
		default:
			break;
	}
}

void sendPackets(void) {
	if ((CodeRepeat==0 || InputMode.RepeatEnabled==1) && DecodeRegister !='\0') {
		int numberOfPackets=3;
		if (InputMode.ModeType > 2) numberOfPackets=5; 
		for (i=0;i<numberOfPackets;i++) {
			while (!(UCA0TXIFG));
			UCA0TXBUF = PS2Packet[i];
		}
	}
}

// USCI A0/B0 Receive ISR - Recieve and send pre-formulated reples to the commands
#pragma vector=USCIAB0RX_VECTOR
__interrupt void receiveSerial_ISR(void)
{
  unsigned char SerialCommand = UCA0RXBUF;
  unsigned char SerialResponse ='\0';
  switch (SerialCommand)
  {
	case 0xFF: SerialResponse = 0xAA; break;					// Reset
	case 0xF6: SerialResponse = 0xAA; break;					// Set Default & Reset
	case 0xEE: SerialResponse = 0xEE; break;					// Echo
	case 0xF2: SerialResponse = 0xAB; break;					// Keyboard ID
	case 0xF0: SerialResponse = 0x03; break;					// Get ScanCode Set
	default: break;
  }
  IE2 |= UCA0TXIE; 	                       						// Enable USCI_A0 TX interrupt
  UCA0TXBUF = 0xFA;
  while (!(UCA0TXIFG));
  if (SerialResponse!='\0') UCA0TXBUF = SerialResponse;
}

/* If AlphaMode== True, then decode the numbers pressed in the remote to characters
 * Wait for a 100ms to see if the same key is pressed again, and if it iterate through the
 * characters of the corresponding number, or else proceed to sending the current
 * packet 
 */
unsigned char alphaDecode(void) {

/* If the same key is pressed again, instruct the sendPackets function not
 * send anything by setting the DecodeResigter(Return value) to '/0'
 */
return '\0';	
}

