#ifndef NECRECEIVER_H_
#define NECRECEIVER_H_

#include "msp430x22x4.h"
#include "keycodes.c"

/* Prototypes */
void sysInit(void);					// Initialize System Clocks and GPIO
void setupTimer(void); 					// Initialize Timer A
void resetDetection(void); 				// Reset Timer A and Pulse Train Specific Variables
void decodeCommand(void); 				// Decode the IRCommand just received
void encodePackets(void);				// Encode the packets into PS/2 Format
void sendPackets(void); 				// Send the packets via UART
unsigned char alphaDecode(void);			// Decode alphabet from numbers and return the character

/* Interrupts */
__interrupt void receiveIR_ISR(void); 		        // Receive the IR Pulse train
__interrupt void receiveSerial_ISR(void);	        // Receive Serial Input

/* Pulse Width Constants */
#define BIT_TIME_START 156
#define BIT_TIME_STARTEND 180
#define BIT_TIME_REPEAT 132
#define BIT_TIME_HIGH 24
#define BIT_TIME_MAX 30
#define BIT_TIME_MIN 12

/* GPIO Constants */
#define LED0 0x01
#define LED1 0x02
#define IRIN 0x04

/* Global Variables */
unsigned int IRAddress=0, IRCommand=0;
unsigned int BitStartTime=0, TimeInterval=0;
unsigned int BitCount=0, EdgeCount=0;
unsigned int CodeReady=0, CodeRepeat=1;
unsigned int DecoderInput=0;
unsigned char DecodeRegister=0;
unsigned char PS2Packet[5];
int i=0, j=0;

struct InputModeInterface {
	enum {
		MCUCommand=0,
		KeyBoard=1,
		Mouse=2,
		KeyBoardExtended=3
	} ModeType;
	enum { True=1, False=0 } AlphaMode;
	int RepeatEnabled;
} InputMode;


#endif /*NECRECEIVER_H_*/
