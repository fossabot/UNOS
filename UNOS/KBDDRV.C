
/**************************************************************************/
/*                                                                        */
/*                                                                        */
/*                         KEYBOARD DRIVER MODULE                         */
/*                                                                        */
/*                                  by                                    */
/*                                                                        */
/*                  L. J. Sciacca, P. J. Moylan, A. Murree Allen,         */
/*                                                                and Bob Betz                            */
/*                      Department of Electrical and Computer Engineering         */
/*                            University of Newcastle                     */
/*                                  Austraila                             */
/*                                                                        */
/*                         ( Copyright 1988, 89, 90, 91, 92 )             */
/*                                                                        */
/*                                                                        */
/**************************************************************************/

/*
	HISTORY

LAST MODIFICATION :     19th February, 1992
BY :                    Robert Betz

	Originally, Peter J. Moylan wrote driver for PMOS. Hacked into a sort of
keyboard driver by a fourth year student. Then adapted by L. J. Sciacca
for UNOS.

Modified for the Cook Islands project by Bob Betz. Modifications related
to the provision of an initialisation procedure in UNOS Version 1.5a. In
addition the method of generating semaphores has also been modified. In
general the whole module was tidied up. Because of the evolutionary nature
of the development of this module it would probably best be rewritten to
get it nice.

Revision notes:

		17-10-1990
		Major mods to fit into UNOS. LJS

		3-11-1990
		More major mods to put all keyboard code into one module
		to keep it together with only 1 driver tasks, rather than the 3 tasks
		required in the first version. LJS

		1-3-1991
		More major major mods in attempt to tidy up code. LJS

		2-3-1991
		Really needs to be rewritten as original C conversion was a real
		Hack job.

		16-2-92
		Modified to take advantage of the new UNOS initialisation procedure
		and the new semaphore creation.

		18-2-92
		Changed variable declarations to "Hungarian" notation. Also improved
		format and layout. Added Ctrl-Alt-Del as UNOS abort. Added the exit
		procedure execution so that interrupt vectors etc are cleaned up.
*/
#include <stdio.h>
#include <stdlib.h>
#include <conio.h>
#include <dos.h>
#include "unos.h"
#include "general.h"
#include "hwpc.h"       /*    (this had // in front of it )   */
#include "kbddrv.h"
#include "main_ext.h"  // define LOG_TASK
/*

Known ScanCodeTranslator() "Faults"
1. Does not yet handle Print Screen.

2.  Pause (which BIOS handles by performing a busy wait inside it's
	keyboard interrupt routine) is not implemented nor detected, but can
	be with a little modification.  Since BIOS handles this key with an
	internal routine, it does not return a standard code whenever it is
	pressed (this is why this translator does not return a code when it
	has been pressed!).

3.  Does not check for F11 and F12 of the extended keytboard.

*/





/****************************************************************************/
/*                                                                          */
/*                                                                                                                                                      */
/*                                                                      DEFINES                                                                 */
/*                                                                                                                                                      */
/*                                                                                                                                                      */
/****************************************************************************/


#define LEFT_SHIFT_CODE 0X2A
#define LEFT_SHIFT_RELEASE 0XAA
#define RIGHT_SHIFT_CODE 0X36
#define RIGHT_SHIFT_RELEASE 0XB6
#define CTRL_CODE 0X1D
#define CTRL_RELEASE 0X9D
#define ALT_CODE 0X38
#define ALT_RELEASE 0XB8
#define CAPS_LOCK_CODE 0X3A
#define CAPS_LOCK_RELEASE 0XBA
#define NUM_LOCK_CODE 0X45
#define NUM_LOCK_RELEASE 0XC5
#define SPACE_CODE 0X39
#define KEYPAD_STAR_CODE 0X37
#define EXTENDED_CODE 224
#define MAX_NORMAL_CODE 0X35
#define MIN_SPECIAL_CODE 0X3B
#define MAX_SPECIAL_CODE 0X53
#define CTRL_PRT_SC 114
#define DEL_CODE 0x53                   /* Del Code for the keypad */
#define TRUE 1
#define FALSE 0

#define SPACE_CHARACTER 32
#define STAR_CHARACTER 42

#define INTERRUPT_NUMBER 9
#define CONTROL_8259 0X20
#define KEYBOARD_DATA 0X60
#define KEYBOARD_OUTPUT_PORT 0X60
#define KEYBOARD_CONTROL 0X61
#define KEYBOARD_STATUS_REG 0X64
#define LED_SCROLL_LOCK 1                       /* 00000001 */
#define LED_NUM_LOCK 2                          /* 00000010 */
#define LED_CAPS_LOCK 4                         /* 00000100 */

#define SC_BUFFER_LEN 64





/****************************************************************************/
/*                                                                                                                                                      */
/*                                                                                                                                                      */
/*                                                        FUNCTION PROTOTYPES                                                   */
/*                                                                                                                                                      */
/*                                                                                                                                                      */
/****************************************************************************/

static void fvdPutKeyboardBuffer ( unsigned char uchMainbyte,
										unsigned char uchAuxbyte );

static void fvdWaitForKeyboardReady ( void );

static void fvdSendKbdCommand(unsigned char uchCommand);

static void interrupt fvdKeyboardInterrupt ( void );
static void fvdRestoreOldDriver (void);
static void fvdSetupKbdDriver ( void );
static void fvdResetScBuffer ( void );

static unsigned char fuchGetScanCode ( void );

static void fvdToggleLED ( unsigned char uchLEDcode );

static void fvdClearLED (unsigned char uchLEDcode);






/****************************************************************************/
/*                                                                                                                                                      */
/*                                                                                                                                                      */
/*                                                        VARIABLE DECLARATIONS                                                 */
/*                                                                                                                                                      */
/*                                                                                                                                                      */
/****************************************************************************/


/* Below are the lower case and upper case translations of scan codes 0-53 */
/* into ASCII for the US QWERTY keyboard.  The codes may be customized for */
/* other keyboard configurations, such foreign or Dvorak keyboards.   */

unsigned char a1uchLowerCase[54] = {0,27,49,50,51,52,53,54,55,56,57,48,45,
					61,8,9,113,119,101,114,116,121,117,105,111,112,91,93,
					13,0,97,115,100,102,103,104,106,107,108,59,39,
					96,0,92,122,120,99,118,98,110,109,44,46,47};

unsigned char a1uchUpperCase[54] = {0,27,33,64,35,36,37,94,38,42,40,41,95,
					43,8,15,81,87,69,82,84,89,85,73,79,80,123,125,13,0,65,
					83,68,70,71,72,74,75,76,58,34,126,0,124,90,88,
					67,86,66,78,77,60,62,63};

unsigned char a1uchAltNormalKey[54] ={0,27,120,121,122,123,124,125,126,127,
					128,129,130,131,8,9,16,17,18,19,20,21,22,23,24,25,91,
					93,13,0,30,31,32,33,34,35,36,37,38,59,39,96,
					0,92,44,45,46,47,48,49,50,44,46,47};

/* The following tables are translations for the scan codes of special   */
/* keys (function keys, keypad keys, arrows, etc.). The translated codes */
/* are the same as those returned by the IBM BIOS.  Different codes are  */
/* returned for some combinations of the special keys and SHIFT/CTRL.    */

unsigned char a1uchSpecialKey[26] = {59,60,61,62,63,64,65,66,67,68,0,0,71,
				72,73,45,75,76,77,43,79,80,81,82,83};

unsigned char a1uchShiftSpecialKey[26] = {84,85,86,87,88,89,90,91,92,93,0,
				0,55,56,57,45,52,53,54,43,49,50,51,48,46};

unsigned char a1uchCtrlSpecialKey[26] =  {94,95,96,97,98,99,100,101,102,
				103,0,0,119,0,132,45,115,0,116,43,117,0,118,0,0};

unsigned char a1uchAltSpecialKey[26] = {104,105,106,107,108,109,110,111,
				112,113,0,0,71,72,73,45,75,76,77,43,79,80,81,82,83};


/*------ Scan code buffer data structure - used by interrupt routine ------*/

static struct {
	unsigned char a1uchData [ SC_BUFFER_LEN ];     /* array to hold scan codes */
	int inHead;
	int inTail;           /* Pointers to queue head,tail */
	int inBufferFull;     /* Flag indicating buffer full */
	} stScBuffer;

static unsigned char uchLEDstatus = 0;     /* Current LED status */

/*------ Variable used to store the semaphore number used ------*/
static unsigned int uinKbdCountSem;


/* Location used to store the task name pointer which represents the
task that is connected to the keyboard.
*/
static char *pchConnectedTaskName = NULL;


/*---- Vector of old keyboard driver (DOS) */
void interrupt ( *pfvdOldKeyboardDriver ) ( );






/*
============================================================================
|
| fvdInitScanCodeTranslatorTask
|
| This function initialises the variables and semaphores required for the
| fvdScanCodeTranslatorTask.
|
| Parameters    :       none
|
| Returns               :       nothing
|
============================================================================
*/

void fvdInitScanCodeTranslatorTask (void) {


	/*------ Create and initialise the semaphore used ------*/
	uinKbdCountSem = create_semaphore ();
	init_semaphore (uinKbdCountSem, 0, 1);

	fvdSetupKbdDriver ();

	/* Define an exit procedure which resets up the keyboard interrupt */
	atexit(fvdRestoreOldDriver);

} /* end of fvdInitScanCodeTranslatorTask */





/*-------------------------------------------------------------------------*/





/*
============================================================================
| fvdScanCodeTranslatorTask
|
|       Intermediate keyboard task. This task gets the characters from the scan
| buffer that is being filled by the interrupt routine. The scan code that is
| obtained from the scan code buffer is translated into an ascii code
| plus an auxiliary byte. Some scan codes cannot be converted into ascii as
| they represent special characters on the PC keyboard. Characters are sent
| off to the required task via the operating system's mailbox facility.
|
|       This is A Murree Allens code. Bit messy. Tidied up a bit by Bob Betz.
|
| Parameters    :       none
|
| Returns               :       nothing
|
============================================================================
*/

void fvdScanCodeTranslatorTask ( void* DummyVariable ) {

	unsigned char uchScanCode,uchMainByte,uchAuxByte=0;
	unsigned char uchShiftDown = FALSE;
	unsigned char uchCtrlDown = FALSE;
	unsigned char uchAltDown = FALSE;
	unsigned char uchCapsLockDown = FALSE, uchCapsLock = FALSE;
	unsigned char uchNumLockDown = FALSE, uchNumLock = TRUE;
	unsigned char uchExtended = FALSE;
	FILE *fp;

	/* now make sure that the keyboard leds reflect the state of the
	Caps lock and the Num lock.
	*/
	DummyVariable = DummyVariable;

	if (uchCapsLock) {
		fvdClearLED (LED_CAPS_LOCK);
		fvdToggleLED (LED_CAPS_LOCK);
	} /* if */
	else {
		fvdClearLED (LED_CAPS_LOCK);
	} /* else */

	if (uchNumLock) {
		fvdClearLED (LED_NUM_LOCK);
		fvdToggleLED (LED_NUM_LOCK);
	} /* if */
	else {
		fvdClearLED (LED_NUM_LOCK);
	} /* else */


	/* Infinite loop task */
	while ( 1 ) {

		/* rjlov */
#if LOG_TASK
		fp = get_fileptr ( );
		fwrite((const char *)"Ks",1,2,fp);
#endif

		uchMainByte = 0;

		/*------ Get a scan code from the scan code buffer ------*/
		uchScanCode = fuchGetScanCode ( );

		/* Test if the code indicates that an extended scan code sequence
		follows (ie. 224 followed by another scan code). If so, turn on
		a flag so that we know when we're currently reading an extended
		sequence.
		*/

		if ( uchScanCode == EXTENDED_CODE ){
			/* Get next code */
			uchScanCode = fuchGetScanCode ( );

			/* Beginning of sequence */
			uchExtended = TRUE;

			/* Set bit 16 */
			uchAuxByte= uchAuxByte | 0X40;
		} /* if */
		else {
			/* end of sequence */
			uchExtended = FALSE;

			/* clear bit 16 */
			uchAuxByte= uchAuxByte & 0XBF;
		} /* else */

		/* Check to see if the key being pressed is a modifier key
		(SHIFT, CTRL, ALT) and set the flags accordingly. If the
		scan code is an extended code (indicated by 'uchExtended')
		then the CTRL or ALT key will be the right-hand key.
		*/

		switch ( uchScanCode ) {
			case ALT_CODE:
				uchAltDown = TRUE;         /* ALT key pressed */
				if (uchExtended == FALSE) {
					/* Set bit 5 (Left key) */
					uchAuxByte = uchAuxByte | 0X20;
				} /* if */
				else  {
					/* Set bit 4 (Right key) */
					uchAuxByte = uchAuxByte | 0X10;
				} /* else */
				break;

			case ALT_RELEASE:
				uchAltDown = FALSE;        /* ALT key released */
					if (uchExtended == FALSE) {
						/* Clear bit 5 (Left key) */
						uchAuxByte = uchAuxByte & 0xDF;
					} /* if */
				else {
					/* Clear bit 4 (Right key) */
					uchAuxByte = uchAuxByte & 0xEF;
				} /* else */
				break;


			case CTRL_CODE:
				uchCtrlDown = TRUE;        /* CTRL key pressed */
				if (uchExtended == FALSE) {
					uchAuxByte = uchAuxByte | 0x08;   /* Set bit 3 (Left key) */
				} /* if */
				else {
					uchAuxByte = uchAuxByte | 0x04;   /* Set bit 2 (Right key) */
				} /* else */
				break;


			case CTRL_RELEASE:
				uchCtrlDown = FALSE;               /* CTRL key released */
				if (uchExtended == FALSE) {
					uchAuxByte = uchAuxByte & 0XF7;   /* Clear bit 3 (Left key) */
				} /* if */
				else {
					uchAuxByte = uchAuxByte & 0XFB;   /* Clear bit 2 (Right key) */
				} /* else */
				break;

			case LEFT_SHIFT_CODE:
				uchShiftDown = TRUE;       /* Left SHIFT pressed */
				uchAuxByte = uchAuxByte | 0X02;   /* Set bit 1 */
				break;

			case RIGHT_SHIFT_CODE:
				uchShiftDown = TRUE;       /* Right SHIFT pressed */
				uchAuxByte = uchAuxByte | 0X01;   /* Set bit 0 */
				break;

			case LEFT_SHIFT_RELEASE:
				uchShiftDown = FALSE;      /* Left SHIFT released */
				uchAuxByte = uchAuxByte & 0XFD;   /* Clear bit 1 */
				break;

			case RIGHT_SHIFT_RELEASE:
				uchShiftDown = FALSE;      /* Right SHIFT released */
				uchAuxByte = uchAuxByte & 0XFE;   /* Clear bit 0 */
				break;

			/* Detect the Caps Lock key, but prevent it from
			auto-repeating
			*/
			case CAPS_LOCK_CODE:
				if ( !uchCapsLockDown ) {
					uchCapsLock = !uchCapsLock;
					uchCapsLockDown = TRUE;
					fvdToggleLED (LED_CAPS_LOCK);
				} /* if */
				break;

			case CAPS_LOCK_RELEASE:
				uchCapsLockDown = FALSE;
				break;

			/* Detect the Num Lock key, but prevent it from
			auto-repeating
			*/
			case NUM_LOCK_CODE:
				if (!uchNumLockDown){
					uchNumLock = !uchNumLock;
					uchNumLockDown = TRUE;
					fvdToggleLED ( LED_NUM_LOCK );
				} /* if */
				break;

			case NUM_LOCK_RELEASE:
				uchNumLockDown = FALSE;
				break;

			/* Detect the space bar and keypad star separately since
			their scan codes fall outside the range of the translation
			tables.  For both ASCII characters, clear bit 7 of uchAuxByte;
			for special/function keys
			*/

			/* set bit 7.*/
			case SPACE_CODE:
				uchAuxByte = uchAuxByte & 0X7F;
				fvdPutKeyboardBuffer ( SPACE_CHARACTER, uchAuxByte );
				break;

			case KEYPAD_STAR_CODE:
				if (uchCtrlDown){
					uchAuxByte = uchAuxByte | 0X80;
					fvdPutKeyboardBuffer ( CTRL_PRT_SC, uchAuxByte );
				}  /* if */
				else {
					uchAuxByte = uchAuxByte & 0X7F;
					fvdPutKeyboardBuffer ( STAR_CHARACTER, uchAuxByte);
				} /* else */
				break;

			default: {

				/* All other codes with the high order bit set are ignored,
				since they are only key releases which we don't need to
				know about.
				*/

				if (uchScanCode >= 0X80) {

					;   /* do nothing */

				} /* if */
				else {
					/* All remaining scan codes can be translated by
					doing a table lookup of the translation tables.
					The result may be affected by the current state of
					the Modifier keys (SHIFT,CTRL,ALT). For 'normal'
					scan codes (not function/special keys), clear bit
					7 of the uchAuxByte to  indicate they're ASCII characters.
					ALT combinations are classed as special keys.
					*/
					if (uchScanCode <= MAX_NORMAL_CODE){
						/* Letters, numbers etc */
							if (uchAltDown){
								uchAuxByte = uchAuxByte | 0X80;
								uchMainByte = a1uchAltNormalKey[uchScanCode];
						} /* if */
						else {
							uchAuxByte = uchAuxByte & 0X7F;
							if (uchShiftDown) {
								uchMainByte = a1uchUpperCase[uchScanCode];
								if (uchCapsLock && (uchMainByte >= 65) &&
													(uchMainByte <= 90)) {
									uchMainByte = a1uchLowerCase[uchScanCode];
								} /* if */
							} /* if */
							else {
								uchMainByte = a1uchLowerCase [ uchScanCode ];
								if ( uchCapsLock && ( uchMainByte >= 97 )
											&& ( uchMainByte<= 122 ) ) {
									uchMainByte = a1uchUpperCase [ uchScanCode ];
								} /* if */
							} /* else */
							if (uchCtrlDown) {
								uchMainByte = ( uchMainByte & 0X1F );
							} /* if */
						} /* else */
						fvdPutKeyboardBuffer ( uchMainByte, uchAuxByte );
					} /* if */
					else {
						/* Check for numeric keypad and function keys and
						combination of these with any of the modifier keys
						(SHIFT/CTRL/ALT).   Set bit 7 of the uchAuxByte to
						indicate they are special (non-ASCII) keys.
						*/
						if ( ( uchScanCode >= MIN_SPECIAL_CODE )
							&& ( uchScanCode <= MAX_SPECIAL_CODE ) ) {

							/* now check to see if the Ctrl-Alt-Del key
							has been pressed. If so then stop the system.
							*/
							if (uchCtrlDown && /* uchAltDown && */
										(uchScanCode == 't')) {
								/*
								!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
								NOW LEAVE THE SYSTEM AND RETURN TO DOS
								!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
								*/
								//CONTINUE = FALSE;
								// fvdRestoreOldDriver();
				 //				exit (EXIT_SUCCESS);
							} /* if */

							uchAuxByte = uchAuxByte | 0x80;

							if ( uchAltDown ) {
								uchMainByte = a1uchAltSpecialKey
									[ uchScanCode - MIN_SPECIAL_CODE ];
							} /* if */
							else {
								if ( uchCtrlDown ) {
									uchMainByte = a1uchCtrlSpecialKey
										[ uchScanCode - MIN_SPECIAL_CODE ];
								} /* if */
								else {
									if ( ( uchNumLock == uchShiftDown )
														|| uchExtended) {
										uchMainByte = a1uchSpecialKey
											[ uchScanCode - MIN_SPECIAL_CODE ];
									} /* if */
									else {
										uchMainByte = a1uchShiftSpecialKey
											[ uchScanCode - MIN_SPECIAL_CODE ];
									} /* else */
								} /* else */
							} /* else */
							fvdPutKeyboardBuffer ( uchMainByte, uchAuxByte );
						} /* if */
					} /* else */
				} /* else */
			} /* end of default */
		} /* end of switch statement */
	} /* while */
} /* end of scan_code_task */





/*-------------------------------------------------------------------------*/





/*
============================================================================
| fuchGetScanCode ( )
|
|       Routine that waits for data from the keyboard interrupt routine. This
| should be called from the scan_code_task.
|
| Parameters    :       none
|
| Returns               :       Scan code from the keyboard scan code buffer
|
============================================================================
*/

static unsigned char fuchGetScanCode ( ) {
		      
	unsigned char uchResult;

	/* Wait until there is a scan code available in the buffer (the     */
	/* interrupt routine will signal the semaphore each time it services*/
	/* a keyboard interrupt).*/

	wait ( uinKbdCountSem );

	/* Retrieve the next scan code from the buffer */
	uchResult = stScBuffer.a1uchData [ stScBuffer.inHead ];

	/* Update the inHead pointer - this operation is a Critical section */
	/* and so we inform the operating system that we require critical */
	/* section protection.*/

	disable ( );

	if ( stScBuffer.inHead < ( SC_BUFFER_LEN -1 ) ) {
		stScBuffer.inHead++;
	} /* if */
	else {
		stScBuffer.inHead = 0;
	} /* else */

	stScBuffer.inBufferFull = FALSE;        /* Must be less 1 at least */

	enable ( );

	/* Return the actual scan code through the function fuchGetScanCode() */

	return ( uchResult );
} /* end of fuchGetScanCode */





/*------------------------------------------------------------------------*/





/*
============================================================================
|
| fvdPutKeyboardBuffer
|
|       Routine to send the characters off to the task requiring keyboard input.
| This is done via UNOS's mailbox system rather than using an intermediate
| FIFO buffer.
|
|       Two characters are sent at a time.
|
| Parameters    :       main translated character
|                                       auxiliary translated character
|
| Returns               :       nothing
|
============================================================================
*/
static void fvdPutKeyboardBuffer
			( unsigned char uchMainByte, unsigned char uchAuxByte ) {

	unsigned char a1uchData[2];

	/* check to see if the keyboard task is connected to any other task. If
	not then do nothing - that is the character types in is simply dumped.
	*/


	if (pchConnectedTaskName != NULL) {
		a1uchData[0] = uchMainByte;
		a1uchData[1] = uchAuxByte;

		disable ();
		if (free_mbx (pchConnectedTaskName) >= 2) {
			send_mess (a1uchData, (unsigned int)2,
					pchConnectedTaskName);
		} /* if */
		enable ();
	} /* if */

} /* end of fvdPutKeyboardBuffer */









/*------------------------------------------------------------------------*/





/*
============================================================================
|
| fvdConnectToKeyboard
|
| This function connects a task to the keyboard. This effectively makes the
| name of the task passed in the recipient of any characters typed on the
| keyboard. This function should be called during the initialisation phase
| of the relevant to connect the appropriate task to the keyboard driver.
|
| Parameters    :       pointer to the task name that one wishes to connect to
|                                       the keyboard.
|
| Returns               :       nothing
|
============================================================================
*/

void fvdConnectToKeyboard (char *pchTaskName) {

	pchConnectedTaskName = pchTaskName;

} /* end of fvdConnectToKeyboard */





/*-------------------------------------------------------------------------*/





/* IBM PC/AT "KBDRV.C" keyboard driver routine
   'C' version by Andrew Murree Allen
   Modified: 4/6/90 Finished working version
		   24/7/90 Modified to allow testing
*/
/*
This module includes an interrupt routine which services the
keyboard interrupt request by getting the keyboard scan code
and placing it within the buffer for later retrieval.  No
translation of the scan codes takes place in this module.

*/

/*****************************************************************/
/* Keyboard technical details                                    */
/* The keyboard status register is 8-bit read only at Port 64H.  */
/*    Bit 7   Parity error                                       */
/*    Bit 6   Receive time-out                                   */
/*    Bit 5   Transmit time-out                                  */
/*    Bit 4   Keyboard Clock Enabled (0 means keyboard disabled) */
/*    Bit 3   Command or data                                    */
/*    Bit 2   System flag (0 during self test)                   */
/*    Bit 1   Command/data input buffer full                     */
/*    Bit 0   Data output buffer full (1 means data available)   */
/*                                                               */
/* The keyboard microprocessor deposits the key scan code in the */
/* keyboard output buffer in Port A of the 8255 chip (Port 60H)  */
/* (or the equivalent chip on the AT).                           */
/*                                                               */
/* The keyboard command buffer is 8-bit write-only at Port 64H.  */
/* The only command used by this driver is Set/Reset (0EDH) for  */
/* changing the keyboard lock indicator LEDs.  This is followed  */
/* by a byte specifying which LEDs to turn on:                   */
/*    Bit 0   Scroll lock on/off                                 */
/*    Bit 1   Num lock on/off                                    */
/*    Bit 2   Caps lock on/off                                   */
/*    Bits 3-7 must be 0                                         */
/*                                                               */
/* See IBM AT Technical Reference Manual, pp9-11 Compatibility:  */
/*                                                               */
/*  "Back to back I/O commands to the same I/O ports will not    */
/*   permit enough recovery time for I/O chips"                  */
/*                                                               */
/* Theoretically a delay long enough for the I/O ports to recover*/
/* should be inserted between consecutive I/O  instructions to   */
/* the same I/O chip.  However, tests on a 386SX 16Mhz compatible*/
/* and a PS/2 30/286 10 Mhz have indicated no requirement for    */
/* this delay.                                                                                                   */
/*****************************************************************/




/*********************************************************************/
/* ScanCodeBufferType implementation                                 */
/* The keyboard scan codes are held in a buffer (implemented here as */
/* a circular queue (FIFO buffer)).  The buffer is an array of bytes */
/* whose length is defined by 'SC_BUFFER_LEN'. Two pointers keep track */
/* of the 'head' and 'tail' of the queue of strings in the buffer.   */
/*                                                                   */
/* New keyboard scan codes are placed at the position after the tail */
/* and the 'inTail' pointer is adjusted accordingly.  Once the highest */
/* memory location of the buffer has been filled, the insertion of   */
/* new codes wraps around to the low end of the buffer. 'inHead' points*/
/* to the next scan code which will be retrieved by the consumer.    */
/*                                                                   */
/* 'inBufferFull' indicates whether the buffer is completely full.     */
/* Once the buffer is full, additional incoming characters are       */
/* discarded and the interrupt beeps the speaker.                    */
/*********************************************************************/




/*------------------------------------------------------------------------*/





/*
============================================================================
|
| INTERRUPT KEYBOARD ROUTINE
|
| fvdKeyboardInterrupt
|
|       This interrupt routine which services the keyboard interrupt
| request (INT9) by reading the keyboard scan code and placing it in the
| "scan code buffer" for later retrieval.  No translation of the
| scan codes takes place in "KeyboardInterrupt".
|
| Parameters    :       none
|
| Returns               :       nothing
|
|
============================================================================
*/

static void interrupt fvdKeyboardInterrupt ( ) {

	unsigned char uchInputDatum, uchTemp;
	volatile char chDummyWait = 0;

	/* Tell the 8259A that we've acknowledged the hardware interrupt
	(send the "End Of Interrupt" command to the 8259A PPI's port B).
	*/

	outportb(CONTROL_8259,0X20);

	/*------ Read a scan code from KEYBOARD_DATA (Port A) ------*/
	uchInputDatum = inportb(KEYBOARD_DATA);

	/* See if the scan code buffer is full - if yes, then beep the speaker
	and discard the scan code, otherwise save the scan code in the buffer
	and update the pointers.*/

	if ( stScBuffer.inBufferFull ) {

		/* The buffer is full and the scan code cannot be saved.  The
		solution here is to throw away the scan code and to inform
		the user by beeping the speaker.

		The routines used here to beep the speaker are from "beep.c"
		module. However, the time delay (while the speaker is beeping)
		inside the interrupt routine should not be used.  Therefore,
		we should instead signal a "beep" task to beep once we have
		finished the interrupt routine.
		*/

	} /* if */
	else {

		/* Save the scan code in the buffer position pointed to by the
		tail pointer, then increment the tail. Make sure the tail ptr
		does not exceed the value (SC_BUFFER_LEN-1), since the buffer
		array has the subscripts from 0 --> (SC_BUFFER_LEN-1).  C will
		not bounds check if the array subscript exceeds SC_BUFFER_LEN-1.
		Wrap-around to bottom of buffer if necessary.
		*/

		stScBuffer.a1uchData [ stScBuffer.inTail ] = uchInputDatum;
		if ( stScBuffer.inTail < ( SC_BUFFER_LEN - 1 ) ) {
				stScBuffer.inTail++;
		} /* if */
		else {
				stScBuffer.inTail = 0;
		} /* else */

		/* Check to see if the buffer is full & set the flag "inBufferFull"
		accordingly (to prevent us trying to insert additional data
		while the buffer is full).
		*/

		stScBuffer.inBufferFull = ( stScBuffer.inTail == stScBuffer.inHead );

		/* Signal through the semaphore that a scan code is available in
		the scan code buffer.*/

		_signal ( uinKbdCountSem );

	} /* else */

	/* "Acknowledge" the keyboard interrupt by resetting the keyboard
	interrupt flip-flop (by momentarily turning on Bit 7 of the value
	in the control port (Port 61H) then restoring its original value).
	*/

	uchTemp = inportb(KEYBOARD_CONTROL);

	chDummyWait++;

	outportb(KEYBOARD_CONTROL, uchTemp | 0X80);

	chDummyWait++;

	outportb(KEYBOARD_CONTROL, uchTemp & 0X7F);

} /* end of KeyboardInterrupt  */





/*--------------------------------------------------------------------------*/





/*
===========================================================================
|
| fvdSetupKbdDriver
|
|       Keyboard Driver initialization functions
| SetupKBDriver() will reset the keyboard driver's buffer, initialize
| the keyboard LEDs, initialize a semaphore, and then install the
| keyboard interrupt handler routine.  It should be called when the
| Operating System is first started.
|
|       fvdResetScBuffer() will reset the keyboard driver's buffer, effectively
| clearing any scan code data that was already in the buffer.
| Warning...(LJS)The LED routine appear to cause problems on some PCs.
|
| Parameters    :       none
|
| Returns               : nothing
|
===========================================================================
*/

static void fvdSetupKbdDriver ( void ) {

	fvdResetScBuffer ( );

	/* set up the interrupt vector and save the old interrupt vector */
	pfvdOldKeyboardDriver =
		(void (interrupt*)() )fpvdSetVector ( INTERRUPT_NUMBER, (char *)fvdKeyboardInterrupt );
//		(void *)fpvdSetVector ( INTERRUPT_NUMBER, (void *)fvdKeyboardInterrupt );

} /* end of fvdSetupKbdDriver */





/*-------------------------------------------------------------------------*/







/*
============================================================================
|
| fvdRestoreOldDriver
|
|       Routine to load the old keyboard vector back so one can return to DOS.
|
| Parameters    :       none
|
| Returns               :       nothing
|
============================================================================
*/

void fvdRestoreOldDriver (void) {

//	fpvdSetVector ( INTERRUPT_NUMBER, (void *)pfvdOldKeyboardDriver );
	fpvdSetVector ( INTERRUPT_NUMBER, (char *)pfvdOldKeyboardDriver );

} /* end of fvdRestoreOldDriver */





/*--------------------------------------------------------------------------*/




/*
=============================================================================
|
| fvdResetScBuffer
|
| This routine resets the pointers into the stScBuffer (scan code buffer) and
| initialises the other variables in the structure to indicate that there are
| no scan codes in the buffer.
|
| Parameters    :       none
|
| Returns               :       nothing
|
=============================================================================
*/

static void fvdResetScBuffer ( ) {

	stScBuffer.inHead = 0;      /* Reset keyboard buffer pointers */
	stScBuffer.inTail = 0;
	stScBuffer.inBufferFull = FALSE;

} /* end of fvdResetScBuffer */





/*--------------------------------------------------------------------------*/





/*
===========================================================================
|
| KEYBOARD CONTROL FUNCTIONS
|
| These functions access the keyboard microprocessor to change the
| operating characteristics of the keyboard.  At the moment, only
| operations to control the indicator LEDs are provided.  Other
| operations, such as SetRepeatRate(int rate), may be implemented
| in the future.
|
===========================================================================
*/


/*
===========================================================================
|
| fvdWaitForKeyboardReady
|
| Waits until the keyboard is ready to receive a command or until a
| 'timeout' occurs.  Reads the keyboard controller status register
| and checks to see if the input buffer is busy.  When bit 1 of the
| KBD status register changes to 0, indicating the input buffer is
| not full, then the keyboard is ready to receive the next command.
| The keyboard should always respond to a command within 20 millisec.
|
| Parameters    :       none
|
| Returns               :       nothing
|
===========================================================================
*/

static void fvdWaitForKeyboardReady ( ) {

	unsigned int uinI = 0xffff;

	do {
		uinI--;
	} while ((uinI != 0) && ((inportb(KEYBOARD_STATUS_REG)&2) !=0 ) );
} /* end of fvdWaitKeyboardReady */





/*------------------------------------------------------------------------*/





/*
============================================================================
|
| fvdSendKbdCommand
|
| Sends a command byte to the keyboard controller via the input
| buffer at Port 60H.  The keyboard should respond to a command with
| an ACK code (0FAH), but the acknowledge may arrive interleaved with
| any character data being sent. To simplify things, I assume that the
| keyboard is working properly and ignore any ACK codes which arrive
| with the data.
|
| Parameters    :       command to send
|
| Returns               :       nothing
|
==========================================================================
*/

static void fvdSendKbdCommand (unsigned char uchCommand) {

	fvdWaitForKeyboardReady ( );
	disable ( );                        /* Disable INTs until data sent */
	outportb (KEYBOARD_OUTPUT_PORT, uchCommand); /* Send to keyboard controller */
	enable ( );                         /* Enable INTs again */

} /* end of fvdSendKbdCommand */





/*-------------------------------------------------------------------------*/





/*
=============================================================================
|
| fvdClearLED
|
| Clears one or more of the keyboard lock indicator LEDs.
| LEDcode specifies which LEDs should be toggled.
|
| Parameters    :   The LED code
|
| Returns               :       nothing
|
=============================================================================
*/

static void fvdClearLED (unsigned char uchLEDcode) {

	fvdSendKbdCommand (0XED);
	uchLEDstatus = uchLEDstatus & ~uchLEDcode;
	fvdSendKbdCommand (uchLEDstatus);
} /* end of fvdClearLED */





/*------------------------------------------------------------------------*/





/*
============================================================================
|
| fvdToggleLED
|
| Toggles one or more of the keyboard lock indicator LEDs.
| LEDcode specifies which LEDs should be toggled.
|
| Parameters    :       LED code
|
| Returns               :       nothing
|
============================================================================
*/

static void fvdToggleLED (unsigned char uchLEDcode) {

	fvdSendKbdCommand (0XED);
	uchLEDstatus = uchLEDstatus ^ uchLEDcode;
	fvdSendKbdCommand(uchLEDstatus);
} /* end of fvdToggleLED */






/*--------------------------------------------------------------------------*/





/*------------------------------ End of kbdrv ---------------------------*/
