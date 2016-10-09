
/*
*****************************************************************************

                        
*                         " \"                   /\
*                            \                  | .\___
*                              \____           /    ____|
*                               |    ---------/    /     \     WOOF !
*                               |    WDOG          |
*                                \    /-------\    /
*                                 \   |        |  |\
*                                  | |          | | \\
*                                  | /          | / |/
*                                  \_\          \_\
*		-------------------------------------------------
				Watch Dog Timer Module
							by
		    L. Sciacca


			by
		    L. Sciacca


Latest:		LJS 4-Oct-1991 Creation

Description:
    This module contains the driver code for the Watch Dog timer board.
    There is a routine to initialise the Watch Dog board, NOT including the
    comms port which is setup using the usual UNOS serial configuration
    facility.

	The board has been set up to use 1 timer (counter 2) in mode 0.
	On not having the timer chip (8253) counter reset, the WDOG will
	reset the computer.

	The initialisation must be called at system startup, preferably before
	UNOS starts up. A time-out of 1 second is being used. The Sequencer
	then calls the reload procedure to keep the counter registers
	always preloaded.

******************************************************************************
*/
#include <stdlib.h>
#include <dos.h>
#include <math.h>

#include <wdog.h>	/* Contains prototypes for extern rotuines */
#include "..\sequ\wdog_hw.h"	/* Definitions for the hardware registers etc */
#define DUFUS

#ifdef DUFUS
void init_watch_dog ( void ) {

} /* end of init_watch_dog */


void disable_watch_dog ( void ) {

} /* disable watch dog */

void reset_watch_dog ( ) {

} /* end of reload_watch_dog */



#else
#ifdef IPC_CARD_WDT
/*
****************************************************************************
init_watch_dog

		Routine to set up the watch dog hardware. The following is carried out:
	1. configure 8253 to Mode 0 Counter 2
	2. Preload Counter 2 with watchdog interval
		3. Enable Watchdog Timer output

Parameters:
	None

****************************************************************************
*/
void init_watch_dog ( void ) {

	/* Enable Watchdog on IPC booard*/
	inportb ( ENABLE_WDT );

} /* end of init_watch_dog */


/*
*****************************************************************************
disable_watch_dog

	Routine to disable the Watch dog timer. This routine should be called
	upon exitting the ATCU software.

Parameters:
	NONE

****************************************************************************
*/
void disable_watch_dog ( void ) {

	inportb ( DISABLE_WDT );

} /* disable watch dog */

/*
*****************************************************************************
reset_watch_dog

	Routine used by the sequencer to reload the watchdog timer chip counters.
	This routine should be called by the sequencer to prevent the
	WDOG from resetting the computer.

Paremeters:
	None
****************************************************************************
*/
void reset_watch_dog ( ) {

	inportb ( ENABLE_WDT );

} /* end of reload_watch_dog */



#else
static int watch_dog_count;

/*
****************************************************************************
init_watch_dog

		Routine to set up the watch dog hardware. The following is carried out:
	1. configure 8253 to Mode 0 Counter 2
	2. Preload Counter 2 with watchdog interval
		3. Enable Watchdog Timer output

Parameters:
	None

****************************************************************************
*/
void init_watch_dog ( void ) {

    char temp;

    /* Configure Counter 2 Mode 0 */
    outportb ( WDOG_BASE + 3, 0xB0 );

    /* Load period */
	watch_dog_count = ceil ( WATCHDOG_PERIOD * WATCHDOG_CLOCK_FREQ );
    
    outportb ( WDOG_BASE + 2,   watch_dog_count & 0x00ff );   /*  Low byte  */
    outportb ( WDOG_BASE + 2, ( watch_dog_count & 0xff00 ) >> 8 ); /*  High byte  */
    
	/* Enable WatchDog Timer */
	/* First read MCR and enable bit 2 */
	temp = inportb ( WDOG_COMM_BASE + 4 ) | 0x04;
	outportb ( WDOG_COMM_BASE + 4, temp );

} /* end of init_watch_dog */


/*
*****************************************************************************
disable_watch_dog

	Routine to disable the Watch dog timer. This routine should be called
	upon exitting the ATCU software.

Parameters:
	NONE

****************************************************************************
*/
void disable_watch_dog ( void ) {

	char temp;

	/* Ensure Bit 2 is OFF */
	temp = inportb ( WDOG_COMM_BASE + 4 ) & 0xfb;
	outportb ( WDOG_COMM_BASE + 4, temp );

} /* disable watch dog */

/*
*****************************************************************************
reset_watch_dog

	Routine used by the sequencer to reload the watchdog timer chip counters.
	This routine should be called by the sequencer to prevent the
	WDOG from resetting the computer.

Paremeters:
	None
****************************************************************************
*/
void reset_watch_dog ( ) {

	outportb ( WDOG_BASE + 2,   watch_dog_count & 0x00ff );		/*  Low byte  */
	outportb ( WDOG_BASE + 2, ( watch_dog_count & 0xff00 ) >> 8 );	/*  High byte  */

} /* end of reload_watch_dog */



#endif
#endif
