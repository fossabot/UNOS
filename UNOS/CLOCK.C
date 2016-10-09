
/**************************************************************************/
/*                                                                        */
/*                                                                        */
/*                               CLOCK TIMER MODULE                       */
/*                                                                        */
/*                                     by                                 */
/*                                                                        */
/*                                 L. J. Sciacca                          */
/*         Department of Electrical Engineering and Computer Science      */
/*                            University of Newcastle					  */
/*									Austraila                             */
/*                                                                        */
/*                         ( Copyright 1988, 89, 90, 91 )                 */
/*                                                                        */
/*                                                                        */
/**************************************************************************/
#include <dos.h>
#include <math.h>

	/* User changeable. This contains the extern prototype for the tick 
	routine.
	*/
#include "unos.h"
#include "hwpc.h"		/* contains setvector prototype */


/*
Module Description:

	This module contains the routines to set up the timer chip for the PC 
compatibles. The timer chip is an 8253. It must be programmed in the 
following modes:

	Counter 0
	Mode 2 (rate generation)

	The set_new_tick is used to place the tick interrupt vector into the 
timer interrupt routine (8). During this time, the PC clock will not be 
updated. It is possible to correct the time after changing the vector once 
we are finished with it but this has not been implemented.

The setvector routine (utilpc.c) is used in place of setvect because setvect 
enables interrupts.

*/

static void interrupt ( *old_dos_vector )( );

#define TIMER_CMD_REG	0x43
#define TIMER_COUNT_REG 0x40
#define COM_TIMER		0x34
#define COM_TIMER_M3	0x36

/*
============================================================================
set_new_tick

Parameters:
	newclock_hz = frequency of clock in hz

Entry:
	via main routine before task creation.

============================================================================
*/
void set_new_tick ( float newclock_hz, void interrupt * tick_int ) {

	int	timervalue;

	/* Program counter 0, mode 2, read/load lsb then msb */
	//outportb ( TIMER_CMD_REG, COM_TIMER );

	/*--- Reprogram timer chip to send out clock ticks at the desired rate ----*/
	timervalue = (int)floor ( 1192737.0 / newclock_hz );

	/*  Command to load timer count register  */
	outportb ( TIMER_COUNT_REG, timervalue & 0x000ff );		/*  Low byte  */
	outportb ( TIMER_COUNT_REG, ( timervalue & 0xff00 ) >> 8 );	/*  High byte  */

	/*
	Now install user clock tick routine pointed to by the 
	clock tick interrupt vector.
	*/

	/* Make an address for the vector ( offset = 4xinterrupt no ) */
	//old_dos_vector = getvect ( 8 );
	SetVector ( 8, tick_int );

}		/* End of set_new_tick */

/*
============================================================================
restore_dos_mode

	Due to problems with disk drive BIOS routines requiring mode 3 of the 
system timer chip, this routine should be called when returning back to 
DOS.

Parameters:
	none

Entry:
	DOS program

============================================================================
*/
void restore_dos_mode ( ) {

	int	timervalue;

	/* Program counter 0, mode 3, read/load lsb then msb */
	outportb ( TIMER_CMD_REG, COM_TIMER_M3 );

	/*--- Reprogram timer chip to send out clock ticks at the desired rate ----*/
	timervalue = (int)floor ( 1192737.0 / 18.2 );

	/*  Command to load timer count register  */
	outportb ( TIMER_COUNT_REG, timervalue & 0x000ff );		/*  Low byte  */
	outportb ( TIMER_COUNT_REG, ( timervalue & 0xff00 ) >> 8 );	/*  High byte  */

	/* Restore old DOS vector */
	SetVector ( 8, old_dos_vector );

}		/* End of restore_dos_mode */



