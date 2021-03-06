/*
****************************************************************************
					Antenna Tracking Control Unit

						PLC Screen Routines for PC

_author: L. J. Sciacca

_latest: 22-Oct-1990
		22-oct-1990
		29-march-1991 ljs - Clean up of code.
		6-march-1995 	sto - this module was created as a separate entity
						from plc.c by moving all functions related to
						screen displays. This was done because these routines
						are called from the screen task, whereas the ones
						in plc.c are called from the plc task.
		21-march-1996 ksc/tzqh - added the display_function_key function and
						the draw line functions..
Description.

	These routines are to be called from the screen task to update the
	screen showing PLC variables.


****************************************************************************
*/
#include <stdio.h>
#include <conio.h>
#include <dos.h>
#include <string.h>

#include "unos.h"
#include "general.h"

#include <unosdef.h>
#include "plc.h"

#include "plcscr.h"
#include "pcscr.h"		/* for line drawings */
#include "kbtask.h"		/* Import page definitions	*/
#include "seq.h"		  /* Import definisitions, start_high, stop etc */
#include "scrext.h"		/* Import protect_ and unprotect_screen fns	*/
#include "protocol.h"

extern void ReadPLC ( PLC_struct * PLC );  /* Import from plc.c */

static PLC_struct PLC;
static char display_string [ 100 ];
static int task_count;

/*
********************************************************************************
Init_PLC_Screen

Routine to display template for PLC task variables.

********************************************************************************
*/
static void InitPLC_Screen ( void ) {

int i;

clrscr ( );

protect_screen();

gotoxy(1,1);cprintf(
 "             PLC Screen                  UniMelb Antenna");

pcscr_draw_line ( 79, 1, 2, HOR, 1 );

	gotoxy ( 1, 4 ); cprintf ( "%s", "Fail" );
	gotoxy ( 1, 5 ); cprintf ( "%s" , "Trip1" );
	gotoxy ( 1, 6 ); cprintf ( "%s" , "Trip" );
	gotoxy ( 1, 7 ); cprintf ( "%s" , "C24VDC" );
	gotoxy ( 1, 8 ); cprintf ( "%s" , "AzBrake" );
	gotoxy ( 1, 9 ); cprintf ( "%s" , "ElBrake" );
	gotoxy ( 1, 10 ); cprintf ( "%s" , "AzFinalCW" );
	gotoxy ( 1, 11 ); cprintf ( "%s" , "AzFinalCCW" );
	gotoxy ( 1, 12 ); cprintf ( "%s" , "ElFinalUp" );
	gotoxy ( 1, 13 ); cprintf ( "%s" , "ElFinalDwn" );
	gotoxy ( 1, 14 ); cprintf ( "%s" , "EmergStop" );
	gotoxy ( 1, 15 ); cprintf ( "%s" , "SmokeDetect" );
	gotoxy ( 1, 16 ); cprintf ( "%s" , "Door1" );
	gotoxy ( 1, 17 ); cprintf ( "%s" , "Door2" );
	gotoxy ( 1, 18 ); cprintf ( "%s" , "CB5_6" );
	gotoxy ( 1, 19 ); cprintf ( "%s" , "CB3" );
	gotoxy ( 1, 20 ); cprintf ( "%s" , "CB4" );
	gotoxy ( 1, 21 ); cprintf ( "%s" , "RUN1" );
	gotoxy ( 1, 22 ); cprintf ( "%s" , "RUN2" );
	gotoxy ( 1, 23 ); cprintf ( "%s" , "C1 - C2" );

for ( i=1; i < 79 ; i++ ) {
			gotoxy ( i, 24 );
			cprintf ( "\xc4" );
			}

display_function_keys();

unprotect_screen ();
} // end of InitPLC_Screen


/*
*******************************************************************************
PLC_Screen ( )

Routine to display IO stuff on screen

*******************************************************************************
*/
void PLC_Screen ( ) {

InitPLC_Screen ( );

display_string [ 0 ] = NULL;

while ( return_screen_page() == PLC_PAGE )
	{
	update_screen ( 1 );

	ret_protocol_mess ( display_string );

	task_count = return_plctask_ctr ( );

	ReadPLC ( &PLC );

	protect_screen ();

	gotoxy ( 3, 3 ); cprintf ( "%x %s", display_string[0], display_string );

/*	gotoxy ( 40, 3 ); cprintf ("%s ", rx_mess );
	gotoxy ( 60, 3 ); cprintf ("%x", rx_mess[0] ); */

	gotoxy ( 60, 8 ); cprintf ("C: %u", task_count );

/* Checking the messages sent and received by the PLC task */
/*	gotoxy ( 60, 4 ); cprintf ("Coils: %x   %x", coillower, coilupper );
		gotoxy ( 60, 5 ); cprintf ("       %s", string );
		gotoxy ( 60, 6 ); cprintf ("%x  %x  %x", rx_mess[2], rx_mess[3], word2 ); */

	gotoxy ( 18,  4 ); cprintf ( "%x", PLC.Fail );
	gotoxy ( 18,  5 ); cprintf ( "%x", PLC.Trip1 );
	gotoxy ( 18,  6 ); cprintf ( "%x", PLC.Trip );
	gotoxy ( 18,  7 ); cprintf ( "%x", PLC.C24VDC );
	gotoxy ( 18,  8 ); cprintf ( "%x", PLC.AzBrake );
	gotoxy ( 18,  9 ); cprintf ( "%x", PLC.ElBrake );
	gotoxy ( 18,  10 ); cprintf ( "%x", PLC.AzFinalCW );
	gotoxy ( 18,  11 ); cprintf ( "%x", PLC.AzFinalCCW );
	gotoxy ( 18,  12 ); cprintf ( "%x", PLC.ElFinalUp );
	gotoxy ( 18,  13 ); cprintf ( "%x", PLC.ElFinalDwn );
	gotoxy ( 18,  14 ); cprintf ( "%x", PLC.EmergStop );
	gotoxy ( 18,  15 ); cprintf ( "%x", PLC.SmokeDetect );
	gotoxy ( 18,  16 ); cprintf ( "%x", PLC.Door1 );
	gotoxy ( 18,  17 ); cprintf ( "%x", PLC.Door2 );
	gotoxy ( 18,  18 ); cprintf ( "%x", PLC.CB5_6 );
	gotoxy ( 18,  19 ); cprintf ( "%x", PLC.CB4 );
	gotoxy ( 18,  20 ); cprintf ( "%x", PLC.CB3 );
	gotoxy ( 18,  21 ); cprintf ( "%x", PLC.RUN1 );
	gotoxy ( 18,  22 ); cprintf ( "%x", PLC.RUN2 );
	gotoxy ( 18,  23 ); cprintf ( "%x", PLC.C1 + PLC.C2 );

	unprotect_screen ();
	}


} /* end of PLC_Screen */
