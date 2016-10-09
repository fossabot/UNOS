/*
****************************************************************************
					Antenna Tracking Control Unit

						IO Screen Routines for PC

_author: L. J. Sciacca

_latest: 22-Oct-1990
		22-oct-1990
		29-march-1991 ljs - Clean up of code.
		6-march-1995 	sto - this module was created as a separate entity
						from plc.c by moving all functions related to
						screen displays. This was done because these routines
						are called from the screen task, whereas the ones
						in plc.c are called from the plc task.
		1-June-1995 rjlov - made this file into a screen for IO
						(serial) functions, so that we can see what
						is happening with the serial link.
		21-March-1996 tzqh/ksc - added the function_key_display function.. and
						some tidying up... :)
Description.

	These routines are to be called from the screen task to update the
	screen showing IO parameters.


****************************************************************************
*/
#include <stdio.h>
#include <conio.h>
#include <dos.h>
#include <string.h>
#include "unos.h"
#include "general.h"
#include <unosdef.h>
#include "ioscr.h"
#include "kbtask.h"		/* Import page definitions	*/
#include "seq.h"		// Import definisitions, start_high, stop etc
#include "scrext.h"		/* Import protect_ and unprotect_screen fns	*/
#include "protocol.h"	/* Import return_protocol_mess ()	*/
#include "pcscr.h"
#include "serinfo.h"

extern unsigned int return_slp_ctr ( void );  /* from slp.c */
static char display_string [ 100 ];
extern ser_struct serinfo;

/*
********************************************************************************
InitIO_Screen

Routine to display template for IO task variables.

********************************************************************************
*/
static void InitIO_Screen ( void ) {

int i;

clrscr ( );

protect_screen();

gotoxy(1,1);cprintf(
 "             IO Screen                  UniMelb Antenna");
 pcscr_put_text( 38,3, "Transmit Task Counter", normal );

/* line printing.. :) 21/3/96 */
pcscr_draw_line ( 79, 1, 2, HOR, 1 );
pcscr_draw_line ( 79, 1, 23, HOR, 1 );

display_function_keys();

unprotect_screen ();
} // end of InitIO_Screen


/*
*******************************************************************************
display_serial ( )

Routine to display IO stuff on screen

*******************************************************************************
*/
void display_serial( ) {
static unsigned char mybuff[255];
int i,j,k,blank;
unsigned int slp_ctr = 0;

InitIO_Screen ( );

display_string [ 0 ] = NULL;
i = 13;
while ( return_screen_page() == IO_PAGE )
	{
	slp_ctr = return_slp_ctr (); /* sto 18 sep */
	update_screen ( 1 );

	ret_protocol_mess ( display_string );

	protect_screen ();
	pcscr_put_int( 60,3, "%i", slp_ctr, normal );

	gotoxy ( 3, 3 ); cprintf ( "CRC in        : %2x %2x  ", serinfo.crcin1,
		serinfo.crcin2);
	gotoxy ( 3, 4 ); cprintf ( "CRC calculated: %2x %2x  ", serinfo.crccalc1,
		serinfo.crccalc2);
	gotoxy ( 3, 5 ); cprintf ( "Received ID   :  %d  ", serinfo.inid);
	gotoxy ( 3, 6 ); cprintf ( "Received Size :  %d  ",serinfo.insize);
	gotoxy ( 3, 7 ); cprintf ( "Last ID       :  %d  ",serinfo.lastinid);
	gotoxy ( 3, 8 ); cprintf ( "Next ID       :  %d  ",serinfo.nextinid);
	gotoxy ( 3, 9 ); cprintf ( "Last Byte     : %2x  ",serinfo.lastbyte);
	blank = (serinfo.mesgidx + 1) % serinfo.nummesgs;
	for (i=0;i<serinfo.nummesgs;++i) {
		gotoxy (3,i+10);
		if (i != blank)
			cprintf("%s",serinfo.mesg[i]);
		cprintf("                                                 ");
	}

	unprotect_screen ();
	}


} /* end of IO_Screen */

