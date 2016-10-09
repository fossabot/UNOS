/**************************************************************************/
/*                                                                        */
/*                                                                        */
/*                        TRACK KEYBOARD MODULE                            */
/*                                                                        */
/*                                     by                                 */
/*                                                                        */
/*                                 Khee S Chiam                          */
/*         Department of Electrical Engineering and Computer Science      */
/*                            University of Melbourne					  */
/*									Austraila                             */
/*                                                                        */
/*                                                                        */
/**************************************************************************/


/*
MODULE DESCRIPTION
------------------

	Routines to be called from the keyboard task (kbtask.c). This way static
variables need not clutter the kbtask module and users amy change their own
kb entry routine without intervering with others.

	Called within kbtask when the ^P selection has selected the sequencer
page.

*/

#include <stdio.h>
#include <conio.h>
#include <dos.h>
#include "unos.h"
#include "general.h"
#include "trackkbd.h"
#include "kbtask.h"
#include "seq.h"

extern unsigned char ScreenTaskName[];

/*
****************************************************************************
decode_track_key

	Routine to decode the keys meant for the TRACK module.

Parameters:
	key_pressed - key received from the keyboard

****************************************************************************
*/

/* initialise the tracked star as the sun with tracking mode as FOLLOW */
int star = SUN ;
int track_mode = FOLLOW;
int track_status = OFF;
int track_interval = DEFAULT;
int change = CHANGE_INTERVAL;

void decode_track_key ( unsigned char key_pressed ) {


	/* Do a simple switch statement to send commands to sequencer */
	switch ( key_pressed ) {

		/* change the star to be tracked */
		case 's':
		case 'S':
			/* allow change in the tracked star only if the antenna is not moving */
			if (track_status == OFF)
				star++;
			if (star >= NUMSTARS)
				star = SUN;

			break;

		/* decide on the tracking mode */
		case 'm':
		case 'M':
			if (track_status == OFF)
				track_mode++;
			if (track_mode >= NUMMODES)
				track_mode = FOLLOW;
			break;

		/* start the tracker */
		case 't':
		case 'T':
			track_status++;
			if (track_status > ON)
				track_status = OFF;
			break;

		case 'l':
		case 'L':
			if ( (track_interval+change) <= MAX_INTERVAL)
				track_interval = track_interval + change;
			break;

		case 'k':
		case 'K':
			if ( (track_interval-change) >= MIN_INTERVAL)
				track_interval = track_interval - change;
			break;

		case 'h':
		case 'H':
			/* change can be allowed 3 values - 1,10,100 */
			if (change == CHANGE_INTERVAL)
				change = MAX_CHANGE;
			else if (change == MAX_CHANGE)
				change = MIN_CHANGE;
			else if (change == MIN_CHANGE)
				change = CHANGE_INTERVAL;
    		
			break;

		default:
			break;

	}


} /* end of decode_tune_key */

