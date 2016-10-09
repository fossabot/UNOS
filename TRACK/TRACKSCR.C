/*
	TRACKSCR.C
	New page created to enable the tracking of the sun across the sky
	ksc/tzqh   12/10/96
*/
#include <graphics.h>
#include <stdio.h>
#include <conio.h>
#include <dos.h>
#include <string.h>
#include <stdlib.h>

#include "tracinfo.h"		/* to find the julian day, radec and az/el co-ordinates */
#include "cmclock.h"
#include "pcscr.h"
#include "kbtask.h"
#include "tparser.h"
#include "taskname.h"
#include "trackkbd.h"
#include "unos.h"
#include "general.h"
#include "seq.h"
#include "seqext.h"
#include "seqscr.h"
#include "scrext.h"

static void init_track (void);
static void update_track (TimeRecord *, int);

/* Routine to draw screen and show variables for the tracker */
void display_track (void) {

	TimeRecord previous;

	if ( return_screen_page() == TRACK_PAGE ) {
		init_track ();
		ReadClock (&previous);
		/* initialise the tracked star as the sun */
		update_track (&previous, TRUE);
	}


	while ( return_screen_page() == TRACK_PAGE ) {
		update_track (&previous, FALSE);
	}

}

void init_track (void)
{
	protect_screen();
	clrscr();
	setcursor ( 0x2000 );		/* Turn cursor off */

	pcscr_put_text ( 30, 1," ATCU TRACK PAGE ", BOLD );
	pcscr_draw_line ( 80, 1, 2, HOR, 1 );
	pcscr_draw_line ( 80, 1, 23, HOR, 1 );

	pcscr_put_text ( 5, 5, "Tracking     : Sun", BOLD );
	pcscr_put_text ( 5, 6, "Track Mode   : FOLLOW", BOLD );
	pcscr_put_text ( 5, 7, "Track Status : Off", BOLD );
	pcscr_put_int  ( 5, 8, "Interval     : %4d", track_interval, BOLD);
	pcscr_put_int  ( 5, 9, "Step Change  : %3d", change, BOLD);

	pcscr_put_text ( 50, 5, "s = Toggle Star", BOLD);
	pcscr_put_text ( 50, 6, "m = Toggle Mode", BOLD);
	pcscr_put_text ( 50, 7, "t = Go", BOLD);
	pcscr_put_text ( 50, 8, "k = dec.  l =  inc.", BOLD);
	pcscr_put_text ( 50, 9, "h = Toggle step", BOLD);

	pcscr_put_text ( 20, 11,"WARNING : CMOS Time must be accurate !!!", BOLD);

	display_function_keys();
	unprotect_screen();

}

/* update the values of Julian Days, co-ordinates of the sun.. etc */
void update_track (TimeRecord *previous, int init) {

	double alphah, alpham, alphas, deltah, deltam, deltas, alpha, delta;
	double jday;

	TimeRecord time_of_day;
	static int interval = 0;

	/* print out the current time */
	ReadClock ( &time_of_day );
	protect_screen();

	gotoxy ( 5, 12 ); cprintf ("Time %u:%u:%u | Date %u/%u/%u   ",time_of_day.hours,
													time_of_day.minutes,
													time_of_day.seconds,
													time_of_day.dayofmonth,
													time_of_day.month,
													time_of_day.year );

	/* convert the interval into seconds */
	interval = (time_of_day.hours - (*previous).hours)*3600 +
			   (time_of_day.minutes - (*previous).minutes)*60 +
			   (time_of_day.seconds - (*previous).seconds);

	pcscr_put_int  ( 20, 8, "%4d", track_interval, BOLD);
	pcscr_put_int  ( 20, 9, "%3d", change, BOLD);


	/* star, mode, and status included from trackkbd.h */
	switch (star) {

			case SUN:
				pcscr_put_text (20, 5,"Sun ", NORMAL);
				break;

			case MOON:
				pcscr_put_text (20, 5, "Moon", NORMAL);
				break;

			case VELA:
				pcscr_put_text (20, 5, "Vela", NORMAL);
				break;

			default:
				break;
		}

		switch (track_mode) {

			case FOLLOW:
				pcscr_put_text (20, 6, "FOLLOW", NORMAL);
				break;

			case SCROLL:
				pcscr_put_text (20, 6, "SCROLL", NORMAL);
				break;

			default:
				break;
		}

	switch (track_status) {

			case ON:
				pcscr_put_text (20, 7,"On ", NORMAL);
				break;

			case OFF:
				pcscr_put_text (20, 7, "Off", NORMAL);
				break;

			default:
				break;
		}


	/* calculate the value of Julian days and RADEC only if
	   the tracking interval is passed */
	if ( (interval > track_interval) || (init == TRUE) ) {

		/* calculate the current julian date */
		jday = julian(time_of_day.hours, time_of_day.minutes,
					  time_of_day.dayofmonth, time_of_day.month,
					  time_of_day.year );
		pcscr_put_double (5,13, "Julian Day = %.4lf", jday, NORMAL);

		/* reinitialise the interval */
		(*previous) = time_of_day;

		/* calculate the current RADEC co-ordinates of the sun */
		switch (star) {

			/* calculate the sun's co-ordinates */
			case SUN:
				radec_sun (jday, &alpha, &delta, &alphah, &alpham, &alphas,
						   &deltah, &deltam, &deltas);
				break;

			/* calculate the moon's co-ordinates with the same parameters
			   ie. radec_moon (jday, &alpha, &delta, .... )
			   since we need to find RA/DEC for the moon		*/
			case MOON:

				break;

			case VELA:

				break;

			default:
				break;
		}


		/* include the conversion function that converts from RADEC
		   into Az/El here and print it out */

/*
		pcscr_put_double (5,  5,  "Ra  = %3.2lf", alpha, NORMAL);
		pcscr_put_double (5,  6,  "Dec = %3.2lf", delta, NORMAL);
*/
		pcscr_put_double (5,  14, "Ra  =  %2.0lf", alphah,  NORMAL);
		pcscr_put_double (16, 14, "%2.0lf", alpham, NORMAL);
		pcscr_put_double (19, 14, "%2.3lf", alphas, NORMAL);
		pcscr_put_double (5,  15, "Dec = %2.0lf", deltah, NORMAL);
		pcscr_put_double (16, 15, "%2.0lf", deltam, NORMAL);
		pcscr_put_double (19, 15, "%2.3lf", deltas, NORMAL);





	}

	unprotect_screen ();

}