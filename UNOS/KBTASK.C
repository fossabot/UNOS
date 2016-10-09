
/**************************************************************************/
/*                                                                        */
/*                                                                        */
/*                               KEYBOARD TASK MODULE                     */
/*                                                                        */
/*                                     by                                 */
/*                                                                        */
/*                                 L. J. Sciacca                          */
/*         Department of Electrical Engineering and Computer Science      */
/*                            University of Newcastle					  */
/*									Austraila                             */
/*                                                                        */
/*                         ( Copyright 1988, 89, 90, 91, 94 )             */
/*                                                                        */
/*                                                                        */
/**************************************************************************/

/*
MODULE DESCRIPTION
------------------

	This routine receives characters from the keyboard scan code task.

	Users may add their own key entry routines here. Note that due to
development restrictions a general prupose keyboard decoder was not written.
Ideally, a scanf routine should have been written to enterany numbers or 
strings. Users wishing such a flexible routine may write their own using
the UNOS Command Decoder facility (comdec.c).

	For all tasks, CONTROL_P is a fixed entry that toggles between pages
(screens) on the ATCU PC screen. (see screen.c).

	Users should add their own keyboard entry routines under the following:

		switch ( screen_page ) {

			case ATCU_PAGE:
				decode_atcu_key ( mainbyte );
				break;

			case ORBIT_PAGE:
				break;

			default:
				break;

			}

Latest: May 1994 LJS Brought back to life for JAH-1 project.
Oct 12th 1996, ksc added another page to the deocding sequence
			   to decode inputs from track page
*/

#include <stdio.h>
#include <conio.h>
#include <dos.h>
#include "main_ext.h"       /* define LOG_TASK */
#include "unos.h"
#include "general.h"
#include "kbtask.h"
#include "pcscr.h"

/* Task Prototypes */
void kbd_task ( void *);

/*---- User put externals here */
extern void decode_atcu_key ( unsigned char key_pressed );
//extern void decode_parser_key ( unsigned char key_pressed );
extern void decode_tune_key ( unsigned char key_pressed );
extern void decode_track_key ( unsigned char key_pressed ); /* ksc 12/10/96 */

	/* SCREEN PAGE NUMBER KEPT HERE */
static char screen_page = ATCU_PAGE;
static char special_key;

static char byte_out;

static double increment_out = 1;
static int os_termination = 1;
int open_loop_flag = 0;
int rate_value_count = 0;
int graph_log_flag = 0;
/*
****************************************************************************
kbd_task

	This task will the the one users will largely modify for their own
purposes. This task waits around for keyboard data from the
keyboard intermediate keyboard task.

	This task must be set up at system initialisation.

	This task is a largely modified version of the example keyboard
driver code. It basically gets key commands and send the appropriate
messages to the sequencer. In reality this should be a Control Point as no
commands should ordinarily be permitted to go to the sequencer while one of
the SCADA interfaces is active ( CP ).

	The User modifies this module to cater for the particular keyboard entry
codes. There are various PAGES and keyboard destinations. E.g. the ATCU
page may have two separate menus, a ATCU menu and a MOdel menu.

	Pages               Menues
	-----               ------
	ATCU				ATCU, MODEL
	ORBITTRACK			ORBIT

	Presently it is possible to stop the OS and return to the DOS prompt.
The command to do this is ^T.

****************************************************************************
*/
void kbd_task ( void * Dummy ) {

	unsigned char mainbyte, FunctionKey;
	unsigned int rcv_length;
	static double increment;
	FILE *fp;

	/* Local mbx data buffers */
	unsigned char kbd_data [ 2 ];

	/* increment initialised to 1.0 */
	set_increment ( 1.0 );
	increment = get_increment ( );

	Dummy = Dummy;

	/*---- Infinite Task loop */
	while ( 1 ) {

		/* rjlov */
#if LOG_TASK
		fp = get_fileptr ( );
		fwrite((const char *)"Kp",1,2,fp);
#endif

		/*---- Get data from scan code task */
		rcv_mess ( &kbd_data [ 0 ], &rcv_length, 0);

		mainbyte = kbd_data [ 0 ];			/* lower byte */
		FunctionKey = 0;
		set_byte_out ( mainbyte );

		if ( kbd_data [ 1 ] == 0x80 ) {
			FunctionKey = kbd_data [ 0 ] - 0x53; // Function key
			set_screen_page ( FunctionKey - 1 );
			if ( return_screen_page () > NUM_PAGES )
				set_screen_page ( 0 );
			}

		/*---- Grab some data from other modules */
		increment = get_increment ();

		/*---- Begin decoding key entry */
		/* These are special characters that are independent of the page
		being displayed. e.g. change page command, refresh, etc. It is up to
		the writer of a screen page to handle these characters
		appropriately.
		*/
		if ( !FunctionKey )
			switch ( mainbyte ) {
				case CONTROL_C:
					/* ^C adjust increment down by a factor of 10 */
					special_key = TRUE;
					increment = increment / 10.0;
					if ( increment < 0.001 )
						increment = 0.001;

					set_increment ( increment );
					break;

				case CONTROL_R:
					/* ^R adjust increment up by a factor of 10 */
					special_key = TRUE;
					increment = increment * 10.0;
					if ( increment > 100.0 )
						increment = 100.0;

					set_increment ( increment );
					break;


				case CONTROL_P:
					/* ^P changes screen page of monitors */

					special_key = TRUE;
					set_screen_page ( return_screen_page () + 1 );

					if ( return_screen_page() > NUM_PAGES )
						set_screen_page ( 0 );

					break;

				case CONTROL_I:
					/* ^I increments the signal level of the open_loop_controller */

					special_key = TRUE;
					if (rate_value_count > 5) /*this condition allows for constant looping  between 0 and -5*/
						rate_value_count = 0;
					else
					rate_value_count++;

					break;

				case CONTROL_D:
					/* ^D decrements the signal level of the open_loop_controller */

					special_key = TRUE;
					if (rate_value_count < -5) /*this condition allows for constant looping between 0 and -5 */
						rate_value_count = 0;
					else
					rate_value_count--;

					break;

				case CONTROL_L:
					/* ^L logs the data to be plotted by gdraw.exe */

					special_key = TRUE;
					graph_log_flag = 1;

					break;

				case CONTROL_O:
					/* ^O changes pid_controller to open_loop_controller, in azimuth only */
					special_key = TRUE;
					open_loop_flag++;
					if (open_loop_flag == 1)
					{
					/* open_loop_controller on */
					open_loop_flag = 1;
					}
					else
					{
					/* pid_controller on */
					open_loop_flag = 0;
					}

					//open_loop_flag %= 2;

					break;

				case CONTROL_T:
					/* ^T Terminates the programme and returns to DOS */

					special_key = TRUE;
					set_os_termination ( 0 );	/* Kill OS */

					break;

				default:
					special_key = FALSE;
					break;

				} /* switch */

		/*----- Now decode for menu pages */
		/* Only pages that require kbd entry routines should be placed here !! */
		/*  ksc /tzqh  12/4/96   :)  */

		if ( ( ! special_key ) && ( ! FunctionKey ) )
			switch ( return_screen_page () ) {

				case ATCU_PAGE:
					decode_atcu_key ( mainbyte );
					break;

				case TUNE_PAGE:
					decode_tune_key ( mainbyte );
					break;

				/* for the decoding of inputs from track page ksc 12/10/96 */
				case TRACK_PAGE:
					decode_track_key ( mainbyte );
					break;
				default:
					break;

				} /* switch */

		}	/* while ( 1 ) */

} /* end of kbtask */


/*
****************************************************************************
return_screen_page ( )

	Routine to pass back the current page. Returns a variable reflecting
	which page should be active on the screen.

Called from screen task when required.

****************************************************************************
*/
unsigned int return_screen_page ( ) {

	unsigned int temp;

	disable ( );
	temp = screen_page;
	enable ( );

	return ( temp );

} /* end of return_screen_page */

/*
****************************************************************************
set_screen_page ( )

	Added by DJLB 18/5/93, to allow screen task to set the screen
	page after leaving the new startup page.

	Routine to set the current page. Sets screen_page to
	reflect which page should be active on the screen.

Called from any task when required.

****************************************************************************
*/
void set_screen_page ( char page )
{

	disable ( );
	screen_page = page;
	enable ( );

} /* end of set_screen_page */

/*
*************************************************************************
get_increment

	Procedure to return the current increment for values to be entered
	from the keyboard.

*************************************************************************
*/
double get_increment ( void )
{
	double result;

	disable ();
	result = increment_out;
	enable ();

	return result;
}


/*
*************************************************************************
set_increment

	Procedure to alter the current increment for values to be entered
	from the keyboard.

*************************************************************************
*/
void set_increment ( double value )
{

	disable ();
	increment_out = value ;
	enable ();

}

/*
*************************************************************************
set_os_termination

	Routine to indicate whther we want to kill the OS and return to DOS prompt.

	Set integer value to 1 if OS is to remain or 0 to kill

*************************************************************************
*/
void set_os_termination ( int result ) {

	disable ( );
	os_termination = result;
	enable ( );

} /* end of set_os_termination */

/*
*************************************************************************
return_os_termination

	Routine to check whther we want to kill the OS and return to DOS prompt.
This shouldbe checked by the null task. It is necessary to drop out of
the OS thru the null task as it is the last task created by the main
task. We need to fall thru the main task to preserve stack deallocation.

Parameters:

	Returns integer value of 1 if OS is to remain or 0 to kill

*************************************************************************
*/
int return_os_termination ( void ) {

	int result;

	disable ( );
	result = os_termination;
	enable ( );

	return ( result );

} /* end of return_os_termination */

/*
****************************************************************************
set_byte_out ( )

	Routine used to pass back the current mainbyte.

Called from screen task when required.

****************************************************************************
*/
void set_byte_out(char result)
{

	disable ();
	byte_out = result;
	enable ();

}   /* end of set_byte_out */

/*
****************************************************************************
return_mainbyte ( )

	Routine to pass back the current mainbyte.
	Used for diagnostic to check the keyboard task is alive

Called from screen task when required.

****************************************************************************
*/
char return_mainbyte(void)
{
	char	result;

	disable ();
	result = byte_out;
	enable ();

	return ( result );
}   /* end of return_mainbyte */




/*------------------------ END OF KBTASK MODULE --------------------------*/



