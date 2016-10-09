/*****************************************************************************
* POLYTRON VERSION CONTROL SOFTWARE _ PVCS (tm) Header Information
*
* 	$Revision:   1.8  $
*	$Date:   12 Oct 1991 16:48:28  $
*
*****************************************************************************/
/*
****************************************************************************
					Antenna Tracking Control Unit

						Sequencer Screen Routines for PC

_author: L. J. Sciacca
				 Tony Huang  (tzqh)
				 Daryl Chiam (ksc)

_latest:
		10 March 1992	great way to spend ones b'day
		18 April 1996 added credit page

Description.

	These routines are to be called from the screen task to update the
	Tuning parameters.

****************************************************************************
*/

#include <graphics.h>
#include <stdio.h>
#include <conio.h>
#include <dos.h>
#include <string.h>
#include <stdlib.h>

#include "unos.h"
#include "general.h"

#include "seq.h"
#include "seqext.h"
#include "scrext.h"

#include "pcscr.h"
#include "kbtask.h"
#include "tparser.h"
#include "taskname.h"


static void init_tune_screen ( void );
static void init_tune_id_screen ( void );
static void update_tune_id_screen ( void );

static void help_screen ( void );
static void update_tune_screen ( void );

static char display_string [ 80 ];
static char error_string [ 80 ];
static char lock_string [ 10 ];
static controller_parameter_struct parser_par;

extern CListNode *clist;
extern open_loop_flag;
extern rate_value_count;
/*
****************************************************************************
display_tune

	Routine to draw screen and show variables for atcu and model screen.

****************************************************************************
*/
void display_tune ( ) {

	char init_once, screen_flag, old_screen_flag;

	if  ( return_screen_page ( ) == TUNE_PAGE ) {
		init_tune_screen ( );
		initialise_tune_decoder ( );

		init_once = 1;
		old_screen_flag = 0; /* want to init first time around */
		}

	while ( return_screen_page ( ) == TUNE_PAGE ) {

			update_screen ( 1 );

			screen_flag = return_tune_screen ( );

			if ( screen_flag == old_screen_flag )
				init_once = 0;
			else
				init_once = 1;

			switch ( screen_flag ) {

				case TUNE_HELP_SCREEN:

					if ( init_once )
						help_screen ( );

					break;

				case TUNE_AZEL_SCREEN:

					if ( init_once )	/* must have been a help previously */
						init_tune_screen ( );

					update_tune_screen ( );

					break;

				case TUNE_ID_SCREEN:

					if ( init_once )
						init_tune_id_screen ( );

					update_tune_id_screen ( );

					break;

				default:
					break;

				} /* switch */

			tune_parser ( );/* A WAIT WILL BE DONE IN HERE */

			old_screen_flag = screen_flag;

		} /* end of while */

} /* end of display_tune */



/*
****************************************************************************
init_tune_screen ( )

	Routine to draw screen and show variables for atcu screen.

****************************************************************************
*/
void init_tune_screen ( void ) {

	protect_screen ();

	clrscr ( );
	setcursor ( 0x2000 );		/* Turn cursor off */

	pcscr_put_text ( 15, 1,
"ATCU Position Loop Tuning Panel - TUNRA IED - 10/March/92", BOLD );

	pcscr_draw_box ( 1, 2, 79, 20, "", NULL );


	/* Now setup parameter names */
	pcscr_put_text ( 3, 4, "PROP gain       :", BOLD );
	pcscr_put_text ( 3, 5, "INTE gain       :", BOLD );
	pcscr_put_text ( 3, 6, "RES  rolloff Hz :", BOLD );
	pcscr_put_text ( 3, 7, "DIFF rolloff    :", BOLD );
	pcscr_put_text ( 3, 8, "INTP + limit    :", BOLD );
	pcscr_put_text ( 3, 9, "INTN - limit    :", BOLD );
	pcscr_put_text ( 3,10, "SIGN            :", BOLD );
	pcscr_put_text ( 3,11, "MAXV max volts  :", BOLD );
	pcscr_put_text ( 3,12, "MINV min volts  :", BOLD );
	pcscr_put_text ( 3,13, "RREG rate region:", BOLD );
	pcscr_put_text ( 3,14, "MAXA max accel  :", BOLD );
	pcscr_put_text ( 3,15, "RATG rate gain  :", BOLD );
	pcscr_put_text ( 3,16, "CONM maxcon rate:", BOLD );

	pcscr_put_text ( 45, 4, "ALPH alpha      :", BOLD );
	pcscr_put_text ( 45, 5, "BETA            :", BOLD );
	pcscr_put_text ( 45, 6, "FFHI Max FF gain:", BOLD );
	pcscr_put_text ( 45, 7, "FFLO Min FF gain:", BOLD );
	pcscr_put_text ( 45, 8, "MITI mit increm :", BOLD );
	pcscr_put_text ( 45, 9, "MITR mit max ref:", BOLD );
	pcscr_put_text ( 45,10, "FFIN initial FF :", BOLD );

	pcscr_put_text ( 3,17, "DATA OUT:", BOLD );
	pcscr_put_text ( 3,18, "DATA    :", BOLD );
	pcscr_put_text ( 3,19, "HELP - Help is available", BOLD );

	display_function_keys ( );
	unprotect_screen ( );

} /* end of init_tune_screen */





/*
****************************************************************************
help_screen ( )

	Routine to draw screen and show help screen.

****************************************************************************
*/
void help_screen ( void ) {

	protect_screen ( );

	clrscr ( );
	setcursor ( 0x2000 );		/* Turn cursor off */

	pcscr_put_text ( 15, 1,
"ATCU Position Loop Tuning Panel - TUNRA IED - 10/March/92", BOLD );

	pcscr_draw_box ( 1, 2, 79, 20, "", NULL );

	/* Now setup parameter names */
	pcscr_put_text ( 20,3, "HELP SCREEN - Press Esc to return to main menu", BOLD );

	pcscr_put_text ( 3,5, "SEL AZ/EL", BOLD );
	pcscr_put_text ( 13,5, "- selects active axis", NORMAL );

	pcscr_put_text ( 3,6, "SEL ID", BOLD );
	pcscr_put_text ( 13,6, "- selects system identification screen", NORMAL );

	pcscr_put_text ( 3,7, "SET/GET", BOLD );
	pcscr_put_text ( 13,7, "- sets/gets into/from NVRAM", NORMAL );

	pcscr_put_text ( 3,8,"DEF", BOLD );
	pcscr_put_text ( 13,8,"- loads default from ROM", NORMAL );

	pcscr_put_text ( 3,9,"OUT TUNE", BOLD );
	pcscr_put_text ( 13,9,"- Turn tune dataport output on, protocol one off", NORMAL );

	pcscr_put_text ( 3,10,"OUT NOR", BOLD );
	pcscr_put_text ( 13,10,"- Turn tune dataport off, protocol one on", NORMAL );

	pcscr_put_text ( 3,11,"OUT TOG", BOLD );
	pcscr_put_text ( 13,11,"- Turn dataport OFF/ON", NORMAL );

	pcscr_put_text ( 3,12,"OUT AZ/EL", BOLD );
	pcscr_put_text ( 13,12,"- Change which axis data is output to dataport", NORMAL );

	pcscr_put_text ( 3,18, "ESC ", BOLD );
	pcscr_put_text ( 13,18, "- Delete current input", NORMAL );

	pcscr_put_text ( 3,19, "Enter command e.g. >>", BOLD );
	pcscr_put_text ( 25,19, "prop 12000.0<cr>", NORMAL );

	unprotect_screen ( );

} /* end of help_screen */


/*
****************************************************************************
update_tune_screen ( )

	Routine to update variables for the ATCU Tune screen page.

****************************************************************************
*/
void update_tune_screen ( ) {

	int spew_axis;
	unsigned char dataport_switch;
	int axis_type;

	return_display_string ( (unsigned char *)&display_string );
	return_error_string ( (unsigned char *)&error_string );
	return_lock_string ( (unsigned char *)&lock_string );
	parser_par = return_parser_par_copy ( &axis_type );
	spew_axis = return_axis_tune_data_out ( );
	dataport_switch = return_dataport_switch ( );

	protect_screen ( );

	pcscr_put_text ( 1, 1, lock_string, BOLD );

	if ( axis_type == AZ_DRIVE )
		pcscr_put_text ( 40, 3, "AZIMUTH  ", BOLD );
	else
		pcscr_put_text ( 40, 3, "ELEVATION", BOLD );

	pcscr_put_text ( 1, 21, ">> ", BOLD );
	pcscr_put_text ( 4, 21, display_string, NORMAL );
	pcscr_put_text ( 2, 23, error_string, BOLD );

	pcscr_put_double ( 22, 4, "%8.3f", parser_par.prop_gain, NORMAL );
	pcscr_put_double ( 22, 5, "%8.3f", parser_par.integral_gain, NORMAL );
	pcscr_put_double ( 22, 6, "%8.3f", parser_par.res_bw, NORMAL );
	pcscr_put_double ( 22, 8, "%8.3f", parser_par.int_positive_limit, NORMAL );
	pcscr_put_double ( 22, 9, "%8.3f", parser_par.int_negative_limit, NORMAL );
	pcscr_put_int ( 22,10, "%i", parser_par.sign, NORMAL );
	pcscr_put_double ( 22,11, "%8.3f", parser_par.max_volts, NORMAL );
	pcscr_put_double ( 22,12, "%8.3f", parser_par.min_volts, NORMAL );
	pcscr_put_double ( 22,13, "%8.3f", parser_par.rate_limit_region, NORMAL );
	pcscr_put_double ( 22,14, "%8.3f", parser_par.max_acceleration, NORMAL );
	pcscr_put_double ( 22,15, "%8.3f", parser_par.rate_gain, NORMAL );

	pcscr_put_double ( 63, 4, "%8.3f", parser_par.mit_alpha, NORMAL );
	pcscr_put_double ( 63, 5, "%8.3f", parser_par.mit_beta, NORMAL );
	pcscr_put_double ( 63, 6, "%8.3f", parser_par.mit_ff_high,NORMAL );
	pcscr_put_double ( 63, 7, "%8.3f", parser_par.mit_ff_low, NORMAL );
	pcscr_put_double ( 63, 8, "%8.3e", parser_par.mit_maxincr, NORMAL );
	pcscr_put_double ( 63, 9, "%8.3f", parser_par.mit_maxref, NORMAL );
	pcscr_put_double ( 63,10, "%8.3f", parser_par.initial_mit_ff_gain, NORMAL );


	if ( spew_axis == AZ_DRIVE )
		pcscr_put_text ( 13, 17, "AZ", BOLD );
	else
		pcscr_put_text ( 13, 17, "EL", BOLD );

	switch ( dataport_switch ) {

		case 0:
			pcscr_put_text ( 14, 18, "NORM", BOLD );

			break;

		case 1:
			pcscr_put_text ( 14, 18, "TUNE", BOLD );

			break;

		case 2:
			pcscr_put_text ( 14, 18, "STOP", BOLD );

			break;

		default:
			break;
		} /* switch */

	unprotect_screen ( );

} /* end of update_tune_screen */




/*
****************************************************************************
init_tune_id_screen ( )


	Routine to update the System ID Screen.

****************************************************************************
*/
void init_tune_id_screen ( void ) {

	protect_screen ( );

	clrscr ( );
	setcursor ( 0x2000 );		/* Turn cursor off */

	pcscr_put_text ( 15, 1,
		"ATCU Position Loop Tuning Panel - TUNRA IED - 10/March/92", BOLD );

	pcscr_draw_box ( 1, 2, 79, 20, "", NULL );

	pcscr_put_text ( 30, 4, "System Identification - Open Loop Data", BOLD );

	pcscr_put_text ( 3, 7, "Az V Step           :", BOLD );
	pcscr_put_text ( 3, 8, "El V Step           :", BOLD );
	pcscr_put_text ( 3, 9, "Step Period ( Sec ) :", BOLD );

	pcscr_put_text ( 3, 15, "To Change step type", BOLD );
	pcscr_put_text ( 3, 16, ">> step az 0.1<cr>", NORMAL );
	pcscr_put_text ( 3, 17, ">> step per 5<cr> ", NORMAL );

	pcscr_put_text ( 3, 19, "To return to other menu type >> sel az/el", BOLD );

	unprotect_screen ( );

} /* end of tune_id_screen */

/*
****************************************************************************
update_tune_id_screen ( )

	Routine to update variables for the ATCU Tune screen page.

****************************************************************************
*/
void update_tune_id_screen ( ) {


	systemid_struct systemid;

	return_display_string ( (unsigned char *)&display_string );
	return_error_string ( (unsigned char *)&error_string );
	return_lock_string ( (unsigned char *)&lock_string );
	return_systemid ( &systemid );

	protect_screen ( );

	pcscr_put_text ( 1, 1, lock_string, BOLD );

	pcscr_put_text ( 1, 21, ">> ", BOLD );
	pcscr_put_text ( 4, 21, display_string, NORMAL );
	pcscr_put_text ( 2, 23, error_string, BOLD );

	pcscr_put_double ( 25, 7, "%8.3f", systemid.azv, NORMAL );
	pcscr_put_double ( 25, 8, "%8.3f", systemid.elv, NORMAL );
	pcscr_put_double ( 25, 9, "%8.3f", systemid.step_per, NORMAL );

	unprotect_screen ( );

} /* end of update_tune_screen */


/*
****************************************************************************
	This routine uses BGI standard library in compliation.
	May not work in a non-BGI environment. Unable to use BGI graphics,
	this can be attributed to the large arrays used in this program.
	Refer to gdraw to execute Tacho information.

	This function displays the information required to
	switch controllers and for the logging of signals for graphing

	R.Lj 	16.9.95

****************************************************************************
*/
void display_graph(void)
{
	double start_x;

	int i,x,y;

	protect_screen();
	clrscr();
	setcursor ( 0x2000 );		/* Turn cursor off */


	pcscr_draw_box ( 1, 2, 79, 25, "", NULL );
	pcscr_put_text ( 15, 4," GRAPH AND CONTROLLER INFORAMTION PAGE", BOLD );
	pcscr_draw_line ( 37, 15, 5, HOR, 1 );

	pcscr_draw_box ( 8, 8, 69, 16, "", NULL );
	pcscr_put_text( 10, 9, "TACHO (rev/sec) : ", NORMAL);
	pcscr_put_text( 10, 11," TO VIEW GRAPH OF TACHO vs TIME:",NORMAL);
	pcscr_put_text( 20, 12,"1. Log data (press ^L to begin 360 samples)", NORMAL);
	pcscr_put_text( 20, 13,"2. Terminate this program ", NORMAL);
	pcscr_put_text( 20, 14,"3. Execute GDRAW.EXE ", NORMAL);

	pcscr_draw_box ( 8, 17, 69, 24, "", NULL );
	pcscr_put_text( 10, 22,"CONTROL STATUS: ", BOLD);
	pcscr_put_text( 10, 18,"Controller toggle: ^O", NORMAL);
	pcscr_put_text( 10, 19,"Increase signal level(open loop): ^I", NORMAL);
	pcscr_put_text( 10, 20,"Decrease signal level(open loop): ^D", NORMAL);
	pcscr_put_text( 10, 23,"Signal level(open loop): ",BOLD);

	unprotect_screen();
	start_x = clist->d_time;


while(return_screen_page() == GRAPH_PAGE)
{
	update_screen(1);
	x = (int) ((clist->d_time) - start_x);
	y = (int) (10.0 * (clist->Az_Vel));
	y = y+10; /* scaled value to plot */

	pcscr_put_int( 30, 9, "%4d", clist->Az_Vel, NORMAL);
	/* unable to use BGI functions due to excessive memory usage in arrays */
	/* the values x,y are here ready to plot. For a crude plot put in 		 */
	/*  gotoxy(x,y); cprintf("*");																				 */

	if (open_loop_flag ==1)    {
		pcscr_put_text( 27, 22, "OPEN LOOP CONTROLLER ", NORMAL);
		pcscr_put_int( 34, 23, "%i", rate_value_count, NORMAL);
		}
	else
		pcscr_put_text( 27, 22, "PID CONTROLLER       ", NORMAL);

}
}/*end display_graph */

