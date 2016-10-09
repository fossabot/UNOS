/*--------------------------------------------------------------------------
|
| FILENAME: SEQSCR.C
|
| DESCRIPTION:
|				This file contains routines called from the screen task to display
|			and update the screen displaying the status of the tracking system.
|			Information displayed includes the commanded and measured pointing
|			angles in each axis, together with the state of the sequencer, which
|			really oversees the overall behaviour of the system.
|				At the time of writing (12/3/96), the screen contains too much
|			extraneous debugging information, and not enough information on what
|			are the valid commands.
|
|	HISTORY:
|			22-Oct-1990		first documented version (Len Sciacca)
|			29-Mar-1991		Len Sciacca does some cleaning up
|			12-Mar-1996		Steve Weller does some cleaning up and documentation
|			14-Mar-1996		Tony Huang(tzqh) & Daryl Chiam (ksc) :)
|
|	Copyright (c) University of Melbourne, 1996.
|
---------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------
|			INCLUDE FILES
---------------------------------------------------------------------------*/

/*----- system and platform files -----------------------------------------*/

#include <stdio.h>
#include <conio.h>
#include <dos.h>
#include <string.h>
#include <math.h>

/*----- program files -----------------------------------------------------*/

#include "unos.h"
#include "general.h"
#include "screen.h"
#include "seq.h"
#include "antsim.h"
#include "antext.h"
#include "seqext.h"
#include "pcscr.h"
#include "posext.h"
#include "kbtask.h"
#include "err_nums.h"
#include "beacon.h"
#include "kbtask.h"			/* import return_screen_page() */
#include "seqscr.h"
#include "scrext.h"			/* import protect and unprotect routine prototypes */
#include "cmclock.h"
#include "simintel.h"		/* import get_intelsat_sim_bcn() */
#include "nvramext.h"		/* import get nvram routines */
#include "startrak.h"

/*--------------------------------------------------------------------------
|			EXTERNAL REFERENCES
---------------------------------------------------------------------------*/

/*----- data declarations -------------------------------------------------*/

extern int open_loop_flag;
extern int rate_value_count;

/*---- function prototypes ------------------------------------------------*/

extern unsigned int return_plctask_ctr ( void );
extern unsigned long ret_free_mem ( void );

/*--------------------------------------------------------------------------
|			PRIVATE DECLARATIONS
---------------------------------------------------------------------------*/

/*----- data declarations -------------------------------------------------*/

static axis az;
static axis el;
static atcu_struct atcu;
static cp_data_struct cp_data;
static unsigned int mode;
static unsigned int atcu_command;
static unsigned int last_atcu_command;
static unsigned int latest_error = 0;
static unsigned int speaker_cnt;
static unsigned int new_error;
static double star_az, star_el;

static char * error_str [ ] = {
						"                      ",
						"Unable to control     ",
						"Drive interface error ",
						"Drive failure         ",
						"Bad key entry         ",
						"Spare                 ",
						"Spare                 ",
						"Invalid Command       ",
						"AZ CW  Software Limit ",
						"AZ CCW Software Limit ",
						"EL UP  Software Limit ",
						"EL DOWN Software Limit",
						"Serious Math Error    ",
						"Drives are Off        ",
						"PLC Forced SEQ OFF    ",
						"Az Cmd CW Limit Clip  ",
						"Az Cmd CCW Limit Clip ",
						"El Cmd UP Limit Clip  ",
						"El Cmd DOWN Limit Clip",
						"SEQ NVRAM Error       "
						};

static char * state_str [ ] = {
						"Off     ",
						"Holding ",
						"Stowing ",
						"Starting",
						"Stopping",
						"Standby ",
						"Tracking",
						"Slewing ",
						"Stowed  ",
						"Position",
						"Scanning",
						"External",
						"Idle    ",
						"Reset   ",
						"Reseting",
						"Ready ? ",
						"RUN On  ",
						"Start Hi",
						"Check On",
						"Ready ? ",
						"Enb Drv ",
						"Drv Enb ",
						"Hold    ",
						"Drv Stow",
						"External",
						"Hold    ",
						"SlewPos ",
						"SlewTrck",
						"SlewWait",
						"WaitSrc",
						"      31",
						"      31",
						"Power On"
						};

static char * cmd_str [ ] = {
						"Idle      ",
						"Stow      ",
						"Drv Start ",
						"Drv Stop  ",
						"Go        ",
						"Hold      ",
						"Power On  ",
						"Power Off ",
						"Lights On ",
						"lights Off",
						"Scan On   ",
						"Scan   Off",
						"Log Data  ",
						"Logger Off",
						};

static char * mode_str [ ] = {
						"LeoTrack         ",
						"StarTrack        ",
						"Position - Az/El ",
						"Position - RA/Dec",
						"Rate             ",
						"Intelsat         ",
						"StepTrack        ",
						"KalmanTrack      "
						};


/*---- function prototypes ------------------------------------------------*/

static void init_atcu_screen ( void );
static void update_atcu_screen ( void );
static void update_atcu_menu ( void );

/*--------------------------------------------------------------------------
|			PRIVATE FUNCTION DEFINITIONS
---------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------
|
|	init_atcu_screen
|
| DESCRIPTION: Draw screen template for the display of key ATCU variables.
|
---------------------------------------------------------------------------*/

void init_atcu_screen ( void )
{
	protect_screen ();

	pcscr_put_text ( 5, 1, "University of Melbourne Radio-Telescope Tracking System 24/10/96", BOLD );


/*
 * removed as instructed :) 14/3/96
 *
	pcscr_draw_box ( 40, 10, 78, 15, "", NULL ); * Tacho box *
 *
 */


/*
	Draw a box within the screen
	pcscr_draw_box ( x, y, l, h, "" , NULL );

	where
						 (x,y)
							---------------------  ^
							|										|  |
							|										|  |
							|										|  | h
							|										|  |
							---------------------  v

							<------------------->
												width
										 and  l = (width + x)

				x						= x-co-ord
				y						= y-co-ord
				h						= height of box
				l						= (value of x-co-ord) + (width of the box)


	To draw a line across the screen
	pcscr_draw_line ( length, x, y, orientation, n );
	where
				length 		  = length of line
				x			 			= x-co-ord
				y           = y co-ord
				orientation = HOR(izontal) or VER(ticle)
				n						= (n-1) spaces between dashes
												ie if n=2, we get   "- - - - -"


	Function which completes the drawing of a line within a box
	It is not worth the effort using this !!!!!
	pcscr_corner ( x, y, orientation )
	where
				x 					= x-co-ord
				y 					= y-co-ord
				orientation = TLEFT for a left corner
												TRIGHT for a right corner

																						ksc / tzqh
																						:) 15/3/96


*/

	/* status box */
	/* ATCU = Antenna Tracking Control Unit */
	pcscr_draw_box ( 1, 2, 38, 19, "", NULL );
	pcscr_put_text ( 4, 3, "  ATCU Functional Information", BOLD );
	pcscr_put_text ( 3, 4, "Azimuth          Elevation", NORMAL );
	pcscr_draw_line ( 37, 1,  5, HOR, 1 );
	pcscr_corner ( 1, 5, TLEFT );
	pcscr_corner ( 38, 5, TRIGHT );

	/* Menu box */
	pcscr_draw_box ( 40, 2, 78, 19, "", NULL ); /* menu box */
	pcscr_put_text ( 49, 3, " ACTU Command Keys", BOLD );
	pcscr_draw_line ( 37, 41, 5, HOR, 1 );
	pcscr_corner ( 40, 5, TLEFT);
	pcscr_corner ( 78, 5, TRIGHT);

	highvideo ( );

	/* status box information */
	pcscr_put_text ( 3, 6, "Cmd  :           Cmd  :          ø ", NORMAL );
	pcscr_put_text ( 3, 7, "Msd  :           Msd  :          ø ", NORMAL );
	pcscr_put_text ( 3, 8, "Error:           Error:          ø ", NORMAL );
	pcscr_put_text ( 3, 9, "Rate :           Rate :          V ", NORMAL );
	pcscr_put_text ( 3,10, "State:           State:            ", NORMAL );
	pcscr_put_text ( 3,11, "Speed:           Speed:         ø/s", NORMAL );
	pcscr_put_text ( 3,12, "ACU state    :", NORMAL );
	pcscr_put_text ( 3,13, "Mode         :", NORMAL );
	pcscr_put_text ( 3,14, "Last command :", NORMAL );
	pcscr_put_text ( 3,15, "Up time      :           hours", NORMAL );
	pcscr_put_text ( 3,16, "Wrap  :         Missed:", NORMAL );
	pcscr_put_text ( 3,17, "Last error :"	, NORMAL );
	pcscr_put_text ( 3,18, "CMOS time :", NORMAL );

	/* menu box information :) tzqh/ksc on 15/3/96 */
	pcscr_put_text ( 42, 6,  "P = power on,    p = power off"   , NORMAL );
	pcscr_put_text ( 42, 7,  "d = drive_on     o = drive_off"	 	, NORMAL );
	pcscr_put_text ( 42, 8,  "g = go           c = change mode"	, NORMAL );
	pcscr_put_text ( 42, 9,  "h = hold         s = stow"				, NORMAL );
	pcscr_put_text ( 42, 10, "z = toggle scan ^t = exit program", NORMAL );
	pcscr_put_text ( 42, 12, "                ^W"					, NORMAL );
	pcscr_put_text ( 42, 13, "                 ^"					, NORMAL );
	pcscr_put_text ( 42, 14, " ^A <--                     --> ^S"		, NORMAL );
	pcscr_put_text ( 42, 15, "                 v"					, NORMAL );
	pcscr_put_text ( 42, 16, "                ^Z"					, NORMAL );
/*	pcscr_put_text ( 42, 17, " ^B = beacon"		, NORMAL ); */
	pcscr_put_text ( 42, 18, " ^R = inc.               ^C =dec."		, NORMAL );


/*
 * removed these displays as instructed... :) 14/3/96

	pcscr_put_text ( 45, 17, "kbd psr: ", NORMAL );
	pcscr_put_text ( 45, 18, "plc task: ", NORMAL );

	pcscr_put_text ( 45, 7, "Scanning mode : ", NORMAL );
	pcscr_put_text ( 45, 8, "Data logging  : ", NORMAL );
	pcscr_put_text ( 45, 6, "Signal level(V):", NORMAL );
	pcscr_put_text ( 42, 11, "Tacho (rev/sec): ", NORMAL );
	pcscr_put_text ( 42, 14, "0                                 20", NORMAL );
	pcscr_put_text( 42, 3,"CONTROL STATUS:", NORMAL);
 *
 */

	lowvideo ( );

	unprotect_screen ();

} /* init_atcu_screen */

/*--------------------------------------------------------------------------
|
|	update_atcu_screen
|
| DESCRIPTION: Display key ATCU variables.
|
---------------------------------------------------------------------------*/

void update_atcu_screen ( ) {

	int						i;
	double				az_rate, el_rate;
	TimeRecord		time_of_day;
	char					kbd_byte;
	unsigned int	plctask_ctr = 0;

	/* get axis information structures */
	az = return_azimuth ( );
	el = return_elevation ( );
	az_rate = return_az_control ( );
	el_rate = return_el_control ( );

	/* get atcu modes and states */
	atcu = return_atcu ( );

	/* get last byte received by keyboard parser task */
	kbd_byte = return_mainbyte ();
	plctask_ctr = return_plctask_ctr ();

	/* get current CP data; What is CP?? (SRW 13/3/96) */
	cp_data = return_cp_data ( );

	/* make a noise on error */
	if ( new_error )
	{
		new_error = false;
		outportb(97,(inportb(97) | 0X03));		/* turn speaker on */
		speaker_cnt = 3;
	}

	if ( speaker_cnt == 1 )
	{
		outportb(97,(inportb(97) & 0XFC));	/* turn speaker off */
	}
	else if ( speaker_cnt != 1 )
	{
		speaker_cnt--;
	}

	protect_screen ();

/*
 * removed as instructed :) 14/3/96
 *
	* task info *
	pcscr_put_int ( 55, 17, "%i", kbd_byte, NORMAL );
	pcscr_put_int ( 55, 18, "%i", plctask_ctr, NORMAL );

	if ( atcu.state == SCANNING )
		pcscr_put_text ( 65, 7, "ON ", BOLD );
	else
		pcscr_put_text ( 65, 7, "OFF", NORMAL );

	if ( return_collect_data () )
		pcscr_put_text ( 65, 8, "ON ", BOLD );
	else
		pcscr_put_text ( 65, 8, "OFF", NORMAL );

	* signal info *
	pcscr_put_double ( 65, 6, "%8.3f", cp_data.signal, NORMAL );

	* tacho info *
	pcscr_put_double ( 62, 11, "%8.3f", cp_data.velocity, NORMAL);
	pcscr_draw_hor_bar ( abs(cp_data.velocity), 20.0, 42, 13, 35);



	* control status information *
	if (open_loop_flag ==1)
		pcscr_put_text( 58, 3,"OPEN LOOP CONTROLLER ", NORMAL);
	else
		pcscr_put_text( 58, 3,"PID CONTROLLER       ", NORMAL);
 *
 */

	/* az info */
	pcscr_put_double ( 10, 6, "%8.3f", az.cmd_absolute, NORMAL );
	pcscr_put_double ( 10, 7, "%8.3f", az.msd_absolute, NORMAL );
	pcscr_put_double ( 10, 8, "%8.3f", az.err, NORMAL );
	pcscr_put_double ( 10, 9, "%8.3f", az_rate, NORMAL );
	if (open_loop_flag)
	{
		if (rate_value_count)
		{
			gotoxy(10,10); cprintf("%s","Slewing ");
		}
		else
		{
			gotoxy(10,10); cprintf("%s","Holding ");
		}
	}
	else
	{
		gotoxy ( 10, 10 ); cprintf ("%s", state_str [ atcu.az_internal ] );
	}

	pcscr_put_double ( 10,11, "%8.3f", return_az_speed (), NORMAL );

	/* el info */
	pcscr_put_double ( 27, 6, "%8.3f", el.cmd, NORMAL );
	pcscr_put_double ( 27, 7, "%8.3f", el.msd, NORMAL );
	pcscr_put_double ( 27, 8, "%8.3f", el.err, NORMAL );
	pcscr_put_double ( 27, 9, "%8.3f", el_rate, NORMAL );
	gotoxy ( 27, 10 ); cprintf ("%s", state_str [ atcu.el_internal ] );
	pcscr_put_double ( 27,11, "%8.3f", return_el_speed(), NORMAL );

	gotoxy ( 19,  12 ); cprintf ("%s", state_str [ atcu.state_summary ] );

	gotoxy ( 19, 13 ); cprintf ("%s", mode_str [ atcu.mode ] );
	gotoxy ( 19, 14 ); cprintf ("%s", cmd_str [ atcu.last_cmd ] );

	pcscr_put_double ( 19, 15, "%8.4f", atcu.up_time/3600.0, NORMAL );
	pcscr_put_int ( 27, 16, "%u", return_sequencer_missed_count ( ), NORMAL );

 /*	pcscr_put_double ( 70, 22, "%10.4f", get_increment (), NORMAL ); */
 /* ie print the number after 69 spaces on line 22 :)  21/3/96  */

	pcscr_put_double ( 52, 18, "%10.4f", get_increment (), NORMAL );


	gotoxy ( 14, 16);
	if ( /* cp_data.wrap */ az.msd_wrap == CW_WRAP )
		cprintf ( "CW " );
	else
		if ( /* cp_data.wrap */ az.msd_wrap == CCW_WRAP )
			cprintf ( "CCW" );
	else
		cprintf ( "Error" );

	gotoxy ( 16, 17 ); cprintf ("%s", error_str [ latest_error ] );

	ReadClock ( &time_of_day );
	gotoxy ( 13, 18 ); cprintf ("%u:%u:%u %u/%u/%u   ",time_of_day.hours,
													time_of_day.minutes,
													time_of_day.seconds,
													time_of_day.dayofmonth,
													time_of_day.month,
													time_of_day.year );


	switch ( atcu.mode ) {

		case POSITION_MODE:

			/* replaced following screen outputs to the main command box
				 ksc/tzqh
				 21/3/96
			*/

			pcscr_put_text ( 42, 12, "Az/El ø    ", NORMAL);
			gotoxy ( 49, 14 );
			cprintf ( "%8.3f  |%8.3f", cp_data.az_cmd, cp_data.el_cmd );
			break;

		case POSITION_RADEC_MODE:

			pcscr_put_text ( 42, 12, "Ra/Dec h   ", NORMAL);
			gotoxy ( 49, 14 );
			cprintf ( "%8.3f  |%8.3f", cp_data.ra_cmd, cp_data.dec_cmd );
			break;

		case RATE_MODE:

			pcscr_put_text ( 42, 12, "Az/El ø/sec", NORMAL);
			gotoxy ( 49, 14 );
			cprintf ( "%8.3f  |%8.3f ", cp_data.az_dem_rate, cp_data.el_dem_rate );
			break;

		case STARTRACK_MODE:
			break;

		default:
			break;
		}

	unprotect_screen ();

} /* update_atcu_screen */

/*--------------------------------------------------------------------------
|
|	update_atcu_menu
|
| DESCRIPTION: Display a menu of commands available at the main screen.
|		This menu is far from complete, and is difficult to understand; it
|		needs rewriting (SRW 13/3/96)
|
---------------------------------------------------------------------------*/

void update_atcu_menu ( void ) {

	protect_screen ();

	gotoxy ( 3, 20 );

/* replaced function display
	ksc/tzqh   21/3/96

	cprintf ("  = hold,   = stow,   = drive_On,   = Drive_off,   = Go,   = Change Mode" );

	pcscr_put_text ( 3, 20, "h", BOLD );
	pcscr_put_text ( 13, 20, "s", BOLD );
	pcscr_put_text ( 23, 20, "d", BOLD );
	pcscr_put_text ( 37, 20, "o", BOLD );
	pcscr_put_text ( 52, 20, "g", BOLD );
	pcscr_put_text ( 59, 20, "c", BOLD );

	pcscr_put_text ( 3,  21, "Az:    CCW,   CW ³ El:    Up,    Dwn | Beacon:", NORMAL );
	pcscr_put_text ( 67, 21, "Exit: ^T", NORMAL );
	pcscr_put_text ( 7,  21, "^A", BOLD );
	pcscr_put_text ( 14, 21, "^S", BOLD );
	pcscr_put_text ( 26, 21, "^W", BOLD );
	pcscr_put_text ( 32, 21, "^Z", BOLD );
	pcscr_put_text ( 49, 21, "^B", BOLD );

	pcscr_put_text ( 45, 22, "Incr: ^C Dwn, ^R Up", BOLD );
	pcscr_put_text ( 67, 22, "Incr = ", BOLD );

*/

	pcscr_draw_line ( 80, 1, 23, HOR, 1 );
	unprotect_screen ();

} /* update_atcu_menu */

/*--------------------------------------------------------------------------
|
|	display_atcu
|
| DESCRIPTION: This routine sets up the main screen of the system. A template
|		is written to the screen, and the key ATCU variables are displayed.
|		The data on the screen is then updated at least once per UNOS clock tick,
|		or whenever there is new data to be displayed (I *think*). This is
|		achieved by performing a timed wait on the semaphore "update_screen_sem".
|		If no other task has signalled on this semaphore after one clock tick,
|		then the UNOS kernel signals the semaphore, and the current data is
|		(re-)displayed.
|
---------------------------------------------------------------------------*/

void display_atcu ( )
{
	if ( return_screen_page ( ) == ATCU_PAGE )
	{
		clrscr ( );
		setcursor ( 0x2000 );		/* turn cursor off */

		last_atcu_command = 3;	/* make the initial "Last command" be "Power off" */
		speaker_cnt = 1;

		init_atcu_screen ( );
		update_atcu_menu ( );
		display_function_keys ( );
	}

	while ( return_screen_page ( ) == ATCU_PAGE )
	{
		update_screen ( 1 );
		update_atcu_screen ( );
	}

} /* display_atcu*/

/*--------------------------------------------------------------------------
|
|	init_parser
|
| DESCRIPTION: Inititialise the CMU (??) commands to sensible values. This
|		routine is called by the sequencer task before starting the state machine.
|		This handles the situation where the sequencer starts up before the parser,
|		which may well be the case if the sequencer task has a higher priority.
|
---------------------------------------------------------------------------*/

void init_parser ( )
{
	last_atcu_command = IDLE_CMD;
	atcu_command			= IDLE_CMD;
	mode							= POSITION_MODE;

} /* init_parser */

/*--------------------------------------------------------------------------
|
|	process_error
|
| DESCRIPTION: Set a flag to display the most recent error on the screen, and
|		"Sound the bell", as Johnny Winter would say. The error flag is "new_error",
|		and the error is actually displayed in the routine "update_atcu_screen".
|
---------------------------------------------------------------------------*/

void process_error ( unsigned int error_code )
{
	latest_error	= error_code;
	new_error			= true;

	/* Sound an error */

	outportb(67,10);	/* get timer ready to accept a frequency */
	outportb(66,0);		/* send low order byte */
	outportb(66,8);		/* send high order byte */

} /* process_error */

/*--------------------------------------------------------------------------
|
|	return_mode
|
| DESCRIPTION: Return the "mode" variable, using disable/enable to prevent
|		corruption by other tasks using this variable.
|
---------------------------------------------------------------------------*/

unsigned int return_mode ( )
{
	int intermediate;

	disable ( );
	intermediate = mode;
	enable ( );

	return ( intermediate );

} /* return_mode */

/*--------------------------------------------------------------------------
|
|	return_atcu_command
|
| DESCRIPTION: Return the "atcu_command" variable, using disable/enable to
|		prevent corruption by other tasks using this variable.
|		In addition to returning the value of "atcu_command", this variable is
|		set to IDLE_CMD, for reasons I do not understand (SRW 13/3/96).
|
---------------------------------------------------------------------------*/

unsigned int return_atcu_command ( )
{
	int intermediate;

	last_atcu_command = atcu_command;

	disable ( );
	intermediate = atcu_command;
	atcu_command = IDLE_CMD;
	enable ( );

	return ( intermediate );

} /* return_atcu_command */

/*--------------------------------------------------------------------------
|
|	set_latest_error
|
| DESCRIPTION: Return the "latest_error" variable, using disable/enable to
|		prevent corruption by other tasks using this variable. Previous
|		documentation indicated that this function was called from the
|		key-pressed decoder functions in kbseq.c, though I haven't verified
|		this statement (SRW 13/3/96).
|
---------------------------------------------------------------------------*/

void set_latest_error ( unsigned int value )
{
	disable ();
	latest_error = value;
	enable ();
} /* set_latest_error */

/*---------------------------------------------------------------------------*/

/* What is this next routine doing in this file? Is it used anywhere?
SRW 13/3/96 */

void star_update ( void ) {
				#define DEG_TO_RAD 57.29577951
	station_position	station_location;
	int star_num;

	TimeRecord time;
	//star_details_struct star_details;

	ReadClock ( &time );

	get_station_position( &station_location );

	/* NB Time must be UTC in final version */
	// DO RISETIME INSTEAD!!!
/*	star_track (
			star_details.right_ascension/DEG_TO_RAD,
			star_details.declination/DEG_TO_RAD,
								star_details.epoch_mode,
			time.dayofmonth, time.month,
			( time.century*100 + time.year ),
			time.hours, time.minutes,
			(time.seconds + time.hundredths*0.01),
			station_location.longitude/DEG_TO_RAD,
			station_location.latitude/DEG_TO_RAD,
			&star_az,
			&star_el );
*/
	star_az = star_az*DEG_TO_RAD;
	star_el = star_el*DEG_TO_RAD;

} /* star_update() */

