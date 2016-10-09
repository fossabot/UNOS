/***************************************************************************
								TIED SOFTWARE
 PROJECT	:- TS3000

 MODULE 	:- OFFSETS

 FILE 		:- OFFSETS.C

 PROGRAMMER :- R Mortenson

 Copyright 1994. The University of Newcastle Research Associates Ltd.

****************************************************************************
* POLYTRON VERSION CONTROL SOFTWARE _ PVCS (tm) Information
*
*      $Revision:   1.1.1.0  $
*      $Date:   29 Apr 1994 19:41:16  $
*
****************************************************************************/

#include "seq.h"      /* IMPORT CONTROL_PERIOD */
#include "pars_ext.h"    /* IMPORT scan key status functions */
#include "offsets.h"
#include "unos.h"        /* IMPORT return_interrupt_status */
#include "unosasm.h"	   // import return_interrupt_status
#include <dos.h>         /* IMPORT enable & disable */
#include <math.h>

#define NUMBER_OF_STATES 4
#define NUMBER_OF_AXES 2

/*
 * Define the states in which the antenna may be at any point in time.
 * The meaning of these states is :-
 *      i) idle -
 *         No scan has been initiated. The delta is simply the fixed offset
 *         for the scan vector.
 *     ii) going_to_scan -
 *         A scan has been initiated. The antenna is being moved from its
 *         initial position to the "start of scan" position, ready to start
 *         a scan.
 *    iii) scanning :-
 *         A scan is being performed in one vector.
 *     iv) waiting :-
 *         The scan has been completed. The antenna is being moved back to the
 *         "start of scan" position, unless a "stop scan" command is given. 
 *         In the latter case, the antenna must be moved to the idle position.
 */         

typedef enum { idle, going_to_scan, scanning, waiting } state;

/*
 * Define a data structure for the values for the scan in one scan vector.
 * This data structure contains :-
 *     1) the fixed offset, which is always applied.
 *     2) the offset for the start of the scan,
 *     3) the offset for the end of the scan,
 *     4) the offset rate from th current rate, for the scanning process,
 *     5) the size of the increment ( step ) to add at each invocation, for
 *        the scanning state,
 *     6) the total number of such increments to add, for the scanning state.
 *     7) the number of such increments added so far,
 *     8) the current state of the antenna.
 */

typedef struct {
           double fixed_offset;              /* units - degrees */
           double scan_offset_start;         /* units - degrees */
           double scan_offset_end;           /* units - degrees */
           double scan_offset_rate;          /* units - degrees per second */
           double step_size;                 /* units degrees */
           long int number_of_steps_to_end;
           long int steps;
           state mode;
} relative_offsets;

/*
 * Define a table of the scan requirements, one entry per scan vector. 
 */

static relative_offsets relative[NUMBER_OF_AXES];

/*
 * The parser will be asked for the status of the start/stop controls.
 * Define variables to record these status values.
 */

static control start_scan, stop_scan;

/*
 * Likewise, the parser will be asked which vector is the scanning vector.
 * Define a variable to record this.
 */

static scan_vector scanning_axis;

/*
 * The parser will periodically ask for the delta which was applied to each
 * vector. Define a table in which these deltas will be stored. Eac entry
 * will be updated each time a new delta is computed.
 */

static double delta_in[NUMBER_OF_AXES] = { 0, 0 };

/*
 * Declare a function which returns the next state for the scan vector, based on the data in the
 * table ( or database ) of scan parameter values.
 */

static state next_state_for ( scan_vector a );

/*
 * Declare a function which returns the "dynamic" component of the delta, ie.
 * without the fixed offset lumped in.
 */

static double scan_delta_in ( scan_vector a );

/*
 * Declare a function which returns the next count of steps in the 
 * "scanning state".
 */

static long int step_count_for ( scan_vector a );

/*
 * Declare a function which asks the parser for the latest scan offsets and
 * stores them into the table.
 */

static void update_scan_values_for
     ( scan_vector a,
       double fixed_offset,
       double offset_start,
       double offset_end,
       double offset_rate );

/* 
 * Declare and define a function which clips a double value to within an
 * upper limit.
 */

static double clip ( const double value, const double upper_limit );

static double clip ( const double value, const double upper_limit )
{
    return value < upper_limit ? value : upper_limit;
}

/*
 * The parser will periodically ask for the delta which was applied to each
 * vector. Define a function which will return the most recent delta value
 * which was applied to the angle for an axis.
 */

double offset_delta_for ( scan_vector vec )
{
    double temp;
    int interrupts_on_at_entry = return_interrupt_status ();

    disable ();
    temp = delta_in[vec];
    if ( interrupts_on_at_entry )
        enable ();
    return temp;
}

/*
 * Define a function which asks the parser for the most recent scan parameter
 * values, and stores these in the internal table ( or database ). At the same
 * time, the new values for the increment, and the total number of increments
 * are updated.
 */

static void update_scan_values_for
             ( scan_vector a,
               double fixed_offset, 
               double offset_start,
               double offset_end,
               double offset_rate ) 
{
   /*
    * Update the offsets for this scan vector.
    */
   
   relative[a].fixed_offset = fixed_offset;
   relative[a].scan_offset_start = offset_start;
   relative[a].scan_offset_end = offset_end;
   relative[a].scan_offset_rate = offset_rate;

   /*
    * Re-compute the step size needed for the scanning state.
    */

   relative[a].step_size = relative[a].scan_offset_rate * CONTROL_PERIOD;

   /*
    * Re-compute the number of steps needed for the scanning state.
    */

   if ( relative[a].scan_offset_rate != 0.00000000 )
       relative[a].number_of_steps_to_end =  ceil (
           ( relative[a].scan_offset_end - relative[a].scan_offset_start )
		 / ( relative[a].scan_offset_rate * CONTROL_PERIOD ) );

}


static state next_state_for ( scan_vector a )
   {
      state new_mode;                         /* temporary for next state */

       if ( stop_scan == ON || a != scanning_axis )
           new_mode = idle;
       else {
            switch ( relative[a].mode ) {
               case idle :
                  new_mode = start_scan == ON ? going_to_scan : idle;
                  break;
               case going_to_scan :
                  new_mode = scanning;
                  break;
               case scanning :
                  new_mode = relative[a].steps >= relative[a].number_of_steps_to_end ?
                             waiting : scanning;
                  break;
               case waiting :
                  new_mode = start_scan == ON ? scanning : waiting;
                  break;
               default : break;
            }
        }
        return new_mode;
    }




static double scan_delta_in ( scan_vector a )
{

   double degrees_of_scan;

   switch ( relative[a].mode ) {
      case idle : degrees_of_scan = 0;
                  break;
      case going_to_scan : degrees_of_scan = relative[a].scan_offset_start;
                           break;
      case scanning : degrees_of_scan = relative[a].scan_offset_start +
                                  relative[a].step_size * relative[a].steps;
                      break;
      case waiting : degrees_of_scan = relative[a].scan_offset_start;
                     break;
      default : break;
   }
   return degrees_of_scan;
}

static long int step_count_for ( scan_vector a )
{
   return relative[a].mode == scanning ?  relative[a].steps + 1 : 0;
}


void add_scan_delta_to
   ( double *azimuth_angle,
     double *elevation_angle )
{

    /*
     * Define the mapping from a scan vector into an antenna axis.
     */

    typedef enum { elevation_scan, azimuth_scan } antenna_axis;
    const antenna_axis scan_direction_for[NUMBER_OF_AXES] =
        { elevation_scan, azimuth_scan };

    /*
     * Define a safe limit for the elevation angle, to avert division by
     * zero.
     */

	const double MAX_ELEVATION = 89.9;		// Was 88.0 !!! found this
											// after tracking failed on a
                                            // satellite!! LJS
    const double PI = 3.14159;                            /* wot it sez */
    const double DEGREES_TO_RADIANS  = 2 * PI / 360.0;    /* wot it sez */

    /*
     * Define a table of the adjusted commanded position in each axis.
     */

    double new_position_in[NUMBER_OF_AXES];

    scan_vector scan_direction;           /* axis for scan vector */
    double X_elevation_factor;            /* cos (el) factor for azimuth */
    int scan;                             /* loop counter */

    /*
     * Define the parameters read from the parser.
     */
     
    double fixed_off, start_off, end_off, rate_off;
    state previous_mode_of_scanning_axis;

    /*
     * Initialize the table of adjusted positions to te current commanded
     * positions.
     */

    new_position_in[elevation_scan] = clip ( *elevation_angle, MAX_ELEVATION );
    new_position_in[azimuth_scan] = *azimuth_angle;

    /*
     * Get current softswitch settings from the parser module.
     */

    scanning_axis = get_scan_vector ();
    start_scan = get_start_scanning_sweep ();
    stop_scan = get_remove_scanning_condition ();

    /*
     * Note the state of the vector for the current scan, before it is
     * updated.
     */

    previous_mode_of_scanning_axis = relative[scanning_axis].mode;
 
    /*
	 * If a scan is not in progress, update the scan parameter tables with
     * the latest values from the parser.
     */

      for ( scan = elev_axis ; scan <= cross_elevation_vector ; scan++ ) {
         if ( start_scan == OFF ) {
            get_offsets_for
               ( scan, &fixed_off, &start_off, &end_off, &rate_off );
            update_scan_values_for
               ( scan, fixed_off, start_off, end_off, rate_off );
          }

       /*
	* Change state if appropriate, and increment the count of increments
	* to be applied for the current state.
	*/

       relative[scan].mode = next_state_for (scan );
       relative[scan].steps = step_count_for ( scan );

       /*
	* If the scan vector is cross-elevation, the azimuth angle must
	* be adjusted by the 1 / cosine ( elevation angle ).
	*/

       scan_direction = scan_direction_for[scan];
       X_elevation_factor = scan_direction == elevation_scan ? 1 :
	       1 / cos ( ( *elevation_angle + relative[elev_axis].fixed_offset )
                  *  DEGREES_TO_RADIANS );

       /* 
	* Compute and store the new adjustment for the commanded angle. Then
	* apply the adjustment to the commanded angle.
	*/

       delta_in[scan] = relative[scan].fixed_offset  + scan_delta_in ( scan );
       new_position_in[scan_direction] += delta_in[scan] * X_elevation_factor;
   }

    /*
     * If the mode of the scanning axis has just changed from "scanning" to "waiting", then intenally
     * reset the "start_scan_sweep" switch to "off" and wait for the next scan sweep.
     */
   
   if ( previous_mode_of_scanning_axis == scanning
        && relative[scanning_axis].mode == waiting )
       reset_start_scanning_sweep ();

   /*
    * If the mode of the scanning axis is idle, then internally reset the "remove_scanning_condition" to             * "off" and wait for  the next command.
    */

   if ( relative[scanning_axis].mode == idle )
       reset_remove_scanning_condition ();

   /*
    * If the mode of the scanning axis has changed to idle from another state ( due to the "remove_
    * scanning_condition" switch being pressed ) then internally reset the "start_scan_sweep" swicth
    * to "off" and wait for the next command.
    */

   if ( relative[scanning_axis].mode == idle
        && previous_mode_of_scanning_axis != idle )
       reset_start_scanning_sweep ();

   /*
    * Write the new commanded angles into the output parameters.
    */

   *azimuth_angle = new_position_in[azimuth_scan];
   *elevation_angle = new_position_in[elevation_scan];
}
