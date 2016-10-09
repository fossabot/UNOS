/*
References:

L. Sciacca, R. J. Evans, An MIT-Rule Adaptive Control System for
			Antenna Tracking... IFAC Workshop, Tbilisi.

Sciacca, Evans, et al. High Accuracy Adaptive Servo Control of Large Antenna 
			Structures. Automatica, 1991 ?

*/

/*
Version Notes:

Circa 1986 	RJ Evans, RE Betz Australia Telecope.

1989 Original Version (Adaptive Algorithm)
	Developed as part of a M.E.. PI controller +
	Feed-forward originally designed by R. J. Evans
	for the Australia Telscope.

14 Sept 1990	LJS	Port to C from Pascal version used on Australia
				Telescope and adaptive control test program on Apollo.

Modifications:
14-Sept-1990 LJS Changed PI+FF controller to be adaptive.
				Changed filters to be general.
8-Oct-1990	LJS Added Elevation code.
15-Oct-1990 LJS added s/w limit saturation on az/el_cmd
4-Nov-1990 LJS Added comments and cleaned up code in general.
10-Dec-1990 LJS Added filter of Feed-forward component

28-July-1991 LJS Added NVRAM support for POSCON.

28-Sept-1991 LJS Cleaned up code. Made all variables, regressors,
			and controller parameters fields of structures. This
			will ensure easier modifications in the future and interfacing
			to the calling modules.

8-10-1991   LJS Changed rate_limiter to return the rate voltage. This
			was required due to the changes in the way the sequencer
			interfaced to hardware.

*/

/*
Module Description

This module contains the position control algorithm used for position and 
tracking for Antenna position control systems. This module can be used
as is for integration in a real-time adaptive system or as part of a 
development environment in which the user may change system parameters
interactively and run simulations using a digitised plant model.

Initialisation

To initialise the position controller module the following routines must be
called:

	init_position_controllers ( );

	init_default_controller_parameters()
		may be called in the NVRAM setup routine to fill NVRAM
		and test it. This routine loads NVRAM with sensible
		controller parameters to be found in poscon.h

	init_position_controllers ( ) should also be called after changing
		the position parameters in NVRAM. Note that these are only
		read into the POSCON copy when changed and are NOT read in every
		call to control_loop or rate_limiter.

Once initialised, there are other routines that must be called
at certain times to update internal controller variables. These
include:
	reset_pos_parameters ( )
		This resets the PI integrator and loads the measured
		position into controller variables that need to be reset
		to enable smooth changeover.

	reset_pos_filters ( )
		This should be called while slewing to enable smooth changeover
		to closed position loop.

	Remember that driving in rate mode is NOT open-loop but is position
	closed loop and merely tracking a constant ramp trajectory.

Slewing

	When slewing, the user normally applies a constant rate command
	to the drives. This may be done by using the rate_limiter
	routine. By passing in the desired MOTOR speed, not the load speed,
	we can safely accelerate the motor and antenna system. The rate
	limiter is used in BOTH the position loop and slewing loops.
	The rate limiter does BOTH resonance energy filtering and
	rate limiting.

	e.g. SLEW_SPEED = 10500;  / 1750rpm =10500 deg/sec /

			volts = rate_limiter ( SLEW_SPEED );

Positioning

	When in precision position closed loop, one calls the control loop
	routine, passing in the appropriate arguments, and the desired
	MOTOR speed is returned. This is then passed, as above, to the
	rate_limiter.

	e.g.	motor_speed = control_loop ( axis, adapt_switch, cmd, msd );

			volts = rate_limiter ( motor_speed );

	where axis is the axis to control
			adapt_switch turns adaptation on or off
			cmd is the desired commanded position in degrees
			msd is the measured position in degrees

Description of Position Loop

	The position loop is a PI controller with anti-windup running sum
	integrator. There is a second order feed-forward component that
	is used in tracking mode ( when the adaptation switch is on ).

	The first-order derivative feed-forward component is adaptive. It
	is a performance oriented adaptive control whose criterion is to
	reduce position error as well as reduce integrator action in the
	feed-back PI loop. The method used to adapt is the MIT-Rule
	which is merely a gradient type estimator. There are a number of
	parameters that must be setup for this and for the PI loop.

	All of the parameters that need to be setup may be found in
	the structure par[axis]. This is kept in NVRAM.

Structures

	There are 4 main structures that allow one to look into the controller
	operation. This is particularly handy when debugging as all variables may
	be inspected in structure con [axis].

	controller_parameter_struct par [ 2 ];
		Contains controller parameters. e.g. proportional and integral
		gains, adaptive controller constants. All are constant
		until changed in NVRAM. Note

	controller_struct con [ 2 ];
		Contains all temporary variables that belong to each axis.

	res_filter_struct res_filter [ 2 ];
		Contains regressor and coefficient values for the anti-resonance
		filters.

	diff_filter_struct diff_filter [ 2 ];
		Contains the regressor and coefficient values for the
		differential filters used in the feed-forward.

	mit_adaptation_struct mit [ 2 ];
		Contains all temporary variables used in the adaptation routine.

*/

#include <math.h>
#include <stdlib.h>
#include <dos.h>

#include "..\posloop\poscon.h"	/* default definitions */

#include <seq.h>
#include <seqext.h>
#include <posext.h>
#include <nvramext.h>	/* Import NVRAM routines */
#include <conio.h>

#include <pcscr.h>

#define pi 3.141592654

/*---- Azimuth Prototypes -----*/
static void init_position_controller ( unsigned int axis );
static void ff_adaptation ( unsigned int axis, double cmd );
static void pid_controller ( unsigned int axis,
							unsigned int adaptation_switch,
							double cmd, double msd );
extern int rate_value_count;

static double resonance_filter ( unsigned int axis, double filter_in );
static double rate_to_volts ( unsigned int axis, double az_control );

/*-------------------- Local Variables for Module ------------------------*/

/*---- General Variables ----*/

		/*---- Structures (typedefed in posext.h ) ----*/

/* Control algorithm constants read in from NVRAM */
static controller_parameter_struct par [ 2 ];

/* Structures that allow generic controller routines */

static controller_struct con [ 2 ];
static res_filter_struct res_filter [ 2 ];
static diff_filter_struct diff_filter [ 2 ];

static mit_adaptation_struct mit [ 2 ];

int az_flag;


/*--------------------------------- Module Start -------------------------*/

/*
****************************************************************************
init_position_controllers

	Routine to initialise both axes position controllers.

Parameters:
	None

Returns:
	NVRAM Error
****************************************************************************
*/
int init_position_controllers ( ) {

	int status1, status2;

	/* Get controller Parameters from NVRAM */
	status1 = get_controller_parameters ( AZ_AXIS, &par [ AZ_AXIS ] );
	status2 = get_controller_parameters ( EL_AXIS, &par [ EL_AXIS ] );

	init_position_controller ( AZ_AXIS );
	init_position_controller ( EL_AXIS );

	return ( status1 | status2 );

} /* end of init_position_controllers */

/*
****************************************************************************
reset_pos_parameters

	Routine to reset the position controller parameters for azimuth drives.
This is necessary to ensure smooth changeover to precision positioning. The
regressor vectors ( previous values ) in the controller filters have to
reset to avoid transients.

Parameters:
	el - Pointer to axis structure for elevation

****************************************************************************
*/
void reset_pos_parameters ( unsigned int axis, double msd ) {

	/*---- Reset some controller Parameters */
	con [ axis ].cmd_prev = msd;
	con [ axis ].cmd_mit_prev = msd;

	con [ axis ].rate_volts = 0;
	con [ axis ].cmd_diff_1 = 0;
	con [ axis ].cmd_dotdot_1 = 0;
	con [ axis ].cmd_dot = 0;

	/* pid_controller */
	con [ axis ].int_sum = 0;
	con [ axis ].control_limited = 0;

	/* Slew rate limiter */
	con [ axis ].dem_rate_1 = 0;

	/* Initialise the ff gain */
	mit [ axis ].ff_gain = par [ axis ].initial_mit_ff_gain  ;

	/* Set Resonance filter regressors */
	res_filter [ axis ].ym_2 = 0;
	res_filter [ axis ].ym_1 = 0;
	res_filter [ axis ].um_1 = 0;

} /* end of reset_pos_parameters */

/*
****************************************************************************
reset_pos_filters

	Routine to reset the position controller parameters for azimuth drives.
This is necessary to ensure smooth changeover to precision positioning. The
regressor vectors ( previous values ) in the controller filters have to
reset to avoid transients.

This routine should be called when slewing to ensure that when we change
over to position loop mode we have smooth changeover.

Parameters:
	axis
	el - msd position

Returns:
	None

****************************************************************************
*/
void reset_pos_filters ( unsigned int axis, double msd ) {

	/*---- Reset some controller Parameters */
	con [ axis ].cmd_prev = msd;
	con [ axis ].cmd_mit_prev = msd;

	con [ axis ].cmd_diff_1 = 0;
	con [ axis ].cmd_dotdot_1 = 0;
	con [ axis ].cmd_dot = 0;

	/* pid_controller */
	con [ axis ].int_sum = 0;

	/* Initialise the ff gain */
	mit [ axis ].ff_gain = par [ axis ].initial_mit_ff_gain  ;

} /* end of reset_pos_filters */

/*
****************************************************************************
init_position_controller

	Routine to initialise local variables in this module. This routine should be
called prior to carrying out real-time control and ideally before applying
power to the drive system. This may be an important issue when attempting to
avoid the situation where we may have initialised the last position value,
gone into slew mode and then gone into position mode.

Because this routine modifies controller filter regressors, it is essential
to call this routine after a demanded position has been determined.

Parameters:
	axis - axis number AZ_AXIS or EL_AXIS

Returns:
	None

****************************************************************************
*/
static void init_position_controller ( unsigned int axis ) {

	double a,w,b,g,wr;

	/* for resonance filter */
	con [ axis ].control_1 = 0;

	/* pid_controller */
	con [ axis ].int_sum = 0;
	con [ axis ].cmd_prev = 0;
	con [ axis ].cmd_dot = 0;
	con [ axis ].control_limited = 0;

	/* Slew rate limiter */
	con [ axis ].control_rate_1 = 0;
	con [ axis ].dem_rate_1 = 0;

	/* Get Resonance filter coefficients */
	/* This WAS the first order approximation. I left it here in case
		we had trouble in the field.
	*/
/*	res_filter [ axis ].alpha = exp ( -par [ axis ].res_bw * CONTROL_PERIOD * 2 * pi );
	res_filter [ axis ].beta = ( 1 - res_filter [ axis ].alpha );
*/
#define DAMP 0.6

	/* Resonance ZOH Equivalent */
	/* Second order filter */
	wr = par [ axis ].res_bw*2.0*pi;
	a = exp ( - DAMP * wr * CONTROL_PERIOD );
	w = wr * sqrt ( 1 - DAMP*DAMP );
	b = cos ( w*CONTROL_PERIOD );
	g = sin ( w*CONTROL_PERIOD );

	res_filter [ axis ].b1 = -( 1 - a*( b + (DAMP*wr*g/w) ) );
	res_filter [ axis ].b2 = -(a*a + a* ( (DAMP*wr*g/w) - b ));
	res_filter [ axis ].a1 = -2*a*b;
	res_filter [ axis ].a2 = a*a;

	/* Get differentiator Filter coefficients */
	diff_filter [ axis ].alpha = exp ( -par [ axis ].diff_bw * CONTROL_PERIOD * 2 * pi );
	diff_filter [ axis ].beta = ( 1 - diff_filter [ axis ].alpha);

} /* end of init_position_controller */


/*
****************************************************************************
void init_default_controller_parameters

	Routine to initialise the position controller parameters. This should be
called to load the controller parameters that are kept in NVRAM. Note that
what we should do is read in values from NVRAM into this structure only
once when we need to.

****************************************************************************
*/
void init_default_controller_parameters ( unsigned int axis,
						controller_parameter_struct * par_temp ) {

	/* setup default parameters */
	if ( axis == AZ_AXIS ) {
		par_temp->prop_gain 		= AZ_PROP_GAIN;
		par_temp->integral_gain		= AZ_INTEGRAL_GAIN;
		par_temp->res_bw 			= AZ_RES_BW;
		par_temp->diff_bw			= AZ_DIFF_BW;
		par_temp->int_positive_limit	= AZ_INT_POSITIVE_LIMIT;
		par_temp->int_negative_limit	= AZ_INT_NEGATIVE_LIMIT;
		par_temp->mit_alpha			= AZ_MIT_ALPHA;
		par_temp->mit_beta			= AZ_MIT_BETA;
		par_temp->mit_ff_high		= AZ_MIT_FF_HIGH;
		par_temp->mit_ff_low		= AZ_MIT_FF_LOW;
		par_temp->mit_maxincr		= AZ_MIT_MAXINCR;
		par_temp->sign				= AZ_SIGN;
		par_temp->mit_maxref		= AZ_MIT_MAXREF;
		par_temp->parabolic_gain	= AZ_PARABOLIC_GAIN;
		par_temp->initial_mit_ff_gain  = AZ_MIT_FF_INITIAL_GAIN;
		par_temp->max_volts			= AZ_MAX_VOLTS;
		par_temp->min_volts			= AZ_MIN_VOLTS;
		par_temp->sign				= AZ_SIGN;
		par_temp->max_acceleration	= AZ_MAX_ACCELERATION;
		par_temp->gear_ratio		= AZ_GEAR_RATIO;
		par_temp->rate_gain			= AZ_RATE_GAIN;
		par_temp->rate_limit_region	= AZ_RATE_LIMIT_REGION;
		par_temp->slew_region		= SLEW_REGION_DEF;
		}

	if ( axis == EL_AXIS ) {
		par_temp->prop_gain 		= EL_PROP_GAIN;
		par_temp->integral_gain		= EL_INTEGRAL_GAIN;
		par_temp->res_bw 			= EL_RES_BW;
		par_temp->diff_bw			= EL_DIFF_BW;
		par_temp->int_positive_limit	= EL_INT_POSITIVE_LIMIT;
		par_temp->int_negative_limit	= EL_INT_NEGATIVE_LIMIT;
		par_temp->mit_alpha			= EL_MIT_ALPHA;
		par_temp->mit_beta			= EL_MIT_BETA;
		par_temp->mit_ff_high		= EL_MIT_FF_HIGH;
		par_temp->mit_ff_low		= EL_MIT_FF_LOW;
		par_temp->mit_maxincr		= EL_MIT_MAXINCR;
		par_temp->sign				= EL_SIGN;
		par_temp->mit_maxref		= EL_MIT_MAXREF;
		par_temp->parabolic_gain	= EL_PARABOLIC_GAIN;
		par_temp->initial_mit_ff_gain  = EL_MIT_FF_INITIAL_GAIN;
		par_temp->max_volts			= EL_MAX_VOLTS;
		par_temp->min_volts			= EL_MIN_VOLTS;
		par_temp->sign				= EL_SIGN;
		par_temp->max_acceleration	= EL_MAX_ACCELERATION;
		par_temp->gear_ratio		= EL_GEAR_RATIO;
		par_temp->rate_gain			= EL_RATE_GAIN;
		par_temp->rate_limit_region	= EL_RATE_LIMIT_REGION;
		par_temp->slew_region		= SLEW_REGION_DEF;
		}

} /* end of init_default_controller_parameters */


/*
****************************************************************************
pid_control_loop

	Routine that does does the control loop. It calculates the measured error
decides on whether to do precision control or rate control. If it does
precision control then it calls the PID and adaptive algorithms.

	This routine should be called by the external module that wishes to do
closed-loop control.

	It is possible to call the precision position loop with adaptation turned
off. This may be required in certain modes that want to do small step
perturbations.

Parameters:
	(unsigned int)axis = axis to be controlled
	(unsigned int)adaptation_switch : = 0 turn adaptation off and do not use feed-forward.
	(double)cmd = commanded az position in degrees
	(double)msd = measured az position in degrees

Returns:
	(double) control deg/sec

****************************************************************************
*/
double pid_control_loop ( unsigned int axis,
					unsigned int adaptation_switch,
					double cmd,
					double msd ) {

		/*---- Measured Position Error */
		con [ axis ].err = cmd - msd;
		/*---- Do precision loop ----*/
		pid_controller ( axis, adaptation_switch, cmd, msd );

		if ( adaptation_switch )
			ff_adaptation ( axis, cmd );

		return ( con [ axis ].control_unfilt );

} /* end of pid_control_loop */


/*
****************************************************************************
ff_adaptation

	This routine carries out the MIT-Rule adaptation as described in the
references. Briefly, the aim of the algorithm is to dynamically adjust the
position feed-forward parameter to reduce measured Antenna error with
respect to the demanded position. It does this by integrating the sum of the
position error and the integral error. By using the integral error signal
derived from the PID controller, it is possible to reduce the energy stored
in the integral action.

	Tuning the adaptive algorithm is particularly important and can have a
marked affect on the performance of the Antenna. Careless choice of the
'alpha' and 'beta' coefficients may in fact result in instability or
oscillations. Some guidelines will be presented in late 1990 as to the
tuning procedure.

Parameter:
	axis = axis to control
	cmd = commanded position

****************************************************************************
*/
static void ff_adaptation ( unsigned int axis, double az_cmd ) {

	/* Do differentiator */
	con [ axis ].cmd_dot = ( az_cmd - mit [ axis ].cmd_prev ) / CONTROL_PERIOD;
	mit [ axis ].cmd_prev = az_cmd;

	/* Normalise for one rate */
	mit [ axis ].norm = ( par [ axis ].mit_maxref ) /
						( fabs ( con [ axis ].cmd_dot ) + 1e-10 );

	mit [ axis ].sign_cmd_dot = ( con [ axis ].cmd_dot /
					( fabs ( con [ axis ].cmd_dot ) + 1e-10 ) );

	mit [ axis ].alpha_err = par [ axis ].mit_alpha *
						mit [ axis ].norm * con [ axis ].err;

	mit [ axis ].beta_err = par [ axis ].mit_beta * con [ axis ].int_sum;

	mit [ axis ].incr = -( mit [ axis ].alpha_err + mit [ axis ].beta_err )
							 * mit [ axis ].sign_cmd_dot;

	/* Apply some adaptation limiting for robustness */
	if ( mit [ axis ].incr > par [ axis ].mit_maxincr )
			mit [ axis ].incr = par [ axis ].mit_maxincr;

	if ( mit [ axis ].incr < -par [ axis ].mit_maxincr )
			mit [ axis ].incr	= -par [ axis ].mit_maxincr;

	mit [ axis ].ff_gain = mit [ axis ].ff_gain - mit [ axis ].incr;

	/* Apply limits to the adapted feed-forward parameter */
	if ( mit [ axis ].ff_gain > par [ axis ].mit_ff_high )
		 mit [ axis ].ff_gain = par [ axis ].mit_ff_high;
	if ( mit [ axis ].ff_gain < par [ axis ].mit_ff_low )
		 mit [ axis ].ff_gain = par [ axis ].mit_ff_low;

} /* end of mit_rule_adaptation */


/*************************************************************************/
/* open_loop_controller 																								 */
/*                                                                       */
/* returns the value of the cyclic flag from -5 to 5 										 */
/*************************************************************************/
double open_loop_controller(void)
{
		return(rate_value_count);
}


/*
****************************************************************************
pid_controller

Routine to carry out PID feed-back and Feed-Forward Control. Although the
original version did not incorporate derivative action, it has been found
that some D action may help to reduce the effects of stiction.


Calls:
	resonance_filter

Arguments:
	axis
	adaptation switch
	cmd
	msd

****************************************************************************
*/
static void pid_controller ( unsigned int axis,
					unsigned int adaptation_switch,
					double cmd,
					double msd ) {

	/*---- Differentiate the commanded position */
	con [ axis ].cmd_diff = ( cmd - con [ axis ].cmd_prev ) / CONTROL_PERIOD;
	con [ axis ].cmd_prev = cmd;

	if ( con [ axis ].cmd_diff > 0.3 )
		con [ axis ].cmd_diff = 0.3;
	if ( con [ axis ].cmd_diff < -0.3 )
		con [ axis ].cmd_diff = -0.3;

	con [ axis ].cmd_dotdot = ( con [ axis ].cmd_diff -
						con [ axis ].cmd_dotdot_1 ) / CONTROL_PERIOD;

	con [ axis ].cmd_dotdot_1 = con [ axis ].cmd_diff;

	if ( con [ axis ].cmd_dotdot > 0.2 )
		con [ axis ].cmd_dotdot = 0.2;
	if ( con [ axis ].cmd_dotdot < -0.2 )
		con [ axis ].cmd_dotdot = -0.2;

	con [ axis ].cmd_diff = diff_filter [ axis ].beta * con [ axis ].cmd_diff +
						con [ axis ].cmd_diff_1 * diff_filter [ axis ].alpha;
	con [ axis ].cmd_diff_1 = con [ axis ].cmd_diff;

	if ( fabs ( con [ axis ].err ) < 0.2 )
		con [ axis ].err = 0;

	/*---- Do integrator */
	con [ axis ].int_sum = con [ axis ].int_sum + con [ axis ].err;

	if ( con [ axis ].int_sum > par [ axis ].int_positive_limit )
		con [ axis ].int_sum = par [ axis ].int_positive_limit;
	if ( con [ axis ].int_sum < par [ axis ].int_negative_limit )
		con [ axis ].int_sum = par [ axis ].int_negative_limit;

	con [ axis ].int_action = par [ axis ].integral_gain * con [ axis ].int_sum;

	/*---- Do feed-forward on reference and find new error */
	if ( adaptation_switch == 0 )
		con [ axis ].pred = cmd;
	else
		con [ axis ].pred = cmd + mit [ axis ].ff_gain *
							con [ axis ].cmd_diff +
							par [ axis ].parabolic_gain * con [ axis ].cmd_dotdot;

	con [ axis ].control_unfilt = par [ axis ].prop_gain *
							( con [ axis ].pred - msd ) +
							con [ axis ].int_action;

} /* end of pid_controller */

/*
****************************************************************************
resonance_filter

Routine to filter control signal to avoid exciting resonance in the Antenna
structure.

1st or 2nd order.

****************************************************************************
*/
double resonance_filter ( unsigned int axis, double filter_in ) {

	double control_tmp;

	control_tmp = ( -res_filter [ axis ].a2 * res_filter [ axis ].ym_2 -
						res_filter [ axis ].a1 * res_filter [ axis ].ym_1
						- res_filter [ axis ].b1 * filter_in +
						res_filter [ axis ].b2 * res_filter [ axis ].um_1 );

	res_filter [ axis ].ym_2 = res_filter [ axis ].ym_1;
	res_filter [ axis ].ym_1 = control_tmp;
	res_filter [ axis ].um_1 = -filter_in;

	return ( control_tmp );

} /* end of resonance filter */

/*
****************************************************************************
rate_limiter

Routine to limit rate action applied to the motor drives. This is primarily
designed to avoid damaging the motor/gear system. Also scales and
outputs the control to the DACs.

This routine also calls the resonance filter before applying the rate limits.
Note that the limiting was done after the filter in case the filter caused
large spikes to enter the system.

The maximum acceleration is specified in terms of load speed (this could
be made motor side if desired), and if we exceed this limit over the control
period, we increment theprevious control value ( .dem_rate_1 ) by the
maximum allowable rate in the control period (that is whay we multiply
by the CONTROL_PERIOD).

Also we reset the integrator when we limit.


Parameters:
	double dem_rate - demanded rate in deg/sec

Returns:
	(double)voltage to be applied to drives

****************************************************************************
*/
double rate_limiter ( unsigned int axis, double dem_rate ) {

	double control, volts; // max_acc;

	//max_acc = par [ axis ].max_acceleration * par [ axis ].gear_ratio;

	/*---- Smooth demanded rate using a second order filter to ensure
		we do not excite resonances. Removed the call to this
		routine on 4/4/95 (ST and SRW)
	*/
 /*	control = resonance_filter ( axis, dem_rate ); */
	control = dem_rate;

	/* Only apply rate limit outside a small rate region */
/*	if ( fabs( control ) >= par [ axis ].rate_limit_region )
		{
		if ( ( control - con [ axis ].dem_rate_1 ) / CONTROL_PERIOD > max_acc )
			{
			control = con [ axis ].dem_rate_1 + max_acc * CONTROL_PERIOD;
			con [ axis ].int_sum = 0;
			}
		else
			if ( ( control - con [ axis ].dem_rate_1 ) / CONTROL_PERIOD < -max_acc )
				{
				control = con [ axis ].dem_rate_1 - max_acc * CONTROL_PERIOD;
				con [ axis ].int_sum = 0;
				}
		}

*/
	/*---- This value is used by the control output routines */
	con [ axis ].control_limited = control;

	/*---- Form Voltage and clip if necessary. N.B. This routine
		does NOT do I/O. It merely determines voltage. */
	volts = rate_to_volts ( axis, control );
	con [ axis ].dem_rate_1 = volts * par [ axis ].rate_gain * par [ axis ].sign;

	return ( volts );

} /* end of rate_limiter */

/*
****************************************************************************
rate_to_volts

Routine to convert rate to volts.

	This routine merely forms the DAC word to be output at next control 
period.

parameters:
	axis    - az or el
	control - deg/sec

Returns:
	(double)volts to be applied to drives

****************************************************************************
*/
static double rate_to_volts ( unsigned int axis, double control_out ) {

	/* we should output the last value to the Azimuth DAC */
	con [ axis ].rate_volts = ( control_out / par [ axis ].rate_gain );

	if ( con [ axis ].rate_volts > par [ axis ].max_volts )
		con [ axis ].rate_volts = par [ axis ].max_volts;
	if ( con [ axis ].rate_volts < par [ axis ].min_volts )
		con [ axis ].rate_volts = par [ axis ].min_volts;

	/* Apply drive direction sign */
	con [ axis ].rate_volts = par [ axis ].sign * con [ axis ].rate_volts;

	return ( con [ axis ].rate_volts );

} /* end of rate_to_volts */


/*======================= Support Routines ===============================*/
/*
****************************************************************************
set_az_prop_gain

Routine to allow another module set az_prop_gain. NOTE: we should place
limits on the amount the user can change the gains here to avoid rediculous
gains.

****************************************************************************
*/
void set_az_prop_gain ( double az_prop_gain_new ) {

	disable ( );
	par [ AZ_AXIS ].prop_gain = az_prop_gain_new;
	enable ( );

} /* end of set_az_prop_gain */

/*
****************************************************************************
set_az_integral_gain

Routine to allow another module set az_integral_gain
****************************************************************************
*/
void set_az_integral_gain ( double az_integral_gain_new ) {

	disable ( );
	par [ AZ_AXIS ].integral_gain = az_integral_gain_new;
	enable ( );

} /* end of set_az_integral_gain */

/*
****************************************************************************
set_res_bw

Routine to allow another module set az_res_bw, the resonance filter
roll-off frequency.

****************************************************************************
*/
void set_res_bw ( unsigned int axis, double az_res_bw_new ) {

	double a,w,b,g,wr;

	par [ axis ].res_bw = az_res_bw_new;
/*	az_res_filt_alpha = exp ( -az_res_bw * CONTROL_PERIOD * 2 * pi );
	az_res_filt_beta = ( 1 - az_res_filt_alpha);
*/
#define DAM 0.8

	wr = par [ axis ].res_bw * 2 * pi;
	a = exp ( - DAM * wr * CONTROL_PERIOD );
	w = wr * sqrt ( 1 - DAM*DAM );
	b = cos ( w*CONTROL_PERIOD );
	g = sin ( w*CONTROL_PERIOD );
	res_filter [ axis ].b1 = -( 1 - a*( b + ( DAM * wr * g / w ) ) );
	res_filter [ axis ].b2 = -(a*a + a* ( ( DAM * wr * g / w ) - b ));
	res_filter [ axis ].a1 = -2*a*b;
	res_filter [ axis ].a2 = a*a;

} /* end of set_res_bw */


/*
****************************************************************************
set_diff_bw

Routine to allow another module set az_res_bw, the controller differentotaor
filter roll-off frequency.

****************************************************************************
*/
void set_diff_bw ( unsigned int axis, double diff_bw_new ) {

	par [ axis ].diff_bw = diff_bw_new;
	diff_filter [ axis ].alpha = exp ( -par [ axis ].diff_bw * CONTROL_PERIOD * 2 * pi );
	diff_filter [ axis ].beta = ( 1 - diff_filter [ axis ].alpha);

} /* end of set_diff_bw */

/*
****************************************************************************
set_az_int_positive_limit

Routine to allow another module set az_int_positive_limit
****************************************************************************
*/
void set_az_int_positive_limit ( double az_int_positive_limit_new ) {

	disable ( );
	par [ AZ_AXIS ].int_positive_limit = az_int_positive_limit_new;
	enable ( );

} /* end of set_az_int_positive_limit */


/*
****************************************************************************
set_az_int_negative_limit

Routine to allow another module set az_int_negative_limit
****************************************************************************
*/
void set_az_int_negative_limit ( double az_int_negative_limit_new ) {

	disable ( );
	par [ AZ_AXIS ].int_negative_limit = az_int_negative_limit_new;
	enable ( );

} /* end of set_az_int_negative_limit */


/*
****************************************************************************
set_az_mit_alpha

Routine to allow another module set az_mit_alpha
****************************************************************************
*/
void set_az_mit_alpha ( double az_mit_alpha_new ) {

	disable ( );
	par [ AZ_AXIS ].mit_alpha = az_mit_alpha_new;
	enable ( );

} /* end of set_az_mit_alpha */


/*
****************************************************************************
set_az_mit_beta

Routine to allow another module set az_mit_beta
****************************************************************************
*/
void set_az_mit_beta ( double az_mit_beta_new ) {

	disable ( );
	par [ AZ_AXIS ].mit_beta = az_mit_beta_new;
	enable ( );

} /* end of set_az_mit_beta */


/*
****************************************************************************
set_az_mit_ff_high

Routine to allow another module set az_mit_ff_high
****************************************************************************
*/
void set_az_mit_ff_high ( double az_mit_ff_high_new ) {

	disable ( );
	par [ AZ_AXIS ].mit_ff_high = az_mit_ff_high_new;
	enable ( );

} /* end of set_az_mit_ff_high */


/*
****************************************************************************
set_az_mit_ff_low

Routine to allow another module set az_mit_ff_low
****************************************************************************
*/
void set_az_mit_ff_low ( double az_mit_ff_low_new ) {

	disable ( );
	par [ AZ_AXIS ].mit_ff_low = az_mit_ff_low_new;
	enable ( );

} /* end of set_az_mit_ff_low */


/*
****************************************************************************
set_az_mit_maxincr

Routine to allow another module set az_mit_maxincr
****************************************************************************
*/
void set_az_mit_maxincr ( double az_mit_maxincr_new ) {

	disable ( );
	par [ AZ_AXIS ].mit_maxincr = az_mit_maxincr_new;
	enable ( );

} /* end of set_az_mit_maxincr */


/*
****************************************************************************
set_az_mit_maxref

Routine to allow another module set az_mit_maxincr
****************************************************************************
*/
void set_az_mit_maxref ( double az_mit_maxref_new ) {

	disable ( );
	par [ AZ_AXIS ].mit_maxref = az_mit_maxref_new;
	enable ( );

} /* end of set_az_mit_maxref */


/*
****************************************************************************
set_az_sign

Routine to allow another module set az_sign
****************************************************************************
*/
void set_az_sign ( int az_sign_new ) {

	disable ( );
	par [ AZ_AXIS ].sign = az_sign_new;
	enable ( );

} /* end of set_az_sign */

/*
****************************************************************************
return_az_rate_dac

Routine to allow another module to read the control action signal as it is
fed into the DACs. This is picked up by the az_model_task ( if in simulation
mode ) as an integer into the DAcs. The model should model the quantisation 
of the DACs as well.

****************************************************************************
*/
int return_az_rate_dac ( ) {

	int intermediate;

	disable ( );
	intermediate = con [ AZ_AXIS ].rate_dac;
	enable ( );

	return ( intermediate );

} /* end of return_az_rate_dac */


/*
****************************************************************************
return_az_control

Routine to allow another module to read the control action signal.

****************************************************************************
*/
double return_az_control ( ) {

	double intermediate;

	disable ( );
	intermediate = con [ AZ_AXIS ].rate_volts;
	enable ( );

	return ( intermediate );

} /* end of return_az_control */

/*
****************************************************************************
return_az_int_action

Routine to allow another module to read the control integral action signal.

****************************************************************************
*/
double return_az_int_action ( ) {

	double intermediate;

	disable ( );
	intermediate = con [ AZ_AXIS ].int_action/1312.5;
	enable ( );

	return ( intermediate );

} /* end of return_az_int_action */

/*
****************************************************************************
return_az_mit_ff_gain

Routine to allow another module to read the az_mit_ff_gain adapted
feed-forward gain.

****************************************************************************
*/
double return_az_mit_ff_gain ( ) {

	double intermediate;

	disable ( );
	intermediate = mit [ AZ_AXIS ].ff_gain;
	enable ( );

	return ( intermediate );

} /* end of return_az_mit_ff_gain */

/*
****************************************************************************
return_az_err

Routine to allow another module to read the measured position error, az_err.

****************************************************************************
*/
double return_az_err ( ) {

	double intermediate;

	disable ( );
	intermediate = con [ AZ_AXIS ].err;
	enable ( );

	return ( intermediate );

} /* end of return_az_err */


/*
****************************************************************************
return_az_cmd_dot

Routine to allow another module to read the measured az command
differential.

****************************************************************************
*/
double return_az_cmd_dot ( ) {

	double intermediate;

	disable ( );
	intermediate = con [ AZ_AXIS ].cmd_dot;
	enable ( );

	return ( intermediate );

} /* end of return_az_cmd_dot */


/*
****************************************************************************
set_az_rate_dac

	Routine to allow another module set az_rate_dac. Dac voltage.
This is done by the sequencer task to ensure the rate will be set to zero on
standby.

****************************************************************************
*/
void set_az_rate_dac ( unsigned int rate ) {

	disable ( );
	con [ AZ_AXIS ].rate_dac = rate;
	enable ( );

} /* end of set_el_rate_dac */


/*
****************************************************************************
set_az_initial_ff_gain

	Routine to set the initial feed-forarwd gain.

****************************************************************************
*/
void set_az_initial_ff_gain ( double value_in ) {

	disable ( );
	par [ AZ_AXIS ].initial_mit_ff_gain = value_in;
	enable ( );

} /* end of set_az_initial_ff_gain */


/*
****************************************************************************
set_el_prop_gain

Routine to allow another module set el_prop_gain. NOTE: we should place
limits on the amount the user can change the gains here to avoid rediculous
gains.

****************************************************************************
*/
void set_el_prop_gain ( double el_prop_gain_new ) {

	disable ( );
	par [ EL_AXIS ].prop_gain = el_prop_gain_new;
	enable ( );

} /* end of set_el_prop_gain */

/*
****************************************************************************
set_el_integral_gain

Routine to allow another module set el_integral_gain
****************************************************************************
*/
void set_el_integral_gain ( double el_integral_gain_new ) {

	disable ( );
	par [ EL_AXIS ].integral_gain = el_integral_gain_new;
	enable ( );

} /* end of set_el_integral_gain */

/*
****************************************************************************
set_el_int_positive_limit

Routine to allow another module set el_int_positive_limit
****************************************************************************
*/
void set_el_int_positive_limit ( double el_int_positive_limit_new ) {

	disable ( );
	par [ EL_AXIS ].int_positive_limit = el_int_positive_limit_new;
	enable ( );

} /* end of set_el_int_positive_limit */


/*
****************************************************************************
set_el_int_negative_limit

Routine to allow another module set el_int_negative_limit
****************************************************************************
*/
void set_el_int_negative_limit ( double el_int_negative_limit_new ) {

	disable ( );
	par [ EL_AXIS ].int_negative_limit = el_int_negative_limit_new;
	enable ( );

} /* end of set_el_int_negative_limit */


/*
****************************************************************************
set_el_mit_alpha

Routine to allow another module set el_mit_alpha
****************************************************************************
*/
void set_el_mit_alpha ( double el_mit_alpha_new ) {

	disable ( );
	par [ EL_AXIS ].mit_alpha = el_mit_alpha_new;
	enable ( );

} /* end of set_el_mit_alpha */


/*
****************************************************************************
set_el_mit_beta

Routine to allow another module set el_mit_beta
****************************************************************************
*/
void set_el_mit_beta ( double el_mit_beta_new ) {

	disable ( );
	par [ EL_AXIS ].mit_beta = el_mit_beta_new;
	enable ( );

} /* end of set_el_mit_beta */


/*
****************************************************************************
set_el_mit_ff_high

Routine to allow another module set el_mit_ff_high
****************************************************************************
*/
void set_el_mit_ff_high ( double el_mit_ff_high_new ) {

	disable ( );
	par [ EL_AXIS ].mit_ff_high = el_mit_ff_high_new;
	enable ( );

} /* end of set_el_mit_ff_high */


/*
****************************************************************************
set_el_mit_ff_low

Routine to allow another module set el_mit_ff_low
****************************************************************************
*/
void set_el_mit_ff_low ( double el_mit_ff_low_new ) {

	disable ( );
	par [ EL_AXIS ].mit_ff_low = el_mit_ff_low_new;
	enable ( );

} /* end of set_el_mit_ff_low */


/*
****************************************************************************
set_el_mit_maxincr

Routine to allow another module set el_mit_maxincr
****************************************************************************
*/
void set_el_mit_maxincr ( double el_mit_maxincr_new ) {

	disable ( );
	par [ EL_AXIS ].mit_maxincr = el_mit_maxincr_new;
	enable ( );

} /* end of set_el_mit_maxincr */


/*
****************************************************************************
set_el_mit_maxref

Routine to allow another module set el_mit_maxincr
****************************************************************************
*/
void set_el_mit_maxref ( double el_mit_maxref_new ) {

	disable ( );
	par [ EL_AXIS ].mit_maxref = el_mit_maxref_new;
	enable ( );

} /* end of set_el_mit_maxref */


/*
****************************************************************************
set_el_sign

Routine to allow another module set el_sign
****************************************************************************
*/
void set_el_sign ( int el_sign_new ) {

	disable ( );
	par [ EL_AXIS ].sign = el_sign_new;
	enable ( );

} /* end of set_el_sign */


/*
****************************************************************************
set_el_rate_dac

	Routine to allow another module set el_rate_dac. Dac voltage.
This is done by the sequencer task to ensure the rate will be set to zero on 
standby.

****************************************************************************
*/
void set_el_rate_dac ( unsigned int rate ) {

	disable ( );
	con [ EL_AXIS ].rate_dac = rate;
	enable ( );

} /* end of set_el_rate_dac */

/*
****************************************************************************
return_el_rate_dac

Routine to allow another module to read the control action signal as it is
fed into the DACs. This is picked up by the el_model_task ( if in simulation 
mode ) as an integer into the DACs. The model should model the quantisation
of the DACs as well.

****************************************************************************
*/
int return_el_rate_dac ( ) {

	int intermediate;

	disable ( );
	intermediate = con [ EL_AXIS ].rate_dac;
	enable ( );

	return ( intermediate );

} /* end of return_el_rate_dac */


/*
****************************************************************************
return_el_control

Routine to allow another module to read the control action signal.

****************************************************************************
*/
double return_el_control ( ) {

	double intermediate;

	disable ( );
	intermediate = con [ EL_AXIS ].rate_volts;
	enable ( );

	return ( intermediate );

} /* end of return_el_control */

/*
****************************************************************************
return_el_int_action

Routine to allow another module to read the control integral action signal.

****************************************************************************
*/
double return_el_int_action ( ) {

	double intermediate;

	disable ( );
	intermediate = con [ EL_AXIS ].int_action;
	enable ( );

	return ( intermediate/1312.5 );

} /* end of return_el_int_action */

/*
****************************************************************************
return_el_mit_ff_gain

Routine to allow another module to read the el_mit_ff_gain adapted
feed-forward gain.

****************************************************************************
*/
double return_el_mit_ff_gain ( ) {

	double intermediate;

	disable ( );
	intermediate = mit [ EL_AXIS ].ff_gain;
	enable ( );

	return ( intermediate );
} /* end of return_el_mit_ff_gain */

/*
****************************************************************************
return_el_err

Routine to allow another module to read the measured position error, el_err.

****************************************************************************
*/
double return_el_err ( ) {

	double intermediate;

	disable ( );
	intermediate = con [ EL_AXIS ].err;
	enable ( );

	return ( intermediate );

} /* end of return_el_err */


/*
****************************************************************************
return_el_cmd_dot

Routine to allow another module to read the measured el command
differential.

****************************************************************************
*/
double return_el_cmd_dot ( ) {

	double intermediate;

	disable ( );
	intermediate = con [ EL_AXIS ].cmd_dot;
	enable ( );

	return ( intermediate );

} /* end of return_el_cmd_dot */
 
/*
****************************************************************************
get_slew_region

Routine to allow another module to read the slew_region

Used by sequencer

****************************************************************************
*/
double get_slew_region ( ) {

	double intermediate;

	disable ( );
	intermediate = par [ AZ_AXIS ].slew_region;
	enable ( );

	return ( intermediate );

} /* end of get_slew_region */

/*---------------------------- End of poscon Module ----------------------*/



