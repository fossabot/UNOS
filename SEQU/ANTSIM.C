/***************************************************************************
 FILE 		:- ANTSIM.C

 PROGRAMMER :- L Sciacca

*/

/*****************************************************************************

 DISCUSSION :
					Antenna Simulation Facility

								by

							L. Sciacca

		 Task to simulate motor drives and Antenna Structure.

	Originally developed for Australia Telescope Simulations and Adaptive
	Controller test bed on an Apollo Workstation.

Latest: 1-9-1991  LJS Made routines generic, fixed comments.
		10-10-1991 LJS Integrated new i/o routines. These are imported
				using antsimex.h

Module Description
------------------

	This module contains the antenna simulator routines. These routines
have been designed to be called from a task running at the control period.
In hindsight a better arrangement would have been to have a different
sampling rate so we could have a faster sampled model with a slower
control.

	The simulator interfaces to the controller or sequencer via
a memory mapping of i/o. This i/o space is assummed to have been setup
at system initialisation. The hardware routines will either write to the
actual hardware ports (if in "real" mode) or to the memory mapped i/o
if in simulation mode.

	This arrangement ensures complete isolation from the rest of the system.
Ideally these routines should reside in another computer and real hardware
used to simulate the antenna.

	Because we are assumming the simulator will be called from the
controller/sequencer, there may be no critical protection in passing
of parameters. Thus the sequencer (or more generally, the controller)
will look as follows:

	initialise antenna simulator ()

	while ( task_running )
		{

		sleep ( control_period );

		do input/output (real or simulated doesn't matter )
				digital i/o, volts

		determine new states, commanded angles

		calculate control (volts)

		run simulator

		}

Initialisation
--------------

	The module may be initialised into a known state by calling:

		init_antenna_simulator ( );

	This should be done prior to calling the simulator routine
	and only once.

	It is possible to change the starting antenna position by
	changing the #defines below:
		INITIAL_AZ_DEG
		INITIAL_EL_DEG

Running the Simulator
---------------------

	The routine to call every control period is:
		antenna_simulator ( );

Simulation Model
----------------

	Simplicity in model formulation was the initial design approach.
Experiments on the Australia Telescope enabled a ARX model of the
antenna to very high orders. It was felt to be more flexible
and easier to change however to have adjustable bandwidth and resonance
models based on simple digitisation of continuous systems.

	Each antenna model includes the following:

	Motor drive logic ( including reset/fail/starting/forward/reverse)
	Integrator ( rate to position )
	Resonance
	Stiction ( although this may not be as great on the elevation drive)
	Time constant (i.e. a lowpass filter )
	Noise on the drive system (constant DC bias and random noise )
		( This includes noise entering the system from the DACs )


****************************************************************************
*/

#include <stdio.h>
#include "dos.h"
#include <stdlib.h>
#include <conio.h>
#include <math.h>

#include <unosdef.h>
#include <seq.h>
#include <seqext.h>	/* import return_az_cmd, az_msd */
#include <antsim.h>
#include <antsimex.h>	/* Import put and get routines for simulator I/O */
#include "main_ext.h"

#define INITIAL_AZ_DEG	15
#define INITIAL_EL_DEG	25

static void antenna_model ( unsigned int axis );
static void init_antenna_model ( unsigned int axis ); /* must be called from main task creation */
static void init_state_machines ( void );

char dio_word [ 2 ]; /* one for each axis */

model_struct model_axis [ 2 ];
model_parameter_struct modpar [ 2 ];

	/* Extra model parameters wind speed etc... */
model_data_struct model;

/*
****************************************************************************
init_antenna_simulator ( )

	Routine to initialise drive simulation. Should be called at beginning of
task using the simulator.

****************************************************************************
*/
void init_antenna_simulator ( ) {

	init_antenna_model ( AZ_AXIS );
	init_antenna_model ( EL_AXIS );
	init_state_machines ( );
} /* end of init_antenna_simulator */


/*
****************************************************************************
void antenna_simulator ( )

	Task to do drive simulation.

****************************************************************************
*/
void antenna_simulator ( ) {

	/*---- Get Hardware I/O */
	/* Only uncomment when integrating new hardware i/o interface
		routines.
	*/

	double V1, V2;
	int az_v1, az_v2, el_v1, el_v2;

	get_rate_volts ( &az_v1, &el_v1 );

	V1 = ( az_v1 * 0.0048828125 - 10 );
	model_axis [ AZ_AXIS ].rate_volts = modpar [ AZ_AXIS ].sign *( V1 );

	V1 = ( el_v1 * 0.0048828125 - 10 );
	model_axis [ EL_AXIS ].rate_volts = modpar [ AZ_AXIS ].sign *( V1 );

	antenna_model ( type );

	put_encoder ( AZ_AXIS, model_axis [ AZ_AXIS ].enc_integer );
	put_encoder ( EL_AXIS, model_axis [ EL_AXIS ].enc_integer );

} /* end of antenna_simulator */



/*
****************************************************************************
void antenna_model ( void )

Routine to simulate plant. Digitised transfer function.

****************************************************************************
*/
static void antenna_model ( unsigned int axis ) {

	double msd, v_noise;
	double wind = 0;
	double msda = 0;
	//double wind_accel;

	/*---- Apply saturation ( this is SWEO side ) */
	if ( model_axis [ axis ].rate_volts > modpar [ axis ].max_volts )
		model_axis [ axis ].rate_volts = modpar [ axis ].max_volts;

	if ( model_axis [ axis ].rate_volts < -modpar [ axis ].max_volts )
		model_axis [ axis ].rate_volts = -modpar [ axis ].max_volts;

	model_axis [ axis ].motor_deg_per_sec = model_axis [ axis ].rate_volts *
									modpar [ axis ].rate_gain;

	/* Resonance Filter Model */
	msda = -modpar [ axis ].ra2 * model_axis [ axis ].res_ym_2 -
			modpar [ axis ].ra1 * model_axis [ axis ].res_ym_1 -
			modpar [ axis ].rb1 * model_axis [ axis ].motor_deg_per_sec +
			modpar [ axis ].rb2 * model_axis [ axis ].res_um_1;

	model_axis [ axis ].res_ym_2 = model_axis [ axis ].res_ym_1;
	model_axis [ axis ].res_ym_1 = msda;
	model_axis [ axis ].res_um_1 = -model_axis [ axis ].motor_deg_per_sec;

	/* Add noise and dc bias effects:-
		Determine some noise for the volts in the drive */
	v_noise = modpar [ axis ].dc_noise * rand ( ) / RAND_MAX ;

	msda = msda + ( ( v_noise + modpar [ axis ].dc_bias )
					* modpar [ axis ].rate_gain );

	/* Add wind effect:-
		Apply wind model - simple LP */
	wind = modpar [ axis ].wind_alpha * model_axis [ axis ].wind_prev +
				modpar [ axis ].wind_beta * model.wind;
	model_axis [ axis ].wind_prev = wind;

	msda = msda; //	+ wind_accel * sample_per;

	/* Rate-Position Integrator and Time-constant */
	msd = modpar [ axis ].b1 * msda +
			modpar [ axis ].a1 * model_axis [ axis ].int_ym_1 + 
			modpar [ axis ].a2 * model_axis [ axis ].int_ym_2;

	model_axis [ axis ].int_ym_2 = model_axis [ axis ].int_ym_1;
	model_axis [ axis ].int_ym_1 = msd;

	/* Motor speed in RPM */
	model_axis [ axis ].rpm = ( msd - model_axis [ axis ].int_ym_2 )/
									(6.0*sample_per);

	model_axis [ axis ].ant_deg_per_sec = 
		6.0*model_axis [ axis ].rpm / modpar [ axis].gear_ratio;

	model_axis [ axis ].msd_integer = (long)( ( ( msd / modpar [ axis ].gear_ratio ) ) 
					/ modpar [ axis ].induct_prec ) ;

	model_axis [ axis ].msd_pos = (double)( model_axis [ axis ].msd_integer *
						modpar [ axis ].induct_prec );

	/* Determine wrap */
	if ( axis == AZ_AXIS )
		{
		if ( model_axis [ axis ].msd_pos < 0 )
			{
			model_axis [ axis ].wrap = CCW_WRAP;
			model_axis [ axis ].enc_integer =
				(long)( ( ( msd/modpar [ axis ].gear_ratio) + 360 ) /
						modpar [ axis ].induct_prec);
			}

		if ( model_axis [ axis ].msd_pos >= 0 )
			{
			model_axis [ axis ].wrap = CW_WRAP;
			model_axis [ axis ].enc_integer = (long)( ( ( msd/modpar [ axis ].gear_ratio ) /
							modpar [ axis ].induct_prec) ) ;
			}
		}
	else
		model_axis [ axis ].enc_integer = (long)( ( ( msd/modpar [ axis ].gear_ratio ) /
					modpar [ axis ].induct_prec) ) ;

} /* end of antenna_model */



/*
****************************************************************************
init_state_machines ( )

****************************************************************************
*/
void init_state_machines ( ) {

	unsigned int i;

	model_axis [ AZ_AXIS ].msd_pos = INITIAL_AZ_DEG;
	model_axis [ EL_AXIS ].msd_pos = INITIAL_EL_DEG;

	/* Initialise the axis structures */
	for ( i = 0; i < 2; i++ ) {
		model_axis [ i ].PowerOn = 0;
		model_axis [ i ].rate_volts = 0;
		model_axis [ i ].reset = 0;
		model_axis [ i ].start = 0;
		model_axis [ i ].stop = 0;
		model_axis [ i ].fail = 0;
		model_axis [ i ].wind_prev = 0;
		model_axis [ i ].wrap = CW_WRAP;
		model_axis [ i ].motor_deg_per_sec = 0;

		model_axis [ i ].res_ym_2 = 0;		/* Resonance model regressors */
		model_axis [ i ].res_ym_1 = 0;
		model_axis [ i ].res_um_1 = 0;
		model_axis [ i ].int_ym_2 = 0;		/* Integrator regressors */
		model_axis [ i ].int_ym_1 = 0;

		if ( i == AZ_AXIS )
			model_axis [ i ].msd_pos = INITIAL_AZ_DEG;
		else
			model_axis [ i ].msd_pos = INITIAL_EL_DEG;

		model_axis [ i ].enc_integer = 0;
		model_axis [ i ].rate_dac = 2048;
		}

	/* Initialise the model parameters */
	modpar [ AZ_AXIS ].sign = AZ_SIGN;
	modpar [ AZ_AXIS ].wind_alpha = WIND_ALPHA;
	modpar [ AZ_AXIS ].wind_beta = WIND_BETA;
	modpar [ AZ_AXIS ].max_volts = AZ_MAX_VOLTS;
	modpar [ AZ_AXIS ].rate_gain = AZ_RATE_LOOP_GAIN;
	modpar [ AZ_AXIS ].dc_bias = AZ_DC_BIAS;

	/* Initialise the model parameters */
	modpar [ EL_AXIS ].sign = EL_SIGN;
	modpar [ EL_AXIS ].wind_alpha = WIND_ALPHA;
	modpar [ EL_AXIS ].wind_beta = WIND_BETA;
	modpar [ EL_AXIS ].max_volts = EL_MAX_VOLTS;
	modpar [ EL_AXIS ].rate_gain = EL_RATE_LOOP_GAIN;
	modpar [ EL_AXIS ].dc_bias = EL_DC_BIAS;

	model.wind = 0;
	model.external_plug = 1;
	model.system_ok = 1;
	model.stiction = 1;

}


/*
****************************************************************************
void init_antenna_model

Routine to initialise plant model.

Parameters:
	axis - unsigned int specifying which axis to initialise.

****************************************************************************
*/
void init_antenna_model ( unsigned int axis ) {

	static double a, b, g, w, initial_deg;

	modpar [ axis ].gear_ratio = 15000L;
	modpar [ axis ].induct_prec = 360.0/(pow(2,20)-1);	/* 19bits (1bit in degrees)*/
	/*	induct_prec = 0.00004291534424; */ /* 23bit */

	/* Model filter storage */
	if ( axis == AZ_AXIS )
		w = INITIAL_AZ_DEG;
	else
		w = INITIAL_EL_DEG;

	model_axis [ axis ].int_ym_1 = w * modpar [ axis ].gear_ratio;
	model_axis [ axis ].int_ym_2 = w * modpar [ axis ].gear_ratio;

	model_axis [ axis ].res_ym_2 = 0;		/* Resonance model regressors */
	model_axis [ axis ].res_ym_1 = 0;
	model_axis [ axis ].res_um_1 = 0;

	/* z transform approximation */
	modpar [ axis ].time_constant = TIME_CONSTANT;

	modpar [ axis ].b1 = sample_per * sample_per * modpar [ axis ].time_constant;
	modpar [ axis ].a1 = ( 2 - modpar [ axis ].time_constant * sample_per );
	modpar [ axis ].a2 = modpar [ axis ].time_constant * sample_per - 1;

	/* Resonance ZOH Equivalent */
	modpar [ axis ].model_damp = ANT_MODEL_DAMPING_FACTOR;
	modpar [ axis ].w0 = ANT_MODEL_RESONANCE_FREQ;

	a = exp ( - modpar [ axis ].model_damp * modpar [ axis ].w0 * sample_per );
	w = modpar [ axis ].w0 * sqrt ( 1 - modpar [ axis ].model_damp * modpar [ axis ].model_damp );
	b = cos ( w*sample_per );
	g = sin ( w*sample_per );
	modpar [ axis ].rb1 = -( 1 - a*( b + ( modpar [ axis ].model_damp * 
									modpar [ axis ].w0*g/w) ) );
	modpar [ axis ].rb2 = -(a*a + a* ( (modpar [ axis ].model_damp *
							modpar [ axis ].w0*g/w) - b ));
	modpar [ axis ].ra1 = -2*a*b;
	modpar [ axis ].ra2 = a*a;

	if ( axis == AZ_AXIS )
		initial_deg = INITIAL_AZ_DEG;
	else
		initial_deg = INITIAL_EL_DEG;

	model_axis [ axis ].msd_integer = (long)( ( ( initial_deg ) /
						modpar [ axis ].induct_prec) ) ;

	/* Encoder reads 0-360 */
	if ( axis == AZ_AXIS )
		if ( model_axis [ AZ_AXIS ].wrap == CCW_WRAP )/* by DJLB. Changed CCW_WRAP to SWITCH_OPEN */
			model_axis [ axis ].enc_integer = (long)( ( ( INITIAL_AZ_DEG + 360 ) /
					modpar [ axis ].induct_prec) ) ;
		else
			model_axis [ axis ].enc_integer = (long)( ( ( INITIAL_AZ_DEG ) /
					modpar [ axis ].induct_prec) ) ;
	else
		model_axis [ axis ].enc_integer = (long)( ( ( INITIAL_EL_DEG ) /
			modpar [ axis ].induct_prec) ) ;

} /* end of init_antenna_model */





/*----------------------------- End of antsim Module --------------------*/










