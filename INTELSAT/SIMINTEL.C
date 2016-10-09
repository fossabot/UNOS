/*
********************************************************************************

								SIMINTEL.c
Module: Intelsat Simulator

Author: L. J. Sciacca
		Australian Systems Research (NSW) Pty Limited

********************************************************************************
*/
/*
Module Description

	Routine to simulate tracking an intelsat satellite based on knowledeg of
	antenna beam pattern and satellite position.

	Based on code developed by Rick Middleton and Adrian Bastiani for the
	TS2000.

*/

#include <stdlib.h>
#include <math.h>
#include <dos.h>		// Import enable()/disable()
#include "cmclock.h"	// import TimeRecord and ReadClock()
#include "intelsat.h"	// Import fdfDurationInDaysFrom ()

#include "nvramext.h"	// Import get_satellite_num

#define ANTENNA_BEAMWIDTH_SIM	0.2 /*0.0035 */
#define DEG_TO_RAD	0.017453292
#define NOISE_LEVEL 0.1

static double antenna_power = 0;
static double sat_az = 0;
static double sat_el = 0;
static TimeRecord test_epoch;
static 	int sat_num;

/*
********************************************************************************
init_simintel ()

Initialise Intelsat simulator.
Called from sequencer at start up.

********************************************************************************
*/
void init_simintel ( ) {

	// Get the epoch for that satellite
	get_current_satellite_num ( &sat_num );
	get_intelsat_epoch ( sat_num, &test_epoch );

} // end of init_simintel

void simintel ( double dish_az, double dish_el ) {

	double time_from_epoch;
	TimeRecord current_time;
	double sinc_multiplier, x;
	double off_bore_sight_angle, antenna_gain;
	double az_off_bore_sight_angle;
	double aa,ba,ca,da;

	ReadClock ( &current_time );

	time_from_epoch = fdfDurationInDaysFrom ( &test_epoch, &current_time );

	//========== Run Intelsat equations
	GetIntelsatAzEl ( time_from_epoch, &sat_az, &sat_el );

	//========== Get antenna pattern
	sinc_multiplier = 2.8/ANTENNA_BEAMWIDTH_SIM;

	az_off_bore_sight_angle = (sat_az - dish_az);

	// Off Bore-sight Angle
	aa =  ( sat_el - dish_el )*( sat_el - dish_el );
	ba =   cos ( sat_el*DEG_TO_RAD ) * cos ( dish_el*DEG_TO_RAD );
	ca =  ( az_off_bore_sight_angle * az_off_bore_sight_angle );
	da = aa+ba*ca;
	//off_bore_sight_angle = sqrt ( ( sat_el - dish_el )*( sat_el - dish_el ) +
	  //					cos ( sat_el*DEG_TO_RAD ) * cos ( dish_el*DEG_TO_RAD ) *
		//				( az_off_bore_sight_angle * az_off_bore_sight_angle ) );
	off_bore_sight_angle = sqrt(da);

	x = off_bore_sight_angle * sinc_multiplier;

	// Now find SINC function value
	if ( off_bore_sight_angle < 0.01 )
		antenna_gain = ( 1 - (x*x)/6.0 + (x*x*x*x)/120);
	else
		antenna_gain = sin (x) /x;

	// Add noise (-10dB) and convert power to dB
	antenna_power = antenna_gain * antenna_gain + 0.1;

	antenna_power = 10.0*log10(antenna_power) + NOISE_LEVEL*2.0*
			(((double)rand()/(double)RAND_MAX)-0.5);

	//========== Output beacon level to Beacon receiver code

} // end of simintel

/*
********************************************************************************
get_intelsat_sim_bcn

Routine to return the beacon power.

********************************************************************************
*/
double get_intelsat_sim_bcn ( void ) {

	double temp;

	disable ();
	temp = antenna_power;
	enable ();

	return temp;

} // end of get_intelsat_sim_bcn ()

/*
********************************************************************************
get_intelsat_sim_az

Routine to return the current satellite position (Intelsat model)

********************************************************************************
*/
double get_intelsat_sim_az ( void ) {

	double temp;

	disable ();
	temp = sat_az;
	enable ();

	return temp;

} // end of get_intelsat_sim_az ()

/*
********************************************************************************
get_intelsat_sim_az

Routine to return the current satellite position (Intelsat model)

********************************************************************************
*/
double get_intelsat_sim_el ( void ) {

	double temp;

	disable ();
	temp = sat_el;
	enable ();

	return temp;

} // end of get_intelsat_sim_el ()


