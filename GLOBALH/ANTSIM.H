/*
****************************************************************************
					Antenna Simulation Facility

								by


							L. Sciacca

	Task to be run in UNOS to simulate motor drives and Antenna Structure.

Date:	26 October 1990


****************************************************************************
*/

#ifndef __ANTSIM_H
#define __ANTSIM_H

typedef
	struct {
		double rpm;
		double ant_deg_per_sec;		/* load side deg/sec */
		double motor_deg_per_sec;	/* motor side deg/sec */
		double rate_volts;
		double msd_rate;
		char wrap;
		char PowerOn;
		char reset;
		char start;
		char stop;
		char Run;
		char enable;
		char fail;				/* hardware failure contact */
		char over_speed;
		char cw_prelimit;
		char ccw_prelimit;
		char up_prelimit;
		char down_prelimit;
		double wind_prev;		/* wind filter regressor */
		double res_ym_2;		/* resonance regressors */
		double res_ym_1;
		double res_um_1;
		double int_ym_1;		/* Antenna model integrator regressors */
		double int_ym_2;
		int rate_dac;			/* DAC integer word */
		long enc_integer;		/* Integer encoder word */
		long msd_integer;
		double msd_pos;
		} model_struct;

typedef struct {
	char begin_trans;
	char az_signals;
	char el_signals;
	char az_enc_raw [ 3 ];
	char el_enc_raw [ 3 ];
	char system_signals;
	} sim_to_atcu_struct;

typedef struct {
	char begin_trans;
	char signals;
	char az_rate_raw [ 3 ];
	char el_rate_raw [ 3 ];
	} from_atcu_struct;

typedef struct {
	double wind;
	char external_plug;
	char system_ok;
	char fail;
	char trip;
	char stiction;
	char FlCW;
	char FlCCW;
	char FlUp;
	char FlDown;
	} model_data_struct;

	/* Model Parameters structure. This sructure contains all of the
	model parameters that are normally FIXED during a simulation run.
	*/
typedef
	struct {
		double dc_noise;		/* Variance of noise on input */
		double wind_alpha;		/* Wind filter coefficients */
		double wind_beta;
		int sign;				/* Motor drive sign */
		double max_volts;		/* Max allowable drive volts (FS) */
		double rate_gain;		/* rate gain in V/deg/sec */
		double ra1;				/* resonance filter coeff */
		double ra2;
		double rb1;
		double rb2;
		double dc_bias;			/* steady DC bias in drives */
		double b1;				/* LP filter coeff to model antenna */
		double a1;
		double a2;
		double gear_ratio;		/* main gear ratio */
		long msd_integer;		/* Antenna position as a word */
		double induct_prec;
		double time_constant;	/* time constant to model antenna */
		double model_damp;			/* damping of resonance */
		double w0;
		} model_parameter_struct;


/*---- Simulate DC bias in the drives and noise */
#define AZ_DC_BIAS  0.05  		/* Volts */
#define EL_DC_BIAS	0.02		/* Volts */
#define AZ_DC_NOISE	0.005		/* Volts */
#define EL_DC_NOISE	0.005		/* Volts */

#define AZ_MAX_VOLTS	10		/* Full Speed setting */
#define EL_MAX_VOLTS	10

#define AZ_RATE_LOOP_GAIN      		1500.0 	/* 10V = 0.32deg/sec */
#define EL_RATE_LOOP_GAIN			1500.0 

#define ANT_MODEL_RESONANCE_FREQ	22   	/* rad/sec, = 3.5 Hz  */
#define ANT_MODEL_DAMPING_FACTOR	0.05  

#define AZ_SIGN		1  
#define EL_SIGN		1  

#define CW_WRAP		0
#define CCW_WRAP	1

#define SWITCH_OPEN		0  /*Real-world switch is closed if cable */
#define SWITCH_CLOSED	1  /*   is CW-wrapped. */

#define AZ_CW_FINALLIMIT 	280.0
#define AZ_CCW_FINALLIMIT -280.0

#define EL_UP_FINALLIMIT 	97.0
#define EL_DOWN_FINALLIMIT 0.0

#define false 	0
#define true 	1
#define pi 		3.141592654
#define twopi 	6.283185307

#define AZ_AXIS 		0
#define EL_AXIS 		1
#define SPEW_ORBIT 		2

#define chan1 0
#define chan2 1

#define POWER_ON 	1
#define POWER_OFF 	0

/* Wind Filter Model. Only a first order model */
#define WIND_ALPHA	0.9
#define WIND_BETA	0.1

/* Antenna inertia time-constant */
#define TIME_CONSTANT		5

/* Antenna stiction constant */
#define STICTION_CONSTANT   30

/* Drive States */
#define _WAITING_FOR_RESET	0
#define _RESET				1
#define _START				2
#define _RUNNABLE			3
#define _ENABLE_DRIVES		4
#define _DRIVES_ON			5
#define _MODEL_OFF			6
#define _MODEL_POWER_OFF	7

/* Counts */
#define reset_count_value	1
#define start_count_value	2

/* Model Definitions */
#define model_az_sign		1 
#define model_el_sign		1 

#define sample_per			0.066

/* Bit definitions for word to atcu */
#define b_ready			0x03
#define b_on			0x0c
#define b_cw_prelimit	0x10
#define b_ccw_prelimit	0x20

#define b_up_prelimit	0x10
#define b_down_prelimit	0x20
#define b_stowpin_in	0x40

#endif



