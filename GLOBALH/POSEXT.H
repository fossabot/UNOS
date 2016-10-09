/*
****************************************************************************

							POSEXT.H

		Adaptive Digital Controller for Antenna Position Control System

								by

							L. Sciacca

			INCLUDE FILE FOR EXTERNAL PROTOTYPES FOR POSCON.C

Latest:	10-10-1991 LJS Improved comments on each structure element.

****************************************************************************
*/
#ifndef __POSEXT_H
#define __POSEXT_H

#include "seq.h"	/* I didn't want to do this but NVRAM system made me do it */

	/* Controller parameters structure. Holds the controller constants.
		and is usually read into and from NVRAM. For both axes.
		approximately 22 values per axis. The file poscon.h holds the
		default values that work with the simulator.
	*/
typedef struct {
		double prop_gain;		/* Controller Proportional gain (Motor deg)*/
		double integral_gain;	/* Integral Gain */
		double res_bw;			/* Resonance Filter cut-off frequency Hz */
		double diff_bw;			/* Feed-forward differentiator filter cut-off */
		double int_positive_limit;	/* Integrator positive limit */
		double int_negative_limit;	/* Integrator negative limit */
		int sign;					/* Motor drive sign. +V*sign -> rate */
		double mit_alpha;		/* MIT-Rule alpha gain */
		double mit_beta;		/* MIT-Rule beta gain */
		double mit_maxref;		/* MIT-Rule Maximum tracking reference */
		double mit_maxincr;		/* Maximum allowed adaptation increment */
		double mit_ff_high;		/* Feed-forward adaptation limit High */
		double mit_ff_low;		/* Feed-forward adaptation limit Low */
		double initial_mit_ff_gain;	/* Initial Feed-forward gain. Reset value */
		double parabolic_gain;		/* Second order feed-forward coefficient */
		double max_volts;		/* Full-speed volts on drive */
		double min_volts;		/* -Full speed volts (probably could lump with max_volts */
		double max_acceleration;	/* maximum acceleration of drive in deg/s/s */
		double gear_ratio;		/* Gear ratio N */
		double rate_gain;		/* Motor drive rate gain. deg/sec/V */
		double rate_limit_region;   /* region where rate limiting begins */
		double slew_region;		/* used by sequencer to determine slew region */

				} controller_parameter_struct;

		/* Controller variables structure */
typedef
		struct {
			double err;			/* measured - commanded */
			double cmd_dot;
			double pred;
			double control_unfilt;
			double dem_rate;
			double dem_rate_1;
			double cmd_prev;	/* previous reference command */
			double cmd_mit_prev;
			double rate_volts;
			double cmd_diff_1;
			double cmd_dotdot_1;
			double int_sum;			/* integrator summation */
			double control_limited;
			double control_rate_1;
			double control_1;
			int rate_dac;		/* DAC word */
			double cmd_diff;
			double cmd_dotdot;
			double int_action;				/* integrator action */
		} controller_struct;


		/* Anti-resonance filter structure. Holds regressor and coefficient
			values.
		*/
typedef
		struct {
			double ym_2;
			double ym_1;
			double um_1;
			double a1;
			double a2;
			double b1;
			double b2;
			} res_filter_struct;


            /* Command differentiator filter structure */
typedef
		struct {
			double alpha;
			double beta;
			} diff_filter_struct;


		/* MIT-Rule structure */
typedef
		struct {
			double norm;
			double alpha_err;
			double beta_err;
			double incr;
			double ff_gain;
			double cmd_prev;
			double sign_cmd_dot;
			} mit_adaptation_struct;

		/* This routine should be called by the sequencer
			when sitting in standby to allow smooth changeover.
		*/
extern void reset_pos_parameters ( unsigned int axis, double msd );

		/* Reset filter coefficients only in slewing mode to allow
			smooth changeover to position loop.
		*/
extern void reset_pos_filters ( unsigned int axis, double msd );

		/* Routine to initialise position controllers. This should be done
			at system startup as well as after changing controller
			NVRAM parameters.
		*/
extern int init_position_controllers ( void );

		/* This routine can be called in the nVRAM setup routine to
		load sensible values into NVRAM
		*/
extern void init_default_controller_parameters ( unsigned int axis,
					controller_parameter_struct * con_temp );

		/* Control Position loop. Returns the MOTOR demanded speed */
extern double pid_control_loop ( unsigned int axis,
				unsigned int adaptation_switch,
				double cmd,
				double msd );

extern double open_loop_controller(void);

		/* rate and resonance filter. Should be called in BOTH
			position and slewing modes. dem_rate is MOTOR speed
			in deg/sec. e.g. 1750rpm = 10500 deg/sec.
			Returns the voltage required to be placed on the drives.
		*/
extern double rate_limiter ( unsigned int axis, double dem_rate );

/* These routines were originally used to adjust controller
parameters on the fly. Only for benefit of developer and will
possibly be scrapped in the future.
*/
extern void set_az_prop_gain ( double value_in );
extern void set_az_integral_gain ( double value_in );
extern void set_az_int_positive_limit ( double value_in );
extern void set_az_int_negative_limit ( double value_in );
extern void set_az_mit_alpha ( double value_in );
extern void set_az_mit_beta ( double value_in );
extern void set_az_mit_ff_high ( double value_in );
extern void set_az_mit_ff_low ( double value_in );
extern void set_az_mit_maxincr ( double value_in );
extern void set_az_sign ( int sign );
extern void set_az_mit_maxref ( double value_in );
extern void set_az_rate_dac ( unsigned int rate );
extern void set_az_initial_ff_gain ( double value_in );

extern double return_az_err ( void );
extern double return_az_control ( void );
extern double return_az_int_action ( void );
extern double return_az_mit_ff_gain ( void );
extern double return_az_cmd_dot ( void );
extern int return_az_rate_dac ( void );

/*----- Some external Elevation prototypes ----*/
extern void set_el_prop_gain ( double value_in );
extern void set_el_integral_gain ( double value_in );
extern void set_el_int_positive_limit ( double value_in );
extern void set_el_int_negative_limit ( double value_in );
extern void set_el_mit_alpha ( double value_in );
extern void set_el_mit_beta ( double value_in );
extern void set_el_mit_ff_high ( double value_in );
extern void set_el_mit_ff_low ( double value_in );
extern void set_el_mit_maxincr ( double value_in );
extern void set_el_sign ( int sign );
extern void set_el_mit_maxref ( double value_in );
extern void set_el_rate_dac ( unsigned int rate );

extern double return_el_err ( void );
extern double return_el_control ( void );
extern double return_el_int_action ( void );
extern double return_el_mit_ff_gain ( void );
extern double return_el_cmd_dot ( void );
extern int return_el_rate_dac ( void );

/* Some common routines */
extern void set_diff_bw ( unsigned int axis, double diff_bw_new );
extern void set_res_bw ( unsigned int axis, double az_res_bw_new );

extern void set_slew_region ( double slew_region );
extern double get_slew_region ( void );

#endif

/*---------------------------- End of posext.h ---------------------------*/
