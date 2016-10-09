/*This file contains duplicates of routines used by NVRAM.C but contained
	in other ATCU files. Duplicated here to reduce the code needed to
	run NVRAMTST.
*/
/**************************************************************************/
#include <stdio.h>
#include <dos.h>
#include "seq.h"
#include "posext.h"
#include "..\posloop\poscon.h"

/* From SEQ.C :- */
void set_default_software_limits ( software_limit_struct * sw_l ) {

	sw_l->az_cw_limit = AZ_CW_LIMIT;
	sw_l->az_ccw_limit = AZ_CCW_LIMIT;
	sw_l->el_up_limit = 95.0; /* EL_UP_LIMIT; */
	sw_l->el_down_limit = -5.0; /* EL_DOWN_LIMIT; */

} /* end of set_default_software_limits */

void set_default_stow_positions ( stow_pos_struct * stow ) {

	stow->az = 0.0;
	stow->el = 90.0;

} /* end of set_default_software_limits */
/*
***************************************************************************/
/* From POSCON.C */
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
		}

} /* end of init_default_controller_parameters */
/*
***************************************************************************/
/* From UNOSASM.C */
char return_interrupt_status ( void )
{
	asm pushf;
	asm pop ax;

	return ( ( _AX & 0x0200 ) >> 9 );
}
/*
***************************************************************************/
