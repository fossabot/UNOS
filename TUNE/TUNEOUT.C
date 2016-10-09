/****************************************************************************
* POLYTRON VERSION CONTROL SOFTWARE _ PVCS (tm) Information
*
*      $Revision:   2.6  $
*      $Date:   06 Mar 1992 17:57:52  $
* 
****************************************************************************/
/****************************************************************************
* Geraldton ADSCS Project.
*****************************************************************************
* Copyright (c) 1991. The University of Newcastle Research Associates Ltd.
*****************************************************************************
*
* PROGRAMMER :-     Len J. Sciacca,
*					Servo Consultant and general sucker for punishment
*                   Industrial Electronics Division,
*                   TUNRA Ltd.,
*                   University of Newcastle, NSW 2308.
*
* FUNCTION REFERENCE :- SC???
*
****************************************************************************/
/****************************************************************************
* MODULE :- ATCU Dataport Tuning Data output        File : tuneout.c
*****************************************************************************
*
* DISCUSSION :  This module provides the routine to be called by the
*		Sequencer every control period to send data to the ATCU
*		data port.
*****************************************************************************
*/

#include <stdio.h>
#include <string.h>

#include <unos.h>

#include <posext.h>		/* return_ () routines */
#include <seq.h>
#include <seqext.h>     /* Import return_az/el_msd ( ) */
#include <antsim.h>
#include <tparser.h>   /* return_axis_tune_data_out() */
#include "taskname.h"

static char tx_mess [ 100 ];

/*
***************************************************************************

			spew_tuning_data

Description:

	Routine to output control data from the sequencer once every control
period to the serial port. Note this requires more mbx queues than
the dataport.

Author: L. J. Sciacca   1 May 1992

***************************************************************************
*/
void spew_tune_data ( ) {

	double az_msd, el_msd, az_err, el_err, az_control, el_control;
	double az_int_action, el_int_action, az_mit_ff_gain, el_mit_ff_gain;
	int spew_axis;

	az_msd = return_az_msd ( );
	az_err = return_az_err ( );
	az_control = return_az_control ( );
	az_int_action = return_az_int_action ( );
	az_mit_ff_gain = return_az_mit_ff_gain ( );

	el_msd = return_el_msd ( );
	el_err = return_el_err ( );
	el_control = return_el_control ( );
	el_int_action = return_el_int_action ( );
	el_mit_ff_gain = return_el_mit_ff_gain ( );

	spew_axis = return_axis_tune_data_out ( );

	tx_mess [ 0 ] = NULL;

	/* Form it up into a character string. */
	if ( spew_axis == AZ_AXIS )
		sprintf( tx_mess, "%8.3f %8.3f %8.3f %8.3f %8.3f\n",
			az_msd, az_err, az_control, az_int_action,
			az_mit_ff_gain );
	else
		sprintf( tx_mess, "%8.3f %8.3f %8.3f %8.3f %8.3f\n",
			el_msd, el_err, el_control, el_int_action,
			el_mit_ff_gain );

	//send_mess ( (unsigned char *)&tx_mess, strlen ( tx_mess ),  );

} /* end of spew_tuning_data */

