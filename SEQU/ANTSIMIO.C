/*
****************************************************************************

						ANTSIMIO.c

				Antenna Simulator I/O Routines

							by

						L.J. Sciacca

Date: 7-10-1991

Latest:	7-10-1991	LJS Creation



*****************************************************************************
*/

/*
Description

This module contains the simulation I/O routines. They have been written
to mimic the real I/O for the antenna drive and encoder interfaces.
Ideally to test the ATCU software, extra hardware should have been used
and loopbacks to the ATCU digital I/O ports used. Also a separate computer
with all the simulators installed. Perhaps this could be an enhancement
of the present system.

To isolate both the interface software and the antenna simulator, we have
mapped the simulated i/o ports at startup into a linear address space
that corresponds to the mapping used in the i/o hardware.

*****************************************************************************
*/

#include <stdlib.h>
#include <stdio.h>	/* Import exit() */
#include <math.h>

#include "..\sequ\antsimio.h"	/* Import definitions for I/O */
#include "antsimex.h"			/* Extern prototypes */
#include "hwio.h"		/* Import shared_port_addresses() */

static char * base_ptr;

#define AZ_AXIS 0
#define EL_AXIS 1

/*
***************************************************************************
init_antenna_io


****************************************************************************
*/
void init_antenna_io ( ) {

	base_ptr = (char *)shared_port_addresses ( );
	if ( base_ptr == NULL )
		{
		printf ( "Error allocating memory for simulator \n");
	    exit ( EXIT_FAILURE );
	    }

} /* end of init_antenna_io */


/*
****************************************************************************
put_encoder

	Routine to put the encoder reading into the simulator memory.
This routine should be called at the end of the simulator code
before returning.

	The error bits correspond to the Ferranti encoder interface spec:
	0
	1
	2	(Accuracy level 3 ) must be 1 for normal operation
	3

Parameters:
	axis - AZ_AXIS or EL_AXIS
	encoder - (unsigned long) 20 bits of encoder position
	error - (int) 4 error bits same specs as Ferranti encoder

	Note that there is also a strobe line.

Returns:
	NONE
****************************************************************************
*/
void put_encoder ( unsigned int axis, unsigned long enc ) {

	unsigned long * base;

	/* Check here for strobe line being high. If so do not place value in!! */

	if ( axis == AZ_AXIS ) {
		base = (unsigned long*)((char*)(base_ptr + AZ_ENC_IO));
		*base = ( enc & 0x0fffffL );
		}
	else {
		base = (unsigned long*)((char*)(base_ptr + EL_ENC_IO));
		*base = ( enc & 0x0fffffL );
		}

} /* end of put_encoder */



/*
*****************************************************************************
put_drive_signals

	Routine to place the motor drive signals into memory.

*****************************************************************************
*/
void put_drive_signals ( char controlV, char azbrake, char elbrake,
		char azflcw, char azflccw, char elflup, char elfldown,
		char emstop, char smoke, char door1, char door2, char cb36, char cb4,
		char cb5, char azrun, char elrun, char trip, char fail ) {

	char * base_tmp;
	base_tmp = base_ptr + DRIVE_SIGNALS_OUT; // Board 1

	*base_tmp = ( ( controlV ) |
				( ( azbrake & 0x01 ) << 1 ) |
				( ( elbrake & 0x01 ) << 2 ) |
				( ( azflcw   & 0x01 ) << 3 ) |
				( ( azflccw   & 0x01 ) << 4 ) |
				( ( elflup     & 0x01 ) << 5 ) |
				( ( elfldown & 0x01 ) << 6 ) |
				( ( emstop & 0x01 ) << 7 ) |
				( ( smoke & 0x01 ) << 8 ) |
				( ( door1 & 0x01 ) << 9 ) |
				( ( door2 & 0x01 ) << 10 ) |
				( ( cb36 & 0x01 ) << 11 ) |
				( ( cb4 & 0x01 ) << 12 ) |
				( ( cb5 & 0x01 ) << 13 ) |
				( ( azrun & 0x01 ) << 14 ) |
				( ( elrun & 0x01 ) << 15 ) |
				( ( trip &0x01 ) << 16 ) |
				( ( fail & 0x01 ) << 17 ) );

} /* end of put_drive_signals */



/*
*****************************************************************************
get_drive_signals

	Routine to return the drive signals comming from the ATCU. These
	include the Start, RUN lines that start/stop the drives. Also the
	reset line.

Parameters:
	az_stop
	el_stop
	az_enable
	el_enable
	start both drives
	reset both drives

*****************************************************************************
*/
void get_drive_signals ( char * stop,
						char * start,
						char * reset,
						char * poweron ) {

	char * base_tmp;
	base_tmp = base_ptr + DRIVE_SIGNALS_IN;

	*start = ((*base_tmp) & 0x01);
	*stop = ((*base_tmp) & 0x02) >> 1;
	*poweron = ((*base_tmp) & 0x04) >> 3;
	*reset = ((*base_tmp) &0x10 ) >> 4;

} /* end of get_drive_signals */

/*
*****************************************************************************
get_rate_volts

	Routine to return the demanded drive rate volts.

	Note that the current hardware writes the 12bit word to the
	higher 12 bits. This accounts for the right 4bit shift.

Parameters:
	az_volts1 - (int *) address of az volts 1
	az_volts2 - (int *)
	el_volts1 - (int *)
	el_volts2 - (int *)

Returns:
	None

*****************************************************************************
*/
void get_rate_volts ( int * az_v1, int * el_v1 ) {

	*az_v1 =  ( (
		( ( *(base_ptr + AZ_V1_INDEX + 1) & ( 0x00ff ) ) << 8 )+
			( *(base_ptr + AZ_V1_INDEX) & ( 0x00ff ) )
		 ) >> 4 ) & 0x0fff;

	*el_v1 = ( (
		( ( *(base_ptr + EL_V1_INDEX + 1) & ( 0x00ff ) ) << 8 )+
			( *(base_ptr + EL_V1_INDEX) & ( 0x00ff ) )
		 ) >> 4 ) & 0x0fff;

} /* end of get_rate_volts */

