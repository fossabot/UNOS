/*****************************************************************************
* POLYTRON VERSION CONTROL SOFTWARE _ PVCS (tm) Header Information
*
* 	$Revision:   3.1.1.0  $
*	$Date:   06 Oct 1993 10:36:08  $
*
*****************************************************************************/
/**************************************************************************/
/*                                                                        */
/*                               MONITOR TASK MODULE                      */
/*                                                                        */
/*                                     by                                 */
/*                                                                        */
/*                                 L. J. Sciacca                          */
/*         Department of Electrical Engineering and Computer Science      */
/*                            University of Newcastle					  */
/*									Australia                             */
/*                                                                        */
/*                         ( Copyright 1990, 91 )                         */
/*                                                                        */
/*                                                                        */
/**************************************************************************/
#include <stdio.h>
#include <conio.h>
#include <dos.h>
#include <mem.h>
#include <string.h>

#include "general.h"
#include "unos.h"

#include "circbuf.h"

/*
HISTORY

	Began typing: 12-April-1991	L. Sciacca

	Latest: 13-April-1991 LJS Fix bug in put pointer resetting.
			20-April-1991 LJS General clean up of comments.

/*
					Circular Buffer Facility Documentation
					--------------------------------------
MODULE DESCRIPTION
------------------

	This is a circular buffer facility for UNOS. The following routines are 
provided to maintain and use the buffer facility:

.	circ_buff_struct * init_central_circ_table ( max_number_circ_buffers );
			Done at system initialisation

.	circ_buff_struct * create_circ_buffer ( circ_buf_num, size, type );
			Done at system startup or during task startup.

.	int put_circ_buffer ( circ_buf_num, void * data );
.	int get_circ_buffer ( circ_buf_num, void * data );
			Carried out at any time.

.	reset_circ_buffer ( int buf_num );
			Carried out at any time.

	No deallocation of buffers has been provided. This may easily be done 
using the ufree ( ) routine in the future.

	This module contains a static variable location holding the address of a 
Central Circular Buffer Table ( CCBT ). The CCBT is a list of pointers to 
circular buffers set up by the umalloc routine. The size of the CCBT should 
be setup at initialisation using the init_central_circ_buffer routine.

	This has been left out of the UNOS initialisation routine in case the 
user does not wish to use circular buffers. In fact this module does not 
have to access UNOS static variables to keep the flavour of other utilities 
such as the command decoder.

HOW TO USE THE CIRCULAR BUFFERS
-------------------------------

	Each circular Buffer must be created like system timers. Each time a 
buffer is to be setup one must call the create_circ_buffer routine. This 
allocates memory for the buffer, sets up pointers in the CCBT and 
initialises the put and get pointers in the circ_buffer structure. Each 
buffer has a unique identifcation number that is merely the index into the 
CCBT. To access data or insert data into the buffers, the buffer 
identification should be passed as an argument. It is recommended that one 
uses defined names that correspond to buffer numbers.


*/

/*---- Define a local copy of the circular buffer table address */

static circ_buf_struct** circ_buf_table;

/*
****************************************************************************
init_central_circ_table

	Routine to allocate memory for the Central Circular Buffer table.

Parameters:
	max_num_buffers - ( int ) Maximum number of buffers in the Table

Returns:
	address of buffer structure allocated. 
	If NULL then memory allocation error.

****************************************************************************
*/
circ_buf_struct * init_central_circ_table ( int max_num_buffers ) {

	circ_buf_table = ( void * )ucalloc ( max_num_buffers, sizeof ( void * ) );

	return ( circ_buf_table );

} /* end of init_central_circ_table */



/*
****************************************************************************
create_circ_buffer

	Routine to allocate memory and initialise each buffer. This routine must 
be called for each circular buffer created. Each buffer has a unique 
identification number that is really only an index into the CCBT.

Parameters:
	circ_buf_num - ( int ) Unique identification number of the buffer 
						(actually index into the table ).

	size - ( int ) size of the buffer in units of "type". i.e. if we want a 
			100 integer buffer then type = INT_TYPE, and size = 100.

	type - ( char ) identifer saying the type of the buffer to yield the 
			true buffer size in bytes.
			Allowable types:
				char	= CHAR_TYPE
				int		= INT_TYPE
				long	= LONG_TYPE
				float	= FLOAT_TYPE
				double	= DOUBLE_TYPE
				user defined = USER_TYPE

	type_size = used only when user defined type ( size in bytes of type )

Returns:
	address of buffer structure allocated. 
	If NULL then memory allocation error.

****************************************************************************
*/
void * create_circ_buffer ( int circ_buf_num, int size, char type,
							int type_size ) {

	/*---- First thing is allocate the circ_buffer */
	circ_buf_table [ circ_buf_num ] = 
			( circ_buf_struct * ) umalloc ( sizeof ( circ_buf_struct ) );

	/*---- Now allocate the buffer that will hold the data */
	switch ( type ) {
		case CHAR_TYPE:
			circ_buf_table [ circ_buf_num ]->buf = 
								( char * )ucalloc ( size, sizeof ( char ) );
			circ_buf_table [ circ_buf_num ]->bytes_type = sizeof ( char );
			break;

		case INT_TYPE:
			circ_buf_table [ circ_buf_num ]->buf = 
								( int * )ucalloc ( size, sizeof ( int ) );
			circ_buf_table [ circ_buf_num ]->bytes_type = sizeof ( int );
			break;

		case LONG_TYPE:
			circ_buf_table [ circ_buf_num ]->buf = 
								( long * )ucalloc ( size, sizeof ( long ) );
			circ_buf_table [ circ_buf_num ]->bytes_type = sizeof ( long );
			break;

		case FLOAT_TYPE:
			circ_buf_table [ circ_buf_num ]->buf = 
								( float * )ucalloc ( size, sizeof ( float ) );
			circ_buf_table [ circ_buf_num ]->bytes_type = sizeof ( float );
			break;

		case DOUBLE_TYPE:
			circ_buf_table [ circ_buf_num ]->buf = 
								( double * )ucalloc ( size, sizeof ( double ) );
			circ_buf_table [ circ_buf_num ]->bytes_type = sizeof ( double );
			break;

		case USER_TYPE:
			circ_buf_table [ circ_buf_num ]->buf = 
						( void * )ucalloc ( size, type_size );
			circ_buf_table [ circ_buf_num ]->bytes_type = type_size;
			break;

		} /* switch */

	/*--- Now initialise the put and get pointers */
	circ_buf_table [ circ_buf_num ]->put = 0;
	circ_buf_table [ circ_buf_num ]->get = 0;
	circ_buf_table [ circ_buf_num ]->buf_size = size;
	circ_buf_table [ circ_buf_num ]->buf_free = size;

	return ( circ_buf_table [ circ_buf_num ]->buf );

} /* end of create_circ_buffer */


/*
****************************************************************************
put_circ_buffer


	Routine to place data into the circular buffer and maintain the put 
pointer in the circ_buffer structure. As this facility was written for a 
particular application need at the time that did not make use of widespread 
semaphore protection, the philosophy of disabling interrupts was maintained 
here.

	A note of warning. For large user defined data types, a memory copy will 
be performed while interrupts disabled. This may cause problems with 
critical sections of code such as device drivers working under interrupt, or 
where precise timing of timers is required. If this becomes a problem it is 
possible to selectively disable interrupts to enable say serial interrupts 
to remain being serviced etc.

	For this version we use the convention used by UNOS of storing the 
interrupt status of the system before disabling interrupts and restoring 
after the critical section.


Parameters:
	circ_buf_num - number of buffer to place data into.

	void * data - pointer to data. As we have permitted generic buffer 
					types, we must pass the address and get the appropriate 
					number of bytes that way.

Returns:
	buffer status - if 0 then an overrun error has occurred. The user may 
		modify this if necessary depending on what action they may want to take 
		if the put pointer has reached the get pointer.

		1 - No error occurred.
 
****************************************************************************
*/
int put_circ_buffer ( int bufn, void * data ) {

	int error = 1;
	char int_status;

	int_status = return_interrupt_status ( );

	disable ( );

	if ( circ_buf_table [ bufn ]->buf_free > 0 ) {

		memcpy ( (char *)circ_buf_table [ bufn ]->buf 
			+ (int)(circ_buf_table [ bufn ]->put * circ_buf_table [ bufn ]->bytes_type),
			data, circ_buf_table [ bufn ]->bytes_type );

		if ( circ_buf_table [ bufn ]->put == ( circ_buf_table [ bufn ]->buf_size - 1 ))
			circ_buf_table [ bufn ]->put = 0;
		else
			circ_buf_table [ bufn ]->put++;
	
		--circ_buf_table [ bufn ]->buf_free;

		} /* if */
	else
		error = 0;

	if ( int_status )
		enable ( );

	return ( error );

} /* end of put_circ_buffer */


/*
****************************************************************************
get_circ_buffer

	Routine to get data from the circular buffer specified. Once again, 
interrupt disabling protection was placed around the buffer manipulation 
section. Semaphores may be added by appropriately modifying the 
circ_buff_structure.

	This routine will return a 1 if there is more data in the buffer, 
otherwise it returns a zero.

	It is up to the user to provide protection around the get routine in 
this version.

Parameters:
	circ_buf_num - number of buffer to get data out of.

	void * data - pointer to data. As we have permitted generic buffer 
					types, we must pass the address and get the appropriate 
					number of bytes that way.

Returns:
	buffer status - 0 = no data left in buffer
 					> 0 = data available

****************************************************************************
*/
int get_circ_buffer ( int bufn, void * data ) {

	int get_tmp;	/* get pointer may change when interrupts reenabled */
	char int_status;

	int_status = return_interrupt_status ( );

	disable ( );

	if ( circ_buf_table [ bufn ]->buf_free < 
					circ_buf_table [ bufn ]->buf_size ) {

		get_tmp = 1;

		memcpy ( data,
			(char *)circ_buf_table [ bufn ]->buf 
			+ (int)(circ_buf_table [ bufn ]->get * circ_buf_table [ bufn ]->bytes_type),
			circ_buf_table [ bufn ]->bytes_type );

		if ( circ_buf_table [ bufn ]->get == ( circ_buf_table [ bufn ]->buf_size-1 ) )
			circ_buf_table [ bufn ]->get = 0;
		else
			circ_buf_table [ bufn ]->get++;

		circ_buf_table [ bufn ]->buf_free++;
		}
	else
		get_tmp = 0;

	if ( int_status )
		enable ( );

	return ( get_tmp );

} /* end of get_circ_buffer */


/*
****************************************************************************
reset_circ_buffer

	Routine to reset the pointers into the circular buffers.
	So, get = 0; put = 0; buf_free=buf_size;

	There are no interrupt disables around this routine so a user may reset 
several buffers at once without problems by putting their own protection 
mechanism.

Parameters:
	bufn - buffer to reset.

****************************************************************************
*/
void reset_circ_buffer ( int buf_num ) {

/*	char int_status;

	int_status = return_interrupt_status ( );

	disable ( );
*/
	circ_buf_table [ buf_num ]->put = 0;
	circ_buf_table [ buf_num ]->get = 0;
	circ_buf_table [ buf_num ]->buf_free = 
								circ_buf_table [ buf_num ]->buf_size;

/*	if ( int_status )
		enable ( );
*/

} /* end of reset_circ_buffer */

/*-------------------------- End of Circular Buffer Module ----------------*/


