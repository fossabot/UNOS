
/*******************************************************************************
*
*
* DISCUSSION : This module contains the routines to perform the error handling
*	facility. The current errors are stored in the "current" field of the
*	"error_handler_struct" (since these may be cleared before being read by
*	the Command Parser) and the errors to be read by the next poll from the
*	Command Parser is in the "scada" field. When a poll occurs, all of the
*	automatically cleared error conditions (as indicated by a "0" in the bit
*	position of the "mask" field) are cleared but the other errors will require
*	an "error_reset()" call before being cleared from "current" (& "scada").
*
******************************************************************************/

#include <conio.h>
#include <dos.h>

#include "err_td1.h"	/* Error list definitions
				and error_register_struct */

#include "err_nums.h"	/* Definitions of the error codes
				used throughout the ATCU */

/* Global variables. */

static error_register_struct	error_register[ 3 ];
			/* (2 CMU's + 1 index) Initialisation - NVRAM ?? */

static error_list_struct	mask;
		/* Remove "static" when testing with "err_test.c" */

/***************************************************************************/
/* Initialisation routine which returns "1" if OK, "0" otherwise.          */

int	error_handler_init( void )
{
int	i,
	max_error_words = 4;     /* Error code is written around 64 bits
					ie four words. 	*/

error_list_struct	zero;

	if (MAX_ERROR_WORDS != max_error_words)
		return 0;

	for ( i=0; i < MAX_ERROR_WORDS; i++ ) zero.errors[ i ] = 0;

	for ( i = 1; i <= 1; i++ )
	{
		error_register[ i ].current = zero;
		error_register[ i ].scada = zero;
	}
	mask = zero;

    return 1;
}

/***************************************************************************
* This routine sets the error register variables to indicate an error
* condition has just been reported. The error codes range is from (1 - 64).
*/

void error_set( int code )
{
error_list_struct	temp;
int	i,j;

	if ( (code < 1) || (code > 64) )
		error_set( INVALID_ERROR_CODE_ERR );
	else
	{
	    if ( code <= 16 )
	    {
	    	temp.errors[ 0 ] = 1;
		temp.errors[ 1 ] = 0;
		temp.errors[ 2 ] = 0;
		temp.errors[ 3 ] = 0;
		temp.errors[ 0 ] <<= (code - 1);
				/* (No shifting for error code = 1) */
	    }
	    else if ( code <= 32 )
	    {
	        temp.errors[ 0 ] = 0;
		temp.errors[ 1 ] = 1;
		temp.errors[ 2 ] = 0;
		temp.errors[ 3 ] = 0;
		temp.errors[ 1 ] <<= (code - 17);
	    }																						/* Update the error data read by Command Parsers and also. */
	    else if ( code <= 48 )
	    {
	        temp.errors[ 0 ] = 0;
		temp.errors[ 1 ] = 0;
		temp.errors[ 2 ] = 1;
		temp.errors[ 3 ] = 0;
		temp.errors[ 2 ] <<= (code - 33);
	    }																						/* Update the error data read by Command Parsers and also. */
	    else /* if ( code <= 64 ) */
	    {
	       	temp.errors[ 0 ] = 0;
		temp.errors[ 1 ] = 0;
		temp.errors[ 2 ] = 0;
		temp.errors[ 3 ] = 1;
		temp.errors[ 3 ] <<= (code - 49);
	    }																						/* Update the error data read by Command Parsers and also. */

	    /* Set the "current" ATCU errors, and SCADA-readable errors.*/

	    disable();
		for( i = 1; i <= 1; i++)
	    	for( j = 0; j < MAX_ERROR_WORDS; j++)
		{
		    error_register[i].scada.errors[j] |= temp.errors[j];
		    error_register[i].current.errors[j] |= temp.errors[j];
		}
	    enable();
	}
}

/***************************************************************************
* This routine allows resetting of errors that are not automatically cleared
* by the SCADA reading function.
*/

void error_reset( int code )
{
error_list_struct	temp;
int	i,
	j;

	if ( (code < 1) || (code > 64) )
		error_set( INVALID_ERROR_CODE_ERR );
	else
	{
	    if ( code <= 16 )
	    {
	    	temp.errors[ 0 ] = 1;
		temp.errors[ 1 ] = 0;
		temp.errors[ 2 ] = 0;
		temp.errors[ 3 ] = 0;
		temp.errors[ 0 ] <<= (code - 1);
				/* (No shifting for error code = 1) */
	    }
	    else if ( code <= 32 )
	    {
	    	temp.errors[ 0 ] = 0;
		temp.errors[ 1 ] = 1;
		temp.errors[ 2 ] = 0;
		temp.errors[ 3 ] = 0;
		temp.errors[ 1 ] <<= (code - 17);
	    }																						/* Update the error data read by Command Parsers and also. */
	    else if ( code <= 48 )
	    {	
	    	temp.errors[ 0 ] = 0;
		temp.errors[ 1 ] = 0;
		temp.errors[ 2 ] = 1;
		temp.errors[ 3 ] = 0;
		temp.errors[ 2 ] <<= (code - 33);
	    }																						/* Update the error data read by Command Parsers and also. */
	    else /* if ( code <= 64 ) */
	    {
	    	temp.errors[ 0 ] = 0;
		temp.errors[ 1 ] = 0;
		temp.errors[ 2 ] = 0;
		temp.errors[ 3 ] = 1;
		temp.errors[ 3 ] <<= (code - 49);
	    }																						/* Update the error data read by Command Parsers and also. */

	    /* Delete the error from the list of current ATCU errors but
	    do not remove it from the SCADA-readable errors. This ensures
	    that a short duration error is reported. */

	    disable();
		for( i = 1; i <= 1; i++)
	    	for( j = 0; j < MAX_ERROR_WORDS; j++)
		    error_register[i].current.errors[j] &= ~(temp.errors[j]);
	    enable();
	}
}

/***************************************************************************
* This routine is called by the Command Parsers to read the current list of
* errors. When it is activated, it clears all errors not masked from this
* clearing action.
*/

error_list_struct	get_errors( int which_CMU )
{
error_list_struct temp;
int	i;

	disable();

	temp = error_register[ which_CMU ].scada;

	/* Clear current non-sticky errors (the mask defines errors which 
		are to be individually reset). */

	for ( i=0; i < MAX_ERROR_WORDS; i++)
		error_register[ which_CMU ].current.errors[ i ]
							&= mask.errors[ i ];

	/* Set the SCADA-readable errors to what remains from the masking
	operation. That is, all others should be cleared by this action. */

	error_register[ which_CMU ].scada
					= error_register[ which_CMU ].current;

	enable();

	return( temp );
}

/***************************************************************************
* This routine sets the error register mask to make an error `sticky' ie a
* call to read_errors will not clear it. The routine should be called in
* the initialising phase of your code.
*/

void make_sticky_error( int code )
{

	if ( (code < 1) || (code > 64) )
		error_set( INVALID_ERROR_CODE_ERR );
	else
	{
	    if ( code <= 16 )
		mask.errors[ 0 ] |= ( 1 << (code - 1) );
				/* (No shifting for error code = 1) */
	    
	    else if ( code <= 32 )
		mask.errors[ 1 ] |= ( 1 << (code - 17) );
	    																						/* Update the error data read by Command Parsers and also. */
	    else if ( code <= 48 )
		mask.errors[ 2 ] |= ( 1 << (code - 33) );
	 																						/* Update the error data read by Command Parsers and also. */
	    else /* if ( code <= 64 ) */
		mask.errors[ 3 ] |= ( 1 << (code - 49) );																						/* Update the error data read by Command Parsers and also. */
	}
}

/***************************************************************************/
