
/****************************************************************************
* MODULE :- UNOS                  File : COMDEC.C
****************************************************************************/
/**************************************************************************/
/*                                                                        */
/*                                                                        */
/*                               DECODER MODULE                           */
/*                                                                        */
/*                                     by                                 */
/*                                                                        */
/*                                 Robert Betz                            */
/*         Department of Electrical Engineering and Computer Science      */
/*                            University of Newcastle					  */
/*									Australia                             */
/*                                                                        */
/*																		  */
/*                         ( Copyright 1988, 89, 90 )                     */
/*                                                                        */
/*                                                                        */
/**************************************************************************/



/*

HISTORY

22/1/88

Began typing in the first version of the decoder module for use on the PCIV
project.

6/2/90

Started to modify the original version of this module which was used in the 
pipline project by converting it to 'C' to be used in version 1.5 of UNOS.
The modifications involve not just the conversion but also changing the 
decoder so it will decode messages coming from a UNOS mail box. Since data 
arriving at a task can come from a variety of sources the address of the 
sending task has to be used to determine which set of character command 
tables will be used to carry out the decoding task. In addition a message 
can be multi-character in nature therefore the routine has been modified so 
that a whole message is decoded on one call to the decoder. This involves 
introducing some new data structures.

*/


/*
MODULE DESCRIPTION

For a detailed description of how this module works and how to us it please 
refer to the UNOS documentation or to the UNOS.TXT file which is distributed 
with UNOS.
*/
#include <dos.h>
#include "general.h"
#include "unos.h"

#include "comdec.h"

/************************************************************************/
/*																		*/
/*							FUNCTION PROTOTYPES							*/
/*																		*/
/************************************************************************/


static void decode_binary ( dec_input_struc *input_ptr );

static void decode_ascii ( dec_input_struc *input_ptr );

static void char_decode ( dec_input_struc *input_ptr );





/*--------------------------------------------------------------------------*/





/*
==========================================================================
|
| decode_binary
|
| This routine allows binary data to be decoded.  Because the input byte can
| take on an arbitrary value the table structure is more complicated than the
| ascii data case.  This complication arises because of the support for input
| ranges. The high bit of the byte can no longer be used to signify the
| beginning of a range because binary data is now allowed.  A third table is
| used to overcome this problem.  Each bit in the bit table corresponds to a
| valid input in the valid data tbale.  If the bit is 0 the the corresponding
| valid data is not part of a range.  If a bit is 1 and the next bit a 0 then
| the currently pointed to valid data and the next one are the limiting values
| of a range.  If a bit is 1 and the next bit 1 then the end of the valid data
| table has been reached and the vector table index is pointing to the default
| handling routine ( which is often an error handling routine ).
| 
| Not withstanding the added complication of the bit table, operation of the
| routine is essentially the same as the decode_ascii routine.  If the data
| byte to be decoded lies within a range or equals one of the commands of the
| valid_data_table then a call is made to the routine whose address is contained
| in the currently indexed location of the vector_table.
| 
| NB. bit patterns start from the lsb of each byte in the bit table.
| 
| 
| Parameters :- pointer to the input data structure.
|
| Entry via :-    
|
===========================================================================
*/

static void decode_binary ( dec_input_struc *input_ptr ) {

    void ( *context_handler ) ( dec_input_struc *input_ptr );
	unsigned char *valid_data_table_ptr, *bit_table_ptr;
	void ( **vector_table_ptr ) ( dec_input_struc *input_ptr );
    unsigned char valid_data_index = 0,
    			  vector_table_index = 0,
    			  bit_table_index = 0;
    unsigned char bit_test_pattern;


	valid_data_table_ptr = input_ptr->valid_data_table_ptr;
	vector_table_ptr = input_ptr->vector_table_ptr;
	bit_table_ptr = input_ptr->bit_table_ptr;

	while ( 1 ) {
        /* firstly calculate the byte of interest in the bit table and then
        determine the test bit pattern for the required bit in the bit table
        */
        bit_table_index = valid_data_index / 8;
        bit_test_pattern = 0x01 << ( valid_data_index % 8 );

        /* now test the contents of the bit table */
		if ( bit_table_ptr [ bit_table_index ] & bit_test_pattern ) {
			/* possibility of a range or end of table.	Test the next bit
			to find if it is 1 or 0
			*/
			bit_table_index = ( valid_data_index + 1 ) / 8;
			bit_test_pattern = 0x01 << ( ( valid_data_index + 1 ) % 8 );

			if ( bit_table_ptr [ bit_table_index ] & bit_test_pattern ) {
				/* have hit the end of the table - the vector table
				index should be pointing to the default handler
				*/
				context_handler = vector_table_ptr [ vector_table_index ];
				( *context_handler ) ( input_ptr );
				return;
			} /* if */
			else {
				/* have a valid command range - check whether data in
				the range */

				if ( ( input_ptr->data_byte >= valid_data_table_ptr [
											   valid_data_index ] ) &&
						( input_ptr->data_byte <= valid_data_table_ptr [
												 valid_data_index + 1 ] ) ) {
					/* data within range */
					context_handler = vector_table_ptr [ vector_table_index ];
					( *context_handler ) ( input_ptr );
					return;
				} /* if */
				else {
					/* data not within range so update the table
					indexes */
					valid_data_index += 2;
					vector_table_index++;
				} /* else */
			} /* else */
		} /* if */
		else {
			/* bit in bit_table a zero so simply compare data with the
			currently pointed to location in the valid_data_table
			*/
			if ( input_ptr->data_byte ==
							valid_data_table_ptr [ valid_data_index ] ){
				context_handler = vector_table_ptr [ vector_table_index ];
				( *context_handler ) ( input_ptr );
				return;
			} /* if */
			else {
				/* data not in valid_data_table so increment the table
				indexes
				*/
				valid_data_index++;
				vector_table_index++;
			} /* else */
		} /* else */
	} /* while */
} /* end of decode_binary */
                




/*--------------------------------------------------------------------------*/





/*
=============================================================================
|
| decode_ascii
|
| This routine is entered if the data to be decoded is ascii data.	This routine
| uses two tables to do the decoding - a valid data table and a vector table.
| The valid data table contains all the valid values for the input.in_data byte
| passed in.  Corresponding to these valid values there is a pointer to a
| procedure which carries out some action for the particular input.
|
| In order to allow the tables to be reduced in size this routine allows valid
| command ranges, that is a data byte is valid if it lies between two values in
| the valid data table.  The start of a range is indicated by the high bit
| being set in the valid data byte.  The next byte in the table is then
| considered to be the end of the range.  Only one pointer in the vector table
| is associated with the range.  Note that the first byte in the range is assumed
| to have a lower numerical value than the second byte.
|
| The valid command table is terminated with 0xffh.
|
|
| Parameters :- pointer to the input data structure
|
| Entry via :-
|
=============================================================================
*/

static void decode_ascii ( dec_input_struc *input_ptr ) {

	void ( *command_handler ) ( dec_input_struc * );
	unsigned char *valid_data_table_ptr;
	void ( **vector_table_ptr ) ( dec_input_struc * );
	unsigned char valid_data_index = 0, vector_table_index = 0;

	valid_data_table_ptr = input_ptr->valid_data_table_ptr;
	vector_table_ptr = input_ptr->vector_table_ptr;

	while ( valid_data_table_ptr [ valid_data_index ] != 0xff ) {
		if ( valid_data_table_ptr [ valid_data_index ] & 0x80 ) {
			/* high bit is set so must be the beginning of a valid data
			range
			*/
			if ( ( input_ptr->data_byte >= ( valid_data_table_ptr [
				  valid_data_index ] & 0x7f ) ) &&
				  ( input_ptr->data_byte <=
				  valid_data_table_ptr [ valid_data_index + 1 ] ) ) {
				/* input belongs to the valid command range so vector
				to the handling routine
				*/
				command_handler = vector_table_ptr [ vector_table_index ];
				( *command_handler ) ( input_ptr );
				return;
			}  /* if */
			else {
				/* not in valid command range so increment the table
				indexes appropriately
				*/
				valid_data_index += 2;
				vector_table_index++;
			} /* else */
		} /* if */
		else {
			/* does not belong to a command range so process normally */
			if ( input_ptr->data_byte ==
						 valid_data_table_ptr [ valid_data_index ] ) {
				command_handler = vector_table_ptr [ vector_table_index ];
				( *command_handler ) ( input_ptr );
				return;
			} /* if */
			else {
				/* not equal to the currently indexed valid command so
				increment the table pointers
				*/
				valid_data_index++;
				vector_table_index++;
			} /* else */
		} /* else */
	} /* while */

    /* only enter this section if the end of the table has been reached - have
	to vector to the currently pointed to procedure
	*/
	command_handler = vector_table_ptr [ vector_table_index ];
	( *command_handler ) ( input_ptr );
} /* end of decode_ascii */





/*---------------------------------------------------------------------------*/
            
    



/*
=========================================================================
|
| char_decode
|
| This function is the entry point to the character decoding section of the
| decoder. It checks the mode that the decoder must work in and then calls
| the correct function. The modes which the decoder can operate in are the
| ascii mode and the binary mode. How this routine fits into the overall
| structure of the decoding system is explained in the UNOS documentation.
|
| Parameters : - pointer to the dec_input_struc
|
| Entry via : -
|
=========================================================================
*/

static void char_decode ( dec_input_struc *input_ptr ) {

    /* firstly determine the mode - 0=> ascii, 1=> binary and then branch to
	the appropriate handling routine.
	*/
	switch ( input_ptr->char_dec_mode ) {
		case 0 :
			decode_ascii ( input_ptr );
			break;
		case 1 :
			decode_binary ( input_ptr );
			break;
	} /* switch */
} /* end of char_decode */





/*-----------------------------------------------------------------------*/





/*
==========================================================================
|
| addr_decode
|
| The function of this routine is to carry out the decoding of the address 
| structure. The function of the address structure is to determine whether
| the task which has sent a message is a valid task to talk to the receiving
| task. If the sending task number appears in the address table then the
| associated address vector table contains the address of the 
| dec_input_struc which will be used to decode the message whose pointer
| is contained in the address structure. If the sending task number is not
| in the address table then this routine will eventually reach the
| terminating address and then return with the address of the terminating
| dec_input_struc. It is the responsibility of the handling routines pointed
| to by this structure to carry out the required error handling for this
| condition.
|
| The structure returns a pointer to a dec_input_struc.
|
| Parameters :- pointer to an addr_struc.
|
| Entry via :- decode_mbx_mess or directly from tasks
|
==========================================================================
*/

dec_input_struc *addr_decode ( addr_struc *addr_struc_ptr ) {

	unsigned int addr_tab_index;

	/* search through the address table until the task address is found or
	the terminating address is located in the table
	*/

	for ( addr_tab_index = 0; addr_struc_ptr->valid_addr_tab_ptr [
							addr_tab_index ] != NULL; addr_tab_index++ )
		if ( addr_struc_ptr->valid_addr_tab_ptr [ addr_tab_index ] ==
										addr_struc_ptr->rtn_addr )
			return ( addr_struc_ptr->addr_vector_ptr [ addr_tab_index ] );

	/* if control reaches here then the end of the table has been reached */
	return ( addr_struc_ptr->addr_vector_ptr [ addr_tab_index ] );
} /* end of addr_decode */





/*-----------------------------------------------------------------------*/





/*
============================================================================
|
| decode_mbx_mess
|
| This function decodes a message from a mail box. The routine assumes that
| the mail box already contains a message. If it does not then the calling
| task will be blocked on the mail boxes semaphore for the timeout period
| or until a message arrives in the mail box. When a message arrives it is
| then passed to the message decoder which calls the character decoder to
| decode the message character by character. Because at the lowest level
| the decoding is carried out on a character basis messages can actually
| be spread across several mail box envelopes without any problems. However
| is it the responsiblility of the handling routines to account for this.
| For example the call to this routine from the task requiring the decoding
| would have to be placed in a loop terminated by a flag. When the required
| data has been decoded across multiple messages then one of the handling
| routines would have to clear the flag.
|
| The return value from this routine obeys similar rules to the return
| values from the rcv_mess routine in the mail box interface system. This
| is obviously related to the fact that this routine also uses time limits 
| in the recv_mess routine. A return value of 0xffff indicates that there
| has been a timeout without decoding a message. If a timer is not
| available for the timeout then the  return value is 0xfffe. If everything
| is ok then the routine returns the address of the task whose message
| was decoded.
|
| Parameters : - pointer to the addr_struc 
|			   - time limit on wait for a message - 0=> infinite wait
|
| Entry via : - multiple places
|
============================================================================
*/

char * decode_mbx_mess ( addr_struc *addr_struc_ptr, unsigned long
								time_limit ) {

	unsigned int mess_index, left_to_decode;
	char * return_code;
	dec_input_struc *input_ptr;


	/* the first thing to do is to get the message from the mail box */
	return_code = rcv_mess ( addr_struc_ptr->mess_ptr,
								&addr_struc_ptr->mess_lgth, time_limit );
	/* now check to see if a message has been received */

	if ( ( return_code == NULL ) || ( return_code == MK_FP ( 0xffff, 0x000f ) ) )
		return return_code;

	/* return code contains the address of the task from which the message
	has been sent so put it into the addr_struc
	*/
	addr_struc_ptr->rtn_addr = return_code;

	/* now decode the return address via the addr_struc. This is done in the
	addr_decode function. If the address decoding is successful then the
	value returned is the address of the dec_input_struc for the current
	sending task. If the address is not a legal one the routine still returns
	with a pointer to a dec_input_struc. However it is a special one which
	will handle the error and effectively flush the message from the illegal
	sending task.
	*/
	input_ptr = addr_decode ( addr_struc_ptr );

	/* now enter the character decode loop an carry out the decoding of the
	message received.
	*/
	mess_index = 0;
	left_to_decode = addr_struc_ptr->mess_lgth;
	while ( left_to_decode-- ) {
		input_ptr->data_byte = addr_struc_ptr->mess_ptr [ mess_index ];
		char_decode ( input_ptr );
		mess_index++;
	} /* while */

	return return_code;
} /* end of decode_mbx_mess */

