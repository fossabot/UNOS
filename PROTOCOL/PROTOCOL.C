
/*****************************************************************************
*
*
* DISCUSSION : This module contains the routines used to implement the Mitsubishi
*	like protocol for ACU/PLC serial communications.
*****************************************************************************/

#include <stdio.h>
#include <dos.h>
#include <string.h>
#include <conio.h>
#include <process.h>

#include "unos.h"      /* Calls for UNOS routines. */
#include "unosdef.h"
#include "screen.h"

#include "main_ext.h"   // define LOG_TASK

#include "taskname.h"
#include "protocol.h"

#include "keyboard.h"

#define SLAVEPARITYERROR			1
#define SLAVECHECKSUMERROR			2
#define SLAVEFRAMINGERROR			4
#define SLAVEOVERRUNERROR			5
#define SLAVECHARAREAERROR			6
#define SLAVEGOTACKNAK	        	7
#define SLAVENOTGOTACKNAK       	8
#define SLAVEGOTWRONGLENGTH     	9
#define SLAVEUNKNOWNMESSAGE 		10
#define SLAVEINPUTBUFFEROVERFLOW 	11
#define SLAVEGOTBREAKERROR			12

/* RAC - The following definitions are also found in the ATCU\UNOS\SERINT.C
file. */

#define	OVERRUN_ERROR_BYTE	0x80
#define	PARITY_ERROR_BYTE	0x81
#define	FRAMING_ERROR_BYTE	0x82
#define	BREAK_INTERRUPT_ERROR_BYTE	0x83

static char display_mess [ 30 ];

static char checksum_count;
static unsigned int prot_sem;
static unsigned char rx_mess [ 20 ];
static unsigned int rx_mess_length;
static char count;

/****************************
* 	FORMSENDMESSAGE()       *
****************************/
/************************************************************************
*
* This routine is passed a character string from either Command Parser, formats
* as per the Mitsubishi protocol and sends a mailbox message off to the
* transmit handler task.
* (N.B. Re-entrant routine.)
* Returns the result of the "send_mess()" call, i.e logical true for success.
*
*************************************************************************/
int	form_and_send_message( char * message, char * TX_handler_task_name,
											char type ) {

	int	error, status, i, checksum;
	char buf [ CMU_MESS_SIZE ];

	switch ( message[ 0 ] )
	{
		case ACK :
					/* Send an ACK to the PLC */
					sprintf( buf, "%c",ACK);
					break;

		case NAK :
					/* Format the error code from the message, and return that */
					error = (int)message[ 1 ];
					sprintf( buf , "%c",NAK);
					break;

		default  :
					/* If the first character is not one of the two above, then
						the data is all data to be returned */
					sprintf( buf, "%c%c%s%c", STX, type, message, ETX );
					//sprintf ( buf, "%c%c%c%c%c%c%c%c%c", STX, 0x37, 0x31, 0x33, 0x30, 0x35, 0x03, 0x30, 0x33 );
					/* Calculate the Checksum */
					checksum = 0;
					for( i = 1; i < strlen( buf ); i++ )
						checksum = checksum + (int)buf[ i ];

					checksum = checksum % 256;

					sprintf( &buf[ strlen(buf) ],"%02X", checksum );

					break;
	}

	if ( free_mbx ( TX_handler_task_name ) > 1 )
		status = send_mess( (unsigned char *)buf, strlen( buf ),
								TX_handler_task_name );

	return( status );
}

/****************************
* 	RX_AND_PARSE_MESSAGE()  *
****************************/
/************************************************************************
*
* This routine is designed to receive data from a Rx serial handler task,
* and collects the string until a LF is received or an error has occured.
* It then performs checking/stripping as per the Mitsubishi protocol.
* An error code is returned if required. The string will be returned via
* the calling parameter string.
* (N.B. Re-entrant routine.)
*
*************************************************************************/

static unsigned char rx_and_parse_message( char *message,
										int time_out )
{
	unsigned int error = 0, readchecksum = 0;
	unsigned char checksum = 0;
	char buf[ 30 ];
	int enddata, i, message_index = 0;
	char EndMessage;

	display_mess[0]=count+0x30;
	if ( count > 9 )
		count = 0;

	display_mess[1]='\0';

	rx_mess[0]='\0';

	message_index = 0;
	EndMessage = 0;

	i=1;

	count++;
	checksum_count = 0;

	do
	{
		/* Wait for bytes to arrive from the serial handler. */
		rcv_mess( &rx_mess[0], &rx_mess_length, time_out );

/*		if ( rx_mess[0] >= OVERRUN_ERROR_BYTE )
		{
			if ( rx_mess[0] == OVERRUN_ERROR_BYTE )
				error = SLAVEOVERRUNERROR;
			else if ( rx_mess[0] == PARITY_ERROR_BYTE )
				error = SLAVEPARITYERROR;
			else if ( rx_mess[0] == FRAMING_ERROR_BYTE )
				error = SLAVEFRAMINGERROR;
			else if ( rx_mess[0] == BREAK_INTERRUPT_ERROR_BYTE )
				error = SLAVEGOTBREAKERROR;
		}
*/

		if ( (rx_mess[0] == NAK) || ( rx_mess[0] ==ACK) )
			{
			/* A brief message */
			message_index = 0;
			message [ message_index ] = rx_mess[0];
			display_mess[i]='-'; // message[message_index]; //rx_mess[0];

			EndMessage = 1;
			}

		if (( rx_mess[0] == STX ))
			{
			if (message_index>0)
				EndMessage = 1;
			else {
			message_index = 0;
			i=1;
			message [ message_index ] = rx_mess[0];
			display_mess[i]='%'; //message[message_index]; //rx_mess[0];
			checksum_count = 1;
			message_index++;
			}
			}
		else if ( message_index > 0 )
			{
			message[ message_index ] = rx_mess[0];
			display_mess[i]=message[message_index]; //rx_mess[0];
			message_index++;

			checksum_count++;

			if ((checksum_count == 10))
				{
				EndMessage = 1;
				message[message_index]=NULL;
				display_mess[i+1]='\0';
				message_index = 0;
				}

			// now grab the 2 checksum bytes
 /*			if (( rx_mess[0] == ETX )) // && ( checksum_count == 0 ))
				checksum_count = 0;

			if ( checksum_count == 2 )
				{
				message [ message_index ] = NULL;
				message_index = 0;
				EndMessage = 1;
				}
			else
				checksum_count++;
*/
			}

		display_mess[i+1]='\0';
		i++;

	} while ( !EndMessage );	 // DO


	if ( error )
		return error;

	/* The message passed to this function should start with an ACK, NAK, STX*/
	enddata = strlen( message );      /* N.B. Length does not include NULL. */
//	strcpy ( &message_disp, message );

	switch ( message[ 0 ] )
	{
		case ACK :
		case NAK :
                    message [ 1 ] = NULL;
					return 0;
		case STX:
					/* Calculate the checksum */
					checksum = 0;
					for( i = 1; i < (enddata-2); i++ )
						{
						checksum += (unsigned char)message[ i ];
						}

					/* Now read the checksum and compare */
					strcpy( buf , &message[ enddata-2 ] );
                    //sscanf(buf, "%x",&readchecksum);
					//gotoxy(20,20);cprintf("%s", message );

					if ( (sscanf( buf, "%x", &readchecksum ) == 1) &&
							((((unsigned int)checksum)&0xff) == readchecksum) )
						{
						/* Remove the checksum and the ETX */
						message[ enddata-3 ] = NULL;

						/* Now shift the data area */
						enddata = strlen(message);
						for( i = 1; i <= enddata; i++ )
							message[i-1] = message[i];

						return 0;
						}
					else
						{
						/* We have a problem, return the error */
						/* Either the scan failed, or the checksum != */
						return SLAVECHECKSUMERROR;
						}

		default  : return SLAVEUNKNOWNMESSAGE;
	}
	return 0;
}


/****************************
* 	       RX_PROTOCOL()    *
****************************/
/************************************************************************
*
* This routine is the "sequencer" for the RX protocol tasks.
* (N.B. Re-entrant routine.)
*
*************************************************************************/

static void	rx_protocol( void )
{

	unsigned char message[ 30 ], rx_error;
	char *command_parser_task_name;
	FILE *fp;

	//TX_interface_task_name = ch_1_tx;
	command_parser_task_name = chPLCTaskName;

	while ( 1 )
	{
		/* Wait for a message to arrive */
		rx_error = rx_and_parse_message ( (char*)message , 0 );

		/* rjlov */
#if LOG_TASK
		fp = get_fileptr ( );
		fwrite((const char *)"Rp",1,2,fp);
#endif

		if ( !rx_error )
			{
				/* All OK. Send the message onto the command parser. */
				if ( free_mbx ( command_parser_task_name ) > 1 )
					send_mess(	message, strlen(message),
									command_parser_task_name );
			}	/* end if !rx_error */
		else
				if ( free_mbx ( command_parser_task_name ) > 1 )
							send_mess(	"Error" , strlen( "Error" ) + 1,
									command_parser_task_name );

	}	/* end while */
}

void ret_protocol_mess ( char * display_str ) {

	disable ( );
	strcpy ( display_str, display_mess );
	enable ( );

} /* end of ret_protocol_mess */


void rx_1_protocol_task ( void * dummy )
{

	dummy = dummy;

	enable ( );

	if ((prot_sem = create_semaphore ()) != 0xffff)
		init_semaphore (prot_sem, 0, 1 );

	/* Attach to serial channel 2 receiver task. */
	send_mess( (unsigned char *)"?",1, ch_1_rx );

	rx_protocol ( );
}
