/***************************************************************************
								TIED SOFTWARE
 PROJECT	:- TS3000

 MODULE 	:- BCNTEST

 FILE 		:- ASYNCXX.H

 PROGRAMMER :- R Middleton

 Copyright 1994. The University of Newcastle Research Associates Ltd.

****************************************************************************
* POLYTRON VERSION CONTROL SOFTWARE _ PVCS (tm) Information
*
*      $Revision:   1.1.1.0  $
*      $Date:   29 Apr 1994 19:41:28  $
*
****************************************************************************/

/* Prototype file for asyncpec.c */
/* Quinn-Curtis 1989 */
void open_com(int Cport, int baud,int parity, int stopbits,int numbits,
		  int *err_code);
	/* Initialise communications, ready buffers etc. on the port. */
void close_com( void );
	/* Shut down all comms, remove interrupts etc. */
void reset_buffer( void );
	/* clear buffer (all data lost) */
void send_com( char c,int *error_code );
	/* send a single character down the serial line */
void writeln_com( char *str,int *error_code );
	/* send a string (with <cr> terminator). */
void readln_com( char *str,int *error_code );
	/* read a string with a <cr> terminator. */
void check_com( char *c, int *error_code );
	/* get a single character (if available) */
void un_check_com( char c );
	/* put a character back in the buffer, ready for the next */
	/* read_com, or check_com operation. */