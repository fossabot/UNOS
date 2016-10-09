/***************************************************************************
								TIED SOFTWARE
 PROJECT	:- TS3000

 MODULE 	:- BCNTEST

 FILE 		:- BCNTEST.C

 PROGRAMMER :- R Middleton

 Copyright 1994. The University of Newcastle Research Associates Ltd.

****************************************************************************
* POLYTRON VERSION CONTROL SOFTWARE _ PVCS (tm) Information
*
*      $Revision:   1.1.1.0  $
*      $Date:   29 Apr 1994 19:41:30  $
*
****************************************************************************/
/*
* DISCUSSION :


	BCNTEST.C
	C program to test the beacon communications protocol for the
	project Geraldton ATCU. This program, together with the Turbo C++
	project file, BCNTEST.PRJ, and the files ASYNCXX.C and ASCYNCXX.H
	should be compiled, and then run on a separate PC. A serial link
	between this PC and the ATCU beacon serial channel should then be
	provided.
	The test software is based on timing and protocol information
	supplied by Mitec in 1991 as follows:
		-ATCU is master
		-ATCU poll is one of the following two digit commands:
			?<cr>	(normal poll)
			1<cr>	(change to channel 1 and poll)
			2<cr>	(change to channel 2 and poll)
			3<cr>	(change to channel 3 and poll)
			4<cr>	(change to channel 4 and poll)
			5<cr>	(change to channel 5 and poll)
		-Beacon receiver response is the eight character string:
			[dddd][e][f][g]<cr>
			where:
				[dddd] is four ascii numerals representing
					the beacon value ( -dddd/100 db )
				[e] is a single ascii character (see later)
				[f] is a single ascii numeral (1-5) denoting
					the beacon number currently selected
				[g] is a single character checksum, computed
					as: mod ( sum, 40h ) + 20h
			-All comms are 9600 baud
			-The beacon R/X takes about 100-200mSec to respond
				to a poll
			-upon changing beacon channels, the level becomes
				temporarily unavailable for about 0.5sec.



	Programmer:	Rick Middleton
	1st Entered:	5/10/91
	Last Updated:	6/2/92
	Status:
			Began by copying TEST_SCP.C from the ORBTRACK
			project, and starting to modify the code from that.

			Removed most of the pre-existing code, and added
			much of the user interface routines.

			Added software to test the tester (i.e. use
			"un_check_com" to put characters back into the
			PC receive buffer to simulate ATCU Polls).

			As far as I can tell, the software works correctly.
			Tested only using the keyboard to "fudge" serial
			communications, testing with a serial channel, and
			also with the ATCU is still required.

			5/2/92: RHM: Having discovered that the protocol
			has changed slightly from that assumed, I now have
			to change the test software to conform to the new
			protocol. There are also some slight modifications
			to the functional operation of the system. These
			changes are:
			- the error character, [e], is now defined as:
				01XXXXXX (binary) where each bit denotes
			the following possibilities:
				bit 0: Mode selected (0=remote, 1=local)
				bit 1: ATCU command status (0=valid)
				bit 2: ATCU command timeout
				bit 3: Summary (uWave) alarm
				bit 4: Beacon Level Unavailable
				bit 5: Beacon Level out-of-range.
			- the timeout is a 0.5 second timeout between
			receiving the first character in the poll and the 2nd.
			- the valid range is 00.00db to (-)40.00db. At either
			extreme, error bit 5 is set.
			- On receipt of a poll (of any kind) the mode
			selected automatically changes to remote.

			6/2/92 RHM: Finished preliminary checking of the
			changes, it seems (from tests at home) to be OK
                        now.
*/

#include "asyncxx.h"

#include <conio.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <bios.h>
#include <time.h>
#include <dos.h>
#include <ctype.h>

/**************************************************************************/
/*	Basic Constant Definitions                                        */
/**************************************************************************/

	/* Serial channel definitions */
#define BAUD_RATE 9600
#define PARITY 0
	/* Parity = 3 means EVEN parity; 1 is ODD, 0 is none 	*/
	/* according to the Turbo C++ Library Reference (since	*/
	/* asyncxx.c uses BIOSCOM to initialise the com port.)	*/
#define DATA_BITS 8
#define STOP_BITS 1

	/* Character Definitions */
#define ESCAPE '\x1B'
#define CR '\x0D'

	/* Rows, and Columns for polling displays */
#define START_COLUMN 55
#define STOP_COLUMN 60
#define START_ROW 3
#define STOP_ROW 13

	/* Timeout(sec) allowed between characters received by beacon RX */
#define TIMEOUT 0.5

	/* Range of delay times (mSec) allowed by beacon RX */
#define MAX_DELAY 999
#define MIN_DELAY 50

	/* Time(sec) for the values to become valid after a channel change */
#define CHANGE_TIME 0.5

	/* Maximum and minimum beacon range allowed by the recevier */
#define MAX_BEACON_VALUE 0000
#define MIN_BEACON_VALUE 4000

	/* Error bit definitions */
#define BEACON_RX_LOCAL '\x01'
#define PROTOCOL_ERROR '\x02'
#define TIMEOUT_ERROR '\x04'
#define LOCK_ERROR '\x08'
#define LEVEL_UNAVAILABLE '\x10'
#define OUT_OF_RANGE '\x20'

/**************************************************************************/
/*		Global Variables (for this module)			  */
/**************************************************************************/

static int BcnErrorCode=0;
	/* Error Code that the simulator responds with. */

static float BeaconLevel=-43.21;
	/* Beacon Level that will be sent to the ATCU. */

static int DelayTime=MIN_DELAY;
	/* response delay time of simulator */

static int BeaconChannel=1;
	/* Beacon Channel that has been selected. */

static int uPFault=0, BeaconRXLocal=0, uWLOLock=1, BcnRXLock=1;
	/* uPFault=1 denotes a faulty beacon receiver which sends	*/
	/* 	garbage messages to the ATCU				*/
	/* BeaconRXLocal=1 denotes a beacon receiver which is in local	*/
	/*	mode, and hence will not respond to ATCU channel change */
	/* uWLOLock=1 denotes that the uWave Local Oscilator is in lock	*/
	/* BcnRXLock=1 denotes same for the beacon receiver LO.		*/

static int ChangingChannels=0, ProtocolError=0, TimeoutError=0;
	/* ChangingChannels non-zero denotes that a change channel	*/
	/* 	command has recently been received.			*/
	/* ProtocolError=1 denotes an error in the current Poll		*/
	/* TimeoutError=1 denotes a timeout error for the pending Poll	*/

static clock_t StartTime, FirstCharTime, ChannelChangeTime;
	/* Time at which we started recording the number of protocol	*/
	/* errors, the number of polls, etc is StartTime; the time at	*/
	/* which we last detected a valid first character in a poll	*/
	/* is FirstCharTime; and ChangeChannelTime is the	*/
	/* last time at which we detected a Channel change command.	*/

static unsigned int ProtocolErrors=0, Polls=0, TimeoutErrors=0;

static int PollRow=START_ROW;
	/* variable used to allow a sensible polling display	*/


/**************************************************************************/
/*	Internal Function Prototypes                                      */
/**************************************************************************/

void InsertCheckSum ( char *String );
	/* Computes the checksum based on the characters in string from	*/
	/* position 0 to (DataLength-1) and inserts the checksum	*/
	/* in String[DataLength], (followed by a null terminator)	*/

int InitialiseComms ( void );
	/* Tries to set up the ASYNC serial communications on the port	*/
	/* specified by the user. Returns "0" if all is OK, "1" else.	*/

void InitialiseScreen ( void );
	/* Writes initial messages etc on the screen */

int CheckKB ( void );
	/* This procedure checks the keyboard for input, and handles	*/
	/* any input that it finds.					*/

void ShowVariables ( void );
	/* Display global variables on the screen. */

void CheckForPoll ( void );
	/* Function to check for characters on the serial channel, and	*/
	/* handle them if there are by updating screen information. It	*/
	/* also checks for protocol timeouts (assuming it is called	*/
	/* about once every 100mSec).					*/

void RespondToPoll ( void );
	/* Sends the appropriate response to the poll sent.	*/

void ComputeErrorCode ( int range_error );
	/* Computes the appropriate error code to be returned to the	*/
	/* ATCU. Note that as there are only six error codes, only the	*/
	/* first (i.e. lowest value) error is reported by this software.*/

void PrintYesNo ( int condition );
	/* prints either "Yes" if condition is true, else "No " */

int ValidFirstChar ( char ch );
	/* returns true if the character is a valid first character	*/
	/* for a poll of any kind.					*/

int ValidTerminator ( char ch );
	/* returns true if the character is equal to the terminator	*/
	/* character for the serial comms protocol.			*/

void ClearPollLine ( void );
	/* increments PollRow, and clears an entire line 	*/

/**************************************************************************/
/*			Internal Function Implementations		  */
/**************************************************************************/

void ClearPollLine ( void )
{
	PollRow += 1;
	if ( PollRow > STOP_ROW ) PollRow = START_ROW;
	gotoxy ( START_COLUMN, PollRow );
	textattr ( WHITE+16*LIGHTGRAY );
	clreol ();
	textattr ( YELLOW+16*BLUE );
}

/*------------------------------------------------------------------------*/

int ValidTerminator ( char ch )
{
	return ( ch == CR );
}

/*------------------------------------------------------------------------*/

int ValidFirstChar ( char ch )
{
	return ( ( ch == '?' ) || ( ( ch >= '1' ) && ( ch <= '5' ) ) );
	/* the valid first characters are '?' and '1'..'5' */
}

/*------------------------------------------------------------------------*/

void InsertCheckSum ( char *String )
{
	int i,CheckSum;
	char CheckSumChar;

	CheckSum = 0; i=0;
	while ( String[i]!=0 )
	    {
		CheckSum = CheckSum + String[i];
		i = i+1;
	    }
	CheckSum = CheckSum % '\x40'; /* leave only the bottom four bits */
	CheckSumChar = ( CheckSum + '\x20' );

	String[i] = CheckSumChar;	/* insert checksum */
	String[i+1] = 0;	/* ensure terminating null */
}

/*--------------------------------------------------------------------------*/

int InitialiseComms ( void )
{
	int ErrorCode, Port=0;

	while ( ( Port<1 ) || ( Port>2 ) )
	{
		clrscr();
		printf("Enter the port number for serial comms (1 or 2) :");
		scanf("%d", &Port);
	}

	printf("\n\nTrying to open comms on port #%1d ...", Port );
	open_com ( (Port-1), BAUD_RATE, PARITY, STOP_BITS, DATA_BITS,
			 &ErrorCode );

	if ( ErrorCode != 0 )
	{
		printf("\n\nError %x opening comms port",ErrorCode);
	}

        return ErrorCode;

}

/*-------------------------------------------------------------------------*/

void InitialiseScreen ( void )
{
	clrscr ();
	window ( 15,1,65,5 );
	textattr ( BLACK+16*GREEN );
	gotoxy ( 1,1 );
	cprintf ( "  Project Geraldton Beacon Receiver Simulator " );
	cprintf ( "\n\r______________________________________________" );
	cprintf ( "\n\rCopyright TUNRA Industrial Electronics 5/10/91");

	window ( 1,5,80,25 );
	textattr ( BLACK+16*LIGHTGRAY );
	gotoxy ( START_COLUMN,1 );
	cprintf ( "ATCU" );
	gotoxy ( 65,1 );
	cprintf ( "Simulator" );
	gotoxy ( START_COLUMN,2 );
	cprintf ( "Poll" );
	gotoxy ( 65,2 );
	cprintf ( "Response" );
	gotoxy ( 1,1 );
	cprintf ( "Valid Commands" );
	textattr ( RED+16*LIGHTGRAY );
	cprintf ( "\n\r<esc> return to DOS                       ");
	cprintf ( "\n\rB     alter the beacon value (random)     ");
	cprintf ( "\n\rC     force the simulator to a new channel");
	cprintf ( "\n\rD     alter the simulator response delay  ");
	cprintf ( "\n\rF     toggle beacon RX uP fault status    ");
	cprintf ( "\n\rL     toggle beacon RX local/remote       ");
	cprintf ( "\n\rR     reset poll/error counts and time    ");
	cprintf ( "\n\rU     toggle uWave LO Lock                ");
	cprintf ( "\n\rX     toggle beacon RX LO Lock            ");
	cprintf ( "\n\relse  simulate sending of character       ");

	textattr ( BLACK+16*LIGHTGRAY );
	gotoxy ( 20,17 );
	cprintf ( "Beacon Simulator Variables" );
	cprintf ( "\n\rLevel Channel Status Delay  uW LO Bcn RX"
		  " Changing Bcn uP   Prot. Time  Bcn.Er." );
	cprintf ( "\n\r(db)  number  (L/R)  (mSec) Lock  Lock  "
		  " Channel  Faulty   Error Error Code(H)" );

	gotoxy ( 20,14 );
	cprintf ( "Polling Statistics" );
	cprintf ( "\n\rProtocol Errors      Timeout Errors      Polls      "
		  "UpTime(Hr)" );

	textattr ( YELLOW+16*BLUE );

	StartTime = clock ();
	FirstCharTime = StartTime;
	ChannelChangeTime = StartTime;


}

/*------------------------------------------------------------------------*/

int CheckKB ( void )
{
	char ch;

	if ( bioskey(1) == 0 )
		return 0;	/* no keys to get, so return */

	ch = bioskey(0);
	switch ( toupper(ch) )
	{
		case ESCAPE :
			return 1;	/* escape was typed, so return 1 */

		case 'B':
			BeaconLevel = -100.0 * ( rand () ) / RAND_MAX;
			break;

		case 'C':
			BeaconChannel = BeaconChannel + 1;
			if ( BeaconChannel > 5 )
				BeaconChannel = 1;
                        ChannelChangeTime = clock ();
			break;
		case 'D':
			DelayTime = DelayTime * 2;
			if ( DelayTime > MAX_DELAY )
				DelayTime = MIN_DELAY;
			break;
		case 'F':
			uPFault = ! uPFault;
			break;
		case 'L':
			BeaconRXLocal = ! BeaconRXLocal;
                        break;
		case 'R':
			StartTime = clock();
			ProtocolErrors = 0;
			TimeoutErrors = 0;
			Polls = 0;
			break;
		case 'U':
			uWLOLock = ! uWLOLock;
			break;
		case 'X':
			BcnRXLock = ! BcnRXLock;
                        break;
		default: /* send the character typed */
			un_check_com ( ch );
			break;
	}
	return 0;
}

/*------------------------------------------------------------------------*/

void ShowVariables ( void )
{
	float UpTime;

	gotoxy ( 1,20 );
	cprintf ( "%6.2f", BeaconLevel );
	gotoxy ( 9,20 );
	cprintf ( "%1d", BeaconChannel );
	gotoxy ( 15,20 );

	if ( BeaconRXLocal )
		cprintf ("Local ");
	else
		cprintf ("Remote");

	gotoxy ( 22,20 );
	cprintf ( "%4d", DelayTime );

	gotoxy ( 29,20 );
	PrintYesNo ( uWLOLock );

	gotoxy ( 35,20 );
	PrintYesNo ( BcnRXLock );

	ChangingChannels =
		( (clock()-ChannelChangeTime) < (CHANGE_TIME*CLK_TCK) );
	/* Allow some time between a command to change channel, till 	*/
	/* the values are available.					*/
	gotoxy ( 42,20 );
	PrintYesNo ( ChangingChannels ) ;

	gotoxy ( 51,20 );
	PrintYesNo ( uPFault );

	gotoxy ( 60,20 );
	PrintYesNo ( ProtocolError );

	gotoxy ( 66,20 );
	PrintYesNo ( TimeoutError );

	gotoxy ( 72,20 );
	cprintf( "Hex%2x", BcnErrorCode );

	gotoxy ( 16,15 );
	cprintf ( "%5d", ProtocolErrors );
	gotoxy ( 36,15 );
	cprintf ( "%5d", TimeoutErrors );
	gotoxy ( 47,15 );
	cprintf ( "%5d", Polls );
	gotoxy ( 63,15 );
	UpTime = ( ( clock() - StartTime ) / CLK_TCK / 3600.0 );
	cprintf ( "%7.3f", UpTime );

}

/*------------------------------------------------------------------------*/

void CheckForPoll ( void )
{
	static int Column=START_COLUMN;
	static char PreviousChar=CR;
	static char CurrentChar=0;
	char TempChar;
	int Error, CharFound;
	int NewBeaconChannel;

	check_com ( &TempChar, &Error );	/* Check for chars in */

	if ( uPFault )	/* simulating a uP fault */
		{
		while ( Error==0 )	/* while there are characters */
                	check_com ( &TempChar, &Error ); /* get them */
		delay ( rand() % 2000 );	/* random delay 0-2sec */
		PreviousChar = 0;
		CurrentChar = 0;
		RespondToPoll();	/* send garbage characters */
		return;
		}
        CharFound = ( Error == 0 );

	/* Logic Modified by RHM 5/2/92 to correspond with what we	*/
	/* observed, and what we have in information from Mitec.	*/

	if ( CharFound )
	/* we've found a character! */
	{
	    PreviousChar = CurrentChar;
	    CurrentChar = TempChar;
	    gotoxy ( Column, PollRow );	/* Display the character */
	    if ( TempChar == CR )
		{
		cprintf ( "<cr>" );
		Column = Column + 4;
		while ( Column < (STOP_COLUMN+4) )
		    {
		    Column = Column + 1;
		    cprintf ( " " );
		    }
		Column = START_COLUMN;
		}
	    else
		{
                cprintf ( "%c", TempChar );
	    	if ( Column < STOP_COLUMN )
		    Column = Column + 1;
		}
	    /* finished displaying the character */

	    ProtocolError = ValidTerminator( CurrentChar ) &&
		ValidTerminator ( PreviousChar );
	    /* its certainly a protocol error if both chars = CR */
	    ProtocolError = ProtocolError ||
		( ValidFirstChar ( CurrentChar ) &&
		ValidFirstChar ( PreviousChar ) );
	    /* its also a protocol error if we get two first characters */
	    ProtocolError = ProtocolError ||
		( ( ! ValidFirstChar( CurrentChar ) ) &&
			( ! ValidTerminator( CurrentChar ) ) );
	    /* its also a protocol error if char is unknown */

	    /* if appropriate, record the time at which we got a valid	*/
	    /* first character.						*/
	    if ( ValidFirstChar ( CurrentChar ) )
	    {
		FirstCharTime = clock();
		TimeoutError = 0;
	    }

	    /* now deal with a protocol error */
	    if ( ProtocolError )
	    {
		ProtocolErrors += 1;
		RespondToPoll();
		Column = START_COLUMN;
		return;
	    }

	    /* now check for a valid poll */
	    if ( ValidTerminator ( CurrentChar ) &&
			ValidFirstChar ( PreviousChar ) )
	    {
		switch ( PreviousChar )
		    {
		    case '1' :
		    case '2' :
		    case '3' :
		    case '4' :
		    case '5' :
			/* May need to switch beacon frequencies. */
			NewBeaconChannel = PreviousChar - '0';
			if ( NewBeaconChannel !=  BeaconChannel )
				{
				ChannelChangeTime = clock();
                                ChangingChannels = 1;
				}
			BeaconChannel = NewBeaconChannel;
		    case '?':	/* Now do regular poll bit */
			Polls = Polls + 1;
			RespondToPoll();
			break;
		    }	/* end of switch statement */
		    return;
	    }   /* end of if valid poll */
	    else if ( ValidTerminator ( CurrentChar ) )
	    /* Previous char was garbage, but this was a <CR> */
	    {
		ClearPollLine();
		Column = START_COLUMN;
	    }
	    TimeoutError = 0;
	}	/* end of if found character */

	if ( ( ! TimeoutError ) && ValidFirstChar( CurrentChar ) &&
		( ( clock() - FirstCharTime ) > ( TIMEOUT*CLK_TCK ) ) )
	/* the last character typed was a valid first character for	*/
	/* which I have not yet registered a timeout error, and the	*/
	/* time since that character is excessive.			*/
	{
		TimeoutError = 1;
		TimeoutErrors += 1;
		RespondToPoll();
		CurrentChar = 0;
		Column = START_COLUMN;
		return;
	}
}

/*-------------------------------------------------------------------------*/

void RespondToPoll ( void )
{
	char response[20];
	int bcn, error, i, garbage_length, range_error;

	if ( uPFault ) 	/* Generate garbage */
	    {
	    garbage_length = rand() % 10; /* random length, 0-9 */
	    for ( i=0; i<garbage_length; i=i+1 )
		response[i] = rand() % 128; /* random characters 00h-7Fh */
	    response[garbage_length] = 0; /* terminating null */
	    }
	else	/* generate correct response */
            {
	    FirstCharTime = clock();
	    delay ( DelayTime );
	    bcn = (-100.0)*BeaconLevel+0.5;
	    range_error = ( bcn <= MAX_BEACON_VALUE ) ||
		( bcn >= MIN_BEACON_VALUE );
	    if ( bcn < MAX_BEACON_VALUE ) bcn=MAX_BEACON_VALUE;
	    if ( bcn > MIN_BEACON_VALUE ) bcn=MIN_BEACON_VALUE;
	    ComputeErrorCode( range_error );
	    sprintf( response,"%04d %1d",bcn,BeaconChannel);
	    /* Now add in the error code field */
	    response[4] = BcnErrorCode;
	    /* Next, add in the checksum */
	    InsertCheckSum ( response );
	    }

	/* Now send the response to the serial channel */
	writeln_com ( response, &error );

	/* and also print the response on the screen */
	gotoxy ( 65, PollRow ); clreol();
	cprintf ( response ); cprintf ( "<cr>" );
	ClearPollLine();

	/* On any Poll, beacon receiver goes to remote */
	BeaconRXLocal = 0;
}

/*------------------------------------------------------------------------*/

void ComputeErrorCode ( int range_error )
{
	BcnErrorCode = '\x40';	/* Innocent until proven guilty */
	if ( BeaconRXLocal )
		BcnErrorCode |= BEACON_RX_LOCAL;
	if ( ProtocolError )
		BcnErrorCode |= PROTOCOL_ERROR;
	if ( TimeoutError )
		BcnErrorCode |= TIMEOUT_ERROR;
	if ( (!uWLOLock) || (!BcnRXLock) )
		BcnErrorCode |= LOCK_ERROR;
	if ( ChangingChannels )
		BcnErrorCode |= LEVEL_UNAVAILABLE;
	if ( range_error )
		BcnErrorCode |= OUT_OF_RANGE;


}

/*------------------------------------------------------------------------*/

void PrintYesNo ( int condition )
{
	if ( condition )
		cprintf ( "Yes" );
	else
		cprintf ( "No " );
}

/**************************************************************************/
/*				Main Program				  */
/**************************************************************************/

void main ( void )
{
	int finished=0;


	if ( InitialiseComms() )
	{
        	close_com ();	/* close all communications */
		exit ( EXIT_FAILURE );	/* we couldn't open comms */
	}

        InitialiseScreen ();

	while ( ! finished )
	{
		finished = CheckKB ();
		CheckForPoll();
		ShowVariables();

	}
	gotoxy(1,21);
	cprintf("Done...(Hit any Key to continue)");
	while ( bioskey(1)==0 );
	bioskey(0);	/* remove leftover character */
	system( "cls" );   /* Added by DJLB */
	close_com ();	/* close communications, reset interrupt vector */
	exit ( 0 );	/* normal exit */
}