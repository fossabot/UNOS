/*
********************************************************************************
PLC.C

Task to communicate to the PLC.

Author:	Len Sciacca
Date:	1994
********************************************************************************
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <dos.h>
#include <conio.h>

#include "general.h"
#include "taskname.h"   // ch_1_tx
//#include "kbtask.h"

#include "unos.h"

#include <unosdef.h>
#include "plc.h"

#include "plcext.h"
#include "prot_ext.h"
#include "main_ext.h"  //LOG_TASK definition
#include "protocol.h"
#include "seq.h"		// Import definisitions, start_high, stop etc
//#include "scrext.h"		/* Import protect_ and unprotect_screen fns	*/


#define PLC_Fail 		0x01
#define PLC_Trip1		0x02
#define PLC_Trip		0x04
#define PLC_Spare_M99 	0x08
#define PLC_24VDC   	0x10
#define PLC_AzBrake		0x20
#define PLC_ElBrake		0x40
#define PLC_AzFinalCW 	0x80

#define PLC_AzFinalCCW	0x01
#define PLC_ElFinalUp	0x02
#define PLC_ElFinalDwn	0x04
#define PLC_EmergStop	0x08
#define PLC_SmokeDetect	0x10
#define PLC_Door1		0x20
#define PLC_Door2		0x40
#define PLC_CB5_6		0x80

#define PLC_CB4			0x01
#define PLC_CB3			0x02
#define PLC_RUN1		0x04
#define PLC_RUN2		0x08
#define PLC_C1			0x10
#define PLC_C2			0x20


static timer_struc* plc_cycle_timer;
static unsigned int plc_sem;

static unsigned char rx_mess [ 30 ];
static unsigned int rx_mess_length;

static void plc_timer1 ( int * temp );
static char word1, word2, word3;
	// PLC structure contains the actual fault/conditions
	// PLC-bit contains the bit image of the PLC words
static PLC_struct PLC;
static PLC_struct PLC_bit;
static unsigned int task_count= 0;

static char coillower = 0;
static char coilupper = 0;
static char string [ 20 ];

/*
****************************************************************************
plc_task ( )

 Data is encripted as follows
		 Word1: 	Bit 0 = Fail 	M96
					Bit 1 = Trip1	M97
					Bit 2 = Trip	M98
					Bit 3 = Spare   M99
					Bit 4 = 24VDC OK   		M100
					Bit 5 = Az Brake		M101
					Bit 6 = El Brake		M102
					Bit 7 = Az Final CW 	M103
		Word2:
					Bit 0 = Az Final CCW	M104
					Bit 1 = El Final UP		M105
					Bit 2 = El Final DWN	M106
					Bit 3 = Emerg. Stop		M107
					Bit 4 = Smoke Detect	M108
					Bit 5 = Door 1			M109
					Bit 6 = Door 2			M110
					Bit 7 = CB5/6			M111
		word 3:
					Bit 0 = CB4				M112
					Bit 1 = CB3				M113
					Bit 2 = RUN 1			M114
					Bit 3 = RUN 2			M115
					Bit 4 = C1				M116
					Bit 5 = C2				M117
					Bit 6 = Spare			M118
					Bit 7 = Spare			M119

****************************************************************************
*/
void plc_task ( void * Dummy ) {

	int temp, i;
	unsigned int return_status;
	char * status;
	char ch1, ch2;
	FILE *fp;

	enable ( );
	Dummy = Dummy;

	plc_sem = create_semaphore();
	if(plc_sem != 0xffff)
		init_semaphore (plc_sem, 0, 1 );

	/*---- Set up any task dependent variables, incl timers */
	/* Need to run every 2 time ticks if tick = 30Hz */
	plc_cycle_timer = start_timer ( (unsigned char)REPETITIVE,
								(unsigned long)10,
								(void (*)(int *))plc_timer1, (void*)&temp );


	while ( 1 ) {

		/* rjlov */
#if LOG_TASK
		fp = get_fileptr ( );
		fwrite((const char *)"P\n",1,2,fp);
#endif

		task_count++;
		if (task_count > 500)
			task_count = 0;

		wait ( plc_sem );
		// send a poll to the PLC and wait for a reply.
		// 010c = M
		// 0080 = x0

		form_and_send_message ( "010C03", ch_1_tx, '0' );
		status = rcv_mess( rx_mess, &rx_mess_length, 5 );

		// returns a NULL if timeout
		// Only process if ACK is returned and not time-out
		if ( ( status != NULL ) && ( rx_mess [0] != 0x15 ) )
			{
			for (i=0; i < 6; i++ )
				{
				if ( rx_mess [ i ] == 'F' )
					rx_mess [ i ] = 0x3f;
				}

			// Now strip off the data from the PLC reply
			word1 = ((rx_mess [ 0 ]-0x30 )<<4) | ( rx_mess[1]-0x30);
			word2 = ((rx_mess [ 2 ]-0x30 )<<4) | ( rx_mess[3]-0x30);
			word3 = ((rx_mess [ 4 ]-0x30 )<<4) | ( rx_mess[5]-0x30);

			// Now send the set data
			// 0990 = 9009 => M400
			//form_and_send_message ( "9009", ch_1_tx, '7' );
			// send 1 byte of data to set coils M400 - M407
			// defined by coilupper and coillower
			// Group address for M400 is 0132
			string [ 0 ] = NULL;
			strcat ( string, "013201" );
			ch1 = coillower + 0x30;	// convert to ascii
			ch2 = coilupper + 0x30;

			// Now add ch1 and ch2 to string
			strncat ( string, &ch2, 1 );
			strncat ( string, &ch1, 1 );

			form_and_send_message ( string, ch_1_tx, '1' );
			status = rcv_mess( rx_mess, &rx_mess_length, 3 ); // wait for ACK

			// if status ==  NULL then timeout, if rx_mess == NACK then error

			PLC_bit.Fail = (( PLC_Fail & word1 ) == PLC_Fail );
			PLC_bit.Trip1= (( PLC_Trip1 & word1 ) == PLC_Trip1 );
			PLC_bit.Trip = (( PLC_Trip & word1 ) == PLC_Trip );
			PLC_bit.C24VDC = (( PLC_24VDC & word1 ) == PLC_24VDC );
			PLC_bit.AzBrake	= (( PLC_AzBrake & word1 ) == PLC_AzBrake );
			PLC_bit.ElBrake	= (( PLC_ElBrake & word1 ) == PLC_ElBrake );
			PLC_bit.AzFinalCW = (( PLC_AzFinalCW & word1 ) == PLC_AzFinalCW );

			PLC_bit.AzFinalCCW = (( PLC_AzFinalCCW & word2 ) == PLC_AzFinalCCW);
			PLC_bit.ElFinalUp  = (( PLC_ElFinalUp & word2 ) == PLC_ElFinalUp );
			PLC_bit.ElFinalDwn = (( PLC_ElFinalDwn & word2 ) == PLC_ElFinalDwn );
			PLC_bit.EmergStop  = (( PLC_EmergStop & word2 ) == PLC_EmergStop );
			PLC_bit.SmokeDetect	= (( PLC_SmokeDetect & word2 ) == PLC_SmokeDetect );
			PLC_bit.Door1	    = ((PLC_Door1 & word2 ) == PLC_Door1 );
			PLC_bit.Door2		= ((PLC_Door2 & word2 ) == PLC_Door2 );
			PLC_bit.CB5_6		= ((PLC_CB5_6 & word2 ) == PLC_CB5_6 );

			PLC_bit.CB4			= ((PLC_CB4 & word3 ) == PLC_CB4 );
			PLC_bit.CB3			= ((PLC_CB3 & word3 ) == PLC_CB3 );
			PLC_bit.RUN1		= ((PLC_RUN1 & word3 ) == PLC_RUN1 );
			PLC_bit.RUN2		= ((PLC_RUN2 & word3 ) == PLC_RUN2 );
			PLC_bit.C1			= ((PLC_C1 & word3 ) == PLC_C1 );
			PLC_bit.C2			= ((PLC_C2 & word3 ) == PLC_C2 );

			//PLC = PLC_bit;
			PLC.Fail = PLC_bit.Fail;
			PLC.Trip1= !PLC_bit.Trip1;
			PLC.Trip = !PLC_bit.Trip;
			PLC.C24VDC = !PLC_bit.C24VDC;
			PLC.AzBrake	= PLC_bit.AzBrake;
			PLC.ElBrake	= PLC_bit.ElBrake;
			PLC.AzFinalCW = !PLC_bit.AzFinalCW;

			PLC.AzFinalCCW = !PLC_bit.AzFinalCCW;
			PLC.ElFinalUp  = !PLC_bit.ElFinalUp;
			PLC.ElFinalDwn = !PLC_bit.ElFinalDwn;
			PLC.EmergStop  = !PLC_bit.EmergStop;
			PLC.SmokeDetect	= PLC_bit.SmokeDetect;
			PLC.Door1	    = PLC_bit.Door1;
			PLC.Door2		= PLC_bit.Door2;
			PLC.CB5_6		= PLC_bit.CB5_6;

			PLC.CB4			= !PLC_bit.CB4;
			PLC.CB3			= !PLC_bit.CB3;
			PLC.RUN1		= PLC_bit.RUN1;
			PLC.RUN2		= PLC_bit.RUN2;
			PLC.C1			= PLC_bit.C1;
			PLC.C2			= PLC_bit.C2;

			} // if
		else
			flush_mbx ( );		// clear mailbox and start afresh

	} /* end of infinite while loop */

} /* End of plc_task */



/*
****************************************************************************
plc_timer

	Routine called on the time-out of the timer created in PLC task.
This routine merely signals the sequencer semaphore. The sequencer will then
be scheduled as the next task as it should be the highest priority.

****************************************************************************
*/
void plc_timer1 ( int * temp ) {

	_signal ( plc_sem );
	*temp = 1;		/* avoids warning on compilation only */

} /* End of sequencer_timer */


/*
********************************************************************************
ReadPLC ( )

Routine to return pointer to information read by PLC Polls.

********************************************************************************
*/
void ReadPLC ( PLC_struct * tempPLC ) {

	*tempPLC = PLC;
} // end of ReadPLC

/*
********************************************************************************
WritePLC ( )

Routine to set the two bytes to be sent to the PLC

********************************************************************************
*/
void WritePLC ( int drive_word ) {

	if ( start_high & drive_word )	// m400
		coillower = coillower | 0x01;
	else
		coillower = coillower & 0x0e;

	if ( stop_closed & drive_word ) // m401
		coillower = coillower | 0x02;
	else
		coillower = coillower & 0x0d;

	if ( poweron & drive_word )     // m402
		coillower = coillower | 0x04;
	else
		coillower = 0; //coillower & 0x0b;

	if ( lights_on & drive_word )  // m403
		coillower = coillower | 0x08;
	else
		coillower = coillower & 0x07;

	if ( reset_high & drive_word ) // m404
		coilupper = coilupper | 0x01;
	else
		coilupper = coilupper & 0x0e;

	coilupper = coilupper | 0x02;		// Watchdog thing

} // end of WritePLC

/*
****************************************************************************
return_plctask_ctr ( )

	Routine to pass back the plc task counter.
	Used for diagnostic to check the task is alive

Called from screen task when required.

****************************************************************************
*/
unsigned int return_plctask_ctr (void)
{
	unsigned int	result;

	disable ();
	result = task_count;
	enable ();

	return ( result );
}   /* end of return_plctask_ctr */

