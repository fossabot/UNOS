/****************************************************************************
* POLYTRON VERSION CONTROL SOFTWARE _ PVCS (tm) Information
*
*      $Revision:   2.6  $
*      $Date:   06 Mar 1992 17:57:28  $
* 
****************************************************************************/
/****************************************************************************
* Geraldton ADSCS Project.
*****************************************************************************
* Copyright (c) 1991. The University of Newcastle Research Associates Ltd.
*****************************************************************************
*
* PROGRAMMER :-     Peter Moylan,
*                   University of Newcastle, NSW 2308.
*
* FUNCTION REFERENCE :- 
*
****************************************************************************/
/****************************************************************************
* MODULE :- Beacon Task                  File : RECEIVER.C
*****************************************************************************
*
* DISCUSSION :
*
****************************************************************************/
/************************************************************************/
/*									*/
/*				RECEIVER.C				*/
/*									*/
/*      	    SIMULATOR FOR THE BEACON RECEIVER			*/
/*									*/
/*	The purpose of this file is to allow testing of the ATCU	*/
/*	software in the absence of the physical beacon receiver.	*/
/*									*/
/*	Programmer:	P. Moylan					*/
/*	Last modified:	5 February 1992					*/
/*	Status:		Now rewritten to be a real-time simulation.	*/
/*		Testing still in progress.  Seems to work.		*/
/*		Changed PJM 4/2/92 to reflect changes which we've	*/
/*		just discovered in the beacon receiver protocol.	*/
/*	Limitations:							*/
/*	    1.	This module simulates a receiver which never makes	*/
/*		an error, and for which the beacon level is always	*/
/*		available.  There is no provision at present for	*/
/*		simulating faults.					*/
/*									*/
/************************************************************************/

#define private static
#define TRUE 1
#define NULL 0x00

#include <receiver.h>	/* Import my own definition file		*/
#include <dos.h>	/* FROM DOS IMPORT enable			*/
#include <unos.h>	/* FROM UNOS IMPORT free_mbx, send_mess,	*/
			/*	rcv_mess				*/
#include <conio.h>
#include <orb_sim.h>	/* FROM Orb_Sim IMPORT get_orb_sim_bcn		*/
#include <simintel.h>	// From simintel.c IMPORT get_intelsat_sim_bcn()

#define CR '\x0D'

#include <bcn_scrn.h>	/* FROM bcn_scrn IMPORT GetScreen,ReleaseScreen	*/
#include <conio.h>	/* FROM conio IMPORT cprintf, gotoxy,		*/
			/*		text_info, clrscr, clreol	*/
#include <kbtask.h>	/* FROM kbtask IMPORT BEACON_PAGE */

private struct text_info ScreenInfo = {45,6,75,10,0x76,0x76};

/************************************************************************/
/*			VARIABLES GLOBAL TO THIS MODULE			*/
/************************************************************************/

/* CurrentChannel is the currently selected channel.	*/

private unsigned int CurrentChannel = 1;

/* Array Level holds the simulated receiver levels for all channels.	*/
/* (To simplify coding, Level[0] is a dummy entry.)			*/

private double Level[6] = { 0.0,  0.0, 0.0,  0.0, 0.0,  0.0 } ;

/************************************************************************/
/*			    INTERNAL PROCEDURES				*/
/************************************************************************/

private void PutBuffer (unsigned char* p, double value)

	/* Converts value to a scaled four-digit string, and puts the	*/
    /* result into p[0..3].						*/

    {
    int scaledvalue, j;

    scaledvalue = 100.0 * value + 0.5;
    if (scaledvalue < 0) scaledvalue = 0;
    if (scaledvalue > 9999) scaledvalue = 9999;
    for (j=3;  j >= 0;  j--)
	{
	p[j] = scaledvalue % 10 + '0';
	scaledvalue /= 10;
	}
    }

/************************************************************************/

private void UpdateLevels (void)

    /* Simulates a level which is a sawtooth function, except for	*/
	/* channel 1 and 2 where we use a more sophisticated simulation supplied	*/
	/* by function get_orb_sim_bcn in module ORB_SIM.			*/
	/* LJS 6/94 - added Intelsat option based on which mode we are in */
	{
	double incr = 0.1;
	unsigned int j;

	Level[1] = get_intelsat_sim_bcn();
	Level[2] = get_orb_sim_bcn ();

	for (j=3; j<6; j++)
	if (Level[j] <= -99.0)  Level[j] = 0.0;
	else Level[j] -= incr;
    }

/************************************************************************/

private void PreparePollResponse (unsigned char* p, char status)

    /* Called after the successful reception of a command.  Puts an	*/
    /* eight-character response into the output buffer pointed to by p.	*/
    /* The format is:							*/
    /*	p[0..3]	encoded signal level					*/
    /*	p[4]	beacon status, as supplied by caller			*/
    /*	p[5]	current beacon channel, ascii '1'..'5'			*/
    /*	p[6]	checksum						*/
    /*	p[7]	carriage return character				*/

    {
    PutBuffer (p, -Level[CurrentChannel]);
    UpdateLevels();
    p[4] = status;
    p[5] = CurrentChannel + '0';

    /* Compute a checksum */

	{
	unsigned int j;  unsigned char checksum = 0;
	for (j=0; j<=5; j++) checksum += p[j];
	checksum &= 0x3F;  checksum += 0x20;
	p[6] = checksum;
	}

    p[7] = CR;
    }

/************************************************************************/

private void SendMessage (unsigned char* mess, unsigned int count,
				unsigned char *destination)

    /* Sends a message of count characters to a destination task.	*/
    /* For the sake of making this a tougher test, the characters are	*/
    /* sent one at a time.						*/

    {
    unsigned char* p;  unsigned int j;

    p = mess;
    for (j = 0;  j < count;  j++, p++)
    	{
	if ( free_mbx((char*)destination) > 1 )
	    send_mess (p, 1, (char*)destination);
	else if (GetScreen (BEACON_PAGE, &ScreenInfo))
    	    {
            gotoxy (2, 5);  clreol();
	    cprintf ("Can't get a free mailbox.");
	    ReleaseScreen (&ScreenInfo);
            }
	}
    }

/************************************************************************/

private unsigned char AcceptPoll (unsigned char* reply,
					unsigned char* *taskname_ptr)

    /* We expect a two-character message of which the second character	*/
    /* is a carriage return.  If it arrives as expected, we return with	*/
    /* the first character in the first argument, the task number of	*/
    /* the task which sent the poll in the second argument, and the	*/
    /* function result is 0x40.  If something went wrong, we return	*/
    /* with a function result of 0x42.					*/

    {
    unsigned char poll[2];
    unsigned int poll_length = 0;

    /* In this version, we wait 50 ticks for a poll to arrive.	*/
    /* This should, in principle, never time out.		*/

    poll[0] = ' ';  poll[1] = ' ';
	*taskname_ptr = rcv_mess (poll, &poll_length, 50);

    if (poll_length > 2) return 0x42;
    *reply = poll[0];
    if (poll_length == 1)
	{
	*taskname_ptr = rcv_mess (poll, &poll_length, 50);
	if (poll_length != 1) return 0x42;
	poll[1] = poll[0];
	}
    if (poll[1] != CR) return 0x42;
    return 0x40;
    }

/************************************************************************/

void init_receiver_screen (void)

    /* Refreshes the screen.  To be called each time BEACON_PAGE	*/
    /* becomes the active maintenance page.				*/

    {
    if (GetScreen (BEACON_PAGE, &ScreenInfo))
    	{
	clrscr();
	cprintf ("   BEACON RECEIVER SIMULATOR\n\r");
        cprintf ("\n\r Poll           Status\n\r Response\n");
        ReleaseScreen (&ScreenInfo);
        }
    }

/************************************************************************/
/*			     THE SIMULATOR TASK				*/
/************************************************************************/

void BeaconSimulatorTask (void *Dummy)

    /* Runs as a separate task, accepting poll messages and responding	*/
    /* to them by sending a reply back to the beacon task.		*/

	{
    unsigned char response[8], command, status;
	unsigned char *client;

	enable();
	Dummy=Dummy;
    init_receiver_screen ();

    while (TRUE)
    	{
	status = AcceptPoll (&command, &client);
	if (client == NULL )
	    {
	    if (GetScreen (BEACON_PAGE, &ScreenInfo))
		{
		gotoxy (2, 5);  clreol();
	    	cprintf ("Beacon poll never arrived");
	    	ReleaseScreen (&ScreenInfo);
            	}
            }

        /* Check that the poll message format is valid.	*/

	if (command >= '1' && command <= '5')
	    CurrentChannel = command - '0';
	else if (command != '?') status = 0x42;

	if (status != 0x40)
	    if (GetScreen (BEACON_PAGE, &ScreenInfo))
		{
		gotoxy (2, 5);  clreol();
		cprintf ("Invalid command byte %c", command);
        	ReleaseScreen (&ScreenInfo);
        	}

	if (GetScreen (BEACON_PAGE, &ScreenInfo))
    	    {
    	    gotoxy (12, 3);  cprintf ("%c", command);
    	    gotoxy (25, 3);  cprintf ("%c", status);
	    ReleaseScreen (&ScreenInfo);
            }

        /* Put together the response.	*/

	PreparePollResponse (response, status);

	if (GetScreen (BEACON_PAGE, &ScreenInfo))
    	    {
            gotoxy (12, 4);  clreol();
    	    cprintf ("%.7s", &response);
	    ReleaseScreen (&ScreenInfo);
            }

        /* Send the response to the task which sent us the poll.	*/
	/* (In the present version this is always the beacon task, but	*/
	/* allowing polls from any task expands our testing options.)	*/
	/* Note that when the rcv_mess timesout it returns an error code */
	/* negative number. So we should NOT send a reply when there is */
	/* a time-out */
	if ( ! ( client == NULL ) )
		SendMessage (response, 8, client);
	}
    }