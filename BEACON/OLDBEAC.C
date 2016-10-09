/****************************************************************************
* POLYTRON VERSION CONTROL SOFTWARE _ PVCS (tm) Information
*
*      $Revision:   2.6  $
*      $Date:   06 Mar 1992 17:57:22  $
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
* MODULE :- Beacon Task                  File : BEACON.C
*****************************************************************************
*
* DISCUSSION :
*
****************************************************************************/
/************************************************************************/
/*									*/
/*			BEACON.C - BEACON INTERFACE			*/
/*									*/
/*	This is the version which can handle either simulation		*/
/*		mode or talk to the real serial interface.		*/
/*									*/
/*	Programmer:	P. Moylan					*/
/*	Last modified:	2/3/1992					*/
/*	Status:		OK, except as noted below			*/
/*									*/
/*		Modified PJM 4/2/92 and 5/2/92 to change the beacon	*/
/*		receiver protocol, to implement the protocol changes of	*/
/*		June 1991 which were not notified to us but which were	*/
/*		discovered after delivery of the beacon receiver.	*/
/*									*/
/*		Complete in most respects, but some details remain to	*/
/*		be filled in:						*/
/*		    (a) The import of TEMP.H should be eliminated	*/
/*		    (b)	The method of performing serial I/O has not	*/
/*			been tested.					*/
/*									*/
/*		Testing is in progress, using a crude beacon simulator.	*/
/*									*/
/*	UNRESOLVED DESIGN ISSUES:					*/
/*		These are currently listed in BEACON.DOC		*/
/*									*/
/*	DESIGN CHANGES 20/8/91:						*/
/*		1. New error code BEACON_POLARISER_IN_USE added.	*/
/*		   Removed again 24/9/91.
/*		2. The error code BEACON_RATE_HIGH is no longer reset	*/
/*		   by this module; rather, it is assumed that it will	*/
/*		   be reset on each CMU poll.  In conjunction with	*/
/*		   this, the error is reported only if the slew rate	*/
/*		   is excessive at the time that get_beacon(FALSE) is	*/
/*		   called - i.e. the probing parameter in get_beacon	*/
/*		   is no longer treated as a sticky flag.  This change	*/
/*		   was made after discussions with RHM which revealed	*/
/*		   problems in the earlier approach of checking rate	*/
/*		   errors continuously until get_beacon(TRUE) is called.*/
/*		3. The .rms field of the beacon structure is actually	*/
/*		   expected to be the square root of the variance of	*/
/*		   the mean signal, rather than the variance itself.	*/
/*		   Change made by RHM 12/12/91.				*/
/*									*/
/*	After Testing the prototype Mitec beacon receiver, it is clear	*/
/*	Mitec have altered the syntax of the protocol. In particular,	*/
/*	the error code is no longer a digit, but seems to be just about	*/
/*	any ascii digit. Changed the beacon code so far to allow for	*/
/*	the new definition without reporting protocol errors. Further	*/
/*	changes needed when we find out from Mitec what the actual	*/
/*	codes now mean.	       RHM 30/1/92.				*/
/*									*/
/*	PJM 2/3/92: David's timeout flag removed, slightly altered	*/
/*	approach to error conditions.					*/
/*									*/
/************************************************************************/

#include <beacon.h>	/* Import my own definition module.		*/
#include <dos.h>	/* FROM dos.h IMPORT enable, disable		*/
#include <stdlib.h>	/* FROM stdlib IMPORT atexit			*/
#include <unos.h>	/* FROM UNOS IMPORT free_mbx, send_mess,	*/
			/*   used_mbx, rcv_mess, flush_mbx, timed_wait	*/
#include <hwio_ext.h>	/* FROM HWIO IMPORT hardware_mode_is_real	*/
#include <math.h>	/* FROM Math IMPORT fabs			*/
#include <ctype.h>	/* FROM ctype IMPORT isdigit			*/
#include <err_nums.h>	/* FROM ErrorHandler IMPORT			*/
#include <err_ex1.h>	/*	error_set, error_reset.			*/
#include <pars_td1.h>	/* FROM Parser IMPORT beacon_details_struct,	*/
			/*		satellite_details_struct	*/
			/* (Why Parser? Shouldn't these be from NVRAM?) */
#include <nvramext.h>	/* FROM NVRAM IMPORT get_beacon_details,	*/
			/*		get_current_satellite,		*/
			/*		get_satellite_details		*/
#include <kbtask.h>	/* FROM Kbtask IMPORT BEACON_PAGE,		*/
			/*	return_screen_page			*/
#include "scrext.h"	// imports update_screen( time)
#include <taskname.h> // UNOS task names
						// in main routine.

/************************************************************************/
/* REMARK: There is something very fishy, to my way of thinking, in	*/
/* the structure of nvramext.h, because of the way it in turn ends up	*/
/* importing a whole batch of other things.  I'm not sure if this can	*/
/* be fixed, but the end result is extremely unreadable.  Is this an	*/
/* inherent fault of the C language, or is it a design problem?		*/
/************************************************************************/

#define Boolean unsigned char
#define TRUE 1
#define FALSE 0
#define private static

#define CR '\x0D'		/* carriage return character */

/************************************************************************/
/*		FOR SCREEN OUTPUT TO MAINTENANCE PAGE			*/
/************************************************************************/

#include <receiver.h>	/* FROM Receiver IMPORT init_receiver_screen	*/
#include <cmclock.h>	/* FROM CmClock IMPORT BYTE, TimeRecord,	*/
			/*				ReadClock	*/
#include <bcn_scrn.h>	/* FROM bcn_scrn IMPORT GetScreen,ReleaseScreen	*/
#include <conio.h>	/* FROM conio IMPORT cprintf, gotoxy, text_info	*/
private struct text_info WholeScreen = {1,1,80,24,0x71,0x71};
private struct text_info Display = {7,3,30,15,0x71,0x71};
private struct text_info PollScreen = {11,16,36,22,0x13,0x13};

/************************************************************************/
/*									*/
/*		      VARIABLES GLOBAL TO THIS MODULE			*/
/*									*/
/************************************************************************/

/* Data in this group of declarations are shared between the beacon	*/
/* task and the clients of this module, therefore critical section	*/
/* protection is needed.  In the present version, we use disable/enable	*/
/* to protect critical sections.					*/

/* DesiredFrequency is the channel number from which we are supposed	*/
/* to be getting readings.  Set ChannelChangeRequired to TRUE whenever	*/
/* this value is updated.  Note that we use ASCII digits, not integers,	*/
/* as channel numbers.							*/

private unsigned char DesiredFrequency = '1';
private Boolean ChannelChangeRequired = TRUE;

/* CurrentEstimate contains our latest update of the data which may	*/
/* be requested by client modules.					*/

private beacon_struct CurrentEstimate = { 0.0, 0.0, 0.0, not_enough_data };

/* We continuously keep track of whether the slew rate is outside the	*/
/* limit defined in NVRAM, and RateExcessive indicates this condition.	*/
/* However, we report the error only when get_beacon is called.		*/

private Boolean RateExcessive = FALSE;

private unsigned int beacon_task_sleep_sem;

/************************************************************************/
/*									*/
/*		    INTERFACE TO THE CLIENT MODULES			*/
/*									*/
/************************************************************************/

extern ErrorCode get_beacon_strength (double *beacon_strength)

    /* This function returns the beacon strength (in db) averaged over	*/
    /* approximately the past 7 samples, and adjusted by the amount	*/
    /* specified as the beacon offset for the current satellite. The	*/
    /* ErrorCode returned indicates the status of the beacon signal	*/
	/* over the 7-sample period.  The caller is NOT expected to do any	*/
    /* error handling on the returned ErrorCode.			*/

    {
    beacon_struct estimate;

    disable();  estimate = CurrentEstimate;  enable();
    *beacon_strength = estimate.average;
    return estimate.error;
    }

/************************************************************************/

beacon_struct get_beacon (int probing)

    /* The integer, probing, is passed in as a zero if Orbit track is	*/
    /* currently operating at a fixed offset from the satellite track,	*/
    /* and thus checking on the beacon rate of change should be enabled.*/
    /* If any other number is passed in, the beacon task's rate of	*/
    /* change detection should be disabled.				*/

    {
    beacon_struct result;
    Boolean rateproblem;

    disable();
    result = CurrentEstimate;
    rateproblem = RateExcessive;
	enable();

    if (!probing && rateproblem && result.error == OK)
	{
	error_set (BEACON_RATE_HIGH);
	result.error = slew_rate_error;
	}

    return result;
    }

/************************************************************************/

void select_new_beacon (int beacon_number)

    /* This function causes the appropriate messages to be sent to the	*/
    /* beacon receiver to change the selected frequency on the beacon	*/
    /* receiver.  This routine does NOT do any error checking on the	*/
    /* response code, as we cannot afford to wait around for the	*/
    /* response	from the beacon RX.					*/

    {
    disable();
    DesiredFrequency = '0' + beacon_number;
    ChannelChangeRequired = TRUE;
    enable();
    }

/************************************************************************/
/*			MAINTENANCE PAGE OUTPUT				*/
/************************************************************************/

private void WriteDay (unsigned int dayinweek)

    /* Writes the name of the day */

    {
    switch (dayinweek)
	{
	case 1:		cprintf ("Mon");  break;
	case 2:		cprintf ("Tues");  break;
	case 3:		cprintf ("Wednes");  break;
	case 4:		cprintf ("Thurs");  break;
	case 5:		cprintf ("Fri");  break;
	case 6:		cprintf ("Satur");  break;
	case 7:		cprintf ("Sun");  break;
	default:	cprintf ("%02X", dayinweek);
	}
    cprintf ("day");
	}

/************************************************************************/

private void ShowTime (void)

    /* Displays the time and date on the screen.	*/

    {
    #define Txbase 65
    #define Tybase 1
    TimeRecord time;

    if (GetScreen (BEACON_PAGE, &WholeScreen))
	{
	ReadClock (&time);
	//gotoxy (Txbase, Tybase);  WriteDay (time.dayofweek);  clreol();
	gotoxy (Txbase, Tybase+1);
	cprintf ("%02u/%02u/%02u%02u", time.dayofmonth,
		time.month, time.century, time.year);
	clreol();
	gotoxy (Txbase, Tybase+2);
	cprintf ("%02u:%02u:%02u.%02u", time.hours,
		time.minutes, time.seconds, time.hundredths);
	clreol();
	ReleaseScreen (&WholeScreen);
	}
    }

/************************************************************************/

void display_beacon (void)

    /* Refreshes the screen.  To be called each time BEACON_PAGE	*/
    /* becomes the active maintenance page.				*/

    {
    if (GetScreen (BEACON_PAGE, &WholeScreen))
	{
	clrscr();
	_setcursortype (_NOCURSOR);
        gotoxy (28,1);
	cprintf ("BEACON MAINTENANCE PAGE");
	ReleaseScreen (&WholeScreen);
    	}
    if (GetScreen (BEACON_PAGE, &Display))
	{
	clrscr();
	cprintf ("  SMOOTHED BEACON DATA\n\n\r Average\n\r");
	cprintf (" RMS\n\r Rate\n\r Error code\n\n\r");
	cprintf (" Offset\n\r Low limit\n\r High limit\n\r");
	cprintf (" Rate limit\n\n\r    RAW BEACON DATA");
	ReleaseScreen (&Display);
    	}
    if (GetScreen (BEACON_PAGE, &PollScreen))
	{
	clrscr();
	ReleaseScreen (&PollScreen);
    	}
    init_receiver_screen();
	while (return_screen_page() == BEACON_PAGE)
	{
	ShowTime ( );

	update_screen ( 5 );
	}
    }

/************************************************************************/
/*									*/
/*	THE FOLLOWING DATA ARE USED ONLY BY THE BEACON TASK AND BY	*/
/*			ITS SUBSIDIARY PROCEDURES			*/
/*									*/
/************************************************************************/

/* We keep the receiver data in a circular buffer Levels.  At any given	*/
/* time SampleNumber gives the buffer position where we are going to	*/
/* put the next datum (except transiently, when we have just received	*/
/* a new datum and are processing it).  Note: the stored data in Levels	*/
/* are as received, minus the "beacon offset" adjustment from NVRAM.	*/

private unsigned int SampleNumber = 0;

private double Levels[BeaconSampleSize] =
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

/* Each element of array Squares holds the square of the corresponding	*/
/* element of Levels.  The only function of Squares is to speed up	*/
/* variance computations - i.e. the square of a value is computed only	*/
/* once rather than being repeatedly re-computed.			*/

private double Squares[BeaconSampleSize] =
				{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

/* SlewLimit holds the rate limit most recently read from NVRAM.	*/

private float SlewLimit;

/* The Boolean flags Beacon_was_low and Beacon_was_high keep track of	*/
/* whether we have set the error conditions BEACON_IS_LOW and		*/
/* BEACON_IS_HIGH respectively.  Their only function is to allow us to	*/
/* avoid redundant setting or resetting of these error indications.	*/

private Boolean Beacon_was_low, Beacon_was_high;

/************************************************************************/
/*									*/
/*		    FUNCTIONS PRIVATE TO THE BEACON TASK		*/
/*									*/
/************************************************************************/

typedef struct { float HighLimit, LowLimit, offset; } NVRAMinfo;

/************************************************************************/

private Boolean get_NVRAM_information (NVRAMinfo* resultptr)

    /* Updates the values of the parameters in NVRAM which we need to	*/
    /* know about.  Returns TRUE iff all values successfully obtained.	*/

    {
    beacon_details_struct Limits;
    int satellite_number;
    satellite_details_struct satellite_details;
    Boolean success;

    /* Getting the limits is easy, but getting the beacon offset is a	*/
    /* little trickier.  The following code should work when we are	*/
    /* tracking a satellite; not sure what to do in star-tracking mode.	*/

    success = get_beacon_details (&Limits)
	&& get_current_satellite_num (&satellite_number)
	&& get_satellite_details (&satellite_details, satellite_number);

    if (success)
	{
	SlewLimit = Limits.slew_limit;
	resultptr->HighLimit = Limits.hi_limit;
	resultptr->LowLimit = Limits.lo_limit;
	resultptr->offset = satellite_details.beacon_offset;

	if (GetScreen (BEACON_PAGE, &Display))
	    {
	    gotoxy (14,8);  cprintf("%10.6f", resultptr->offset);
            gotoxy (14,9);  cprintf("%10.6f", resultptr->LowLimit);
            gotoxy (14,10);  cprintf("%10.6f", resultptr->HighLimit);
            gotoxy (14,11);  cprintf("%10.6f", SlewLimit);
	    ReleaseScreen (&Display);
	    }
	}
    return success;
    }

/************************************************************************/

private void SendCommand (unsigned char ChannelCode)

    /* Sends a command to the beacon receiver.  Also flushes the	*/
    /* beacon task mailbox to get rid of any rubbish which might have	*/
    /* have accumulated as the result of a communications error after	*/
    /* the last command.						*/

    {
    unsigned char message[2];
    unsigned int dummy_length;
    unsigned char dummy_message[9];

	message[0] = ChannelCode;
    message[1] = CR;

	/*    flush_mbx();	*/
    /* Call to flush_mbx commented out by RHM 6/11/91 as this		*/
	/* procedure in UNOS does not function correctly at pressent.	*/
    /* The following lines of code were added to perform this function.	*/
    /*	PRESUMPTION: the message length returned from the mailbox is	*/
    /* less than [9] characters.					*/
	while ( used_mbx ( chBeaconTaskName ) > 0 )
	 {
	 rcv_mess ( dummy_message, &dummy_length, 1);
	 }
    /* In simulation mode, send a poll both to the real serial	*/
    /* port and to the beacon receiver simulator.  In real	*/
	/* mode, poll only the real hardware.			*/

    if ( !hardware_mode_is_real() )
	{
	if ( free_mbx(chBeaconSimulatorTaskName ) > 1 )
		send_mess (message, 2, chBeaconSimulatorTaskName);
	else
	    {
	    error_set (BEACON_COMMS_ERROR);
	    if (GetScreen (BEACON_PAGE, &PollScreen))
		{
		cprintf ("\n\rProblem in getting a mailbox");
		ReleaseScreen (&PollScreen);
		}
	    }
	}

    /* This next part is the poll to the real hardware, and we do it in	*/
    /* either real or simulation mode.  The presumption is that in	*/
    /* simulation mode there will be no response because the hardware	*/
    /* is not there to receive the poll.				*/

	if ( free_mbx(ch_3_tx) > 1 )
		send_mess ( message, 2, ch_3_tx );
    else
	{
	error_set (BEACON_COMMS_ERROR);
		if (GetScreen (BEACON_PAGE, &PollScreen))
			{
			cprintf ("\n\rProblem in getting a mailbox");
            ReleaseScreen (&PollScreen);
            }
        }

    }

/************************************************************************/

private ErrorCode AssembleResponse (unsigned char* reply)

    /* We expect a message from the receiver which is precisely seven	*/
    /* characters followed by a carriage return.  The nature of the	*/
    /* serial interface is such that the input is sent to us a little	*/
    /* at a time (usually a character at a time).  The function of this	*/
    /* procedure is to put it together, to check that the message is	*/
    /* exactly the right length and that the carriage return occurs at	*/
    /* the right place, and to report communications errors.		*/
    /* Note: reply[0] is unused, it's a relic from an earlier version	*/
    /* of this module where reply[0] was a status indicator.		*/

    {
    unsigned int reply_length, wanted, place, dummy;
    unsigned char CRfound;
	char *pchDummy;

    wanted = 8;  place = 1; CRfound = 0;

    /* addition by RHM 30/1/92: to ensure the maintenance display	*/
    /* is sensible, clear the reply string prior to filling it from	*/
    /* the serial channel. In this way, protocol, timeout errors	*/
    /* etc. give a more sensible display.				*/

    for ( dummy=0; dummy < wanted; dummy++ )
	reply[dummy] = 0;
    reply[0] = ' ';

    /* addition to while loop condition by RHM 6/11/91.			*/
    /* the while loop is now also terminated if a CR is detected.	*/
    /* This seems to be required for correctly "sychronising" the	*/
    /* communications with the Beacon receiver.				*/

    while ( ( wanted > 0 ) && ( ! CRfound ) )
	{
	/* Allow 1 tick for the reply to arrive; it should be almost	*/
        /* instantaneous, since we have already slept for half a second.*/

	pchDummy = rcv_mess (&(reply[place]), &reply_length, 1);
	if (pchDummy == NULL)
	    return timeout_error /* was dummy (DJLB strikes again!)*/ /*comms_error*/;
	if (reply_length > wanted) return protocol_error;
	CRfound = ( reply[place] == CR );
	place += reply_length;
	wanted -= reply_length;
	}

    if ( reply_length < wanted ) /* then we found a CR at the wrong place */
	return protocol_error;

    for (place = 1; place < 8; place++)
	if (reply[place] >= 0x80) return comms_error;
    if (reply[8] != CR) return protocol_error;

    return OK;
    }

/************************************************************************/

private ErrorCode PollResponse (unsigned char channel, unsigned char* reply)

    /* Called after a poll.  This procedure reads the eight-character	*/
    /* response from the beacon receiver, and checks for transmission	*/
    /* errors (including timeout), protocol errors, checksum error,	*/
    /* "wrong channel" error, and error codes passed back from the	*/
    /* beacon receiver.  Apart from these error checks, it does not do	*/
    /* any processing of the received data.				*/

    {
    unsigned char ch, checksum, errorcode;
    unsigned int count;
    ErrorCode status;

    status = AssembleResponse(reply);
    if (status != OK) return status;

    /* The following loop computes the checksum, also checks that the	*/
    /* first six bytes of the message are all ASCII digits.		*/

    /* RHM 30/1/92: Apparently, Mitec have changed the protocol. The	*/
    /* error code field is still a single character, but its not	*/
    /* a digit. Code changed to reflect this. Have no idea, though,	*/
    /* what the error code now means?					*/
    /* PJM 4/2/92: This problem now fixed, I hope.			*/

    for (checksum=0, count=1; count<=6; count++)
	{
	ch = reply[count];
	if ( ( !isascii(ch) ) || ( !isdigit(ch) && ( count != 5 ) ) )
		return protocol_error;
	checksum += ch;
	}

    checksum &= 0x3F;  checksum += 0x20;

    /* The message is now in, we have verified that there was no	*/
    /* communications error and that there was no premature carriage	*/
    /* return.  Now check for other errors.				*/

    if (reply[7] != checksum) return protocol_error;
    if (reply[6] != channel)
	{
	disable();
	DesiredFrequency = channel;  ChannelChangeRequired = TRUE;
	enable();
	return wrong_channel;
	}

    /* New error code field as notified to us 3/2/92: this field is	*/
    /* 01xxxxxx where any x=1 indicates an error condition.  The least	*/
    /* significant bit is "local selected" which we do not necessarily	*/
    /* consider to be an error.						*/

    errorcode = reply[5];
    if ((errorcode & 0xC0) != 0x40) return protocol_error;
    if (errorcode & 0x6) return beacon_poll_error;
    if (errorcode & 0x8) return oscillator_out_of_lock;
    if (errorcode & 0x10) return beacon_level_unavailable;
    if (errorcode & 0x20)
	{
	if (reply[1] == '0') return beacon_level_overflow;
	else return beacon_level_underflow;
	}

    /* Note that we ignore the receiver error bit 1 (local selected)	*/
    /* since we don't care whether the receiver is in local or remote	*/
    /* mode as long as the actual channel matches the desired channel.	*/

    return OK;
    }

/************************************************************************/

private ErrorCode DecodeLevel (unsigned char* message)

    /* Takes the four-character level code in message[1..4], converts	*/
    /* it to a double float, stores it in Levels[SampleNumber], and	*/
    /* stores its square in Squares[SampleNumber].  Checks for datum	*/
    /* out of range and for errors in fetching NVRAM information.	*/

    {
    NVRAMinfo limits;
    double newdatum;
    ErrorCode status;

    status = OK;

    /* Pick up the NVRAM data which we need for error checking.	*/

    if (!get_NVRAM_information(&limits)) return nvram_error;

    /* Convert the four-character response into a floating point	*/
    /* number.  The format is implicitly -99.99 dB.  Note that a	*/
    /* "beacon offset" adjustment is subtracted from the received value.*/

    {
    unsigned int j;

    for (newdatum = 0.0, j=1; j<=4; j++)
	{
	newdatum *= 10.0;
	newdatum += message[j] - '0';
	}
    }
    newdatum = -newdatum/100.0 - limits.offset;

    /* Check for level outside the acceptable range.	*/

    if (newdatum > limits.HighLimit)
    	{
	status = out_of_range;
	if (!Beacon_was_high)
	    { error_set (BEACON_IS_HIGH);  Beacon_was_high = TRUE; }
        }
    else if (Beacon_was_high)
	{ error_reset (BEACON_IS_HIGH);  Beacon_was_high = FALSE; }

    if (newdatum < limits.LowLimit)
    	{
	status = out_of_range;
	if (!Beacon_was_low)
	    { error_set (BEACON_IS_LOW);  Beacon_was_low = TRUE; }
        }
    else if (Beacon_was_low)
	{ error_reset (BEACON_IS_LOW);  Beacon_was_low = FALSE; }

    /* Store the newly acquired value.	*/

    if (GetScreen (BEACON_PAGE, &PollScreen))
    	{
	cprintf ("%7.2f", newdatum);
	ReleaseScreen (&PollScreen);
	}

    Levels[SampleNumber] = newdatum;
    Squares[SampleNumber] = newdatum*newdatum;

    return status;
    }

/************************************************************************/

private void ShowStatus ( unsigned int status )

	/* Prints (using the current cursor location), a message  */
	/* indicating the status passed in. MUST ONLY EVER BE	*/
	/* CALLED IF WE HAVE THE SCREEN.	RHM 30/1/92	*/
	{

	switch ( status )
	{
		case OK:
			cprintf(" OK      "); break;
		case timeout_error:
			cprintf(" Timeout "); break;
		case comms_error:
			cprintf(" Comms    "); break;
		case protocol_error:
			cprintf(" Protocol "); break;
		case wrong_channel:
			cprintf(" Channel  "); break;
		case beacon_poll_error:
			cprintf(" Poll Err "); break;
		case oscillator_out_of_lock:
			cprintf(" Osc Lock "); break;
		case beacon_level_unavailable:
			cprintf(" Unavail  "); break;
		case beacon_level_overflow:
			cprintf(" O/flow   "); break;
		case beacon_level_underflow:
			cprintf(" U/flow   "); break;
		case not_enough_data:
			cprintf(" data #   "); break;
		case out_of_range:
			cprintf(" range    "); break;
		case nvram_error:
			cprintf(" NVRAM    "); break;
		case slew_rate_error:
			cprintf(" rate     "); break;
		case polariser_operated:
			cprintf(" polariser"); break;
		default:
			cprintf(" ?????????");
	}
	}


/************************************************************************/

private void UpdateEstimates (ErrorCode status)

    /* Computes the mean level, rate, etc. from the Levels and Squares	*/
    /* data (see BEACON.H for a more detailed description of what	*/
    /* really needs to be computed).  The input parameter specifies the	*/
    /* error code to be stored with the computed estimate.		*/

    {
    beacon_struct estimate;
    double mean, sumsq, w;
    static double ratescale =
     12.0 / (BeaconSamplingInterval * (BeaconSampleSize*BeaconSampleSize-1));

    estimate.error = status;

    /* From the buffered data, compute the mean, sum of squares, and	*/
    /* weighted sum of values.						*/

    {
    unsigned int j;
    int weight = BeaconSampleSize - SampleNumber;

    for (mean = sumsq = w = 0.0, j = 0;  j < BeaconSampleSize;  j++)
	{
	mean += Levels[j];
	sumsq += Squares[j];
	w += weight*Levels[j];
	weight++;
	if (weight > BeaconSampleSize) weight = 1;
	}
    }

    mean /= BeaconSampleSize;
    w /= BeaconSampleSize;

    estimate.average = mean;

    /* The least squares estimate for the slope turns out to be (after	*/
    /* manipulating the usual least squares formulae into a form	*/
    /* suitable for efficient computation):				*/
    /*				w - ((N+1)/2)*(new mean)		*/
    /*			a   =   ------------------------		*/
    /*				    (T/12)*(N*N-1)			*/
    /* where N is the number of points over which we are doing the fit,	*/
    /* T is the sampling interval, and w is (1/N)sum(i*yi).		*/

    estimate.rate = ratescale*(w - 0.5*(BeaconSampleSize+1)*mean);

    /* Check whether the rate is excessive.  Note that we do not report	*/
    /* this as an error (that is the responsibility of function		*/
    /* get_beacon), we simply note the condition in global variable	*/
    /* RateExcessive.							*/

    disable();
    RateExcessive = (fabs(estimate.rate) > SlewLimit);
    enable();

    /* Now calculate the variance of the mean.  The formula is		*/
    /*					   2      _2			*/
    /*				     sum (x ) - N x			*/
    /*		variance of mean  =  ---------------			*/
    /*                                  N (N - 1)			*/

    /* Modified by RHM 12/12/91 so that we pass the square root of the	*/
    /* variance (i.e. r.m.s. value), consistent with Orbit Track.	*/

    estimate.rms = sqrt ( fabs ( (sumsq - BeaconSampleSize*mean*mean)
		/ (BeaconSampleSize*(BeaconSampleSize-1)) ) );

    estimate.error = status;

    /* End of calculation, store the results into the shared data area.	*/

    disable();  CurrentEstimate = estimate;  enable();

    /* Display the results on the maintenance page.	*/

    if (GetScreen (BEACON_PAGE, &Display))
	{
	gotoxy (14,3);  cprintf("%10.6f", estimate.average);
        gotoxy (14,4);  cprintf("%10.6f", estimate.rms);
        gotoxy (14,5);  cprintf("%10.6f", estimate.rate);
	gotoxy (16,6);  cprintf("%u", estimate.error);
	/* Addition by RHM 30/1/92: Add in code to display the error	*/
	/* type, not just show a number.				*/
	ShowStatus ( estimate.error );
	ReleaseScreen (&Display);
	}

    }

/************************************************************************/

private void Sleep (float seconds)

    /* Puts the caller to sleep for the specified number of seconds.	*/
    /* We do this by performing a timed wait on a semaphore which is	*/
    /* never signalled.							*/

	{
	timed_wait (beacon_task_sleep_sem, TICKS_PER_SECOND*seconds);
	}

/*
*******************************************************************************
get_current_beacon_number ()

Routine to get the beacon selected from NVRAM.

********************************************************************************
*/
unsigned int get_current_beacon_number ( void ) {

	satellite_details_struct sat_details;
	unsigned char sat_num;

	get_current_satellite_num ( &sat_num );
	get_satellite_details ( &sat_details, sat_num );

	return ( sat_details.beacon_select );

} // end of get_current_satellite_number()


/************************************************************************/
/*									*/
/*			      THE BEACON TASK				*/
/*									*/
/* Runs in a continuous loop (approx. 0.5 seconds per loop execution)	*/
/* polling the beacon receiver each time around, and updating the	*/
/* record of short-term mean beacon level, etc., which the client	*/
/* modules want to know about.						*/
/* Installed as a separate task, at priority level intermediate		*/
/*									*/
/************************************************************************/

void BeaconTask ( void *Dummy)

	{
	unsigned char CurrentFrequency = '0';
	unsigned char newfrequency;
    Boolean ChannelChangeFlag;
    ErrorCode status, DLstatus, LastErrorCode;
    unsigned char ReceiverResponse[9];

    /* LastErrorPoint gives the last buffer position where the data are	*/
    /* known to be faulty and LastErrorCode identifies the error kind.	*/
	/* If the whole buffer is OK then LastErrorCode = OK and the value	*/
    /* of LastErrorPoint is irrelevant.  The initial value of TRUE for	*/
    /* ChannelChangeRequired ensures that the very first sample will	*/
    /* produce a non-OK error code (usually not_enough_data, unless a	*/
    /* more serious error is found), which ensures that initially the	*/
	/* data are considered invalid until the first time that a complete	*/
    /* buffer-full of data is available.				*/

    unsigned int LastErrorPoint;

	/***************   END OF BEACON TASK LOCAL VARIABLES ***************/

    enable();
	Dummy=Dummy;
	Beacon_was_low = FALSE;  error_reset (BEACON_IS_LOW);
    Beacon_was_high = FALSE;  error_reset (BEACON_IS_HIGH);
	ReceiverResponse[0] = ' ';
    make_sticky_error (BEACON_IS_LOW);
    make_sticky_error (BEACON_IS_HIGH);


	if( (beacon_task_sleep_sem = create_semaphore()) != 0xffff)
		init_semaphore (beacon_task_sleep_sem, 0, 1 );
	if( beacon_task_sleep_sem == 0xffff )
	{
		cprintf("\n\rCannot generate showtime semaphores.\n\r");
		exit(1);
	}

 	/* Attach to serial channel 3 receiver task. */
	//send_mess( (unsigned char *)"?",1, ch_3_rx );

    while (TRUE)
	{
        /* Perform a poll, which might involve a frequency change.	*/

	ChannelChangeFlag = ChannelChangeRequired;
	if (ChannelChangeFlag)
	    {
	    disable();
	    newfrequency = DesiredFrequency;
	    ChannelChangeRequired = FALSE;
	    enable();
		SendCommand (newfrequency);
	    CurrentFrequency = newfrequency;
	    }
	    /* Note: changing the frequency gives an implied poll of	*/
	    /* the new channel.						*/

	else
	    SendCommand ('?');

        /* Time delay to give approximately the correct polling rate.	*/
        /* This delay incidentally gives ample time for the receiver	*/
        /* to have responded to the poll.				*/

	Sleep (BeaconSamplingInterval - 0.05);

        /* Fetch the poll response, checking for comms errors etc.	*/

	status = PollResponse (CurrentFrequency, ReceiverResponse);

	if (GetScreen (BEACON_PAGE, &PollScreen))
            {
    	    cprintf ("\n\r%.8s", &ReceiverResponse);
            ReleaseScreen (&PollScreen);
	    }

        /* Convert the response to a numerical level, and check for	*/
        /* beacon strength too high or too low.				*/

	DLstatus = DecodeLevel (ReceiverResponse);
	if (status == OK) status = DLstatus;

        /* Process any errors. */

        if (status == OK)
	    {
	    if (the_polariser_is_in_use()) status = polariser_operated;
	    else if (ChannelChangeFlag) status = not_enough_data;
	    }

	/* Extra call added by RHM to show the status of each poll */
	if ( ( status != OK ) && (GetScreen (BEACON_PAGE, &PollScreen)) )
	    {
	    ShowStatus ( status );
	    ReleaseScreen (&PollScreen);
	    }

	if (status != OK) switch (status)
	    {
	     case timeout_error:
		error_set (BEACON_RESPONSE_TIMEOUT);  break;
	     case comms_error:
		error_set (BEACON_COMMS_ERROR);  break;
	     case protocol_error:
		error_set (BEACON_PROTOCOL_ERROR);  break;
	     case beacon_poll_error:
		error_set (BEACON_POLL_ERROR);  break;
	     case oscillator_out_of_lock:
		error_set (BEACON_OSCILLATOR_OUT_OF_LOCK);  break;
	     case beacon_level_unavailable:
		error_set (BEACON_LEVEL_UNAVAILABLE);  break;
	     case beacon_level_overflow:
		error_set (BEACON_LEVEL_OVERFLOW);  break;
	     case beacon_level_underflow:
		error_set (BEACON_LEVEL_UNDERFLOW);  break;
	     case nvram_error:
		error_set (BEACON_DATA_NVRAM_ERROR);  break;
	    }

        /* Note that the following two lines update LastErrorCode if	*/
        /* there is an error in the current sample OR if we have	*/
        /* wrapped around far enough in the buffer to overwrite the	*/
        /* previous last error.						*/

        if (status != OK) LastErrorPoint = SampleNumber;
        if (LastErrorPoint == SampleNumber) LastErrorCode = status;

        /* Compute mean level, variance, slope; then increment the	*/
        /* sample number modulo the buffer size.			*/

	UpdateEstimates (LastErrorCode);
	SampleNumber++;  SampleNumber %= BeaconSampleSize;
	}
	}

/*
*******************************************************************************
return_beacon_number ( )

Routine to return the beacon number. For display purposes.

*******************************************************************************
*/
unsigned char return_beacon_number ( void ) {
	unsigned int temp;

	disable ();
	temp = DesiredFrequency - '0';
	enable ();

	return temp;

} // end of return_beacon_number
