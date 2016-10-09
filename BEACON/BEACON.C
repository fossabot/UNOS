/*
*****************************************************************************
* Copyright (c) 1991. The University of Newcastle Research Associates Ltd.
*****************************************************************************
*
* PROGRAMMER :-     
*                   Industrial Electronics Division,
*                   TUNRA Ltd.,
*                   University of Newcastle, NSW 2308.
*
*****************************************************************************
*****************************************************************************
* MODULE :- BEACON                  File :  BEACON.C
****************************************************************************/

/************************************************************************/
/*																		*/
/*						BEACON.C - BEACON INTERFACE						*/
/*																		*/
/*		Programmer:			P. Moylan									*/
/*		Last modified:		26 May 1992									*/
/*		Status:				Working.									*/
/*																		*/
/*				Tested except for A/D calibration.						*/
/*																		*/
/************************************************************************/

/*
Modified by RHM 7/9/92:
	To avoid problems with dynamic priorities, the protection mechanism
	for both "DataAccess" and "LimitsCopy" has been altered. The mechanism
	is now "centralised" in the sense that there are pairs of procedures:
		ProtectDataAccess and UnprotectDataAccess
		ProtectLimitsCopy and UnprotectLimitsCopy
	which perform all the protection operations required. Also, because of
	the difficulties we've been having with dynamic priorities, and nesting
	of semaphore operations etc., we may have to (in the short term) change
	the protection mechanism.
*/                               


#include <general.h>	/* Import TASK_STACK_SIZE definition */
#include <beacon.h>		/* Import my own definition module.				*/
#include <unos.h>		/* FROM UNOS IMPORT timed_wait,					*/
						/*		create_semaphore, init_semaphore		*/
						/*		create_task, PRIORITY_Q_TYPE			*/
						/* enable_task_switch, disable_task_switch		*/
#include <math.h>		/* FROM Math IMPORT fabs, sqrt					*/

#include <nvramext.h>		/* FROM STORE IMPORT beacon_details_struct,		*/
#include "analogue.h"
						/* FROM Analogue IMPORT StartBeaconSampling,	*/
						/*		SetBeaconChannel, FetchSample,		    */
						/*		InitBeaconInput						    */
#include <err_nums.h>	/* FROM ErrorHandler IMPORT			*/
#include <err_ex1.h>	/*	error_set, error_reset.			*/
#include "kbtask.h"		// Import BEACON_PAGE
#include "bcn_scrn.h"	// Import GetScreen() ReleasScreen()

/************************************************************************/
/*				FOR SCREEN OUTPUT TO MAINTENANCE PAGE					*/
/************************************************************************/

#include "simbeaco.h"
						/* FROM SimBeacon IMPORT						*/
						/*		init_beacon_simulator_screen			*/
#include <cmclock.h>	/* FROM CmClock IMPORT ShowTime					*/

#include <conio.h>		/* FROM conio IMPORT cprintf, gotoxy, text_info	*/
						/*		clreol									*/
#include "pars_td1.h"
#include "main_ext.h"	/* IMPORT getsimulation() */
#include "scrext.h"		// Import display_function_keys()


#define Boolean unsigned char



/************************************************************************/
/*																		*/
/*				      VARIABLES GLOBAL TO THIS MODULE					*/
/*																		*/
/************************************************************************/

/* Some of the data in this group of declarations are shared between	*/
/* the beacon task and the clients of this module, therefore critical	*/
/* section protection is needed.  In the present version, we use		*/
/* semaphores to protect critical sections.								*/

/* Task name, used in creating task.		*/

private char BeaconTaskName[] = "Beacon Task";

/* CurrentEstimate contains our latest update of the data which may		*/
/* be requested by client modules.  Critical sections protected by		*/
/* semaphore DataAccessSem.												*/

private beacon_struct CurrentEstimate = { 0.0, 0.0, 0.0,
										  FALSE, not_enough_data };

/* Semaphore to protect access to the CurrentEstimate structure.		*/

private unsigned int DataAccessSem;

/* Limits most recently read from STORE.  We take two copies of this		*/
/* (the "master" copy is private to the beacon task, the shared copy		*/
/* is declared here) to reduce the chance of blocking the beacon task		*/
/* while the screen is being updated.										*/

typedef struct
		{ float HighLimit, LowLimit, offset, scalefactor, SlewLimit; }
														STOREinfo;

private STOREinfo LimitsCopy;

/* Semaphore to protect access to the LimitsCopy structure.		*/

private unsigned int LimitsCopySem;

/* A flag to say whether we are simulating the beacon receiver.		*/

private BOOLEAN Simulating;
private Boolean RateExcessive = FALSE;

/************************************************************************/
/*																																				*/
/*										Centralised Protection Mechanisms										*/
/*																																				*/
/*		The previous protection mechanism (wait and signal on DataAccessSem		*/
/*		and LimitsCopySem) caused occasional sequencer over-run problems		*/
/*		as we do not yet have the dynamic priorities properly sorted out.		*/
/*				RHM 8/9/92.																												*/
/*																																				*/
/************************************************************************/
// 0 = Black
// 1 = Blue
// 2 = Green
// 3 = Cyan
// 7 = LightGray
// 9 = LightBlue
// 128 = Blink

private struct text_info WholeScreen = {1, 1, 80, 24, 0x71, 0x71};
private struct text_info Display = {2, 3, 35, 10, 0x09, 0x09};
private struct text_info STOREWindow = { 40, 3, 70, 15, 0x13, 0x13};
private struct text_info ChanSatWindow = { 2, 12, 35, 23, 0x09, 0x09};
private struct text_info PollScreen = {40, 17, 70, 23, 0x13, 0x13};

private void ProtectDataAccess ( void )
{
		disable_task_switch ();
}

private void UnprotectDataAccess ( void )
{
		enable_task_switch ();
}

private void ProtectLimitsCopy ( void )
{
		disable_task_switch ();
}

private void UnprotectLimitsCopy ( void )
{
		enable_task_switch ();
}


/************************************************************************/
/*																		*/
/*				    INTERFACE TO THE CLIENT MODULES						*/
/*																		*/
/************************************************************************/

unsigned char BeaconStatusCode (void)

		/* Returns an octal digit ('0' to '7') where bit 0 denotes beacon	*/
		/* low, bit 1 denotes beacon high, and bit 2 denotes rate error.	*/
		/* (Note that the encoding is distinct from the ErrorCode type.)	*/
		/* Errors other than the above three are not reported, since the	*/
		/* CMU command protocol (which we are not at liberty to change)		*/
		/* does not recognise that any other sort of error can exist.		*/

		{
		beacon_struct estimate;
		STOREinfo limits;
		unsigned char result = '0';

		ProtectDataAccess ();
    estimate = CurrentEstimate;
		UnprotectDataAccess ();

		ProtectLimitsCopy ();
	limits = LimitsCopy;
		UnprotectLimitsCopy ();

	/* We re-do the check on the high and low limits, rather than rely		*/
	/* on the CurrentEstimate.error code, because the error code might		*/
	/* indicate an error not recognised by the CMU protocol.				*/

    if (estimate.average < limits.LowLimit) result += 1;
    else if (estimate.average > limits.HighLimit) result += 2;

    if (estimate.RateHigh) result += 4;

    return result;
    }

/************************************************************************/

double GetBeaconLevel (void)

	/* Returns the beacon strength (in db) averaged over approximately		*/
	/* the past three and a half seconds, and adjusted by the amount		*/
	/* specified as the beacon offset for the current satellite.			*/

    {
    double result;

		ProtectDataAccess ();
	result = CurrentEstimate.average;
		UnprotectDataAccess ();

	return result;
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

//beacon_struct get_beacon (void)

	/* Returns the entire beacon structure, i.e. signal average over	*/
	/* the last 3.5 seconds, together with rate, rms, etc.				*/

/*	{
	beacon_struct result;

		ProtectDataAccess ();
	result = CurrentEstimate;
		UnprotectDataAccess ();

	return result;
	}
*/


/************************************************************************/
/*						MAINTENANCE PAGE OUTPUT							*/
/************************************************************************/

private void WriteErrorCode (ErrorCode code)

	/* Writes code to the screen.		*/
	{
	switch (code)
		{
		case OK:				cprintf ("OK       ");  break;
		case level_low:			cprintf ("Level Lo ");  break;
		case level_high:		cprintf ("Level Hi ");  break;
		case timeout_error:		cprintf ("Time-out ");  break;
		case comms_error:		cprintf ("Comms Err");  break;
		case protocol_error:	cprintf ("Prot Err ");  break;
		case wrong_channel:		cprintf ("Wrong Chn");  break;
		case beacon_poll_error:	cprintf ("Poll Err ");  break;
		case nvram_error:		cprintf ("NVRAM Err");  break;
		case slew_rate_error:	cprintf ("Slew Rate");  break;

		case not_enough_data:	cprintf ("Ch Change ");  break;
		}
    }

// Routine for use by others to display beacon errors
void WriteBeaconError ( ) {

	WriteErrorCode ( CurrentEstimate.error );

} // end of WriteBeaconError ()
/************************************************************************/

void RefreshDisplayPage ( )

	/* Refreshes the screen.  Called repeatedly as long as BEACON_PAGE		*/
	/* is the active maintenance page.										*/

	{

	if ( return_screen_page ( ) == BEACON_PAGE )
		{
		if (GetScreen (BEACON_PAGE, &WholeScreen))
			{
			clrscr();
			gotoxy (30,1);
			cprintf ("BEACON INFORMATION");
			gotoxy(42,16);cprintf ("RAW BEACON DATA - dB");
			gotoxy (1,24);
			display_function_keys ();
			ReleaseScreen (&WholeScreen);
			}
		if (GetScreen (BEACON_PAGE, &Display))
			{
			clrscr();
			cprintf ("     SMOOTHED BEACON DATA\n\n\r Average\n\r"
						" RMS\n\r Rate\n\r Status");
			ReleaseScreen (&Display);
				}
		if (GetScreen (BEACON_PAGE, &STOREWindow))
			{
			clrscr();
			cprintf( "    SIMULATOR VALUES");
			ReleaseScreen (&STOREWindow);
				}
		if (GetScreen (BEACON_PAGE, &ChanSatWindow))
			{
			clrscr();
			gotoxy ( 1,1 );cprintf("    BEACON SETUP  \n\r");
			cprintf (" Satellite:\n\r Beacon channel:\n\r");
			cprintf (" Offset:\n\r Scale factor:\n\r Low limit:\n\r"
						" High limit:\n\r Rate limit:");
			ReleaseScreen (&ChanSatWindow);
				}
		if (GetScreen (BEACON_PAGE, &PollScreen))
			{
			clrscr();  gotoxy (1, 7);
			ReleaseScreen (&PollScreen);
				}
		}


		while ( return_screen_page () == BEACON_PAGE )
		{
		//ShowTime (BEACON_PAGE);


		/* Display the limits read from STORE, and the latest estimates		*/
		/* of beacon strength, standard deviation, etc.						*/

		if (GetScreen ( BEACON_PAGE, &Display))
			{
			beacon_struct estimate;

				ProtectDataAccess ();
			estimate = CurrentEstimate;
				UnprotectDataAccess ();
			gotoxy (14,3);  cprintf("%10.6f", estimate.average);
			gotoxy (14,4);  cprintf("%10.6f", estimate.rms);
			gotoxy (14,5);  cprintf("%10.6f", estimate.rate);
			gotoxy (15,6);  WriteErrorCode (estimate.error);
			gotoxy (3, 7);
			if (estimate.RateHigh) cprintf ("*Rate limit exceeded*");
			else clreol();

			ReleaseScreen (&Display);
			}

		if (GetScreen (BEACON_PAGE, &STOREWindow))
			{
			ReleaseScreen (&STOREWindow);
			}

		if (GetScreen ( BEACON_PAGE, &ChanSatWindow))
			{
			STOREinfo Limits;

			BYTE satnum = GetCurrentSatelliteNumber();

			gotoxy (18, 2);  cprintf ("%2d", satnum);
			gotoxy (18, 3);  cprintf ("%2d", get_current_beacon_select (satnum));

			ProtectLimitsCopy (); Limits = LimitsCopy; UnprotectLimitsCopy ();
			gotoxy (18,4);  cprintf("%10.6f", Limits.offset);
			gotoxy (18,5);  cprintf("%10.6f", Limits.scalefactor);
			gotoxy (18,6);  cprintf("%10.6f", Limits.LowLimit);
			gotoxy (18,7);  cprintf("%10.6f", Limits.HighLimit);
			gotoxy (18,8);  cprintf("%10.6f", Limits.SlewLimit);

			ReleaseScreen ( &ChanSatWindow );
				}

		//if (Simulating) DisplaySimulatorLevels(BEACON_PAGE);
		} // while

	//#endif

	}

/************************************************************************/
/*																		*/
/*		THE FOLLOWING DATA ARE USED ONLY BY THE BEACON TASK AND BY		*/
/*						ITS SUBSIDIARY PROCEDURES						*/
/*																		*/
/************************************************************************/

/* We keep the receiver data in a circular buffer Levels.  At any given		*/
/* time SampleNumber gives the buffer position where we are going to		*/
/* put the next datum (except transiently, when we have just received		*/
/* a new datum and are processing it).  Note: the stored data in Levels		*/
/* have already been adjusted by the "beacon offset" from STORE.			*/

private unsigned int SampleNumber = 0;

private double Levels[BeaconSampleSize] =
								{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

/* Each element of array Squares holds the square of the corresponding		*/
/* element of Levels.  The only function of Squares is to speed up			*/
/* variance computations - i.e. the square of a value is computed only		*/
/* once rather than being repeatedly re-computed.							*/

private double Squares[BeaconSampleSize] =
								{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

/* Our own private copy of the limits read from STORE, to give us free		*/
/* access to the limits without having to compete with the screen task.		*/

private STOREinfo Limits;

/* The BOOLEAN flags Beacon_was_low and Beacon_was_high keep track of		*/
/* whether we have set the error conditions BEACON_IS_LOW and				*/
/* BEACON_IS_HIGH respectively.  Their only function is to allow us to		*/
/* avoid redundant setting or resetting of these error indications.			*/

private BOOLEAN Beacon_was_low, Beacon_was_high;

/************************************************************************/
/*																		*/
/*				    FUNCTIONS PRIVATE TO THE BEACON TASK				*/
/*																		*/
/************************************************************************/

private void get_STORE_information (STOREinfo* resultptr)

	/* Takes a copy of the values of the parameters in NVRAM which we		*/
	/* need to know about.													*/

    {
	beacon_details_struct Details;

	get_beacon_details (&Details);
    resultptr->HighLimit = Details.hi_limit;
	resultptr->LowLimit = Details.lo_limit;

	/* The scale factor (dB/V) is the reciprocal of the gradient (V/dB)		*/
	/* stored in STORE, so watch out for overflow.							*/

	if (fabs(Details.gradient) < 0.1)
		{
		if (Details.gradient >= 0.0) resultptr->scalefactor = 10.0;
		else resultptr->scalefactor = -10.0;
		}
    else resultptr->scalefactor = 1.0/Details.gradient;

	resultptr->offset = get_beacon_offset(GetCurrentSatelliteNumber ( ));
	resultptr->SlewLimit = Details.slew_limit;
    }

/************************************************************************/

private ErrorCode GetNextSample (double* pValue)

	/* Fetches the next sample, checking for out-of-range errors.		*/

    {
    ErrorCode status = OK;

	/* Pick up the raw datum.  Function FetchSample contains a pause	*/
	/* until the sample is ready, thus freeing us of the trouble of		*/
	/* doing anything about the sampling rate.						    */

    *pValue = FetchSample();

	/* Display the raw datum.		*/

	if (GetScreen (BEACON_PAGE, &PollScreen))
		{
		cprintf ("\n\r %8.4f", *pValue);
		ReleaseScreen (&PollScreen);
		}


	/* Update the two copies of the STORE information.		*/

	get_STORE_information (&Limits);
		ProtectLimitsCopy ();
    LimitsCopy = Limits;
	UnprotectLimitsCopy ();

	/* Scale the result and adjust by beacon offset.		*/

    *pValue *= Limits.scalefactor;
    *pValue -= Limits.offset;

	/* Check for out-of-range data.		*/

    if (*pValue > Limits.HighLimit)
		{
		status = level_high;
		if (!Beacon_was_high)
			{
			error_set (BEACON_IS_HIGH);
			Beacon_was_high = TRUE;
			}
		}
	else if (Beacon_was_high)
			{
			error_reset (BEACON_IS_HIGH);
			Beacon_was_high = FALSE;
			}

	if (*pValue < Limits.LowLimit)
		{
		status = level_low;
		if (!Beacon_was_low)
			{
			error_set (BEACON_IS_LOW);
			Beacon_was_low = TRUE;
			}
		}
	else if (Beacon_was_low)
		{
		error_reset(BEACON_IS_LOW);
		Beacon_was_low = FALSE;
		}

	return status;
	}

/************************************************************************/

private void UpdateEstimates (ErrorCode status)

	/* Computes the mean level, rate, etc. from the Levels and Squares	*/
	/* data (see BEACON.H for a more detailed description of what		*/
	/* really needs to be computed).  The input parameter specifies the	*/
	/* error code to be stored with the computed estimate.				*/

    {
    beacon_struct estimate;
	double mean, sumsq, w;
    static double ratescale =
     12.0 / (BeaconSamplingInterval * (BeaconSampleSize*BeaconSampleSize-1));

	/* From the buffered data, compute the mean, sum of squares, and	*/
	/* weighted sum of values.											*/

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
	/* manipulating the usual least squares formulae into a form		*/
	/* suitable for efficient computation):								*/
	/*								w - ((N+1)/2)*(new mean)			*/
	/*						a   =   ------------------------			*/
	/*								    (T/12)*(N*N-1)					*/
	/* where N is the number of points over which we are doing the fit,	*/
	/* T is the sampling interval, and w is (1/N)sum(i*yi).				*/

    estimate.rate = ratescale*(w - 0.5*(BeaconSampleSize+1)*mean);

	/* Check whether the rate is excessive.  Note that we do not report	*/
	/* this as an error, we simply note the condition in global			*/
	/* variable CurrentEstimate.RateHigh.								*/

    estimate.RateHigh = (fabs(estimate.rate) > Limits.SlewLimit);

	/* Now calculate the variance of the mean.  The formula is			*/
	/*										   2      _2				*/
	/*								     sum (x ) - N x				    */
	/*				variance of mean  =  ---------------				*/
	/*                                  N (N - 1)						*/

	/* Modified by RHM 12/12/91 so that we pass the square root of the	*/
	/* variance (i.e. r.m.s. value), consistent with Orbit Track.		*/

    estimate.rms = sqrt ( fabs ( (sumsq - BeaconSampleSize*mean*mean)
				/ (BeaconSampleSize*(BeaconSampleSize-1)) ) );

    estimate.error = status;

	/* End of calculation, store the results into the shared data area.	*/

		ProtectDataAccess ();
    CurrentEstimate = estimate;
		UnprotectDataAccess ();

    }

/************************************************************************/
/*																		*/
/*						      THE BEACON TASK							*/
/*																		*/
/* Runs in a continuous loop (approx. 0.5 seconds per loop execution)   */
/* polling the beacon receiver each time around, and updating the		*/
/* record of short-term mean beacon level, etc., which the client		*/
/* modules want to know about.											*/
/* Installed as a separate task, at priority level intermediate			*/
/*																		*/
/************************************************************************/

private void BeaconTask (void* DummyLocalVariable)

    {
    BYTE CurrentChannel = 99;
    BYTE newchannel;
    BOOLEAN ChannelChangeFlag;
    double newlevel;
    ErrorCode status, LastErrorCode;
		unsigned int LastErrorPoint;

		DummyLocalVariable = DummyLocalVariable;

	/* LastErrorPoint gives the last buffer position where the data are		*/
	/* known to be faulty and LastErrorCode identifies the error kind.		*/
	/* If the whole buffer is OK then LastErrorCode = OK and the value		*/
	/* of LastErrorPoint is irrelevant.  The initial illegal value for		*/
	/* CurrentChannel ensures that the very first sample will produce a		*/
	/* not_enough_data error code, which ensures that initially the			*/
	/* data are considered invalid until the first time that a complete		*/
	/* buffer-full of data is available.									*/

    Beacon_was_low = FALSE;  Beacon_was_high = FALSE;
	StartBeaconSampling (100.0 /* Hz */);

	if ( get_simulation () )
		GoBeaconSimulator ( );

    while (TRUE)
		{
		/* Get the desired analogue channel from STORE.  Normally it		*/
		/* will be the same as the last time around; if not, perform a		*/
		/* "channel change" operation.										*/

		newchannel = get_current_beacon_select ( GetCurrentSatelliteNumber());
		ChannelChangeFlag = CurrentChannel NE newchannel;
		if (ChannelChangeFlag)
			{
			SetBeaconChannel (newchannel);
            CurrentChannel = newchannel;
			}

		/* Fetch the next sample, checking for out-of-range errors.		*/
		/* This includes a wait until the next sample is available;		*/
		/* the sampling timing is controlled by the data provider, not	*/
		/* by this module.												*/

		status = GetNextSample (&newlevel);

		/* Store the result in the circular buffer.						*/

        Levels[SampleNumber] = newlevel;
        Squares[SampleNumber] = newlevel*newlevel;

        /* Process any errors. */

        if ((status EQ OK) AND ChannelChangeFlag) status = not_enough_data;

		/* Note that the following two lines update LastErrorCode if	*/
		/* there is an error in the current sample OR if we have		*/
		/* wrapped around far enough in the buffer to overwrite the		*/
		/* previous last error.											*/

        if (status NE OK) LastErrorPoint = SampleNumber;
        if (LastErrorPoint EQ SampleNumber) LastErrorCode = status;

		/* Compute mean level, variance, slope; then increment the		*/
		/* sample number modulo the buffer size.						*/

		UpdateEstimates (LastErrorCode);
		SampleNumber++;  SampleNumber %= BeaconSampleSize;
		}
    }

/************************************************************************/
/*						MODULE INITIALIZATION							*/
/************************************************************************/

BOOLEAN InitBeacon (BOOLEAN SimulateBeacon)

	/* Must be called at the start of program execution.  If the result	*/
	/* FALSE is returned, initialization has failed.					*/

    {
    Simulating = SimulateBeacon;

	/* Create semaphores.		*/
	LimitsCopySem = create_semaphore ();
    if (LimitsCopySem EQ 0xFFFF) return FALSE;
    init_semaphore (LimitsCopySem, 1, 1);

	DataAccessSem = create_semaphore ();
	if (DataAccessSem EQ 0xFFFF) return FALSE;
	init_semaphore (DataAccessSem, 1, 1);

	/* Initialize the analogue I/O module.		*/

	if (  !InitBeaconInput (SimulateBeacon)  ) return FALSE;

		/* Create the beacon task.		*/
		cprintf ("Creating Beacon Task...\n\r");
		if (!create_task ( BeaconTaskName, PRIORITY_3, 0, TASK_RUNNABLE,
								PRIORITY_Q_TYPE, 0, TASK_STACK_SIZE , 0,
						0, NULL, BeaconTask, NULL ))
														return FALSE;

	return TRUE;
	}
