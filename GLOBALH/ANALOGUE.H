/****************************************************************************
* POLYTRON VERSION CONTROL SOFTWARE _ PVCS (tm) Information
*
*      $Revision:   2.3  $
*      $Date:   07 Jan 1994 16:16:16  $
* 
*****************************************************************************
*****************************************************************************
* Tracksat Project.
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
* MODULE :- BEACON                  File : ANALOGUE.H
****************************************************************************/


#ifndef __ANALOGUE_H
#define __ANALOGUE_H

/************************************************************************/
/*									*/
/*				ANALOGUE.H				*/
/*									*/
/*		Analogue input, for reading the beacon signal		*/
/*									*/
/*									*/
/*	Programmer:	P. Moylan					*/
/*	Last modified:	18 May 1992					*/
/*	Status:		OK						*/
/*									*/
/************************************************************************/

#include <general.h>	/* FROM General IMPORT BYTE */

void StartBeaconSampling (double frequency);

    /* Starts a timer-controlled analogue sampling operation.  This	*/
    /* function should be called when the beacon task is ready to start	*/
    /* collecting data - i.e. after the task has started up.  Parameter	*/
    /* frequency, which is in units of samples per second, specifies	*/
    /* the actual A/D sampling rate.  The rate at which FetchSample	*/
    /* (below) returns results is somewhat lower, and is specified in	*/
    /* BEACON.H.  This is because a "sample" returned by FetchSample is	*/
    /* in fact the average of a number of physical samples.  The reason	*/
    /* for this dual-rate approach is to compensate for the lack of an	*/
    /* anti-aliasing filter in the hardware.				*/

double FetchSample (void);

    /* Waits until the next sample is available, then returns it.  (It	*/
    /* is assumed that StartBeaconSampling has been called - otherwise	*/
    /* this function will never return.)  Data overrun errors are	*/
    /* reported to the Alarm module, but not to the caller.		*/
    /* The result is in the range -10.0 to +9.995			*/

void SetBeaconChannel (BYTE newchannel);

    /* Change the analogue input channel.	*/

BOOLEAN InitBeaconInput (BOOLEAN SimulateBeacon);

    /* Must be called at the start of program execution.  If the result	*/
    /* FALSE is returned, initialization has failed.			*/

#endif /* __ANALOGUE_H */
