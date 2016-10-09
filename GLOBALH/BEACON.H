/***************************************************************************
* POLYTRON VERSION CONTROL SOFTWARE _ PVCS (tm) Information
*
*      $Revision:   2.3  $
*      $Date:   07 Jan 1994 16:16:36  $
*
*****************************************************************************
*****************************************************************************
* Tracksat  Project.
*****************************************************************************
* Copyright (c) 1992. The University of Newcastle Research Associates Ltd.
*****************************************************************************
*
* PROGRAMMER :-
*                   Industrial Electronics Division,
*                   TUNRA Ltd.,
*                   University of Newcastle, NSW 2308.
*
* FUNCTION REFERENCE :-
*
*****************************************************************************
*****************************************************************************
* MODULE :-  GLOBALH                 File :  BEACON.H
****************************************************************************/


#ifndef __BEACON_H
#define __BEACON_H

/************************************************************************/
/*									*/
/*				BEACON.H				*/
/*									*/
/*			BEACON INTERFACE MODULE				*/
/*									*/
/*	Programmer:	P. Moylan					*/
/*	Last modified:	18 May 1992					*/
/*	Status:		OK						*/
/*									*/
/*	Note that the functions supplied by this module form a		*/
/*	redundant set.  There are times where the external callers	*/
/*	want only the beacon strength but not the status, other		*/
/*	times when the status is wanted as well, etc.; and it is	*/
/*	convenient to provide multiple functions rather than force	*/
/*	the callers to accept information they don't need.		*/
/*									*/
/************************************************************************/

/* Sampling parameters.	*/

#define BeaconSamplingInterval 0.5		/* seconds per sample */
#define BeaconSampleSize 7	/* # of samples over which we average */

/* The following constant is used by the Orbit Track module to know	*/
/* how long to wait, after a move or channel change, before the beacon	*/
/* signal is stable again.						*/

#define BEACON_SETTLING_TIME	BeaconSamplingInterval*BeaconSampleSize

/* A status indication of type ErrorCode is treated by some callers	*/
/* as a BOOLEAN.  For such callers OK is mapped to FALSE and everything	*/
/* else is mapped to TRUE.						*/
#define BOOLEAN unsigned char

// typedef enum { OK, level_low, level_high, not_enough_data } ErrorCode;
typedef enum {	OK, level_low, level_high, timeout_error, comms_error,
		protocol_error, wrong_channel,
		beacon_poll_error, oscillator_out_of_lock,
		beacon_level_unavailable, beacon_level_overflow,
		beacon_level_underflow, not_enough_data, out_of_range,
		nvram_error, slew_rate_error, polariser_operated }
								ErrorCode;

typedef struct
	{
		double average;
		double rms;
		double rate;
                BOOLEAN RateHigh;
		ErrorCode error;
					} beacon_struct;

/************************************************************************/
/*									*/
/* This structure is returned by the Beacon module function get_beacon.	*/
/* The variables in the above definition are as follows:		*/
/*	average	: average beacon level (db) over the past 7 samples	*/
/*	rms	: an estimate of the standard deviation of the average,	*/
/*		  based on the assumption that the past 7 samples were	*/
/*		  drawn from a Gaussian distribution.			*/
/*	rate	: the rate of change of the beacon signal over the past	*/
/*		  7 samples obtained by a least squares line of best	*/
/*		  fit.  (Added to the definition on 17/11/90 by RHM).	*/
/*	RateHigh: TRUE iff the magnitude of the rate of change is	*/
/*		  above the current slew limit.  This is reported	*/
/*		  separately from the error code since a high rate of	*/
/*		  change is not always considered to be an error.	*/
/*	error	: summary error for beacon. error = OK means no error,	*/
/*		  anything else means an error. The error type is NOT	*/
/*		  decoded by the Orbit Track task, and a code not	*/
/*		  equal to OK will cause the orbit track task to	*/
/*		  return to passive tracking. The Beacon task is to	*/
/*		  "handle" the error in the sense of causing the	*/
/*		  appropriate alarms etc. to be raised.			*/
/*									*/
/*	Any one of the following conditions will cause the beacon	*/
/*	task to return a non-zero error code:				*/
/*		(a) fewer than 7 samples have been collected since	*/
/*		    the last time the beacon channel was changed;	*/
/*		(b) any of the last 7 samples above the specified	*/
/*		    maximum threshold or below the minimum threshold	*/
/*		    (even if the average is inside the thresholds);	*/
/*									*/
/*	Note that condition (b) can be triggered several times during	*/
/*	the last 7 samples.  In the case of multiple errors, the error	*/
/*	code reported is that corresponding to the most recent error.	*/
/*									*/
/************************************************************************/

beacon_struct get_beacon (int probing);

    /* Returns the entire beacon structure, i.e. signal average over	*/
    /* the last 3.5 seconds, together with rate, rms, etc.		*/

unsigned char BeaconStatusCode (void);

    /* Returns an octal digit ('0' to '7') where bit 0 denotes beacon	*/
    /* low, bit 1 denotes beacon high, and bit 2 denotes a high rate of	*/
    /* change.  (Note that the encoding is distinct from the ErrorCode	*/
    /* type.)  Errors other than the above three are not reported,	*/
	/* since the CMU command protocol (which we are not at liberty to	*/
    /* change) does not recognise any other sort of error.		*/

double GetBeaconLevel (void);

    /* Returns the beacon strength (in db) averaged over approximately	*/
    /* the past three and a half seconds, and adjusted by the amount	*/
    /* specified as the beacon offset for the current satellite.	*/

BOOLEAN InitBeacon (BOOLEAN Simulating);

    /* Must be called at the start of program execution.  If the result	*/
    /* FALSE is returned, initialization has failed.  The parameter, if	*/
    /* TRUE, specifies that a beacon simulator be used, rather than the	*/
    /* genuine beacon hardware.						*/

extern void RefreshDisplayPage ( void );

void WriteBeaconError ( void );


#endif /* __BEACON_H */
