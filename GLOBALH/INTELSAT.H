/****************************************************************************
* POLYTRON VERSION CONTROL SOFTWARE _ PVCS (tm) Information
*
*      $Revision:   2.3  $
*      $Date:   07 Jan 1994 16:16:52  $
* 
*****************************************************************************
****************************************************************************
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
* MODULE :- GLOBALH                  File : INTELSAT.H
****************************************************************************/

/************************************************************************

	INTELSAT.H

	Author:     Adrian.Bastiani

	Commenced:      25/2/92

	Last Revised:   11/3/92

	Summary:    Header file for the Intelsat module for the
				Cook Islands Project.


***********************************************************************/

#ifndef __INTELSAT_H
#define __INTELSAT_H

#include "general.h"
#include "cmclock.h"

#define BOOLEAN unsigned char
#define TRUE 1
#define FALSE 0

#define BYTE unsigned char


typedef struct  {
		double dfLm0, dfLm1, dfLm2;
		double dfLonc, dfLonc1, dfLons, dfLons1;
		double dfLatc, dfLatc1, dfLats, dfLats1;
		int duration;
		double xel_offset;
		double elevation_offset;
//      double epoch;
		} IntelsatDataStructure;

extern int GetIntelsatAzEl( double TimeFromEpoch,
							double *SatAz,
							double *SatEl );

// removed day of week to conform with TS3000
typedef struct  {
		BYTE hundredths;
		BYTE seconds;
		BYTE minutes;
		BYTE hours;
//      BYTE dayofweek;
		BYTE dayofmonth;
		BYTE month;
		BYTE year;
		BYTE century;
				} IntelsatEpochStructure;


/**************************************************************************
	Function:   fdfDurationInDaysFrom
	Purpose:    Returns the number of days from the first time record to the
				second. If the first record is later than the first then
				a -1.0 is returned. If the difference is greater than 1 year
				then 1000.0 is returned. ( In reality times greater than 12
				days are way to big for AutoTrack purposes)
**************************************************************************/

extern double fdfDurationInDaysFrom( TimeRecord *ptstFirst,
					TimeRecord *ptstSecond);



#endif
