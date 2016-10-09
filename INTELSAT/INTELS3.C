

/*************************************************************************

	Module:		INTELSAT

	Author:			Adrian.Bastiani

	Commenced:		20/2/92

	Last Modified:	14/6/92

	Status:		Appears to be working.

	Summary:	Implements the intelsat mode for the TRACKSAT
				project.

	Modifications:
			24/2/92:
				Added STORE stubb procedures.
				Reworked code to be more similar to Intelsat's
				POINT36.FOR source code.

			25/2/92:
				Modified the STORE access, using structures.
				Executing station and tilt calculations on
				every call to fvdGetIntelsatAzEl(), smoother
				access to STORE.

			12/3/92:
				Made the duration checking external to the
				intelsat function. The intelsat function
				performs NO checking now.   AB

			3/6/92:
				Added extra function into the code to check if the
				software simulator is active and if so to use data local
				to this module rather than the STORE intelsat satellite
				ephemeris data, and earth station data. AB

			14/6/92:
				Modified the test data for the test satellites. The Lonc1,
				and (probably) Lons1 parameters (growth of amplitude) were
				much larger than realistic. This then gives rise to
				numerous orbit change alarms, as we do not model large
				changes in amplitude. Typical values are not more than
				about 0.8 deg/year or 0.002 deg/day in magnitude. Previous
				values were as large as 0.2deg/day. We can probably tolerate
				up to about 0.01deg/day. RHM.
			14/6/92:
				Modified to give a satellite near azimuth = 0 degrees, to
				properly test possible wrap around problems. RHM.

			6/5/94 LJS Ported Intelsat from TS2000 to JAH-1 (old Geraldton)
				code.

			8/6/94 LJS Changed module name from INTELSAT to INTELS3 to
				distinguish the file from the TS2000 version. Don't want people
				thinking they are the same as integrating Intelsat into
				TS3000 changed structures etc.

**************************************************************************/


#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "intelsat.h"
#include "nvramext.h"	// import get_current_satellite_number
						// import get_intelsat_data_nvram
#include "cmclock.h"	// Import TimeRecord

#define K 0.008726646
#define EQUITORIAL_RADIUS 6378.140
#define RG 42164.57
#define FLATNESS 3.352813178e-3

#define RAD_TO_DEG 57.29577951
#define DEG_TO_RAD 0.017453292

/* Simulator Model Data */

/* Data for satellite 3 */

//static IntelsatDataStructure tstSat3Data = {220.0,0.0003,0.001,0.01,-0.002,0.01,
//											-0.01,3.54,0.06,3.54,0.02};

/* Data for satelllite 7 */

//static IntelsatDataStructure tstSat7Data = {215.0,0.002,0.002,0.1,-0.003,-0.1,
//												-0.003,3,0.002,-4, -0.005};

/* Default satellite data - stationary satellite */

//static IntelsatDataStructure tstDefaultSatData = {200.0,0.0,0.0,0.0,0.0,0.0,
//													0.0,0.0,0.0,0.0,0.0};

/* Simulator Station Data */

// static station_position tstSimulatorStation = {
//										/* latitude */ -35.0,
//										/* longitude*/ 220.0,
//										/* height   */ 0.02,
//										/* mnt tilt */ 0.0,
//										/* mnt twist*/ 0.0,
//										/* fallback */ 0    // unsigned char
//										};

/* prototypes */

static void fvdMatprod(double *a2dfMatrixA,double *a1dfMatrixB,
													double *a1dfMatrixC);

static double fdfRefraction(double dfSatelliteElevation);

static void fvdCalculateTiltMatrix(double dfMountTilt, double dfMountTwist,
				double *a2dfMatrix);

/**************************************************************************
*                            TIME FUNCTIONS                               *
**************************************************************************/

static double fdfTimeToDays(TimeRecord *ptstTime);
static BOOLEAN fboIsLeapYear(TimeRecord *ptstTime);
static int finDaysSinceYearStart (TimeRecord *ptstTime);
static int finDaysFrom (TimeRecord *ptstFirst, TimeRecord *ptstSecond);
//static BYTE fbyDaysInMonth(BYTE byMonth, BYTE byYear);

/********************************************************************

	Function:	Refraction()
	Author:		Adrian.Bastiani

	Written:	12/8/91

	Last Modified:	21/2/92

	Current Status:	was working will need to be tested again

	Summary:	Adds correction to the angle at which the
			signal which arrives due to refraction in the
			atmosphere.                              9

			Assumes that the satellite elevation is in
			degrees.

			Assumes elevation is greater than 0 degrees?

	Modifications:
			21/2/92:	
				Edited the names of the function
				and the variables to meet the standards for the 
				Cook Islands Project.

*********************************************************************/

static double fdfRefraction(double dfSatelliteElevation)
{
	double dfNewElevation,dfTemp;

	if (dfSatelliteElevation>10.2)
	{
		dfNewElevation = dfSatelliteElevation + 0.01617 *
				tan((90.0 - dfSatelliteElevation)*DEG_TO_RAD);
	}
	else
	{
		dfTemp = dfSatelliteElevation + 0.589;
		dfNewElevation = dfSatelliteElevation + 
			(0.82622101e-4 * dfTemp*dfTemp*dfTemp*dfTemp) +
			(-0.25187400e-2 * dfTemp*dfTemp*dfTemp) + 
			(0.29906946e-1*dfTemp*dfTemp) + (-0.17941557 * dfTemp) 
			+ (0.58804392);
	}

	return(dfNewElevation);
}



/****************************************************************

	Function:	fvdMatprod()

	Author:		Adrian.Bastiani

	Written:	21/10/91

	Last Modified:	26/2/92

	Summary:	Performs A*B=C where A is 3x3 B is 3x1 and C
			is 3x1. A is stored and passed as A[9], the
			A[r][c] elements located in row major order.

	Modifications:  Replaced the more generalized with this specific
			matrix multiplication function.

			26/2/92:
				Made naming consistent for Cook Islands
				Project.

******************************************************************/

static void fvdMatprod(double *a2dfMatrixA,double *a1dfMatrixB,
														double *a1dfMatrixC)
{
	a1dfMatrixC[0] = a2dfMatrixA[0]*a1dfMatrixB[0] + a2dfMatrixA[1]*
			a1dfMatrixB[1] + a2dfMatrixA[2]*a1dfMatrixB[2];

	a1dfMatrixC[1] = a2dfMatrixA[3]*a1dfMatrixB[0] + a2dfMatrixA[4]*
			a1dfMatrixB[1] + a2dfMatrixA[5]*a1dfMatrixB[2];

	a1dfMatrixC[2] = a2dfMatrixA[6]*a1dfMatrixB[0] + a2dfMatrixA[7]*
			a1dfMatrixB[1] + a2dfMatrixA[8]*a1dfMatrixB[2];

}


/***********************************************************************

	Function:	fvdCalculateTiltMatrix()

	Author:		Adrian.Bastiani

	Commenced:	9/8/91

	Last Modified:	25/2/92

	Summary:        Computes the elements in the tilt matrix,
			given the mount tilt angles and returns the
			transformation array.

	Modifications:	
			24/2/92:
				Added the STORE interfaces.

			25/2/92:
				Removed STORE interfaces, changed parameters.

**************************************************************************/


static void fvdCalculateTiltMatrix(double dfMountTilt, double dfMountTwist,
												double *a2dfMatrix)
{
	dfMountTwist = dfMountTwist * DEG_TO_RAD;
	dfMountTilt = dfMountTilt * DEG_TO_RAD;

	a2dfMatrix[0] = cos(dfMountTwist) * cos(dfMountTilt);
	a2dfMatrix[1]=  -sin(dfMountTwist) * cos(dfMountTilt);
	a2dfMatrix[2]=  sin(dfMountTilt);
	a2dfMatrix[3]=  sin(dfMountTwist);
	a2dfMatrix[4]=  cos(dfMountTwist);
	a2dfMatrix[5]=  0;
	a2dfMatrix[6]=  -sin(dfMountTilt) * cos(dfMountTwist);
	a2dfMatrix[7]=  sin(dfMountTilt) * sin(dfMountTwist);
	a2dfMatrix[8]=  cos(dfMountTilt);

}




/*************************************************************************

	Function:	GetIntelsatAzEl()
	Author:  	Adrian.Bastiani.
	
	Last modified:	11/3/92

	Current Status:	Appears to be working under tests (2/3/92).

	Summary:        Function calculates the actual az and elevation
			of the satellite as seen from the earth station,
			given the time in days from the epoch time. 
			This model accounts for refraction.

			Assumes that sensible values exist for all variables
			in STORE, no checking except for size of
			TimeFromEpoch is performed (Alarms due to this are
			called internally).


			If the Intelsat data is older than the user set
			age limit but less than the age limit plus 2 days
			then values are returned along with a minor alarm
			being activated. If the age limit plus 2 days is
			exceed then a major alarm is activated and this
			function aborted.

			No checking for -ve times included.

			Returns a number to indicate if nvram fail fas occured.
			Sequencer should check this number and take action on it.

	Variable Descriptions:

			Lm0  - Mean Longitude (degrees East of Greenwich)
			Lm1  - Mean Longitude Drift Rate (deg(East)/day)
			Lm2  - Mean Longitude Drift Acceleration  
				(deg(East)/day/day)
			Lonc - Amplitude of the Cosine component of the
				Longitude Oscillation deg(East)
			Lonc1- Rate of change in Lonc (deg(East)9/day)
			Lons - Amplitude of the Sine Component of the
				Longitude Oscillation (deg(East))
			Lons1- Rate of change in Lons (deg(East)/day)
			Latc - Amplitude of the Cosine component of the
				Latitude Oscillation (deg(North))
			Latc1- Rate of change in Latc (deg(North)/day)
			Lats - Amplitude of the Sine component of the
				Latitude Oscillation (deg(North))
			Lats1- Rate of change in (deg(North)/day)


	Constants:	RG: 42,164.57 Km
			K:  pi/360.0
			PI: 3.14159265
			R:  6378.144

	Modifications:

			21/2/92:
				Editted code to suit Cook Islands Project.
				Modified variable names to meet the project
				coding standards.

			24/2/92:
				Added calls to STORE to access required
				parameters.

			25/2/92:
				Modified the STORE calls and put the earth
				station parameter processing into this
				function so it is recalculated every time
				rather than read the parameters from STORE.

			2/3/92:
				added in deg to rad corrections for the dfRc,dfRs equations.

			11/3/92:
				added in function call to check intelsat data age.

			12/3/92:
				Made data age checking external to this function.
				Removed internal checking.
			4/6/92:
				When the simulator is active local data is used rather
				than data from STORE.
				The Epoch Time of these simulated satellites will need to
				be looked after in the SIMUL.C module.

			6/5/94 LJS modified to use TS3000 NVRAM routines.

*************************************************************************/

int GetIntelsatAzEl(double dfTimeFromEpoch,
							double *dfSatAz,
							double *dfSatEl )
{
	double dfW;
	double dfCosWxTime,dfSinWxTime,dfCos2WxTime,dfSin2WxTime;
	double dfSatelliteGeocentricLatitude,dfSatelliteEastLongitude;
	double dfSatelliteRadius;
	double a1dfxyz[3], a1dfxyzdash[3];
	double dfRhox,dfRhoy,dfRhoz,dfRhoNorth,dfRhoZenith;
	double dfclat,dfslat,dfsra,dfsrz,dfCe;
	double a2dfTiltMatrix[9]; /* 3x3 row major ordered matrix */
	double dfRc,dfRs;
	double dfSatElGeometric;
	int sat_num;
	int nvram_ok, status;

	station_position tstStationData;
	IntelsatDataStructure tstSatData;

	// get station position structure
	nvram_ok = get_station_position ( &tstStationData );
	if ( !nvram_ok )
		status = 0;

	// Get satellite number from NVRAM. should check for error
	// here, don't forget now.
	nvram_ok = get_current_satellite_num ( &sat_num );
	if ( !nvram_ok )
		status = 0;

	nvram_ok = get_intelsat_parameters ( sat_num, &tstSatData );
	if ( !nvram_ok )
		status = 0;

	// Only do computations if nvram was okay
	// I didn't change indenting for convenience of printing and viewing
	// LJS 8 June 1994
	if ( nvram_ok )
	{
	dfW = (tstSatData.dfLm1 + 360.98564)*(DEG_TO_RAD);

	dfCosWxTime = cos(dfW*dfTimeFromEpoch);
	dfSinWxTime = sin(dfW*dfTimeFromEpoch);
	dfCos2WxTime = cos(2*dfW*dfTimeFromEpoch);
	dfSin2WxTime = sin(2*dfW*dfTimeFromEpoch);

	/* LONGITUDE CALCULATIONS */

	dfSatelliteEastLongitude =  tstSatData.dfLm0 + tstSatData.dfLm1*dfTimeFromEpoch +
		tstSatData.dfLm2*dfTimeFromEpoch*dfTimeFromEpoch +
		(tstSatData.dfLonc + tstSatData.dfLonc1*dfTimeFromEpoch)
		*dfCosWxTime + (tstSatData.dfLons + tstSatData.dfLons1*
		dfTimeFromEpoch)*dfSinWxTime + (K/2.0)*(tstSatData.dfLatc
		*tstSatData.dfLatc - tstSatData.dfLats*tstSatData.dfLats)
		*dfSin2WxTime - K*tstSatData.dfLatc*tstSatData.dfLats*dfCos2WxTime;


	/* LATITUDE CALCULATIONS */

	dfSatelliteGeocentricLatitude = (tstSatData.dfLatc + tstSatData.dfLatc1*
		dfTimeFromEpoch) * dfCosWxTime + (tstSatData.dfLats+
		tstSatData.dfLats1*dfTimeFromEpoch) * dfSinWxTime;

	/* RADIUS CALCULATIONS */

	dfSatelliteRadius = RG*(1 - ((2*tstSatData.dfLm1)/(3*(dfW*RAD_TO_DEG -
		tstSatData.dfLm1)))) * (1+K*tstSatData.dfLonc*dfSinWxTime
		-K*tstSatData.dfLons*dfCosWxTime);

	dfRc = dfSatelliteRadius*cos(dfSatelliteGeocentricLatitude*DEG_TO_RAD);
	dfRs = dfSatelliteRadius*sin(dfSatelliteGeocentricLatitude*DEG_TO_RAD);

	/* CALCULATE THE EARTH STATION PARAMETERS */

	tstStationData.latitude = tstStationData.latitude * DEG_TO_RAD;
	dfclat = cos(tstStationData.latitude);
	dfslat = sin(tstStationData.latitude);
	dfCe = EQUITORIAL_RADIUS/sqrt(1-FLATNESS*(2-FLATNESS)*dfslat*dfslat);
	dfsra = (dfCe + tstStationData.mount_height) * dfclat;
	dfsrz = (dfCe * (1 - FLATNESS)*(1 - FLATNESS) + tstStationData.mount_height) *
		dfslat;

	/* COMPUTE DELTA R COMPONENTS */

	dfRhox = dfRc * cos((dfSatelliteEastLongitude - tstStationData.longitude)
		* DEG_TO_RAD) - dfsra;

	dfRhoy = dfRc * sin((dfSatelliteEastLongitude - tstStationData.longitude)
		* DEG_TO_RAD);

	dfRhoz = dfRs - dfsrz;

	dfRhoNorth = -dfRhox * dfslat + dfRhoz * dfclat;
	dfRhoZenith = dfRhox * dfclat + dfRhoz * dfslat;


	*dfSatAz = RAD_TO_DEG * atan2(dfRhoy,dfRhoNorth);
	if (*dfSatAz<0.0)
		*dfSatAz = *dfSatAz + 360.0;

	dfSatElGeometric = RAD_TO_DEG * atan(dfRhoZenith/sqrt(dfRhoNorth*
				dfRhoNorth + dfRhoy*dfRhoy));


	/* 	I think that refraction should be added in here before the 
		transformations */

	/* 	Compensate for atmospheric refraction effects that make the
		satellite appear at an angle different to it's actual 
		geometric angle */

	*dfSatEl = fdfRefraction(dfSatElGeometric);

	/* now add in coordinate transformations to allow for mount tilting */

	a1dfxyz[0] = cos(DEG_TO_RAD * (*dfSatEl)) * cos(DEG_TO_RAD * (*dfSatAz));
	a1dfxyz[1] = cos(DEG_TO_RAD * (*dfSatEl)) * sin(DEG_TO_RAD * (*dfSatAz));
	a1dfxyz[2] = sin(DEG_TO_RAD * (*dfSatEl));

	fvdCalculateTiltMatrix(tstStationData.mount_tilt,
			tstStationData.mount_twist,&a2dfTiltMatrix[0]);

	fvdMatprod(&a2dfTiltMatrix[0],&a1dfxyz[0],&a1dfxyzdash[0]);


/*
	for the demo 30/8/91  - prevent the atan2 domain error
	need to check up on what caused that problem
*/

	if ((a1dfxyzdash[1] != 0) && (a1dfxyzdash[0]!=0))
		{
		*dfSatAz = atan2(a1dfxyzdash[1],a1dfxyzdash[0]) * RAD_TO_DEG;

	/*
		Correction added 27/2/92:
		found that atan2 returns the number in the range -pi/2 to pi/2
		the solution to get this into 0 to 2pi range is to check if it
		is negative and if so add 360.0 degrees
		AB
	*/
		if (*dfSatAz < 0.0)
			*dfSatAz = *dfSatAz + 360.0;
		}

	*dfSatEl = asin(a1dfxyzdash[2]) * RAD_TO_DEG;
	} // if nvram_ok

	return ( status );


} 









/*************************************************************************/
/*				             TIME ROUTINES   							 */
/*************************************************************************/


/********************************************************************

	Function:  			fdfTimeToDays
	Author: 			Adrian.Bastiani revised RHM's code.
	Status:				Untested
	Summary:          	Returns the time expressed in days

	Commenced:			13/3/92
	Last Modified:   	1/4/92

	Modifications:

*********************************************************************/

static double fdfTimeToDays(TimeRecord *ptstTime)
{
	return ( ( ( ( ptstTime->hundredths ) / 100.0 + ptstTime->seconds ) / 60.0
			+ ptstTime->minutes ) / 60.0 + ptstTime->hours ) / 24.0;
}




 
/********************************************************************

	Function:			fboIsLeapYear
	Author:				Adrian.Bastiani revised RHM's code
	Status:         	Not Tested

	Summary:       		This function returns a TRUE if the year passed in
						is a leap year, else it returns FALSE.
						It treats a leap year as any year that is divisible
						by 4, and thus is correct for the range 1901 - 2099.

	Commenced:       	13/3/92
	Last Modified:  	1/4/92

	Modifications:

*********************************************************************/


static BOOLEAN fboIsLeapYear( TimeRecord *ptstTime )
{
	div_t x;

	x = div ( ptstTime->year , 4 );
		/* compute quotient and remainder of the year divided by 4 */
	if ( x.rem == 0 )
		return(TRUE);
	else
		return(FALSE);
}




/********************************************************************

	Function:			finDaysSinceYearStart
	Author:				Adrian.Bastiani revised RHM's code
	Status:				Not tested

	Summary:			This procedure returns the number of days since
						1st January on the year input, to the day,
						month input.


	Commenced:			13/3/92
	Last Modified: 		1/4/92

	Modifications:

*********************************************************************/

static int finDaysSinceYearStart ( TimeRecord *ptstTime )
{
	int result;

	switch ( ptstTime->month )
	{
		case 1: /* January */
			result = 0;
                break;
		case 2: /* February */
			result =31;
		break;
		case 3: /* March */
			result = 59;
		break;
		case 4: /* April */
			result = 90;
		break;
		case 5: /* May */
			result = 120;
		break;
		case 6: /* June */
			result = 151;
		break;
		case 7: /* July */
			result = 181;
		break;
		case 8: /* August */
			result = 212;
		break;
		case 9: /* September */
			result = 243;
		break;
		case 10: /* October */
			result = 273;
		break;
		case 11: /* November */
			result = 304;
		break;
		case 12: /* December */
			result = 334;
		break;
		default: /* Help! */
			result = 0;
	}	/* end of switch statement */

	if ( fboIsLeapYear ( ptstTime ) && ( ptstTime->month > 2 ) )
		/* past February on a leap year */
		result = result + 1;

	result = result + ptstTime->dayofmonth;

	return result;
}



   
/********************************************************************

	Function:			finDaysFrom
	Author:        		Adrian.Bastiani revised RHM's code
	Status:				Not tested yet

	Summary:			The following function returns an integer
						being the number of days from the first_date to the
						second_date. If the second date is earlier than the
						first, -1 will be returned.

						!!! NOT SO !!! (by DJLB).
						The code does not reflect this.
						It returns + or - values, up to +/-1 year, and if
						the difference is > 1 year, + or - 1000 is returned.


													If the dates are
						identical 0 will be returned. All years divisible by
						4 are treated as leap years which is correct until
						2100 (not a leap year). Differences in the date of
						greater than one year may be treated as a difference
						of 1000days (i.e. as far as orbit track is concerned, so big
						the rest doesn't matter).
					Centuries are ignored, and any difference
						in years is treated as modulo 100.

	Commenced: 			13/3/92
	Last Modified: 		2/7/92

	Modifications:

*********************************************************************/


static int finDaysFrom ( TimeRecord *ptstFirst, TimeRecord *ptstSecond)
{
	int result, year_difference;

	result = 0;

/*	year_difference = ptstFirst->year - ptstSecond->year;
	Correction by DJLB 3/11/93.	*/
	year_difference = ptstSecond->year - ptstFirst->year;
	year_difference %= 100;
/*	if ( year_difference > 50 )
		year_difference -= 100;
	Correction by DJLB 3/11/93.	NOTE: I know this is not solid - it fails
	for differences of more than 50 in the same century. But its better
	than what was here. */
	if ( year_difference < -50 )
		year_difference += 100;
	if ( year_difference > 50 )
		year_difference -= 100;
	/* check to see if we're (possibly) more than a year apart */

	if ( year_difference > 1 )
		return 1000;
	if ( year_difference < -1 )
		return -1000;

	/* years are now either the same, or they differ by +/- 1 */

	if ( year_difference > 0 )
		{
		if ( fboIsLeapYear(ptstFirst) )
			result = 366;
		else
			result = 365;
		}
	if ( year_difference < 0 )
		{
		if ( fboIsLeapYear(ptstSecond) )
			result = -366;
		else
			result = -365;
	/* Altered 5/11/93: Previously read -355 */
		}

	result = result + finDaysSinceYearStart( ptstSecond ) -
			finDaysSinceYearStart( ptstFirst );

	return result;


}





/********************************************************************

	Function: 			fdfDurationInDaysFrom
	Author:  			Adrian.Bastiani
	Status:             Not tested yet

	Summary:			Given 2 time records calculate the difference
						in days from the first time record to the second.
						If the first record is later than the first then
						a -1 is returned by this function.
						If the difference is greater than 1 year then
						this function will return 1000.0 ( In reality
						times greater than 12 days are way to big for
						Tracksat purposes)


	Commenced: 			13/3/92
	Last Modified:   	13/3/92

	Modifications:      6/5/94 LJS Changed argument to TimeRecord
						due to changes in structure of timing in TS3000
						system.

*********************************************************************/


double fdfDurationInDaysFrom ( TimeRecord *ptstFirst, TimeRecord *ptstSecond)
{
	double dfDuration = 0;

	dfDuration = finDaysFrom ( ptstFirst, ptstSecond );

	if ((dfDuration < 0.0) || (dfDuration == 1000.0))
		return (dfDuration);
	else
	{
		/* add on the fraction of a day difference between the 2 times */

		dfDuration = dfDuration + fdfTimeToDays( ptstSecond )
				- fdfTimeToDays( ptstFirst );

	}

	if (dfDuration < 0.0)
		return (-1.0);
	else
		return(dfDuration);
}




/********************************************************************

	Function: 			fbyDaysInMonth
	Author:  			Adrian.Bastiani

	Status:             Not tested yet

	Summary:			Given the month and the year and the century
						it returns the number of days in the month.

	Commenced: 			8/5/92
	Last Modified:   	8/5/92

	Modifications:

*********************************************************************/


static BYTE fbyDaysInMonth(BYTE byMonth, BYTE byYear)
{
	BYTE byResult;
	div_t tstTemp;
	BOOLEAN boLeapYear;

	tstTemp = div ( byYear , 4 );
	if (tstTemp.rem == 0)
		boLeapYear = TRUE;
	else
		boLeapYear = FALSE;

	switch(byMonth)
	{
		case 1: /* January */
				byResult = 31;
				break;

		case 2: /* February */
				if (boLeapYear)
					byResult = 29;
				else
					byResult = 28;
				break;

		case 3: /* March */
				byResult = 31;
				break;

		case 4: /* April */
				byResult = 30;
				break;

		case 5: /* May */
				byResult = 31;
				break;

		case 6: /* June */
				byResult = 30;
				break;

		case 7: /* July */
				byResult = 31;
				break;

		case 8: /* August */
				byResult = 31;
				break;

		case 9: /* September */
				byResult = 30;
				break;

		case 10: /* October */
				byResult = 31;
				break;

		case 11: /* November */
				byResult = 30;
				break;

		case 12: /* December */
				byResult = 31;
				break;

		default: /* Help! */
				/* set a software informational error */
				  ;
	 }	/* end of switch statement */

	 return(byResult);
}


