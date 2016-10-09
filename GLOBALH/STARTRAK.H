/***************************************************************************
								TIED SOFTWARE
 PROJECT	:- TS3000

 MODULE 	:- GLOBALH

 FILE 		:- STARTRAK.H

 PROGRAMMER :- L Sciacca (?)

 Copyright 1994. The University of Newcastle Research Associates Ltd.

****************************************************************************
* POLYTRON VERSION CONTROL SOFTWARE _ PVCS (tm) Information
*
*      $Revision:   1.1.1.0  $
*      $Date:   29 Apr 1994 19:41:14  $
*
****************************************************************************/

extern void star_track ( double star_ra, double star_dec, int epoch,
					int day, int month, int year, int hour, int minute,
					double second,
					double station_long, double station_lat,
					double *azcom,
					double *elcom );

extern double juldat(int day, int month, int year,
			  int hour, int minute, double second);