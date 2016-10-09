/***************************************************************************
								TIED SOFTWARE
 PROJECT	:- TS3000

 MODULE 	:- SEQU

 FILE 		:- ANTSIMIO.H

 PROGRAMMER :- L Sciacca

 Copyright 1994. The University of Newcastle Research Associates Ltd.

****************************************************************************
* POLYTRON VERSION CONTROL SOFTWARE _ PVCS (tm) Information
*
*      $Revision:   1.1.1.0  $
*      $Date:   29 Apr 1994 19:41:24  $
*
****************************************************************************/
/*

date: 7-10-1991

Latest: 7-10-1991 LJS Creation

Description:
	Contains the definitions for byte filed offsets for simulated i/o
	memory.
	For details, see hardware specification manual and/or antsimio.c
	desciption.

***************************************************************************
*/

#define AZ_ENC_IO 0			/* long */
#define EL_ENC_IO 4			/* long */

#define DRIVE_SIGNALS_IN    8
#define DRIVE_SIGNALS_OUT     9

#define AZ_V1_INDEX			11
#define AZ_V2_INDEX		13
#define EL_V1_INDEX         	15
#define EL_V2_INDEX         	17

/****************************** End of antsimio.h **********************/


