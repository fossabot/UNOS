/***************************************************************************
								TIED SOFTWARE
 PROJECT	:- TS3000

 MODULE 	:- GLOBALH

 FILE 		:- RECEIVER.H

 PROGRAMMER :- P Moylan

 Copyright 1994. The University of Newcastle Research Associates Ltd.

****************************************************************************
* POLYTRON VERSION CONTROL SOFTWARE _ PVCS (tm) Information
*
*      $Revision:   1.1.1.0  $
*      $Date:   29 Apr 1994 19:41:12  $
*
****************************************************************************/

/************************************************************************/
/*																		*/
/*								RECEIVER.H								*/
/*																		*/
/*      	    	SIMULATOR FOR THE BEACON RECEIVER					*/
/*																		*/
/*		The purpose of this file is to allow testing of the ATCU		*/
/*		software in the absence of the physical beacon receiver.		*/
/*																		*/
/*		Programmer:	P. Moylan											*/
/*		Last modified:	26 September 1991								*/
/*		Status:		Now rewritten to be a real-time simulation			*/
/*																		*/
/*		Limitations:													*/
/*	    	1.	This module simulates a receiver which never makes		*/
/*				an error, and for which the beacon level is always		*/
/*				available.  There is no provision at present for		*/
/*				simulating faults.										*/
/*																		*/
/************************************************************************/

#ifndef __RECEIVER_H
#define __RECEIVER_H

void BeaconSimulatorTask (void *);

    /* A task which should be created and run at initialisation time.	*/

void init_receiver_screen (void);

	/* Refreshes the screen.  To be called each time BEACON_PAGE		*/
	/* becomes the active maintenance page.								*/

#endif /* __RECEIVER_H */
