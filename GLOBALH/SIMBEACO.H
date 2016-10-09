/****************************************************************************
* POLYTRON VERSION CONTROL SOFTWARE _ PVCS (tm) Information
*
*      $Revision:   2.3  $
*      $Date:   07 Jan 1994 16:16:18  $
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
* MODULE :- BEACON                  File : SIMBEACO.H
****************************************************************************/


#ifndef __SIMBEACO_H
#define __SIMBEACO_H

/************************************************************************/
/*									*/
/*				SIMBEACO.H				*/
/*									*/
/*      	    SIMULATOR FOR THE BEACON RECEIVER			*/
/*									*/
/*	The purpose of this file is to allow testing of the ATCU	*/
/*	software in the absence of the physical beacon receiver.	*/
/*									*/
/*	Programmer:	P. Moylan					*/
/*	Last modified:	26 April 1992					*/
/*	Status:		OK						*/
/*									*/
/************************************************************************/

BOOLEAN InitBeaconSimulator (void);

    /* Must be called at the start of program execution.  If the result	*/
    /* FALSE is returned, initialization has failed.			*/

void GoBeaconSimulator (void);

   /* Must be called when it is time to start the simulator task.	*/

double FetchSimulatorSample (void);

    /* Waits until the next sample is available, then returns it.  (It	*/
    /* is assumed that StartBeaconSimulator has been called - otherwise	*/
    /* this function will never return.)  Data overrun errors are	*/
    /* reported to the Alarm module, but not to the caller.		*/

void SetSimulatorChannel (BYTE newchannel);

    /* Sets a new simulated beacon channel.	*/

//void init_beacon_simulator_screen (tstDisplayPage* page);

    /* Puts titles on the screen.	*/

//void DisplaySimulatorLevels (tstDisplayPage* page);

    /* Displays the simulator output on a maintenance page.	*/

#endif /* __SIMBEACO_H */
