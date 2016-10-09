/***************************************************************************
								TIED SOFTWARE
 PROJECT	:- TS3000

 MODULE 	:- GLOBALH

 FILE 		:- DISPLAYS.H

 PROGRAMMER :- DJL Brown

 Copyright 1994. The University of Newcastle Research Associates Ltd.

****************************************************************************
* POLYTRON VERSION CONTROL SOFTWARE _ PVCS (tm) Information
*
*      $Revision:   1.1.1.0  $
*      $Date:   29 Apr 1994 19:41:10  $
*
****************************************************************************/

#ifndef __DISPLAYS_H
#define __DISPLAYS_H

#include "screen.h"

void fvdInitDisplays( void );
void fvdStatusPage( char chKey );
void fvdBeaconPage( char chKey );
void fvdParserPage( char chKey );
void fvdSeqPage( char chKey );
void fvdTunePage( char chKey );

extern struct display_page *pstStatusPage, *pstBeaconPage, *pstOrbitTrackPage,
							*pstParserPage, *pstSeqPage, *pstTunePage;

extern struct window_info 	*pstStWholePage,
							*pstStTitleWindow, *pstStPositionWindow,
							*pstStDetailsWindow, *pstStAlarmsWindow,

							*pstBeaconWholePage,
							*pstBeaconInfo, *pstBeaconPollHeading,
							*pstBeaconPoll, *pstBeaconDisplay,
							*pstBeaconSettings,
							 
							*pstOrbitTrackWholePage, *pstParserInnerPage,

							*pstParserWholePage,

							*pstSeqWholePage,
							*pstSeqMenu,

							*pstTuneWholePage,*pstTuneParametersWindow,
							*pstTuneControlWindow, *pstTuneStepOutWindow;


#endif