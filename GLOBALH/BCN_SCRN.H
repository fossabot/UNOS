/***************************************************************************
								TIED SOFTWARE
 PROJECT	:- TS3000

 MODULE 	:- GLOBALH

 FILE 		:- BCN_SCRN.H

 PROGRAMMER :- P Moylan

 Copyright 1994. The University of Newcastle Research Associates Ltd.

****************************************************************************
* POLYTRON VERSION CONTROL SOFTWARE _ PVCS (tm) Information
*
*      $Revision:   1.1.1.0  $
*      $Date:   29 Apr 1994 19:41:08  $
*
****************************************************************************/

#ifndef __BCN_SCRN_H
#define __BCN_SCRN_H

#include <conio.h>	/* FROM conio IMPORT text_info	*/

unsigned int GetScreen (unsigned int page_number, struct text_info *saveptr);

    /* Attempts to get hold of the screen to write to maintenance page	*/
    /* page_number.  If the result is 0, the screen is currently held	*/
    /* by some other task and the caller must not write to the screen.	*/
	/* If the screen is available, we switch to the screen window		*/
    /* described by *saveptr, give the caller exclusive access to the	*/
    /* screen until such time as ReleaseScreen is called, and return	*/
    /* a value of 1 to indicate success.  It is assumed that the caller	*/
	/* can be trusted to call ReleaseScreen fairly soon, i.e. the		*/
	/* caller must not hog the screen.									*/

void ReleaseScreen (struct text_info *saveptr);

    /* Restores the window coordinates which were in force before the	*/
    /* last successful call to GetScreen, and surrenders control of the	*/
	/* screen by performing a signal on semaphore SCREEN_SEM.			*/

#endif /* __BCN_SCRN_H */
