/****************************************************************************
* POLYTRON VERSION CONTROL SOFTWARE _ PVCS (tm) Information
*
*      $Revision:   2.6  $
*      $Date:   06 Mar 1992 17:57:34  $
* 
****************************************************************************/
/****************************************************************************
* Geraldton ADSCS Project.
*****************************************************************************
* Copyright (c) 1991. The University of Newcastle Research Associates Ltd.
*****************************************************************************
*
* PROGRAMMER :-     
*                   University of Newcastle, NSW 2308.
*
* FUNCTION REFERENCE :- 
*
****************************************************************************/
/****************************************************************************
* MODULE :- Beacon Task                  File : BCN_SCRN.C
*****************************************************************************
*
* DISCUSSION :
*
****************************************************************************/
/************************************************************************/
/*									*/
/*	 BCN_SCRN.H - SCREEN ACCESS UTILITIES FOR WRITING TO THE	*/
/*			      MAINTENANCE PAGES				*/
/*									*/
/*	Programmer:	P. Moylan					*/
/*	Last modified:	1 September 1991				*/
/*	Status:		Working, except that I don't know where to	*/
/*			import return_screen_page from.  At present	*/
/*			it's defined in TEMP.H				*/
/*									*/
/************************************************************************/

#define private static
#include <bcn_scrn.h>	/* Import my own prototypes		*/
#include <conio.h>	/* FROM conio IMPORT text_info, window,	*/
			/*	gotoxy, gettextinfo, textattr	*/
#include "scrext.h" // imports protect..unprotect_screen()

#include <unos.h>	/* FROM UNOS IMPORT wait, _signal	*/

    /* The following line still needs fixing */

#include <kbtask.h>	/* FROM kbtask IMPORT return_screen_page	*/

/************************************************************************/

private void swapto (struct text_info *saveptr)

    /* Sets the window frame and cursor position to that specified	*/
    /* in *saveptr.  On exit, *saveptr holds the window data which	*/
    /* were in force before the call to swapto.				*/

    {
    struct text_info oldinfo;

    gettextinfo (&oldinfo);
    window (saveptr->winleft, saveptr->wintop,
    		saveptr->winright, saveptr->winbottom);
    textattr (saveptr->attribute);
    gotoxy (saveptr->curx, saveptr->cury);
    *saveptr = oldinfo;
    }

/************************************************************************/

unsigned int GetScreen (unsigned int page_number,
					struct text_info *saveptr)

    /* Attempts to get hold of the screen to write to maintenance page	*/
    /* page_number.  If the result is 0, the screen is currently held	*/
    /* by some other task and the caller must not write to the screen.	*/
    /* If the screen is available, we switch to the screen window	*/
    /* described by *saveptr, give the caller exclusive access to the	*/
    /* screen until such time as ReleaseScreen is called, and return	*/
    /* a value of 1 to indicate success.  It is assumed that the caller	*/
    /* can be trusted to call ReleaseScreen fairly soon, i.e. the	*/
    /* caller must not hog the screen.					*/

    {
    if (return_screen_page() == page_number)
	{
	protect_screen();
	swapto (saveptr);
	return 1;
	}
    else return 0;
    }

/************************************************************************/

void ReleaseScreen (struct text_info *saveptr)

    /* Release control of the screen, and revert to the window		*/
    /* coordinates which were in force before the call to GetScreen.	*/

    {
    swapto (saveptr);
	unprotect_screen ();
    }
