/***************************************************************************
								TIED SOFTWARE
 PROJECT	:- TS3000

 MODULE 	:- GLOBALH

 FILE 		:- WDOG.H

 PROGRAMMER :- L Sciacca

 Copyright 1994. The University of Newcastle Research Associates Ltd.

****************************************************************************
* POLYTRON VERSION CONTROL SOFTWARE _ PVCS (tm) Information
*
*      $Revision:   1.1.1.0  $
*      $Date:   29 Apr 1994 19:41:14  $
*
****************************************************************************/

/*
***************************************************************************

		Watch Dog Timer Board Hardware Header File

			Extern Prototypes

			by
			L. J. Sciacca

    Date: 4-10-1991

    Latest: 4-10-1991

***************************************************************************
*/

extern void init_watch_dog ( void );
extern void disable_watch_dog ( void );
extern void reset_watch_dog ( void );
