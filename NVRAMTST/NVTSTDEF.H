/***************************************************************************
								TIED SOFTWARE
 PROJECT	:- TS3000

 MODULE 	:- NVRAMTST

 FILE 		:- NVTSTDEF.H

 PROGRAMMER :- DJL Brown

 Copyright 1994. The University of Newcastle Research Associates Ltd.

****************************************************************************
* POLYTRON VERSION CONTROL SOFTWARE _ PVCS (tm) Information
*
*      $Revision:   1.1.1.1  $
*      $Date:   29 Apr 1994 19:41:32  $
*
****************************************************************************/

#ifndef __NVTSTDEF_H
#define __NVTSTDEF_H

/* NOTE WELL:-
	In TS3000 there are two versions of this file. One is in NVRAMTST, the
	other in GLOBALH. THEY ARE DIFFERENT.

	The idea is that when assembling the ATCU code, the version in GLOBALH
	is used (the current directory is then ATCU, and ..\GLOBALH is listed as
	an `include' directory), but when NVRAMTST is being assembled,
	the version in NVRAMTST is used (that then being the current directory).

	The version in GLOBALH does NOT define __NVRAMTST, but it DOES define
	`private' as `static'.

	The version in NVRAMTST defines __NVRAMTST, but does NOT define
	`private' as `static'.

	This enables `private variables in NVRAM.C to be used externally when
	linking NVRAMTST, but not when linking ATCU.

	It also enables conditional compilation of NVRAM.C in that for memory
	allocation, malloc() is used in NVRAMTST, but umalloc() is used in ATCU.
*/

/*
	THIS IS THE VERION TO BE LOCATED IN NVRAMTST
*/

#define __NVRAMTST
#define private


#endif