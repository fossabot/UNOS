/***************************************************************************
								TIED SOFTWARE
 PROJECT	:- TS3000

 MODULE 	:- GLOBALH

 FILE 		:- DIAG_EX1.H

 PROGRAMMER :- DJL Brown

 Copyright 1994. The University of Newcastle Research Associates Ltd.

****************************************************************************
* POLYTRON VERSION CONTROL SOFTWARE _ PVCS (tm) Information
*
*      $Revision:   1.1.1.0  $
*      $Date:   29 Apr 1994 19:41:10  $
*
****************************************************************************/

/*
* DISCUSSION :  This file contains routines for the ATCU Diagnostic tests.
*	The tests are initiated from Command Parser calls.
*
*****************************************************************************/

#ifndef __DIAG_EX1_H
#define __DIAG_EX1_H

#include "pars_td1.h"    /* For hardware_inputs structure. */
void diag_get_IO( hardware_inputs *hwio );
void diag_set_IO( int which_IO, int test_value );

#endif /* __DIAG_EX1_H */
