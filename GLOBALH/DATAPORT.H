/***************************************************************************
								TIED SOFTWARE
 PROJECT	:- TS3000

 MODULE 	:- GLOBALH

 FILE 		:- DATAPORT.H

 PROGRAMMER :- DJL Brown

 Copyright 1994. The University of Newcastle Research Associates Ltd.

****************************************************************************
* POLYTRON VERSION CONTROL SOFTWARE _ PVCS (tm) Information
*
*      $Revision:   1.1.1.0  $
*      $Date:   29 Apr 1994 19:41:10  $
*
****************************************************************************/

/*****************************************************************************
*
* DISCUSSION :  This header file provides the prototype of the routine
*		to be called by the Sequencer once per sequence cycle,
*		to send data to the ATCU data port.
*****************************************************************************
*/
#include "pars_td1.h"

void	spew_data( void );