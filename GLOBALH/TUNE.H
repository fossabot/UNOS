/***************************************************************************
								TIED SOFTWARE
 PROJECT	:- TS3000

 MODULE 	:- GLOBALH

 FILE 		:- TUNE.H

 PROGRAMMER :- DJL Brown

 Copyright 1994. The University of Newcastle Research Associates Ltd.

****************************************************************************
* POLYTRON VERSION CONTROL SOFTWARE _ PVCS (tm) Information
*
*      $Revision:   1.1.1.0  $
*      $Date:   29 Apr 1994 19:41:14  $
*
****************************************************************************/

#ifndef __TUNE_H
#define __TUNE_H

#define AZ_AXIS 		0
#define EL_AXIS 		1
#define SPEW_ORBIT 		2
#define NORMAL_DATA 	3
#define DATAPORT_OFF 	4


void set_dataport_switch ( int setting );
unsigned char return_dataport_switch ( void );
unsigned char return_tune_op_switch ( void );

void fvdTunePage (char chKey );

#endif