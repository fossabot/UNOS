/***************************************************************************
								TIED SOFTWARE
 PROJECT	:- TS3000

 MODULE 	:- GLOBALH

 FILE 		:- ERR_EX2.H

 PROGRAMMER :- R Cockerell

 Copyright 1994. The University of Newcastle Research Associates Ltd.

****************************************************************************
* POLYTRON VERSION CONTROL SOFTWARE _ PVCS (tm) Information
*
*      $Revision:   1.1.1.0  $
*      $Date:   29 Apr 1994 19:41:10  $
*
****************************************************************************/

#ifndef __ERR_EX2_H
#define __ERR_EX2_H

#include "err_td1.h"

extern int	error_handler_init( void );
extern error_list_struct	get_errors( int which_CMU );

#endif /* __ERR_EX2_H */