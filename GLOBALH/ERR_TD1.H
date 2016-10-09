/***************************************************************************
								TIED SOFTWARE
 PROJECT	:- TS3000

 MODULE 	:- GLOBALH

 FILE 		:- ERR_TD1.H

 PROGRAMMER :- R Cockerell

 Copyright 1994. The University of Newcastle Research Associates Ltd.

****************************************************************************
* POLYTRON VERSION CONTROL SOFTWARE _ PVCS (tm) Information
*
*      $Revision:   1.1.1.0  $
*      $Date:   29 Apr 1994 19:41:10  $
*
****************************************************************************/

#ifndef __ERR_TD1_H
#define __ERR_TD1_H

#define	MAX_ERROR_WORDS	4

typedef struct {
		unsigned int	errors[ MAX_ERROR_WORDS ];
} error_list_struct;

typedef struct {
		error_list_struct	current;
		error_list_struct	scada;
} error_register_struct;

#endif  /* __ERR_TD1_H */