/***************************************************************************
								TIED SOFTWARE
 PROJECT	:- TS3000

 MODULE 	:- GLOBALH

 FILE 		:- ERR_EX1.H

 PROGRAMMER :- R Cockerell

 Copyright 1994. The University of Newcastle Research Associates Ltd.

****************************************************************************
* POLYTRON VERSION CONTROL SOFTWARE _ PVCS (tm) Information
*
*      $Revision:   1.1.1.0  $
*      $Date:   29 Apr 1994 19:41:10  $
*
****************************************************************************/

#ifndef __ERR_EX1_H
#define __ERR_EX1_H

/* Error-handling routines:-
* Error code enums are defined in ERR_NUM.H - there is a capacity of 64 error\
* codes. Codes 0 and 64 are reserved. (Code 0 is no error, code 64 is incorrect
* error code given to error handler.)
*/

extern void	make_sticky_error ( int code );
	/* This routine sets the error register mask to make an error `sticky'
* ie a call to `get_errors' (see ERR_EX2.H) will not clear it. 
* `make-sticky_error' should be called in the initialising phase of your code.
*/

extern void	error_set( int code );
	/* This routine sets the error register variables to indicate an error
* condition has just been reported. 
*/

extern void	error_reset( int code );
	/* This routine allows resetting of errors that are not automatically
* cleared when the error list is read. These are the `sticky' errors created 
* by use of `make_sticky_error'.
*/

#endif /* __ERR_EX1_H */