/***************************************************************************
								TIED SOFTWARE
 PROJECT	:- TS3000

 MODULE 	:- GLOBALH

 FILE 		:- MAIN_EXT.H

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
* DISCUSSION :   This file was added 15/12/91. It provides prototypes of 2
*		functions which provide access to the 2 MAIN-local variables
*		simulator_speed and nvram_sim - previously provided as global
*		variables - a somewhat undesirable method.
*
*		Have since added 2 more functions to pass out mou\re variables.
*		04/05/95 (STO)
*
****************************************************************************/

#define LOG_TASK 0

int get_simulator_speed( void );	/* Returns 0 if normal speed reqd, 
						1 if fast		*/              

int get_nvram_sim( void );		/* Returns 0 if actual nvram to    
						be used, 1 if it should be 
						simulated. 		*/
int get_simulation( void );		/* Returns 0 if real world is to
						be used, 1 if a simulated
						world is to be used.	*/
FILE *get_fileptr( void );	 /* file pointer of data logging file */
FILE *get_graphptr(void);

int get_log_data( void );	/* Returns 1 if requiring data-logging */

/* End of MAIN_EXT.H */