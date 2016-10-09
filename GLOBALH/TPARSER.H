/*****************************************************************************
* POLYTRON VERSION CONTROL SOFTWARE _ PVCS (tm) Header Information
*
* 	$Revision:   1.8  $
*	$Date:   12 Oct 1991 17:03:26  $
*
*****************************************************************************/
/*
****************************************************************************

				tparser.h

				Header file for tparser.c

****************************************************************************
*/

#include "posext.h"

#define TUNE_HELP_SCREEN 1
#define TUNE_AZEL_SCREEN 2
#define TUNE_ID_SCREEN   3

extern void initialise_tune_decoder ( void );
extern void tune_parser ( void );
extern void return_display_string ( unsigned char * );
extern void return_error_string ( unsigned char * );
extern void return_lock_string ( unsigned char * );
extern char return_tune_screen ( void );
extern unsigned char return_dataport_switch ( void );

extern int return_axis_tune_data_out ( void );

extern controller_parameter_struct return_parser_par_copy ( int * drive_type );