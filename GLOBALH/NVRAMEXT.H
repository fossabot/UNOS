/*****************************************************************************
* POLYTRON VERSION CONTROL SOFTWARE _ PVCS (tm) Header Information
*
* 	$Revision:   3.1.1.0  $
*	$Date:   06 Oct 1993 10:39:24  $
*
*****************************************************************************/
/****************************************************************************
* Geraldton ADSCS Project.
*****************************************************************************
* Copyright (c) 1991. The University of Newcastle Research Associates Ltd.
****************************************************************************/
/****************************************************************************
* MODULE :- GLOBALH                  File : NVRAMEXT.H
****************************************************************************/
/* Function prototypes required by all tasks requiring NVRAM access.
	Filename: nvramext.h
	Programmer: RC
	First Entry: 11/1/91
	Last Edit: 25/3/91
*/

#ifndef __NVRAMEXT_H
#define __NVRAMEXT_H

#include "posext.h"
#include "intelsat.h"
/*------------------------------------------------------------------*/

int	nvram_init( int mode );    /* Returns "1" if OK, "0" otherwise. */

#define	NVRAM_INIT_MODE	1
#define	NVRAM_TEST_MODE	2
#define LOAD_DEFAULT_VALUES 3

/* The following routines return "1" if OK, "0" otherwise. */

int	get_current_satellite_num( int * );

int	get_station_position( station_position * );

int get_intelsat_parameters ( int sat_num, IntelsatDataStructure *output );
int get_intelsat_epoch ( int sat_num, TimeRecord *output );

/*---- Sequencer and Position Control prototypes */
void put_controller_parameters ( unsigned int index, controller_parameter_struct * con_t );
int get_controller_parameters ( unsigned int index, controller_parameter_struct * con_t );

void put_seq_software_limits ( software_limit_struct * sw_limit );
int get_seq_software_limits ( software_limit_struct * sw_limit );
int get_stow_positions ( stow_pos_struct * stow );

void put_stow_positions ( stow_pos_struct * stow );

/*-------------------------------------------------------------------*/

void	put_current_satellite_num( int * );
void	put_station_position( station_position * );

void put_intelsat_parameters ( int sat_num, IntelsatDataStructure *input);
void put_intelsat_epoch ( int sat_num, TimeRecord *input);


// Those useful NVRAM routines
extern unsigned int ReturnNVRAMError ( void );

extern void ClearNVRAMError ( void );

extern int GetCurrentSatelliteNumber ( void );

extern void get_parameters_from_nvram ( void );
#endif /* __NVRAMEXT_H */
