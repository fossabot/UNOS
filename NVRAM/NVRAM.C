/***************************************************************************
								TIED SOFTWARE
 PROJECT	:- TS3000

 MODULE 	:- NVRAM              ##### MODIFIED FOR INTELSAT ######

 FILE 		:- NVRAM.C

 PROGRAMMER :- R Cockerell

 Copyright 1994. The University of Newcastle Research Associates Ltd.

****************************************************************************
* POLYTRON VERSION CONTROL SOFTWARE _ PVCS (tm) Information
*
*      $Revision:   1.1.1.0  $
*      $Date:   29 Apr 1994 19:41:16  $
*
****************************************************************************/

/*****************************************************************************
*
* DATE (Last modified) :-
*
* DISCUSSION : This file contains all the routines used to access data stored
*	in NVRAM. These routines are an interface between the rest of the ATCU
*	and the NVRAM hardware. Data is accessed via a pointer addressing scheme
*	which is initialised in the header file "nvram.h".
*   Two bytes are reserved for storing the check-sum and its complement.
*
*	To add new data structures for storage the following steps must be taken:
*		(1) Declare the pointer;
*		(2) Assign absolute address to pointer in "setup_nvram_pointers()";
*		(3) Include initialisation code in "nvram_init()";
*		4. Add the "put()" & "get()" routines;
*		5. Include the function prototypes in "nvramext.h";
*		6. Make sure any necessary type definitions are accessible.
*		7. Ensure appropriate protection, e.g. "disable/enable" is in place.
*
*		8. LJS 2-7-1991 Modified to allow for compilation on any PC (no 
*			NVRAM required.
*		9. LJS 2-7-1991 Modified disable/enable pairs to check for int 
*			status to allow calling these routines with interrupts disabled.
*	      	10. LJS 2-8-1991 Added Position loop and sequencer NVRAM routines.
*		11. DJLB 7-12-91. Changed use of umalloc ( for computers without
*			NVRAM ) to use of malloc. Provided a service file with other
*			NVRAM I/O routines used by NVRAMTST, but located in other parts
*			of the ATCU software, so that NVRAMTST could be compiled from a
*                       much reduced source base, reducing its size.
*               12. DJLB 2-3-92. Added facility to force loading of default
*			values, when used as a part of NVRAMTST.
*
******************************************************************************/

#include <string.h>
#include <stdlib.h>
#include <dos.h>
#include <stdio.h>
#include <stddef.h>
#include <conio.h>

#include "posext.h"
#include "pars_td1.h"
#include "nvram.h"
#include "nvramext.h"
#include "unos.h"
#include "seqext.h"
#include "main_ext.h"

#include "nvtstdef.h"  	/* This file comes in two versions:-
							The copy in the NVRAMTST sub-directory defines
							__NVRAMTST, and also defines `private' as `'.
							The copy in the GLOBALH sub-directory (and thus
							the one used when ATCU is being assembled)
							defines `private' as `static', but DOES NOT define
							__NVRAMTST. This provides a convenient method of
							conditionally compiling NVRAM.C so that if
							simulated NVRAM is required, memory allocation
							is done by malloc() or umalloc(), the former
							for NVRAMTST, the latter for ATCU. */

unsigned int   size_of_nvstore; /* Also used by nvramtst.c */


static int copy_from_nvram ( const void *from, const void *to, const size_t n );
static void copy_into_nvram ( const void *from, const void *to, const size_t n );


/**************  (1) Pointer Declaration  ******************/
/* These are pointers into the NVRAM address space */

/* Note that if NVRAMTST is being used, `private has NOT been
	defined as `static'*/
private unsigned char	*sat_details_ptr[ NUM_SATELLITES ],
				*star_details_ptr[ NUM_STARS ],
				*beacon_details_ptr,
				*scan_data_ptr,
				*orbit_model_ptr[ NUM_SATELLITES ],
				*orbit_covariance_ptr[ NUM_SATELLITES ],
				*intelsat_parameters_ptr[ NUM_SATELLITES ],
				*nv_calibration_coefficients,
				*station_position_ptr;

private unsigned int	*current_sat_num_ptr,
				*orbit_start_ptr,
				*current_star_num_ptr;

private void 	*sw_limits_ptr; /* Software limits */
private void	*stow_pos_ptr;	/* Stow positions */

/* Controller algorithm constants */
private void 	*con_ptr [ 2 ];	/* two axes of controller constants*/

/*************************************
* SETUP_NVRAM_POINTERS()             *
*************************************/

/*************************************************************************
* The following routine must be called upon system initialisation before
* any NVRAM access is attempted. The routine initialises the pointers to
* absolute memory locations in NVRAM.
* The program returns "1" if OK, "0" otherwise.
*
* It must be noted that by using the method of creating an
* address for the next data structure by adding to the base address of
* the previous structure will only be possible whilst the total data size
* does not exceed 64KB. Otherwise there will be wraparound problems because
* of the way the NVRAM_BASE_ADDR is defined.
*************************************************************************/

static int	setup_nvram_pointers(  void )
{
int	i, nvram_sim, stored_data_size;
char *	nvram_addr, *nvram_base_addr;

	/* If being run on a computer without NVRAM card installed,
	the following allows use of some RAM as if it were the NVRAM card.
	the variable nvram_sim is set in main() by the 2nd command-line
	parameter. (nvram_sim = 0 indicates nvram is to be used.)*/

	nvram_sim = get_nvram_sim(); 
	if ( nvram_sim == 0 ) nvram_addr = (char *)NVRAM_BASE_ADDR;
	else
	{
#ifdef __NVRAMTST/* If NVRAMTST is being compiled, malloc() is used.
					If ATCU is being compiled, umalloc() from UNOS is used.*/
		nvram_addr = (char *)malloc( NVRAM_SIZE );
#else
		nvram_addr = (char *)umalloc( NVRAM_SIZE );
#endif
		if( nvram_addr == NULL )
		{
			cprintf(
			"ERROR - Not enough memory available to simulate NVRAM.\n\r");
			exit(1);
		}
	}
	nvram_base_addr = nvram_addr;

	/********  (2) Assign pointer to absolute memory address  *********/

	for( i=0; i < NUM_SATELLITES; i++ )
	{
		sat_details_ptr[i] = (unsigned char*)(nvram_addr);
		stored_data_size = sizeof(satellite_details_struct) + 2;
		nvram_addr += stored_data_size;
	}

	current_sat_num_ptr = (unsigned int*)(nvram_addr);
	stored_data_size = 2 * sizeof( int ); /* Store value and complement */
	nvram_addr += stored_data_size;

	for( i=0; i < NUM_STARS; i++ )
	{
		star_details_ptr[i] = (unsigned char*)(nvram_addr);
		stored_data_size = sizeof(star_details_struct) + 2;
		nvram_addr += stored_data_size;
	}

	current_star_num_ptr = (unsigned int*)(nvram_addr);
	stored_data_size = 2 * sizeof( int ); /* Store value and complement */
	nvram_addr += stored_data_size;

	beacon_details_ptr = (unsigned char*)(nvram_addr);
	stored_data_size = sizeof(beacon_details_struct) + 2;
	nvram_addr += stored_data_size;

	orbit_start_ptr = (unsigned int*)(nvram_addr);
	stored_data_size = 2 * sizeof( int ); /* Store value and complement */
	nvram_addr += stored_data_size;

	scan_data_ptr = (unsigned char*)(nvram_addr);
	stored_data_size = sizeof(scan_data) + 2;
	nvram_addr += stored_data_size;

	nv_calibration_coefficients = ( unsigned char * ) nvram_addr;
	stored_data_size = sizeof ( calibration_coefficients ) + 2;
	nvram_addr += stored_data_size;

	for( i=0; i < NUM_SATELLITES; i++ )
	{
		intelsat_parameters_ptr[i] = ( unsigned char * ) nvram_addr;
		stored_data_size = sizeof ( intelsat_parameters ) + 2;
		nvram_addr += stored_data_size;
	}

	station_position_ptr = ( unsigned char * )nvram_addr;
	stored_data_size = sizeof( station_position ) + 2;
	nvram_addr += stored_data_size;

	/*---- Satellite structures */
	for( i=0; i < NUM_SATELLITES; i++ )
	{
		orbit_model_ptr[i] = (unsigned char*)(nvram_addr);
		stored_data_size = sizeof(orbit_model_struct) + 2;
		nvram_addr += stored_data_size;
	}

	for( i=0; i < NUM_SATELLITES; i++ )
	{
		orbit_covariance_ptr[i] = (unsigned char*)(nvram_addr);
		stored_data_size = sizeof(orbit_covariance_struct) + 2;
		nvram_addr += stored_data_size;
	}

	/************ Sequencer Area *********************************/
	/*---- Controller parameter Algorithm Constants */
	for ( i = 0; i < 2; i++ ) {
		con_ptr [ i ] = ( void * )( nvram_addr );
		stored_data_size = sizeof( controller_parameter_struct ) + 2;
		nvram_addr += stored_data_size;
		}

	/*---- Sequencer Software Limits */
	sw_limits_ptr = ( void * )( nvram_addr );
	stored_data_size = sizeof( software_limit_struct ) + 2;
	nvram_addr += stored_data_size;

	/*---- Sequencer Stow positions */
	stow_pos_ptr = ( void * )( nvram_addr );
	stored_data_size = sizeof( stow_pos_struct ) + 2;
	nvram_addr += stored_data_size;
	/************* End of Sequencer NVRAM ******************/

	size_of_nvstore = (unsigned int)(nvram_addr - nvram_base_addr);

	if ( ( nvram_addr == NULL )
		||  ( size_of_nvstore > NVRAM_SIZE ) )

	{	/* Unsafe side of 64KB */
		printf("\n\aToo much NVRAM to be stored in one group.");
		return 0;
	}

    return 1;
}


/*************************************
* NVRAM_INIT()                       *
*************************************/

/**************************************************************************
* The following procedure is called during the ATCU power-up initialisation
* period. The routine initialises data pointers and data contents if there
* is a detected error.
*
* The "mode" variable allows the routine to be called from both the initialisation
* and diagnostic routines. The difference is that during initialisation if
* the initial "get" of the data is in error, factory settings are loaded and
* another test is performed before judgement is passed. With the "test" mode,
* pass/fail judgement is made at the first read operation.
*
* The program returns a "1" if all is OK. "0" if there is a serious NVRAM
* error which should prevent system start up until it is rectified.
**************************************************************************/

int	nvram_init( int mode )
{
int	index,
	i;

/* Local, temporary variables for checking purposes. */

satellite_details_struct	sat_details;
star_details_struct		star_details;
beacon_details_struct		beacon_details;
int				current_sat_num,
				orbit_start_type,
				current_star_num;
scan_data			scan;
unsigned char	orbit_model[ sizeof(orbit_model_struct) ], 
		       /* (Assume can be legally processed at compile-time) */
		orbit_covariance[ sizeof(orbit_covariance_struct) ];

calibration_coefficients 	cal,
   				default_calibration_coefficients =
     				   { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

intelsat_parameters intelsat,
				default_intelsat_parameters =
					   { 177.0, 0.0, 0.0, 0.0, 0.0, 0.0,
						   0.0, 0.0, 0.0, 0.0, 0.0,
						   1, 1, 50, 0, 0, 0, 10};

station_position		stat_loc, default_stat_loc = { -38.83, 144.9 };

/* Sequencer Structures */
controller_parameter_struct 	con_temp;
software_limit_struct       	sw_limit_temp;
stow_pos_struct			stow_pos_temp;

    /* NVRAM Initialisation */
	if (mode == NVRAM_INIT_MODE)
	{
		/* Write to 6M21 control port to allow access. This presently involves
		   allowing write access to both banks and disabling the global
		   read/write shutdown bit. */

		outportb( 0x200 , 0xe0 );
		if (!setup_nvram_pointers())
			return 0;
	}

    /* Access all NVRAM data to ensure correct operation.
	   Initialise if required. (3)
    */
	for ( index = 0; index < NUM_SATELLITES; index++ )
	{
		if ( !get_satellite_details( &sat_details , index )
			|| mode == LOAD_DEFAULT_VALUES )
		{
			/* Error in data sumcheck. */

			if ( mode == NVRAM_TEST_MODE )
				return 0;		/* Diagnost testing shows a fault. No point
                					in loading factory settings. */

			/* Error due to corrupted or previously unused NVRAM. Flag an error
			   in case this is due to corrupted NVRAM, and initialise data. */

			strcpy( sat_details.name , "Default" );
			sat_details.beacon_offset = 0.0;
			sat_details.beacon_select = 1;
			sat_details.polariser = 0;
			sat_details.move_size = 0.01;
			sat_details.max_time_increment = 10;

			put_satellite_details( &sat_details , index );

			if ( !get_satellite_details( &sat_details , index ) )
				return 0;		/* Must be seriously faulty NVRAM. */
        }
	}

    /* Satellite Data */
	if ( !get_current_satellite_num( &current_sat_num ) 
			|| mode == LOAD_DEFAULT_VALUES )
	{
		if ( mode == NVRAM_TEST_MODE )
			return 0;		/* Diagnost testing shows a fault. No point
								in loading factory settings. */

		/* Error due to corrupted or previously unused NVRAM. Flag an error
		   in case this is due to corrupted NVRAM, and initialise data. */

		current_sat_num = 3;
		put_current_satellite_num( &current_sat_num );

		if ( !get_current_satellite_num( &current_sat_num ) )
			return 0;		/* Must be seriously faulty NVRAM. */
	}

    /* Star Data */
	for ( index=0; index < NUM_STARS; index++ )
	{
		if ( !get_star_details( &star_details , index ) 
			|| mode == LOAD_DEFAULT_VALUES )
		{
			if ( mode == NVRAM_TEST_MODE )
				return 0;		/* Diagnost testing shows a fault. No point
                					in loading factory settings. */

			/* Error due to corrupted or previously unused NVRAM. Flag an error
			   in case this is due to corrupted NVRAM, and initialise data. */

			strcpy( star_details.name , "Default" );
			star_details.right_ascension = 70.0 * index;;
			star_details.declination = -10.034;
			star_details.epoch_mode = 2000;


			put_star_details( &star_details , index );

			if ( !get_star_details( &star_details , index ) )
				return 0;
				/* Must be seriously faulty NVRAM. */
        }
	}

	if ( !get_current_star_num( &current_star_num ) 
			|| mode == LOAD_DEFAULT_VALUES )
	{
		if ( mode == NVRAM_TEST_MODE )
			return 0;		/* Diagnost testing shows a fault. No point
								in loading factory settings. */

		/* Error due to corrupted or previously unused NVRAM. Flag an error
		   in case this is due to corrupted NVRAM, and initialise data. */

		current_star_num = 0;
		put_current_star_num( &current_star_num );

		if ( !get_current_star_num( &current_star_num ) )
			return 0;		/* Must be seriously faulty NVRAM. */
	}


    /* Beacon Data */
	if ( !get_beacon_details( &beacon_details ) 
			|| mode == LOAD_DEFAULT_VALUES )
	{
		if ( mode == NVRAM_TEST_MODE )
			return 0;		/* Diagnost testing shows a fault. No point
								in loading factory settings. */

		/* Error due to corrupted or previously unused NVRAM. Flag an error
		   in case this is due to corrupted NVRAM, and initialise data. */

		beacon_details.half_power_bw[ 0 ] = 0.2;
		beacon_details.half_power_bw[ 1 ] = 0.2;
		beacon_details.half_power_bw[ 2 ] = 0.2;
		beacon_details.half_power_bw[ 3 ] = 0.2;
		beacon_details.half_power_bw[ 4 ] = 0.1;
		beacon_details.slew_limit = 0.3 ;
		beacon_details.hi_limit = 10.0;
        	beacon_details.lo_limit = -10.0;

		put_beacon_details( &beacon_details );

		if ( !get_beacon_details( &beacon_details ) )
			return 0;		/* Must be seriously faulty NVRAM. */
	}

	if ( ! get_calibration_coefficients ( &cal )
			|| mode == LOAD_DEFAULT_VALUES ) 
	{
        if ( mode == NVRAM_TEST_MODE )
           return 0;
		else
		{
           put_calibration_coefficients ( &default_calibration_coefficients );
           if ( ! get_calibration_coefficients ( &cal ) )
              return 0;
        }
	}

	for ( index = 0; index < NUM_SATELLITES; index++ )
	{
		if ( ! get_intelsat_parameters ( &intelsat, index )
			|| mode == LOAD_DEFAULT_VALUES ) 
		{
			if ( mode == NVRAM_TEST_MODE )
				return 0;
			else
			{
				put_intelsat_parameters (&default_intelsat_parameters, index);
				if ( ! get_intelsat_parameters ( &intelsat, index ) )
					return 0;
			}
		}
	}

	if ( ! get_station_position ( &stat_loc )
			|| mode == LOAD_DEFAULT_VALUES ) 
	{
		if ( mode == NVRAM_TEST_MODE )
			return 0;
		else
		{
		   put_station_position ( &default_stat_loc );
           if ( ! get_station_position ( &stat_loc ) )
              return 0;
        }
     }

    /* Orbit Track */
	if ( !get_orbit_start_type( &orbit_start_type ) 
			|| mode == LOAD_DEFAULT_VALUES )
	{
		if ( mode == NVRAM_TEST_MODE )
			return 0;		/* Diagnost testing shows a fault. No point
								in loading factory settings. */

		/* Error due to corrupted or previously unused NVRAM. Flag an error
		   in case this is due to corrupted NVRAM, and initialise data. */

		orbit_start_type = 2;
		put_orbit_start_type( &orbit_start_type );

		if ( !get_orbit_start_type( &orbit_start_type ) )
			return 0;		/* Must be seriously faulty NVRAM. */
	}

	if ( !get_scan_data( &scan ) 
			|| mode == LOAD_DEFAULT_VALUES )
	{
		if ( mode == NVRAM_TEST_MODE )
			return 0;		/* Diagnost testing shows a fault. No point
								in loading factory settings. */

		/* Error due to corrupted or previously unused NVRAM. Flag an error
		   in case this is due to corrupted NVRAM, and initialise data. */

		scan.time_limit = 30;
		scan.separation = 0.01;
		scan.search_speed = 0.01;
		scan.box_size = 20.0;
		scan.method = 1;

		put_scan_data( &scan );

		if ( !get_scan_data( &scan ) )
			return 0;		/* Must be seriously faulty NVRAM. */
	}

	for ( index = 0; index < NUM_SATELLITES; index++ )
    {
		if ( !get_orbit_track_model( (orbit_model_struct *)&orbit_model , index ) 
			|| mode == LOAD_DEFAULT_VALUES )
		{
			/* Error in data sumcheck. */

			if ( mode == NVRAM_TEST_MODE )
				return 0;		/* Diagnost testing shows a fault. No point
                					in loading factory settings. */

			/* Error due to corrupted or previously unused NVRAM. Flag an error
			   in case this is due to corrupted NVRAM, and initialise data. */

/*******
ADD Initialisation. RHM requires nvram error indication if the data is functionally
in error (e.g. uninitalised model) whereas the routines are currently set up
for returning an error if the NVRAM itself is faulty.

Below is a first draft consisting of loading all zeros. In this case, reading the
model's update_date would show 0/00/00 => functional error even if the "get"
routined returned that there was no NVRAM error.
*******/

			for ( i = 0; i < sizeof(orbit_model_struct); i++ )
				orbit_model[ i ] = 0;
/******/

			put_orbit_track_model( (orbit_model_struct *)&orbit_model , index );

			if ( !get_orbit_track_model( (orbit_model_struct *)&orbit_model , index ) )
				return 0;		/* Must be seriously faulty NVRAM. */
        }
	}

	for ( index = 0; index < NUM_SATELLITES; index++ )
    {
		if ( !get_orbit_track_covariance( (orbit_covariance_struct *)&orbit_covariance , index ) 
			|| mode == LOAD_DEFAULT_VALUES )
		{
			/* Error in data sumcheck. */

			if ( mode == NVRAM_TEST_MODE )
				return 0;		/* Diagnost testing shows a fault. No point
                					in loading factory settings. */

			/* Error due to corrupted or previously unused NVRAM. Flag an error
			   in case this is due to corrupted NVRAM, and initialise data. */

/*******
ADD Initialisation. RHM requires nvram error indication if the data is functionally
in error (e.g. uninitalised model) whereas the routines are currently set up
for returning an error if the NVRAM itself is faulty.

Below is a first draft consisting of loading all zeros. In this case, reading the
model's update_date would show 0/00/00 => functional error even if the "get"
routined returned that there was no NVRAM error.
*******/

			for ( i = 0; i < sizeof(orbit_covariance_struct); i++ )
				orbit_covariance[ i ] = 0;

			put_orbit_track_covariance( (orbit_covariance_struct *)&orbit_covariance , index );

			if ( !get_orbit_track_covariance( (orbit_covariance_struct *)&orbit_covariance , index ) )
				return 0;		/* Must be seriously faulty NVRAM. */
        }
	}


	/********************** SEQUENCER AREA ***************************/
	/*----- Check Controller Parameters */
	/* Do once for el and once for az */

	for ( i = 0; i < 2; i++ )
		if ( !get_controller_parameters ( i, (controller_parameter_struct *)&con_temp ) 
			|| mode == LOAD_DEFAULT_VALUES )
			{
			if ( mode == NVRAM_TEST_MODE )
				return ( 0 );

			/* Load up the default values for this test case */
			init_default_controller_parameters ( i, (controller_parameter_struct *)&con_temp );
			put_controller_parameters ( i, &con_temp );
			
			/* Recheck RAM */
			if ( !get_controller_parameters ( i, &con_temp ) )
				return ( 0 );
			}

    /*----- Sequencer Software Limits */
    if ( !get_seq_software_limits ( (software_limit_struct *)&sw_limit_temp ) 
			|| mode == LOAD_DEFAULT_VALUES )
	{
	if ( mode == NVRAM_TEST_MODE )
		return ( 0 );

		/* Load up the default values for now */
    	set_default_software_limits ( (software_limit_struct *)&sw_limit_temp );
     	put_seq_software_limits ( &sw_limit_temp );

    	/* Recheck RAM */
		if ( !get_seq_software_limits ( (software_limit_struct *)&sw_limit_temp ) )
    		return ( 0 );
	}

    /*----- Sequencer Software Limits */
    if ( !get_stow_positions ( (stow_pos_struct *)&stow_pos_temp ) 
			|| mode == LOAD_DEFAULT_VALUES )
	{
		if ( mode == NVRAM_TEST_MODE )
			return ( 0 );

		/* Load up the default values for now */
		set_default_stow_positions ( (stow_pos_struct *)&stow_pos_temp );
		put_stow_positions ( &stow_pos_temp );

    	/* Recheck RAM */
		if ( !get_stow_positions ( (stow_pos_struct *)&stow_pos_temp ) )
    		return ( 0 );
	}
	/************************* END SEQUENCER AREA ************************/

    return 1;
}

/*
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    						PUTS and GETS here


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
*/

/*************************************
* PUT_SATELLITE_DETAILS()            *
*************************************/

/**************************************************************************
 Procedure to be called from MAIN which will assign the absolute NVRAM
	addresses to global data pointers. Also initialises any other global
	data, if required.
This routine is used to store one data structure into NVRAM.

***************************************************************************/

void put_satellite_details( satellite_details_struct *input , int sat_num )
{
unsigned char	*ptr,
				check_sum = 0;
int	i;
	int int_status;

	ptr = sat_details_ptr[ sat_num ];

	int_status = return_interrupt_status ( );

	disable();

	for ( i=0; i < sizeof( satellite_details_struct ) ; i++ )
	{
		/* Store data in consecutive bytes and calculate the XOR check-sum. */

		*(ptr + i) = *((unsigned char *)input + i);
		check_sum ^= *((unsigned char *)input + i);
	}

	/* Now store the check-sum with its complement for error checking. */

	*(ptr + i) = check_sum;
	*(ptr + i + 1) = ~check_sum;

	if ( int_status )
		enable ( );
}


/*************************************
* GET_SATELLITE_DETAILS()            *
*************************************/

/***************************************************************************
 This routine gets a structure from NVRAM, checks validity returning a "0"
   if there is an error, "1" otherwise.
***************************************************************************/

int get_satellite_details( satellite_details_struct *output , int sat_num )
{
unsigned char	*ptr,
				check_sum = 0;
int	i;
	int int_status;

	ptr = sat_details_ptr[ sat_num ];

	int_status = return_interrupt_status ( );

	disable();

	for ( i=0; i < sizeof( satellite_details_struct ) ; i++ )
	{
		*((unsigned char *)output + i) = *(ptr + i);
		check_sum ^= *(ptr + i);
	}

	if ( ( *(ptr + i) == check_sum ) &&
		 ( *(ptr + i + 1) == (unsigned char)(~check_sum) ) )
	{
	if ( int_status )
		enable();
		return 1;
	}
	else
	{
	if ( int_status )
		enable();
		return 0;
	}
}

/*************************************
* PUT_INTELSAT_PARAMETERS()            *
*************************************/

/**************************************************************************
 Procedure to be called from MAIN which will assign the absolute NVRAM
	addresses to global data pointers. Also initialises any other global
	data, if required.
This routine is used to store one data structure into NVRAM.

***************************************************************************/

void put_intelsat_parameters( intelsat_parameters *input , int sat_num )
{
unsigned char	*ptr,
				check_sum = 0;
int	i;
	int int_status;

	ptr = intelsat_parameters_ptr[ sat_num ];

	int_status = return_interrupt_status ( );

	disable();

	for ( i=0; i < sizeof( intelsat_parameters ) ; i++ )
	{
		/* Store data in consecutive bytes and calculate the XOR check-sum. */

		*(ptr + i) = *((unsigned char *)input + i);
		check_sum ^= *((unsigned char *)input + i);
	}

	/* Now store the check-sum with its complement for error checking. */

	*(ptr + i) = check_sum;
	*(ptr + i + 1) = ~check_sum;

	if ( int_status )
		enable ( );
}


/*************************************
* GET_INTELSAT_PARAMETERS()            *
*************************************/

/***************************************************************************
 This routine gets a structure from NVRAM, checks validity returning a "0"
   if there is an error, "1" otherwise.
***************************************************************************/

int get_intelsat_parameters( intelsat_parameters *output , int sat_num )
{
unsigned char	*ptr,
				check_sum = 0;
int	i;
	int int_status;

	ptr = intelsat_parameters_ptr[ sat_num ];

	int_status = return_interrupt_status ( );

	disable();

	for ( i=0; i < sizeof( intelsat_parameters ) ; i++ )
	{
		*((unsigned char *)output + i) = *(ptr + i);
		check_sum ^= *(ptr + i);
	}

	if ( ( *(ptr + i) == check_sum ) &&
		 ( *(ptr + i + 1) == (unsigned char)(~check_sum) ) )
	{
	if ( int_status )
		enable();
		return 1;
	}
	else
	{
	if ( int_status )
		enable();
		return 0;
	}
}


/*************************************
* PUT_CURRENT_SATELLITE_NUM()        *
*************************************/

/* Simplified since there is only one "int" variable. */

void put_current_satellite_num( int *input )
{
unsigned int	*ptr;
    int int_status;

	ptr = current_sat_num_ptr;

    int_status = return_interrupt_status ( );
	disable();

	*ptr = *((unsigned int *)input);
	*(ptr + 1) = ~( *((unsigned int *)input) );   /* Store the complement */

    if ( int_status )
    	enable();
}


/*************************************
* GET_CURRENT_SATELLITE_NUM()        *
*************************************/

/* Simplified since there is only one "int" variable. */

int	get_current_satellite_num( int *output )
{
unsigned int	*ptr,
				complement = 0;
    int int_status;

	ptr = current_sat_num_ptr;

    int_status = return_interrupt_status ( );
	disable();

	*((unsigned int *)output) = *ptr;
	complement = *(ptr + 1);

    if ( int_status )
    	enable();

	if ( *((unsigned int *)output) == ~complement )
		return 1;
	else
		return 0;
}


/*************************************
* PUT_STAR_DETAILS()                 *
*************************************/

/***************************************************************************
This routine is used to store one data structure into NVRAM.
***************************************************************************/

void put_star_details( star_details_struct *input , int star_num )
{
unsigned char	*ptr,
				check_sum = 0;
int	i;
    int int_status;

	ptr = star_details_ptr[ star_num ];

    int_status = return_interrupt_status ( );
	disable();

	for ( i=0; i < sizeof( star_details_struct ) ; i++ )
	{
		/* Store data in consecutive bytes and calculate the XOR check-sum. */

		*(ptr + i) = *((unsigned char *)input + i);
		check_sum ^= *((unsigned char *)input + i);
	}

	/* Now store the check-sum with its complement for error checking. */

	*(ptr + i) = check_sum;
	*(ptr + i + 1) = ~check_sum;

    if ( int_status )
    	enable();
}


/*************************************
* GET_STAR_DETAILS()                 *
*************************************/

/***************************************************************************
 This routine gets a structure from NVRAM, checks validity returning a "0"
   if there is an error, "1" otherwise.
***************************************************************************/

int get_star_details ( star_details_struct *output , int star_num )
{
unsigned char	*ptr,
				check_sum = 0;
int	i;
    int int_status;

	ptr = star_details_ptr[ star_num ];

    int_status = return_interrupt_status ( );
	disable();

	for ( i=0; i < sizeof( star_details_struct ) ; i++ )
	{
		*((unsigned char *)output + i) = *(ptr + i);
        check_sum ^= *(ptr + i);
	}

	if ( ( *(ptr + i) == check_sum ) &&
		 ( *(ptr + i + 1) == (unsigned char)(~check_sum) ) )
	{
    if ( int_status )
    		enable();
		return 1;
	}
	else
	{
    if ( int_status )
    		enable();
		return 0;
    }
}


/*************************************
* PUT_CURRENT_STAR_NUM()             *
*************************************/

/* Simplified since there is only one "int" variable. */

void put_current_star_num( int *input )
{
unsigned int	*ptr;
    int int_status;

	ptr = current_star_num_ptr;

    int_status = return_interrupt_status ( );
	disable();

	*ptr = *((unsigned int *)input);
	*(ptr + 1) = ~( *((unsigned int *)input) );   /* Store the complement */

    if ( int_status )
        enable();
}


/*************************************
* GET_CURRENT_STAR_NUM()             *
*************************************/

/* Simplified since there is only one "int" variable. */

int	get_current_star_num( int *output )
{
unsigned int	*ptr,
				complement = 0;
    int int_status;

	ptr = current_star_num_ptr;

    int_status = return_interrupt_status ( );
	disable();

	*((unsigned int *)output) = *ptr;
	complement = *(ptr + 1);

    if ( int_status )
        enable();

	if ( *((unsigned int *)output) == ~complement )
		return 1;
	else
		return 0;
}


/*************************************
* PUT_BEACON_DETAILS()               *
*************************************/

void put_beacon_details( beacon_details_struct *input )
{
unsigned char	*ptr,
				check_sum = 0;
int	i;
    int int_status;

	ptr = beacon_details_ptr;

    int_status = return_interrupt_status ( );
	disable();

	for ( i=0; i < sizeof( beacon_details_struct ) ; i++ )
	{
		/* Store data in consecutive bytes and calculate the XOR check-sum. */

		*(ptr + i) = *((unsigned char *)input + i);
		check_sum ^= *((unsigned char *)input + i);
	}

	/* Now store the check-sum with its complement for error checking. */

	*(ptr + i) = check_sum;
	*(ptr + i + 1) = ~check_sum;

    if ( int_status )
    	enable();
}


/*************************************
* GET_BEACON_DETAILS()               *
*************************************/

int	get_beacon_details( beacon_details_struct *output )
{
unsigned char	*ptr,
				check_sum = 0;
int	i;
    int int_status;

	ptr = beacon_details_ptr;

    int_status = return_interrupt_status ( );
	disable();

	for ( i=0; i < sizeof( beacon_details_struct ) ; i++ )
	{
		*((unsigned char *)output + i) = *(ptr + i);
        check_sum ^= *(ptr + i);
	}


	if ( ( *(ptr + i) == check_sum ) &&
		 ( *(ptr + i + 1) == (unsigned char)(~check_sum) ) )
	{
    if ( int_status )
    		enable();
		return 1;
	}
	else
	{
    if ( int_status )
    		enable();
		return 0;
    }
}


/*************************************
* PUT_ORBIT_TRACK_START_TYPE()       *
*************************************/

/* Simplified since there is only one "int" variable. */

void put_orbit_start_type( int *input )
{
unsigned int	*ptr;
    int int_status;

	ptr = orbit_start_ptr;

    int_status = return_interrupt_status ( );
	disable();

	*ptr = *((unsigned int *)input);
	*(ptr + 1) = ~( *((unsigned int *)input) );   /* Store the complement */

    if ( int_status )
       enable();
}


/*************************************
* GET_ORBIT_TRACK_START_TYPE()       *
*************************************/

/* Simplified since there is only one "int" variable. */

int	get_orbit_start_type( int *output )
{
unsigned int	*ptr,
				complement = 0;
    int int_status;

	ptr = orbit_start_ptr;

    int_status = return_interrupt_status ( );
	disable();

	*((unsigned int *)output) = *ptr;
	complement = *(ptr + 1);

	if ( int_status )
    	enable();

	if ( *((unsigned int *)output) == ~complement )
		return 1;
	else
		return 0;
}


/*************************************
* PUT_SCAN_DATA()                    *
*************************************/

void put_scan_data( scan_data *input )
{
unsigned char	*ptr,
				check_sum = 0;
int	i;
    int int_status;

	ptr = scan_data_ptr;

    int_status = return_interrupt_status ( );

	disable();

	for ( i=0; i < sizeof( scan_data ) ; i++ )
	{
		/* Store data in consecutive bytes and calculate the XOR check-sum. */

		*(ptr + i) = *((unsigned char *)input + i);
		check_sum ^= *((unsigned char *)input + i);
	}

	/* Now store the check-sum with its complement for error checking. */

	*(ptr + i) = check_sum;
	*(ptr + i + 1) = ~check_sum;

	if ( int_status )
		enable ( );
}



/*************************************
* GET_SCAN_DATA()                    *
*************************************/

int	get_scan_data( scan_data *output )
{
unsigned char	*ptr,
				check_sum = 0;
int	i;
    int int_status;

	ptr = scan_data_ptr;

    int_status = return_interrupt_status ( );

	disable();

	for ( i=0; i < sizeof( scan_data ) ; i++ )
	{
		*((unsigned char *)output + i) = *(ptr + i);
        check_sum ^= *(ptr + i);
	}


	if ( ( *(ptr + i) == check_sum ) &&
		 ( *(ptr + i + 1) == (unsigned char)(~check_sum) ) )
	{
	if ( int_status )
			enable();
		return 1;
	}
	else
	{
	if ( int_status )
			enable();
		return 0;
    }
}

/*************************************
* PUT_ORBIT_TRACK_MODEL()            *
*************************************/

/**************************************************************************
 Procedure to be called from MAIN which will assign the absolute NVRAM
	addresses to global data pointers. Also initialises any other global
	data, if required.
This routine is used to store one data structure into NVRAM.

***************************************************************************/

void put_orbit_track_model( orbit_model_struct *input , int sat_num )
{
unsigned char	*ptr,
				check_sum = 0;
int	i;

	/* N.B. This routine is externally protected by semaphores in the Orbit
	Track module. The diagnostic module which is currently the only other module which
	will call this routine, should be OK since the Orbit Track tasks would
	not be running in this mode. */

	ptr = orbit_model_ptr[ sat_num ];

	for ( i=0; i < sizeof( orbit_model_struct ) ; i++ )
	{
		/* Store data in consecutive bytes and calculate the XOR check-sum. */

		*(ptr + i) = *((unsigned char *)input + i);
		check_sum ^= *((unsigned char *)input + i);
	}

	/* Now store the check-sum with its complement for error checking. */

	*(ptr + i) = check_sum;
	*(ptr + i + 1) = ~check_sum;

}


/*************************************
* GET_ORBIT_TRACK_MODEL()            *
*************************************/

/***************************************************************************
 This routine gets a structure from NVRAM, checks validity returning a "0"
   if there is an error, "1" otherwise.
***************************************************************************/

int get_orbit_track_model( orbit_model_struct *output , int sat_num )
{
unsigned char	*ptr,
				check_sum = 0;
int	i;

	/* N.B. This routine is externally protected by semaphores in the Orbit
	Track module. The diagnostic module which is currently the only other module which
	will call this routine, should be OK since the Orbit Track tasks would
	not be running in this mode. */

	ptr = orbit_model_ptr[ sat_num ];

	for ( i=0; i < sizeof( orbit_model_struct ) ; i++ )
	{
		*((unsigned char *)output + i) = *(ptr + i);
        check_sum ^= *(ptr + i);
	}

	if ( (*(ptr + i) == check_sum) &&
		 (*(ptr + i + 1) == (unsigned char)(~check_sum)) )
	{
		return 1;
	}
	else
		return 0;
}


/*************************************
* PUT_ORBIT_TRACK_COVARIANCE()       *
*************************************/

/**************************************************************************
 Procedure to be called from MAIN which will assign the absolute NVRAM
	addresses to global data pointers. Also initialises any other global
	data, if required.
This routine is used to store one data structure into NVRAM.

***************************************************************************/

void put_orbit_track_covariance( orbit_covariance_struct *input , int sat_num )
{
unsigned char	*ptr,
				check_sum = 0;
int	i;

	/* N.B. This routine is externally protected by semaphores in the Orbit
	Track module. The diagnostic module which is currently the only other module which
	will call this routine, should be OK since the Orbit Track tasks would
	not be running in this mode. */

	ptr = orbit_covariance_ptr[ sat_num ];

	for ( i=0; i < sizeof( orbit_covariance_struct ) ; i++ )
	{
		/* Store data in consecutive bytes and calculate the XOR check-sum. */

		*(ptr + i) = *((unsigned char *)input + i);
		check_sum ^= *((unsigned char *)input + i);
	}

	/* Now store the check-sum with its complement for error checking. */

	*(ptr + i) = check_sum;
	*(ptr + i + 1) = ~check_sum;
}


/*************************************
* GET_ORBIT_TRACK_COVARIANCE()       *
*************************************/

/***************************************************************************
 This routine gets a structure from NVRAM, checks validity returning a "0"
   if there is an error, "1" otherwise.
***************************************************************************/

int get_orbit_track_covariance( orbit_covariance_struct *output , int sat_num )
{
unsigned char	*ptr,
				check_sum = 0;
int	i;

	/* N.B. This routine is externally protected by semaphores in the Orbit
	Track module. The diagnostic module which is currently the only other module which
	will call this routine, should be OK since the Orbit Track tasks would
	not be running in this mode. */

	ptr = orbit_covariance_ptr[ sat_num ];

	for ( i=0; i < sizeof( orbit_covariance_struct ) ; i++ )
	{
		*((unsigned char *)output + i) = *(ptr + i);
        check_sum ^= *(ptr + i);
	}

	if ( ( *(ptr + i) == check_sum ) &&
		 ( *(ptr + i + 1) == (unsigned char)(~check_sum) ) )
	{
		return 1;
	}
	else
		return 0;
}


/*
****************************************************************************
put_controller_parameters

	Routine to put controller parameters into NVRAM.

This routine should only be called in diagnostic or at system startup.

Parameters:
	Address of controller parameter structure

****************************************************************************
*/
void put_controller_parameters ( unsigned int index, 
						controller_parameter_struct *input ) {

	unsigned char	*ptr, check_sum = 0;
	int	i;
    int int_status;

	ptr = con_ptr [ index ];

    int_status = return_interrupt_status ( );
	disable();

	for ( i=0; i < sizeof ( controller_parameter_struct ) ; i++ )
	{
		/* Store data in consecutive bytes and calculate the XOR check-sum. */
		*(ptr + i) = *((unsigned char *)input + i);
		check_sum ^= *((unsigned char *)input + i);
	}

	/* Now store the check-sum with its complement for error checking. */
	*(ptr + i) = check_sum;
	*(ptr + i + 1) = ~check_sum;

	if ( int_status )
		enable ( );

} /* end of put_controller_parameters */



/*
****************************************************************************
get_controller_parameters

	Routine to get controller parameters from NVRAM.

This routine should only be called in diagnostic or at system startup.

Parameters:
	Address of controller parameters structure

Returns:
	NVRAM successful or unsuccessful

****************************************************************************
*/

int	get_controller_parameters ( unsigned int index,
				controller_parameter_struct *output ) {

	unsigned char *ptr, check_sum = 0;
	int	i;
    	int int_status;

	ptr = con_ptr [ index ];

	int_status = return_interrupt_status ( );
	disable();

	for ( i=0; i < sizeof ( controller_parameter_struct ) ; i++ )
		{
		*((unsigned char *)output + i) = *(ptr + i);
        check_sum ^= *(ptr + i);
		}

	if ( ( *(ptr + i) == check_sum ) &&
		 ( *(ptr + i + 1) == (unsigned char)(~check_sum) ) ) {
		if ( int_status )
				enable();

		return 1;
		}
	else
		{
		if ( int_status )
				enable();

		return 0;
	    }

} /* end of get_controller_parameters */



/*
****************************************************************************
put_seq_software_limits

	Routine to put software limits into NVRAM.

This routine should only be called in diagnostic or at system startup.

Parameters:
	Address of software limit structure

Returns:
	NVRAM successful or unsuccessful


****************************************************************************
*/
void put_seq_software_limits ( software_limit_struct * input ) {

	unsigned char	*ptr, check_sum = 0;
	int	i;
    int int_status;

	ptr = sw_limits_ptr;

    int_status = return_interrupt_status ( );
	disable();

	for ( i=0; i < sizeof ( software_limit_struct ) ; i++ )
	{
		/* Store data in consecutive bytes and calculate the XOR check-sum. */
		*(ptr + i) = *((unsigned char *)input + i);
		check_sum ^= *((unsigned char *)input + i);
	}

	/* Now store the check-sum with its complement for error checking. */
	*(ptr + i) = check_sum;
	*(ptr + i + 1) = ~check_sum;

	if ( int_status )
		enable ( );

} /* end of put_seq_software_limits */



/*
****************************************************************************
get_seq_software_limits

	Routine to get sequnecer software limits from NVRAM.

This routine should only be called in diagnostic or at system startup.


Parameters:
	Address of software limit structure

Returns:
	NVRAM successful or unsuccessful

****************************************************************************
*/
int get_seq_software_limits ( software_limit_struct * output ) {

	unsigned char *ptr, check_sum = 0;
	int	i;
    int int_status;

	ptr = sw_limits_ptr;

    int_status = return_interrupt_status ( );
	disable();

	for ( i=0; i < sizeof ( software_limit_struct ) ; i++ )
		{
		*((unsigned char *)output + i) = *(ptr + i);
        check_sum ^= *(ptr + i);
		}

	if ( ( *(ptr + i) == check_sum ) &&
		 ( *(ptr + i + 1) == (unsigned char)(~check_sum) ) ) {
		if ( int_status )
				enable();

		return 1;
		}
	else
		{
		if ( int_status )
				enable();

		return 0;
	    }

} /* end of get_software_limits */


/*
****************************************************************************
get_stow_positions

	Routine to get sequencer stow positions from NVRAM.

Parameters:
	Address of stow position structure

Returns:
	NVRAM successful or unsuccessful

****************************************************************************
*/
int get_stow_positions ( stow_pos_struct * output ) {

	unsigned char *ptr, check_sum = 0;
	int	i;
    int int_status;

	ptr = stow_pos_ptr;

    int_status = return_interrupt_status ( );
	disable();

	for ( i=0; i < sizeof ( stow_pos_struct ) ; i++ )
		{
		*((unsigned char *)output + i) = *(ptr + i);
        check_sum ^= *(ptr + i);
		}

	if ( ( *(ptr + i) == check_sum ) &&
		 ( *(ptr + i + 1) == (unsigned char)(~check_sum) ) ) {
		if ( int_status )
				enable();

		return 1;
		}
	else
		{
		if ( int_status )
				enable();

		return 0;
	    }

} /* end of get_stow_positions */



/*
****************************************************************************
put_stow_positions

	Routine to put stow positions into NVRAM.

Parameters:
	Address of stow position structure

Returns:
	NVRAM successful or unsuccessful


****************************************************************************
*/
void put_stow_positions ( stow_pos_struct * input ) {

	unsigned char	*ptr, check_sum = 0;
	int	i;
    int int_status;

	ptr = stow_pos_ptr;

    int_status = return_interrupt_status ( );
	disable();

	for ( i=0; i < sizeof ( stow_pos_struct ) ; i++ )
	{
		/* Store data in consecutive bytes and calculate the XOR check-sum. */
		*(ptr + i) = *((unsigned char *)input + i);
		check_sum ^= *((unsigned char *)input + i);
	}

	/* Now store the check-sum with its complement for error checking. */
	*(ptr + i) = check_sum;
	*(ptr + i + 1) = ~check_sum;

	if ( int_status )
		enable ( );

} /* end of put_stow_positions */

/*************************************
* GET_CALIBRATION_COEFFICIENTS()       *
*************************************/

/***************************************************************************
 This routine gets a structure from NVRAM, checks validity returning a "0"
   if there is an error, "1" otherwise.
***************************************************************************/

int get_calibration_coefficients ( calibration_coefficients *cal )

{ 
   return copy_from_nvram 
     ( nv_calibration_coefficients, 
       cal,
       sizeof ( calibration_coefficients ) );
}

static int copy_from_nvram ( const void *from, const void *to, const size_t n )

{  
   unsigned char huge *out = ( unsigned char huge * ) to;
   unsigned char huge *in  = ( unsigned char huge * ) from;
   unsigned char checksum;
   int i, valid_copy, interrupts_on_at_entry = return_interrupt_status ();

   disable ();

   for ( i = 0, checksum = 0 ; i < n ; i++ )
       checksum ^= out[i] = in[i];

   valid_copy = ( ( in[n] == checksum )
                    && ( in[n+1] == ( unsigned char ) ~checksum ) );

   if ( interrupts_on_at_entry ) enable ();
   return valid_copy;
}


  

/*************************************
* PUT_CALIBRATION_COEFFICIENTS()            *
*************************************/

void put_calibration_coefficients ( calibration_coefficients *cal )
{
   copy_into_nvram ( cal, nv_calibration_coefficients,
                     sizeof ( calibration_coefficients ) );
}

static void copy_into_nvram
   ( const void *from, const void *to, const size_t n )
{
   unsigned char huge *out = ( unsigned char huge * ) to;
   unsigned char huge *in =  ( unsigned char huge * ) from;
   unsigned char checksum;
   int i, interrupts_on_at_entry = return_interrupt_status ();

   disable ();

   for ( i = 0, checksum = 0 ; i < n ; i++ )
      checksum ^= out[i] = in[i];

   out[n] = checksum;
   out[n+1] = ~checksum;

   if ( interrupts_on_at_entry ) enable ();
}/****************************************************************************/

/*************************************
* GET_STATION_POSITION()       	     *
*************************************/
/***************************************************************************
 This routine gets a structure from NVRAM, checks validity returning a "0"
   if there is an error, "1" otherwise.
***************************************************************************/

int get_station_position( station_position *stat_loc )

{ 
   return copy_from_nvram 
     		( station_position_ptr, stat_loc, sizeof(station_position) );
}

/****************************************************************************/

/*************************************
* PUT_STATION_POSITION()             *
*************************************/

void put_station_position ( station_position *stat_loc )
{
   copy_into_nvram 
   	      ( stat_loc, station_position_ptr, sizeof ( station_position ) );
}

/****************************************************************************/
