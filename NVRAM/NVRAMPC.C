
/******************************************************************************
* MODULE :- ATCU NVRAM Controller					 File : NVRAMPC.C
*******************************************************************************
*
* DATE (Last modified) :- 12 September 1995
*
* UPDATE : Bug hunting ----> modified load_from_nvram () code
*
* DISCUSSION : This file contains all the routines used to access data stored
*	in NVRAM. These routines are an interface between the rest of the ATCU
*	and the NVRAM hardware. Data is accessed via a pointer addressing scheme
*	which is initialised in the header file "nvram.h".
*   Two bytes are reserved for storing the check-sum and its complement.
*
*	To add new data structures for storage the following steps should be taken:
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
*       12. DJLB 2-3-92. Added facility to force loading of default values, 
*			when used as a part of NVRAMTST.
*
*		13. LJS 16-4-94. Changed hardware to PCDISK 2.88MB ALP  card
*			This requires major modifications to the way the NVRAM is used.
*			Designed NVRAM driver using "virtual linear memory" and a memory
*			management model that uses the 2k mapped memory window of the card.
*
*
******************************************************************************/

/*
******************************************************************************
New Hardware/Software Description
---------------------------------

The PCDISK Card is a Lanner card that is normally used as a Disk emulator.
It is possible to access the SRAM chips directly however. This module was
originally designed using a linear mapped NVRAM memory model. To maintain the
NVRAM accessing routines used in the Geraldton system it is easiest to
merely modify the routines that access the structures and variables stored
in NVRAM thus maintaining the protoytpes used by the rest of the system.

The philosophy used to map data into the new memory system is to give the
memory the appearance of being linear where in fact it is accessed using
a 2k window into the PCDISK card.

Hardware Description
--------------------
Bank 0 SRAM Chips.
Installed (say 128k SRAM chips)
J1 - 1:2 Open (LI battery)
J2 - 1:6, 2:5 Shorted (FLASH memory)
J3 - 1:3 2:4 Shorted
J4 - 1:6 Short, 2:5 Open (SRAM)
J5 - 1:6 Open, 2:5 Short 128k SRAM chips installed
J6 - BIOS Address 1:6, 2:5, 3:4 all open E0000 to E1FFF
J7 - 1:2 Open, 2:3 Short (FLASH)

SW0 - 1   2    3    4
	OFF  OFF  OFF  ON

SW1 - 1   2    3    4
	ON    OFF  ON   ON

SRAM disk would be drive E: in DOS v5

Accessing Memory
---------------- 
The card memory is accessed as follows:

The BASE address talked about earlier is the BIOS address set using J6.

1. BASE + 0x147f selects chip and bank

There are 5 chips each with 128k. We will be using Bank 0 (SRAM)

Bit: 3 2 1 0
	 b c c c
	 b=bank
     ccc = chip (0-4)

2. BASE + 0x143f selects page

Each page is 2k in length (the length of the window). Each chip has
128k so there are 64 pages in each chip. 

3. BASE + 0x1800 is the start of the 2k memory window.

Software Description
--------------------

The data to be stored in NVRAM is given a virtual memory location in
routine "setup_nvram_pointers" which
is linear (from 0-128k). This memory is then mapped into the pages of
the NVRAM using the following algorithm.

mem is the linear memory

chip = floor( mem/128k )
remainder = mem - chip*128k
page = floor(remainder/2048)
offset = (remainder - page*2048)

e.g. data.address = 2100

|-----| chip 0, page 0, offset 0
|     |
|     |
|     |
|     |
|-----| chip 0, page 1, offset 0
|     |
|     |<---data = chip 0, page 1, offset = 52
|     |
|-----| chip 0, page 2, offset 0
|     |
|     |

   -------------------------------------------------------------------------------
** Note: The driver assumes that the bit 3 of the chip register is ALWAYS set to 0.
		 This is the bank 0 of the NVRAM card (SRAM chips)
   -------------------------------------------------------------------------------
******************************************************************************
*/

#include <string.h>
#include <stdlib.h>
#include <dos.h>
#include <stdio.h>
#include <stddef.h>
#include <math.h>
#include <alloc.h>

#include "posext.h"
#include "unosasm.h"
#include "nvram.h"
#include "nvramext.h"
#include "unos.h"
#include "seqext.h"
#include "main_ext.h"
#include "inteldef.h"	// has intelsat default data

/**************  (1) Pointer Declaration  ******************/
/* These are pointers into  virtual  NVRAM address space */

/* In the next #define line, the word `static' must be removed when 
	compiling NVRAMTST. For compilation of ATCU, the word `static'
	must follow the word `private'. See also change required in
	routine setup_nvram_pointers().  */

#define private static

void load_nvram ( unsigned long linear_mem, unsigned char * input, int data_size, char prot );
int load_from_nvram ( unsigned long linear_mem, unsigned char * output, int data_size, char prot );

// The following pointers are virtual memory pointers in a flat
// memory space. The flat space is then translated into the
// paged memory space of the PCDISK NVRAM card.
// LJS 16-4-94

// N.B. These are global for the benefit of nvramtst. At some point
// return functions should be built for these to hide them from everyone

unsigned long
			station_position_ptr,
			current_sat_num_ptr,
			sw_limits_ptr,					/* Software limits */
			stow_pos_ptr,					/* Stow positions */
			con_ptr [ 2 ],
			intelsat_param_ptr [ NUM_SATELLITES ],
			intelsat_epoch_ptr [ NUM_SATELLITES ];


private unsigned long mem_pointer;       // Now only used in simulation of NVRAM
										// LJS 16 april 1994


// NVRAM Constants (PCDISK)
#define NVRAMCHIP_SIZE		131072UL
#define NVRAMPAGE_SIZE		63
#define NVRAMPAGE_LENGTH	2047UL
#define NVRAMNUMCHIPS		5

#define PROTECT		1
#define NOPROTECT	0

// Addresses of the NVRAM registers
// NVRAM_BASE_ADDR is defined in nvram.h
//
private unsigned char * chip_reg = (unsigned char*)MK_FP(NVRAM_BASE_ADDR, 0x147f);
private unsigned char * page_reg = (unsigned char*)MK_FP(NVRAM_BASE_ADDR, 0x143f);
private unsigned long load_reg = NVRAM_BASE_ADDR + 0x1800;
static int nvram_sim;

static unsigned int NVRAMError = 0;
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
*
* Note that with the new card memory design nvram_mem was made global to this module
*************************************************************************/

// Routine to return NVRAM error word
unsigned int ReturnNVRAMError ( ) {

	return NVRAMError;
}

// Routine to Clear NVRAM error word
void ClearNVRAMError ( ) {

	NVRAMError = 0;
}

static int	setup_nvram_pointers(  void )
{
int	i;
unsigned long linear_addr, stored_data_size;

	/* If being run on a computer without NVRAM card installed,
	the following allows use of some RAM as if it were the NVRAM card.
	the variable nvram_sim is set in main() by the 2nd command-line
	parameter. (nvram_sim = 0 indicates nvram is to be used.)*/

	nvram_sim = get_nvram_sim();	// get_nvram_sim() is now in nvramtsts.c also

	if ( nvram_sim == 0 )
		linear_addr = 0;
	else {
		// For the NVRAM simulator I found it
		// necessary in borlandc v4 to allocate whatever
		// memory was available using farcoreleft() first.
		// Calling farmalloc or malloc on their own never
		// seemed to allocate enough ram while running
		// the debugger. LJS April 1994
		mem_pointer = (unsigned long)farmalloc( farcoreleft() );
		linear_addr = 0;
		if ( mem_pointer == NULL )
			{
			printf("\nError allocating memory from DOS...");
			exit(1);
			}
		}

	// Build virtual memory map of data

	station_position_ptr = linear_addr;
	stored_data_size = sizeof( station_position ) + 2;
	linear_addr += stored_data_size;

	/************ Sequencer Area *********************************/
	/*---- Controller parameter Algorithm Constants */
	for ( i = 0; i < 2; i++ ) {
		con_ptr [ i ] = ( linear_addr );
		stored_data_size = sizeof( controller_parameter_struct ) + 2;
		linear_addr += stored_data_size;
		}

	/*---- Sequencer Software Limits */
	sw_limits_ptr = ( linear_addr );
	stored_data_size = sizeof( software_limit_struct ) + 2;
	linear_addr += stored_data_size;

	/*---- Sequencer Stow positions */
	stow_pos_ptr = ( linear_addr );
	stored_data_size = sizeof( stow_pos_struct ) + 2;
	linear_addr += stored_data_size;

	/************* End of Sequencer NVRAM ******************/

	for( i=0; i < NUM_SATELLITES; i++ )
	{
		intelsat_param_ptr [ i ] = (linear_addr);
		stored_data_size = sizeof(IntelsatDataStructure) + 2;
		linear_addr += stored_data_size;
	}

	for( i=0; i < NUM_SATELLITES; i++ )
	{
		intelsat_epoch_ptr [ i ] = (linear_addr);
		stored_data_size = sizeof(TimeRecord) + 2;
		linear_addr += stored_data_size;
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
station_position		stat_loc;
station_position		 default_stat_loc = { -38.8, 145.0, 0.0, 0.0, 0.0 };

/* Sequencer Structures */
controller_parameter_struct 	con_temp;
software_limit_struct       	sw_limit_temp;
stow_pos_struct					stow_pos_temp;
IntelsatDataStructure 			intelsat_param;
TimeRecord						intelsat_epoch;

	/* NVRAM Initialisation */
	if (mode == NVRAM_INIT_MODE)
	{
		if (!setup_nvram_pointers())
			return 0;
	}

	/* Access all NVRAM data to ensure correct operation.
	   Initialise if required. (3)
	*/

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

	//********************************************************************
	// Intelsat Area
	//
	for ( index = 0; index < NUM_SATELLITES; index++ )
	{
		if ( !get_intelsat_parameters( index, (IntelsatDataStructure *)&intelsat_param )
			|| mode == LOAD_DEFAULT_VALUES )
		{
			/* Error in data checksum. */

			if ( mode == NVRAM_TEST_MODE )
				return 0;		/* Diagnost testing shows a fault. No point
									in loading factory settings. */

			/* Error due to corrupted or previously unused NVRAM. Flag an error
			   in case this is due to corrupted NVRAM, and initialise data. */
			intelsat_param = (intelsatdefdata [ index ]);

			put_intelsat_parameters( index, (IntelsatDataStructure *)&intelsat_param );

			if ( !get_intelsat_parameters( index, (IntelsatDataStructure  *)&intelsat_param ) )
				return 0;		/* Must be seriously faulty NVRAM. */
		}
	}


	for ( index = 0; index < NUM_SATELLITES; index++ )
	{
		if ( !get_intelsat_epoch( index, (TimeRecord *)&intelsat_epoch )
			|| mode == LOAD_DEFAULT_VALUES )
		{
			/* Error in data checksum. */

			if ( mode == NVRAM_TEST_MODE )
				return 0;		/* Diagnost testing shows a fault. No point
									in loading factory settings. */

			/* Error due to corrupted or previously unused NVRAM. Flag an error
			   in case this is due to corrupted NVRAM, and initialise data. */
			intelsat_epoch = (intelsatdefepochs [ index ]);

			put_intelsat_epoch ( index, (TimeRecord *)&intelsat_epoch );

			if ( !get_intelsat_epoch( index, (TimeRecord *)&intelsat_epoch ) )
				return 0;		/* Must be seriously faulty NVRAM. */
		}
	}


	return 1;
}

/*
***************************************************************************
load_nvram

Reentrant routine to write to NVRAM (PCDISK).

Note that some get/put routines must pass a large number f bytes and
therefore are protected by semaphores in some code (.e.g. getting
models etc). To cater for this we must specify whether we wish to
put disbale/enables around the code. NOTE: that disables are ESSENTIAL
when fiddling with the NVRAM registers.

***************************************************************************
*/
void load_nvram ( unsigned long linear_mem, unsigned char * input, int data_size, char prot )
{

	unsigned char check_sum = 0;
	int	i, int_status;
	unsigned char chip;
	unsigned char page;
	unsigned int offset, rem;

	if ( prot ) {
		int_status = return_interrupt_status ( );
		disable();
		}

	if ( nvram_sim ) {
		// make pointer into the segment and offset for this data
		load_reg = mem_pointer + linear_mem;
		offset = 0;
		}
	else
		{
		// Compute the starting chip, page and offset for this data
		chip = floor(linear_mem/NVRAMCHIP_SIZE);
		rem  = (unsigned int)(linear_mem - chip*NVRAMCHIP_SIZE);
		page = floor ( rem/(NVRAMPAGE_LENGTH + 1) );
		offset = (unsigned int)(rem - ( page*(NVRAMPAGE_LENGTH + 1)));

		}

	for ( i=0; i < data_size; i++ )
		{
		/* Store data in consecutive bytes and calculate the XOR check-sum. */
		disable();
		(*chip_reg)=(char)chip;
		(*page_reg)=(char)page;
		*((unsigned char *)(load_reg + offset)) = *((unsigned char *)input + i);
		check_sum ^= *((unsigned char *)input + i);
		if ( !prot )enable();

		offset++; // index into the next byte

		if ( !nvram_sim )
			// check for the page boundary (only if using real NVRAM card)
			if ( offset > NVRAMPAGE_LENGTH )
				{
				offset = 0;
				page++;

				// check for page overflow to next chip
				if ( page > NVRAMPAGE_SIZE )
					chip++;

				}
		}

	/* Now store the check-sum with its complement for error checking. */
	disable();
	(*chip_reg)=(char)chip;
	(*page_reg)=(char)page;

	*((unsigned char *)(load_reg + offset)) = check_sum;
	if ( !prot )enable();

	// have to check whether the next byte crosses a page boundary
	offset++; // index into the next byte

	if ( !nvram_sim )
		// check for the page boundary (only if using real NVRAM card)
		if ( offset > NVRAMPAGE_LENGTH )
			{
			offset = 0;
			page++;

			// check for page overflow to next chip
			if ( page > NVRAMPAGE_SIZE )
				chip++;

			}

	// Now load complement
	disable();
	(*chip_reg)=(char)chip;
	(*page_reg)=(char)page;
	*((unsigned char *)(load_reg + offset)) = ~check_sum;

	// This next line looks strange considering what follows but
	// we must reenable interrupts if required as this is just
	// register protection.
	if ( !prot )enable();


	if ( prot )
		if ( int_status )
			enable ( );

} /* end of load_nvram */



/*
***************************************************************************
12 september 1995
Trying to find the big but elusive bug in the system
On Len's Advice I am removing all enable/disable  calls
and all chip/page settings etc.

The original code is below

load_from_nvram

generic routine to read from NVRAM (PCDISK).

***************************************************************************
*/
int load_from_nvram ( unsigned long linear_mem, unsigned char * output, int data_size, char prot )
{

	unsigned char check_sum = 0;
	int	i, int_status;
	unsigned char chip;
	unsigned char page, nvramcomp_checksum, nvram_checksum;
	unsigned int offset, status, rem;

		// Get Far pointer into segment and offset for data
		load_reg =  mem_pointer + linear_mem;
		offset = 0;

	for ( i=0; i < data_size; i++ )
		{
		*((unsigned char *)output + i) = *((unsigned char *)(load_reg + offset));
		check_sum ^= *((unsigned char *)(load_reg + offset));

		offset++; // index into the next byte
		}

	/* Now store the check-sum with its complement for error checking. */
	nvram_checksum = *((unsigned char *)(load_reg + offset));
	offset++; // index into the next byte

	nvramcomp_checksum = *((unsigned char *)(load_reg + offset));


	if ( ( nvram_checksum == check_sum ) &&
		 ( nvramcomp_checksum == (unsigned char)(~check_sum) ) )
		status = 1;
	else
		status = 0;

	return status;

} /* end of load_from_nvram */

/*
***************************************************************************
OLD 	load_from_nvram
This is the original code before 12-September 1995

generic routine to read from NVRAM (PCDISK).

***************************************************************************
*/
int load_from_nvram_old ( unsigned long linear_mem, unsigned char * output, int data_size, char prot )
{

	unsigned char check_sum = 0;
	int	i, int_status;
	unsigned char chip;
	unsigned char page, nvramcomp_checksum, nvram_checksum;
	unsigned int offset, status, rem;

	if ( prot ) {
		int_status = return_interrupt_status ( );
		disable();
		}

	if ( nvram_sim ) {
		// Get Far pointer into segment and offset for data
		load_reg =  mem_pointer + linear_mem;
		offset = 0;
		}
	else
		{
		// Compute the starting chip, page and offset for this data
		chip = floor(linear_mem/NVRAMCHIP_SIZE);
		rem  = (unsigned int)(linear_mem - chip*NVRAMCHIP_SIZE);
		page = floor ( rem/(NVRAMPAGE_LENGTH + 1) );
		offset = (unsigned int)(rem - ( page*(NVRAMPAGE_LENGTH + 1)));
		}

	for ( i=0; i < data_size; i++ )
		{
		/* Store data in consecutive bytes and calculate the XOR check-sum. */
		disable();

		(*chip_reg)=(char)chip;
		(*page_reg)=(char)page;
		*((unsigned char *)output + i) = *((unsigned char *)(load_reg + offset));
		check_sum ^= *((unsigned char *)(load_reg + offset));

		if ( !prot )enable();

		offset++; // index into the next byte

		if ( !nvram_sim )
			// check for the page boundary (only if using real NVRAM card)
			if ( offset > NVRAMPAGE_LENGTH )
				{
				offset = 0;
				page++;

				// check for page overflow to next chip
				if ( page > NVRAMPAGE_SIZE )
					chip++;

				}
		}

	/* Now store the check-sum with its complement for error checking. */
	disable();
	(*chip_reg)=(char)chip;
	(*page_reg)=(char)page;
	nvram_checksum = *((unsigned char *)(load_reg + offset));
	if ( !prot )enable();

	// have to check whether the next byte crosses a page boundary
	offset++; // index into the next byte

	if ( !nvram_sim )
		// check for the page boundary (only if using real NVRAM card)
		if ( offset > NVRAMPAGE_LENGTH )
			{
			offset = 0;
			page++;

			// check for page overflow to next chip
			if ( page > NVRAMPAGE_SIZE )
				chip++;

			}

	// No check checksum and its complement and return status
	disable();

	(*chip_reg)=(char)chip;
	(*page_reg)=(char)page;
	nvramcomp_checksum = *((unsigned char *)(load_reg + offset));

	if ( !prot )enable();

	if ( ( nvram_checksum == check_sum ) &&
		 ( nvramcomp_checksum == (unsigned char)(~check_sum) ) )
		status = 1;
	else
		status = 0;

	if ( int_status && prot )
		enable();

	return status;

} /* end of load_from_nvram */


/*
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    						PUTS and GETS here


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
*/

/*************************************
* PUT_CURRENT_SATELLITE_NUM()        *
*************************************/

/* Simplified since there is only one "int" variable. */

void put_current_satellite_num( int *input ) {

load_nvram ( current_sat_num_ptr, (unsigned char *) input, sizeof(  int ), PROTECT);

}


/*************************************
* GET_CURRENT_SATELLITE_NUM()        *
*************************************/

/* Simplified since there is only one "int" variable. */

int	get_current_satellite_num( int *output ) {

int temp;
temp =load_from_nvram ( current_sat_num_ptr, (unsigned char *) output, sizeof( int ), PROTECT );
	if ( !temp )
		NVRAMError |= 0x02;

	return temp;
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

	load_nvram ( con_ptr [index], (unsigned char *) input, sizeof( controller_parameter_struct ), PROTECT );

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
int temp;
temp = load_from_nvram ( con_ptr [ index ], (unsigned char *) output, sizeof(  controller_parameter_struct ),
											  PROTECT );
	if ( !temp )
		NVRAMError |= 0x200;

	return temp;
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

	load_nvram ( sw_limits_ptr, (unsigned char *) input, sizeof( software_limit_struct ), PROTECT );

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

int temp;

temp = load_from_nvram ( sw_limits_ptr, (unsigned char *) output, sizeof( software_limit_struct ), PROTECT );

	if ( !temp )
		NVRAMError |= 0x400;

	return temp;
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

int temp;
temp = load_from_nvram ( stow_pos_ptr, (unsigned char *) output, sizeof( stow_pos_struct ), PROTECT );

	if ( !temp )
		NVRAMError |= 0x800;

	return temp;

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

	load_nvram ( stow_pos_ptr, (unsigned char *) input, sizeof( stow_pos_struct ), PROTECT );

} /* end of put_stow_positions */


/*************************************
* GET_STATION_POSITION()       	     *
*************************************/
int get_station_position( station_position *output ) {

int temp;
temp = load_from_nvram ( station_position_ptr, (unsigned char *) output,
				sizeof( station_position ), PROTECT );
	if ( !temp )
		NVRAMError |= 0x2000;

	return temp;
}

/*************************************
* PUT_STATION_POSITION()             *
*************************************/
void put_station_position ( station_position *input ) {

	load_nvram ( station_position_ptr, (unsigned char *) input, sizeof( station_position ), PROTECT );

}


/*************************************
* PUT_INTELSAT_PARAMETERS ()         *
*************************************/

void put_intelsat_parameters ( int sat_num, IntelsatDataStructure *input)
{
	load_nvram ( intelsat_param_ptr [sat_num ], (unsigned char *) input, sizeof( IntelsatDataStructure ), PROTECT );
}



/*************************************
* GET_INTELSAT_PARAMETERS ()         *
*************************************/

int get_intelsat_parameters ( int sat_num, IntelsatDataStructure *output )
{
int temp;
temp = load_from_nvram (  intelsat_param_ptr [ sat_num ], (unsigned char *) output,
					sizeof( IntelsatDataStructure ), PROTECT );
	if ( !temp )
		NVRAMError = 15;

	return temp;

}

/*************************************
* PUT_INTELSAT_EPOCH ()         *
*************************************/

void put_intelsat_epoch ( int sat_num, TimeRecord *input)
{
	load_nvram ( intelsat_epoch_ptr [sat_num ], (unsigned char *) input, sizeof( TimeRecord ), PROTECT );
}



/*************************************
* GET_INTELSAT_EPOCH ()         *
*************************************/

int get_intelsat_epoch ( int sat_num, TimeRecord *output )
{

return load_from_nvram (  intelsat_epoch_ptr [ sat_num ], (unsigned char *) output,
					sizeof( TimeRecord ), PROTECT );


}



//*****************************************************************************
// Some more useful NVRAM routines
int GetCurrentSatelliteNumber ( ) {
	int sat_num;

	get_current_satellite_num (&sat_num);

	return sat_num;

} // end of fbyGetCurrentSatelliteNumber

/*********************************** END *************************************/
