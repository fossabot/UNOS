/************************************************************************/
/*                                                                      */
/*                                                                      */
/*                                                                      */
/*                    LOW LEVEL SERIAL INTERRUPT ROUTINE MODULE         */
/*                                                                      */
/*                                      by                              */
/*                                                                      */
/*                                  Robert Betz                         */
/*              Department of Electrical and Computer Engineering       */
/*                            University of Newcastle                   */
/*                               (Copyright 1992)                       */
/*                                                                      */
/*                                                                      */
/*                                                                      */
/************************************************************************/


#include <stdlib.h>					/* Inport atexit procedure */
#include "general.h"     /* Import general definitions */
#include "serial.h"         /* Import rx and tx interrupt buffer
									data structures.
									*/
#include "serint.h"					/* Exporting the setup_interrupt_routine
									function.
									*/
#include "hwpc.h"		/* Import SetVector function */
#include <dos.h>					/* Import the I/O port function
									prototypes
									*/



/*

DESCRIPTION

This module contains the routines to allow the low level interrupt routines
to be set up for the serial channels. Specifically a certain number of
interrupt routines have been set up in this module, each of the interrupt
routines being almost identical. In fact the only difference between the
routines is the number stored as a static variable which allows the interrupt
routine to pick up the specific information for the particular serial
channel from a table of data structures which contains this information.

In order to fill out this table with the appropriate information a call is
made to the setup_interrupt_routine which will give one of the generic
interrupt handlers to the caller, set up the interrupt vector, and fill
out the appropriate interrupt routine data structure with the uart type
and the addresses of the interrupt buffers.

To allow more interrupt routines the generic routine has to be copied,
its name changed in accordance with the other examples below (ie. add the
appropriate number to the end of the routine), change the static variable
number within the routine to give the correct index into the interrupt
routine array structure, increase the size of the interrupt array structure
to accommodate the new routines and initialise the interrupt routine
pointer section of the structure to the names of the new interrupt routines.

As each interrupt routine is allocated during system initialisation a
variable keeps count of the next routine to be allocated and checks to
see if there is a routine to allocate. If there isn't then obviously the
interrupt allocation has been a failure.

*/



/************************************************************************/
/*																		*/
/*							FUNCTION PROTOTYPES							*/
/*																		*/
/************************************************************************/


void init_serint (void);
void interrupt generic_1st_level_intr_0 (...);
void interrupt generic_1st_level_intr_1 (...);
void interrupt generic_1st_level_intr_2 (...);
void interrupt generic_1st_level_intr_3 (...);
void interrupt generic_1st_level_intr_4 (...);
void interrupt generic_1st_level_intr_5 (...);
void interrupt generic_1st_level_intr_6 (...);
void interrupt generic_1st_level_intr_7 (...);
void interrupt generic_1st_level_intr_8 (...);
void interrupt generic_1st_level_intr_9 (...);
void interrupt generic_1st_level_intr_10 (...);
void interrupt generic_1st_level_intr_11 (...);
void interrupt generic_1st_level_intr_12 (...);
void interrupt generic_1st_level_intr_13 (...);
void interrupt generic_1st_level_intr_14 (...);
void interrupt generic_1st_level_intr_15 (...);
void interrupt generic_1st_level_intr_16 (...);
void interrupt generic_1st_level_intr_17 (...);
void interrupt generic_1st_level_intr_18 (...);
void interrupt generic_1st_level_intr_19 (...);




/************************************************************************/
/*                                                                      */
/*                          MAIN DATA STRUCTURES                        */
/*                                                                      */
/************************************************************************/


/* Interrupt array structure
This data structure contains the information that the generc interrupt
routines need to know to interact with a particular uart.
*/

typedef
	struct {
		void interrupt (*intr_routine) (...);
		rx_int_buffer* rx_int_bufptr;
		tx_int_buffer* tx_int_bufptr;
		rx_int_buffer* srx_int_bufptr;
		tx_int_buffer* stx_int_bufptr;
		char num_8259s;
		unsigned int addr_8259_mast;
		unsigned int addr_8259_slave;
		char uart_type;
		unsigned char intr_num;
		char intr_type;
		void interrupt (*old_intr_vector)(...);
	} intr_struc;


/* One component in the structure above is the interrupt type. This is used
to distinguish between an interrupt which is tied to the receive channel
or an interrupt which is tied to a transmit channel, or an interrupt
which is used for both transmit and receive interrupts.
*/
#define RXTX 0
				/* interrupt used for both receive and transmit */

#define RX 1
				/* interrupt used only for receive */

#define TX 2
				/* interrupt used only for transmit */



#define MAX_NUM_SERIAL_INTRS 20
					/* This definition contains the total number of interrupt
					routines that can be set up associated with the serial
					interrupts.
					*/


/* Now define the array of structures used for the serial interrupt routines
*/

intr_struc intr_array [MAX_NUM_SERIAL_INTRS];



/* Variable used to keep track of the next of the intr_array structures
to be allocated to an interrupt routine. The value stored in this variable
is compared with the MAX_NUM_SERIAL_INTRS value to determine whether the
interrupt routine can be generated.
*/

unsigned char total_num_serial_intrs = 0;





/*------------------------------------------------------------------------*/





/*
===========================================================================
|
| restore_interrupts
|
| This procedure is the "atexit" procedure that is called to restore the
| serial interrupts of the system.  This routine is used mainly for PC
| implementations.
|
| Parameters:	-	none
|
| Returns:		-	nothing
|
===========================================================================
*/

void restore_serial_interrupts (void) {

	int i;

	for (i = 0; i < total_num_serial_intrs; i++) {
		SetVector (intr_array [i].intr_num,
											intr_array [i].old_intr_vector);
	} /* for */
} /* end of restore_serial_interrupts */





/*------------------------------------------------------------------------*/





/*
============================================================================
|
| setup_interrupt_routine
|
| This function has to purpose of setting up an interrupt routine for a
| particular uart. The setting up process involves initialising the interrupt
| vector and then connecting the interrupt routine to the appropriate
| interrupt buffers and handling routine for the uart type.
|
| If the set up process is successful then a TRUE is returned, else a
| FALSE is returned. If the set up is unsuccessful it ca be due to no
| more interrupt routines being available or the uart type is not a supported
| one.
|
| Parameters:   -   uart_type - the type of uar hardware being used.
|               -   num_8259s - the number of 8259s connect to the uart.
|                   Valid values are 0, 1, 2.
|               -   addr_8259_mast - address of the 8259 master interrupt
|                   controller.
|               -   addr_8259_slave - address of the 8259 slave interrupt
|                   controller.
|               -   rx_intr_num - the number of the interrupt for the
|                   receive interrupt.
|               -   tx_intr_num - the number of the interrupt for the
|                   transmit interrupt.
|               -   rx_int_bufptr - pointer to the 1st receive interrupt
|                   buffer.
|               -   tx_int_bufptr - pointer to the 1st transmit interrupt
|                   buffer.
|               -   srx_int_bufptr - pointer to the 2nd receive interrupt
|                   buffer.
|               -   stx_int_bufptr - pointer to the 2nd transmit interrupt
|                   buffer.
|
| Returns:      -   TRUE if the interrupt set up is successful, else a FALSE.
|
============================================================================
*/

int setup_interrupt_routine (char uart_type, char num_8259s,
			unsigned int addr_8259_mast, unsigned int addr_8259_slave,
			unsigned char rx_intr_num, unsigned char tx_intr_num,
			rx_int_buffer* rx_int_bufptr, tx_int_buffer* tx_int_bufptr,
			rx_int_buffer* srx_int_bufptr, tx_int_buffer* stx_int_bufptr) {

	static char init_module = TRUE;

	/* firstly check to see if the module initialisation has to be called */

	if (init_module) {
		init_serint ();
		init_module = FALSE;
	} /* if */

	/* firstly check to see if there is an interrupt routine to be allocated
	*/

	if (total_num_serial_intrs >= MAX_NUM_SERIAL_INTRS) {
		/* do not have any interrupt routines left to allocate */
		return (FALSE);
	} /* if */

	/* now check for the particular interrupt type that there are enough
	interrupts available.
	*/
	switch (uart_type) {
		case I8251A :
			if ((MAX_NUM_SERIAL_INTRS - total_num_serial_intrs) < 2) {
				/* now enough interrupts left for this uart */
				return (FALSE);
			} /* if */
			tx_intr_num++;
			tx_intr_num--;
		break;

		case I8274 :
		case NS16450 :
			/* must be at least one to have got here */
		break;

		default :
			/* has been an illegal uart type nominated */
			return (FALSE);
	} /* switch */

	/* definitely enough interrupts available to get here. */
	/* now assign the interrupt structures for the interrupt/s and initialise
	the values in it. Set up the interrupt vector and save the old vector
	so that it can be replaced.
	*/
	switch (uart_type) {
		case I8251A :
			/* the 8251a needs two interrupts to be allocated, one for the
			receiver and one for the transmitter. This is probably the
			the normal case, but in certain hardware configurations the
			interrupt lines for these two interrupts are OR'd together
			so that one interrupt is used. At the moment this software
			does not support this hardware option.
			*/

		break;

		case I8274 :
		case NS16450 :
			/* both these uarts have a single interrupt line for all their
			internal interrupt sources
			*/
			intr_array [total_num_serial_intrs].rx_int_bufptr =
														rx_int_bufptr;
			intr_array [total_num_serial_intrs].tx_int_bufptr =
														tx_int_bufptr;
			intr_array [total_num_serial_intrs].srx_int_bufptr =
														srx_int_bufptr;
			intr_array [total_num_serial_intrs].stx_int_bufptr =
														stx_int_bufptr;
			intr_array [total_num_serial_intrs].num_8259s =
														num_8259s;
			intr_array [total_num_serial_intrs].addr_8259_mast =
														addr_8259_mast;
			intr_array [total_num_serial_intrs].addr_8259_slave =
														addr_8259_slave;
			intr_array [total_num_serial_intrs].uart_type =
														uart_type;

			/* now set up the interrupt vector for this routine. The
			previous contents of the interrupt vector is stored so that
			it can be restored at some later stage.
			*/
			intr_array [total_num_serial_intrs].intr_num = rx_intr_num;
			intr_array [total_num_serial_intrs].intr_type = RXTX;
			intr_array [total_num_serial_intrs].old_intr_vector = getvect (rx_intr_num);
			  SetVector (rx_intr_num,
					intr_array [total_num_serial_intrs].intr_routine);
			/* Now set up the atexit procedure to restore the interrupt
			vector.
			*/

		break;

		default :
			/* should never get here */
			return (FALSE);
	} /* switch */
	atexit (restore_serial_interrupts);
	total_num_serial_intrs++;
	return (TRUE);
} /* end of setup_interrupt_routine */





/*-------------------------------------------------------------------------*/





/*
============================================================================
|
| init_serint
|
| This function is called to carry out the initialisation of the serial
| interrupt module (ie. this module). The initialisation functions carried
| out are :-
|
|   -   initialise the array of interrupt structures with the interrupt
|       routine pointers.
|
| Parameters:   -   none
|
| Returns:      -   nothing
|
============================================================================
*/

void init_serint (void) {

	int i;

	/* now initialise the interrupt array */
	for (i = 0; i < MAX_NUM_SERIAL_INTRS; i++) {
		intr_array [i].rx_int_bufptr = NULL;
		intr_array [i].tx_int_bufptr = NULL;
		intr_array [i].srx_int_bufptr = NULL;
		intr_array [i].stx_int_bufptr = NULL;
		intr_array [i].old_intr_vector = NULL;
	} /* for */

	/* now initialise the interrupt routine addresses */
	intr_array [0].intr_routine = generic_1st_level_intr_0;
	intr_array [1].intr_routine = generic_1st_level_intr_1;
	intr_array [2].intr_routine = generic_1st_level_intr_2;
	intr_array [3].intr_routine = generic_1st_level_intr_3;
	intr_array [4].intr_routine = generic_1st_level_intr_4;
	intr_array [5].intr_routine = generic_1st_level_intr_5;
	intr_array [6].intr_routine = generic_1st_level_intr_6;
	intr_array [7].intr_routine = generic_1st_level_intr_7;
	intr_array [8].intr_routine = generic_1st_level_intr_8;
	intr_array [9].intr_routine = generic_1st_level_intr_9;
	intr_array [10].intr_routine = generic_1st_level_intr_10;
	intr_array [11].intr_routine = generic_1st_level_intr_11;
	intr_array [12].intr_routine = generic_1st_level_intr_12;
	intr_array [13].intr_routine = generic_1st_level_intr_13;
	intr_array [14].intr_routine = generic_1st_level_intr_14;
	intr_array [15].intr_routine = generic_1st_level_intr_15;
	intr_array [16].intr_routine = generic_1st_level_intr_16;
	intr_array [17].intr_routine = generic_1st_level_intr_17;
	intr_array [18].intr_routine = generic_1st_level_intr_18;
	intr_array [19].intr_routine = generic_1st_level_intr_19;
} /* end of init_serint */





/*-------------------------------------------------------------------------*/





/*
============================================================================
|
| generic_1st_level_intr_0
|
| This is the generic 1st level interrupt handler for all serial interrupts
|
| Parameters:   -   none
|
| Returns:      -   nothing
|
============================================================================
*/

void interrupt generic_1st_level_intr_0 ( ... ) {

	static unsigned char intr_index = 0;
	unsigned char intr_ident;

	/* Reset the interrupt controller */

	switch (intr_array [intr_index].num_8259s) {
		case 0 :
		break;

		case 1 :
			outportb (intr_array [intr_index].addr_8259_mast, EOI_8259);
		break;

		case 2 :
			outportb (intr_array [intr_index].addr_8259_mast, EOI_8259);
			outportb (intr_array [intr_index].addr_8259_slave, EOI_8259);
		break;
		default:
			;
	} /* switch */


	/* Now process the interrupts */
	switch (intr_array [intr_index].uart_type) {
		case I8251A :
			/* now determine whether the interrupt is for a receive or
			transmit interrupt.
			*/
			if (intr_array [intr_index].intr_type == RX) {
				receive_interrupt_handler_8251A (intr_array [intr_index].
												rx_int_bufptr);
			} /* if */
			else {
				transmit_interrupt_handler_8251A (intr_array [intr_index].
												tx_int_bufptr);
			} /* else */
		break;


		case NS16450 :
			intr_ident = inportb
					(intr_array [intr_index].rx_int_bufptr->com_reg_add + 2);
			while (((intr_ident & 0x01) == 0)) {
				interrupt_handler_16450 (
							intr_array [intr_index].rx_int_bufptr,
							intr_array [intr_index].tx_int_bufptr, intr_ident);
				intr_ident = inportb
					(intr_array [intr_index].rx_int_bufptr->com_reg_add + 2);
			} /* while */
		break;

		case I8274 :
			interrupt_handler_8274 (intr_array [intr_index].rx_int_bufptr,
						intr_array [intr_index].tx_int_bufptr,
						intr_array [intr_index].srx_int_bufptr,
						intr_array [intr_index].stx_int_bufptr);
		break;
	} /* switch */

} /* end of generic_1st_level_intr_0 */





/*-------------------------------------------------------------------------*/





/*
============================================================================
|
| generic_1st_level_intr_1
|
| This is the generic 1st level interrupt handler for all serial interrupts
|
| Parameters:   -   none
|
| Returns:      -   nothing
|
============================================================================
*/

void interrupt generic_1st_level_intr_1 (...) {

	static unsigned char intr_index = 1;
	unsigned char intr_ident;

	/* Reset the interrupt controller */

	switch (intr_array [intr_index].num_8259s) {
		case 0 :
		break;

		case 1 :
			outportb (intr_array [intr_index].addr_8259_mast, EOI_8259);
		break;

		case 2 :
			outportb (intr_array [intr_index].addr_8259_mast, EOI_8259);
			outportb (intr_array [intr_index].addr_8259_slave, EOI_8259);
		break;
	} /* switch */


	/* Now process the interrupts */
	switch (intr_array [intr_index].uart_type) {
		case I8251A :
			/* now determine whether the interrupt is for a receive or
			transmit interrupt.
			*/
			if (intr_array [intr_index].intr_type == RX) {
				receive_interrupt_handler_8251A (intr_array [intr_index].
												rx_int_bufptr);
			} /* if */
			else {
				transmit_interrupt_handler_8251A (intr_array [intr_index].
												tx_int_bufptr);
			} /* else */
		break;


		case NS16450 :
			intr_ident = inportb
					(intr_array [intr_index].rx_int_bufptr->com_reg_add + 2);
			while (((intr_ident & 0x01) == 0)) {
				interrupt_handler_16450 (
							intr_array [intr_index].rx_int_bufptr,
							intr_array [intr_index].tx_int_bufptr, intr_ident);
				intr_ident = inportb
					(intr_array [intr_index].rx_int_bufptr->com_reg_add + 2);
			} /* while */
		break;

		case I8274 :
			interrupt_handler_8274 (intr_array [intr_index].rx_int_bufptr,
						intr_array [intr_index].tx_int_bufptr,
						intr_array [intr_index].srx_int_bufptr,
						intr_array [intr_index].stx_int_bufptr);
		break;
	} /* switch */

} /* end of generic_1st_level_intr_1 */





/*-------------------------------------------------------------------------*/





/*
============================================================================
|
| generic_1st_level_intr_2
|
| This is the generic 1st level interrupt handler for all serial interrupts
|
| Parameters:   -   none
|
| Returns:      -   nothing
|
============================================================================
*/

void interrupt generic_1st_level_intr_2 (...) {

	static unsigned char intr_index = 2;
	unsigned char intr_ident;

	switch (intr_array [intr_index].uart_type) {
		case I8251A :
			/* now determine whether the interrupt is for a receive or
			transmit interrupt.
			*/
			if (intr_array [intr_index].intr_type == RX) {
				receive_interrupt_handler_8251A (intr_array [intr_index].
												rx_int_bufptr);
			} /* if */
			else {
				transmit_interrupt_handler_8251A (intr_array [intr_index].
												tx_int_bufptr);
			} /* else */
		break;


		case NS16450 :
			intr_ident = inportb
					(intr_array [intr_index].rx_int_bufptr->com_reg_add + 2);
			while ((intr_ident & 0x01) == 0) {
				interrupt_handler_16450 (
							intr_array [intr_index].rx_int_bufptr,
							intr_array [intr_index].tx_int_bufptr, intr_ident);
				intr_ident = inportb
					(intr_array [intr_index].rx_int_bufptr->com_reg_add + 2);
			} /* while */
		break;

		case I8274 :
			interrupt_handler_8274 (intr_array [intr_index].rx_int_bufptr,
						intr_array [intr_index].tx_int_bufptr,
						intr_array [intr_index].srx_int_bufptr,
						intr_array [intr_index].stx_int_bufptr);
		break;
	} /* switch */

	/* at this stage the character interrupt processing has been completed.
	Now have to check how many 8259 interrupt controllers we have and
	then send out the appropriate EOI instructions to them.
	*/
	switch (intr_array [intr_index].num_8259s) {
		case 0 :
		break;

		case 1 :
			outportb (intr_array [intr_index].addr_8259_mast, EOI_8259);
		break;

		case 2 :
			outportb (intr_array [intr_index].addr_8259_mast, EOI_8259);
			outportb (intr_array [intr_index].addr_8259_slave, EOI_8259);
		break;
	} /* switch */

} /* end of generic_1st_level_intr_2 */





/*-------------------------------------------------------------------------*/





/*
============================================================================
|
| generic_1st_level_intr_3
|
| This is the generic 1st level interrupt handler for all serial interrupts
|
| Parameters:   -   none
|
| Returns:      -   nothing
|
============================================================================
*/

void interrupt generic_1st_level_intr_3 (...) {

	static unsigned char intr_index = 3;
	unsigned char intr_ident;

	switch (intr_array [intr_index].uart_type) {
		case I8251A :
			/* now determine whether the interrupt is for a receive or
			transmit interrupt.
			*/
			if (intr_array [intr_index].intr_type == RX) {
				receive_interrupt_handler_8251A (intr_array [intr_index].
												rx_int_bufptr);
			} /* if */
			else {
				transmit_interrupt_handler_8251A (intr_array [intr_index].
												tx_int_bufptr);
			} /* else */
		break;


		case NS16450 :
			intr_ident = inportb
					(intr_array [intr_index].rx_int_bufptr->com_reg_add + 2);
			while ((intr_ident & 0x01) == 0) {
				interrupt_handler_16450 (
							intr_array [intr_index].rx_int_bufptr,
							intr_array [intr_index].tx_int_bufptr, intr_ident);
				intr_ident = inportb
					(intr_array [intr_index].rx_int_bufptr->com_reg_add + 2);
			} /* while */
		break;

		case I8274 :
			interrupt_handler_8274 (intr_array [intr_index].rx_int_bufptr,
						intr_array [intr_index].tx_int_bufptr,
						intr_array [intr_index].srx_int_bufptr,
						intr_array [intr_index].stx_int_bufptr);
		break;
	} /* switch */

	/* at this stage the character interrupt processing has been completed.
	Now have to check how many 8259 interrupt controllers we have and
	then send out the appropriate EOI instructions to them.
	*/
	switch (intr_array [intr_index].num_8259s) {
		case 0 :
		break;

		case 1 :
			outportb (intr_array [intr_index].addr_8259_mast, EOI_8259);
		break;

		case 2 :
			outportb (intr_array [intr_index].addr_8259_mast, EOI_8259);
			outportb (intr_array [intr_index].addr_8259_slave, EOI_8259);
		break;
	} /* switch */

} /* end of generic_1st_level_intr_3 */





/*-------------------------------------------------------------------------*/





/*
============================================================================
|
| generic_1st_level_intr_4
|
| This is the generic 1st level interrupt handler for all serial interrupts
|
| Parameters:   -   none
|
| Returns:      -   nothing
|
============================================================================
*/

void interrupt generic_1st_level_intr_4 (...) {

	static unsigned char intr_index = 4;
	unsigned char intr_ident;

	switch (intr_array [intr_index].uart_type) {
		case I8251A :
			/* now determine whether the interrupt is for a receive or
			transmit interrupt.
			*/
			if (intr_array [intr_index].intr_type == RX) {
				receive_interrupt_handler_8251A (intr_array [intr_index].
												rx_int_bufptr);
			} /* if */
			else {
				transmit_interrupt_handler_8251A (intr_array [intr_index].
												tx_int_bufptr);
			} /* else */
		break;


		case NS16450 :
			intr_ident = inportb
					(intr_array [intr_index].rx_int_bufptr->com_reg_add + 2);
			while ((intr_ident & 0x01) == 0) {
				interrupt_handler_16450 (
							intr_array [intr_index].rx_int_bufptr,
							intr_array [intr_index].tx_int_bufptr, intr_ident);
				intr_ident = inportb
					(intr_array [intr_index].rx_int_bufptr->com_reg_add + 2);
			} /* while */
		break;

		case I8274 :
			interrupt_handler_8274 (intr_array [intr_index].rx_int_bufptr,
						intr_array [intr_index].tx_int_bufptr,
						intr_array [intr_index].srx_int_bufptr,
						intr_array [intr_index].stx_int_bufptr);
		break;
	} /* switch */

	/* at this stage the character interrupt processing has been completed.
	Now have to check how many 8259 interrupt controllers we have and
	then send out the appropriate EOI instructions to them.
	*/
	switch (intr_array [intr_index].num_8259s) {
		case 0 :
		break;

		case 1 :
			outportb (intr_array [intr_index].addr_8259_mast, EOI_8259);
		break;

		case 2 :
			outportb (intr_array [intr_index].addr_8259_mast, EOI_8259);
			outportb (intr_array [intr_index].addr_8259_slave, EOI_8259);
		break;
	} /* switch */

} /* end of generic_1st_level_intr_4 */





/*-------------------------------------------------------------------------*/





/*
============================================================================
|
| generic_1st_level_intr_5
|
| This is the generic 1st level interrupt handler for all serial interrupts
|
| Parameters:   -   none
|
| Returns:      -   nothing
|
============================================================================
*/

void interrupt generic_1st_level_intr_5 (...) {

	static unsigned char intr_index = 5;
	unsigned char intr_ident;

	switch (intr_array [intr_index].uart_type) {
		case I8251A :
			/* now determine whether the interrupt is for a receive or
			transmit interrupt.
			*/
			if (intr_array [intr_index].intr_type == RX) {
				receive_interrupt_handler_8251A (intr_array [intr_index].
												rx_int_bufptr);
			} /* if */
			else {
				transmit_interrupt_handler_8251A (intr_array [intr_index].
												tx_int_bufptr);
			} /* else */
		break;


		case NS16450 :
			intr_ident = inportb
					(intr_array [intr_index].rx_int_bufptr->com_reg_add + 2);
			while ((intr_ident & 0x01) == 0) {
				interrupt_handler_16450 (
							intr_array [intr_index].rx_int_bufptr,
							intr_array [intr_index].tx_int_bufptr, intr_ident);
				intr_ident = inportb
					(intr_array [intr_index].rx_int_bufptr->com_reg_add + 2);
			} /* while */
		break;

		case I8274 :
			interrupt_handler_8274 (intr_array [intr_index].rx_int_bufptr,
						intr_array [intr_index].tx_int_bufptr,
						intr_array [intr_index].srx_int_bufptr,
						intr_array [intr_index].stx_int_bufptr);
		break;
	} /* switch */

	/* at this stage the character interrupt processing has been completed.
	Now have to check how many 8259 interrupt controllers we have and
	then send out the appropriate EOI instructions to them.
	*/
	switch (intr_array [intr_index].num_8259s) {
		case 0 :
		break;

		case 1 :
			outportb (intr_array [intr_index].addr_8259_mast, EOI_8259);
		break;

		case 2 :
			outportb (intr_array [intr_index].addr_8259_mast, EOI_8259);
			outportb (intr_array [intr_index].addr_8259_slave, EOI_8259);
		break;
	} /* switch */

} /* end of generic_1st_level_intr_5 */





/*-------------------------------------------------------------------------*/





/*
============================================================================
|
| generic_1st_level_intr_6
|
| This is the generic 1st level interrupt handler for all serial interrupts
|
| Parameters:   -   none
|
| Returns:      -   nothing
|
============================================================================
*/

void interrupt generic_1st_level_intr_6 (...) {

	static unsigned char intr_index = 6;
	unsigned char intr_ident;

	switch (intr_array [intr_index].uart_type) {
		case I8251A :
			/* now determine whether the interrupt is for a receive or
			transmit interrupt.
			*/
			if (intr_array [intr_index].intr_type == RX) {
				receive_interrupt_handler_8251A (intr_array [intr_index].
												rx_int_bufptr);
			} /* if */
			else {
				transmit_interrupt_handler_8251A (intr_array [intr_index].
												tx_int_bufptr);
			} /* else */
		break;


		case NS16450 :
			intr_ident = inportb
					(intr_array [intr_index].rx_int_bufptr->com_reg_add + 2);
			while ((intr_ident & 0x01) == 0) {
				interrupt_handler_16450 (
							intr_array [intr_index].rx_int_bufptr,
							intr_array [intr_index].tx_int_bufptr, intr_ident);
				intr_ident = inportb
					(intr_array [intr_index].rx_int_bufptr->com_reg_add + 2);
			} /* while */
		break;

		case I8274 :
			interrupt_handler_8274 (intr_array [intr_index].rx_int_bufptr,
						intr_array [intr_index].tx_int_bufptr,
						intr_array [intr_index].srx_int_bufptr,
						intr_array [intr_index].stx_int_bufptr);
		break;
	} /* switch */

	/* at this stage the character interrupt processing has been completed.
	Now have to check how many 8259 interrupt controllers we have and
	then send out the appropriate EOI instructions to them.
	*/
	switch (intr_array [intr_index].num_8259s) {
		case 0 :
		break;

		case 1 :
			outportb (intr_array [intr_index].addr_8259_mast, EOI_8259);
		break;

		case 2 :
			outportb (intr_array [intr_index].addr_8259_mast, EOI_8259);
			outportb (intr_array [intr_index].addr_8259_slave, EOI_8259);
		break;
	} /* switch */

} /* end of generic_1st_level_intr_6 */





/*-------------------------------------------------------------------------*/





/*
============================================================================
|
| generic_1st_level_intr_7
|
| This is the generic 1st level interrupt handler for all serial interrupts
|
| Parameters:   -   none
|
| Returns:      -   nothing
|
============================================================================
*/

void interrupt generic_1st_level_intr_7 (...) {

	static unsigned char intr_index = 7;
	unsigned char intr_ident;

	switch (intr_array [intr_index].uart_type) {
		case I8251A :
			/* now determine whether the interrupt is for a receive or
			transmit interrupt.
			*/
			if (intr_array [intr_index].intr_type == RX) {
				receive_interrupt_handler_8251A (intr_array [intr_index].
												rx_int_bufptr);
			} /* if */
			else {
				transmit_interrupt_handler_8251A (intr_array [intr_index].
												tx_int_bufptr);
			} /* else */
		break;


		case NS16450 :
			intr_ident = inportb
					(intr_array [intr_index].rx_int_bufptr->com_reg_add + 2);
			while ((intr_ident & 0x01) == 0) {
				interrupt_handler_16450 (
							intr_array [intr_index].rx_int_bufptr,
							intr_array [intr_index].tx_int_bufptr, intr_ident);
				intr_ident = inportb
					(intr_array [intr_index].rx_int_bufptr->com_reg_add + 2);
			} /* while */
		break;

		case I8274 :
			interrupt_handler_8274 (intr_array [intr_index].rx_int_bufptr,
						intr_array [intr_index].tx_int_bufptr,
						intr_array [intr_index].srx_int_bufptr,
						intr_array [intr_index].stx_int_bufptr);
		break;
	} /* switch */

	/* at this stage the character interrupt processing has been completed.
	Now have to check how many 8259 interrupt controllers we have and
	then send out the appropriate EOI instructions to them.
	*/
	switch (intr_array [intr_index].num_8259s) {
		case 0 :
		break;

		case 1 :
			outportb (intr_array [intr_index].addr_8259_mast, EOI_8259);
		break;

		case 2 :
			outportb (intr_array [intr_index].addr_8259_mast, EOI_8259);
			outportb (intr_array [intr_index].addr_8259_slave, EOI_8259);
		break;
	} /* switch */

} /* end of generic_1st_level_intr_7 */





/*-------------------------------------------------------------------------*/





/*
============================================================================
|
| generic_1st_level_intr_8
|
| This is the generic 1st level interrupt handler for all serial interrupts
|
| Parameters:   -   none
|
| Returns:      -   nothing
|
============================================================================
*/

void interrupt generic_1st_level_intr_8 (...) {

	static unsigned char intr_index = 8;
	unsigned char intr_ident;

	switch (intr_array [intr_index].uart_type) {
		case I8251A :
			/* now determine whether the interrupt is for a receive or
			transmit interrupt.
			*/
			if (intr_array [intr_index].intr_type == RX) {
				receive_interrupt_handler_8251A (intr_array [intr_index].
												rx_int_bufptr);
			} /* if */
			else {
				transmit_interrupt_handler_8251A (intr_array [intr_index].
												tx_int_bufptr);
			} /* else */
		break;


		case NS16450 :
			intr_ident = inportb
					(intr_array [intr_index].rx_int_bufptr->com_reg_add + 2);
			while ((intr_ident & 0x01) == 0) {
				interrupt_handler_16450 (
							intr_array [intr_index].rx_int_bufptr,
							intr_array [intr_index].tx_int_bufptr, intr_ident);
				intr_ident = inportb
					(intr_array [intr_index].rx_int_bufptr->com_reg_add + 2);
			} /* while */
		break;

		case I8274 :
			interrupt_handler_8274 (intr_array [intr_index].rx_int_bufptr,
						intr_array [intr_index].tx_int_bufptr,
						intr_array [intr_index].srx_int_bufptr,
						intr_array [intr_index].stx_int_bufptr);
		break;
	} /* switch */

	/* at this stage the character interrupt processing has been completed.
	Now have to check how many 8259 interrupt controllers we have and
	then send out the appropriate EOI instructions to them.
	*/
	switch (intr_array [intr_index].num_8259s) {
		case 0 :
		break;

		case 1 :
			outportb (intr_array [intr_index].addr_8259_mast, EOI_8259);
		break;

		case 2 :
			outportb (intr_array [intr_index].addr_8259_mast, EOI_8259);
			outportb (intr_array [intr_index].addr_8259_slave, EOI_8259);
		break;
	} /* switch */

} /* end of generic_1st_level_intr_8 */





/*-------------------------------------------------------------------------*/





/*
============================================================================
|
| generic_1st_level_intr_9
|
| This is the generic 1st level interrupt handler for all serial interrupts
|
| Parameters:   -   none
|
| Returns:      -   nothing
|
============================================================================
*/

void interrupt generic_1st_level_intr_9 (...) {

	static unsigned char intr_index = 9;
	unsigned char intr_ident;

	switch (intr_array [intr_index].uart_type) {
		case I8251A :
			/* now determine whether the interrupt is for a receive or
			transmit interrupt.
			*/
			if (intr_array [intr_index].intr_type == RX) {
				receive_interrupt_handler_8251A (intr_array [intr_index].
												rx_int_bufptr);
			} /* if */
			else {
				transmit_interrupt_handler_8251A (intr_array [intr_index].
												tx_int_bufptr);
			} /* else */
		break;


		case NS16450 :
			intr_ident = inportb
					(intr_array [intr_index].rx_int_bufptr->com_reg_add + 2);
			while ((intr_ident & 0x01) == 0) {
				interrupt_handler_16450 (
							intr_array [intr_index].rx_int_bufptr,
							intr_array [intr_index].tx_int_bufptr, intr_ident);
				intr_ident = inportb
					(intr_array [intr_index].rx_int_bufptr->com_reg_add + 2);
			} /* while */
		break;

		case I8274 :
			interrupt_handler_8274 (intr_array [intr_index].rx_int_bufptr,
						intr_array [intr_index].tx_int_bufptr,
						intr_array [intr_index].srx_int_bufptr,
						intr_array [intr_index].stx_int_bufptr);
		break;
	} /* switch */

	/* at this stage the character interrupt processing has been completed.
	Now have to check how many 8259 interrupt controllers we have and
	then send out the appropriate EOI instructions to them.
	*/
	switch (intr_array [intr_index].num_8259s) {
		case 0 :
		break;

		case 1 :
			outportb (intr_array [intr_index].addr_8259_mast, EOI_8259);
		break;

		case 2 :
			outportb (intr_array [intr_index].addr_8259_mast, EOI_8259);
			outportb (intr_array [intr_index].addr_8259_slave, EOI_8259);
		break;
	} /* switch */

} /* end of generic_1st_level_intr_9 */





/*-------------------------------------------------------------------------*/





/*
============================================================================
|
| generic_1st_level_intr_10
|
| This is the generic 1st level interrupt handler for all serial interrupts
|
| Parameters:   -   none
|
| Returns:      -   nothing
|
============================================================================
*/

void interrupt generic_1st_level_intr_10 (...) {

	static unsigned char intr_index = 10;
	unsigned char intr_ident;

	switch (intr_array [intr_index].uart_type) {
		case I8251A :
			/* now determine whether the interrupt is for a receive or
			transmit interrupt.
			*/
			if (intr_array [intr_index].intr_type == RX) {
				receive_interrupt_handler_8251A (intr_array [intr_index].
												rx_int_bufptr);
			} /* if */
			else {
				transmit_interrupt_handler_8251A (intr_array [intr_index].
												tx_int_bufptr);
			} /* else */
		break;


		case NS16450 :
			intr_ident = inportb
					(intr_array [intr_index].rx_int_bufptr->com_reg_add + 2);
			while ((intr_ident & 0x01) == 0) {
				interrupt_handler_16450 (
							intr_array [intr_index].rx_int_bufptr,
							intr_array [intr_index].tx_int_bufptr, intr_ident);
				intr_ident = inportb
					(intr_array [intr_index].rx_int_bufptr->com_reg_add + 2);
			} /* while */
		break;

		case I8274 :
			interrupt_handler_8274 (intr_array [intr_index].rx_int_bufptr,
						intr_array [intr_index].tx_int_bufptr,
						intr_array [intr_index].srx_int_bufptr,
						intr_array [intr_index].stx_int_bufptr);
		break;
	} /* switch */

	/* at this stage the character interrupt processing has been completed.
	Now have to check how many 8259 interrupt controllers we have and
	then send out the appropriate EOI instructions to them.
	*/
	switch (intr_array [intr_index].num_8259s) {
		case 0 :
		break;

		case 1 :
			outportb (intr_array [intr_index].addr_8259_mast, EOI_8259);
		break;

		case 2 :
			outportb (intr_array [intr_index].addr_8259_mast, EOI_8259);
			outportb (intr_array [intr_index].addr_8259_slave, EOI_8259);
		break;
	} /* switch */

} /* end of generic_1st_level_intr_10 */





/*-------------------------------------------------------------------------*/





/*
============================================================================
|
| generic_1st_level_intr_11
|
| This is the generic 1st level interrupt handler for all serial interrupts
|
| Parameters:   -   none
|
| Returns:      -   nothing
|
============================================================================
*/

void interrupt generic_1st_level_intr_11 (...) {

	static unsigned char intr_index = 11;
	unsigned char intr_ident;

	switch (intr_array [intr_index].uart_type) {
		case I8251A :
			/* now determine whether the interrupt is for a receive or
			transmit interrupt.
			*/
			if (intr_array [intr_index].intr_type == RX) {
				receive_interrupt_handler_8251A (intr_array [intr_index].
												rx_int_bufptr);
			} /* if */
			else {
				transmit_interrupt_handler_8251A (intr_array [intr_index].
												tx_int_bufptr);
			} /* else */
		break;


		case NS16450 :
			intr_ident = inportb
					(intr_array [intr_index].rx_int_bufptr->com_reg_add + 2);
			while ((intr_ident & 0x01) == 0) {
				interrupt_handler_16450 (
							intr_array [intr_index].rx_int_bufptr,
							intr_array [intr_index].tx_int_bufptr, intr_ident);
				intr_ident = inportb
					(intr_array [intr_index].rx_int_bufptr->com_reg_add + 2);
			} /* while */
		break;

		case I8274 :
			interrupt_handler_8274 (intr_array [intr_index].rx_int_bufptr,
						intr_array [intr_index].tx_int_bufptr,
						intr_array [intr_index].srx_int_bufptr,
						intr_array [intr_index].stx_int_bufptr);
		break;
	} /* switch */

	/* at this stage the character interrupt processing has been completed.
	Now have to check how many 8259 interrupt controllers we have and
	then send out the appropriate EOI instructions to them.
	*/
	switch (intr_array [intr_index].num_8259s) {
		case 0 :
		break;

		case 1 :
			outportb (intr_array [intr_index].addr_8259_mast, EOI_8259);
		break;

		case 2 :
			outportb (intr_array [intr_index].addr_8259_mast, EOI_8259);
			outportb (intr_array [intr_index].addr_8259_slave, EOI_8259);
		break;
	} /* switch */

} /* end of generic_1st_level_intr_11 */





/*-------------------------------------------------------------------------*/





/*
============================================================================
|
| generic_1st_level_intr_12
|
| This is the generic 1st level interrupt handler for all serial interrupts
|
| Parameters:   -   none
|
| Returns:      -   nothing
|
============================================================================
*/

void interrupt generic_1st_level_intr_12 (...) {

	static unsigned char intr_index = 12;
	unsigned char intr_ident;

	switch (intr_array [intr_index].uart_type) {
		case I8251A :
			/* now determine whether the interrupt is for a receive or
			transmit interrupt.
			*/
			if (intr_array [intr_index].intr_type == RX) {
				receive_interrupt_handler_8251A (intr_array [intr_index].
												rx_int_bufptr);
			} /* if */
			else {
				transmit_interrupt_handler_8251A (intr_array [intr_index].
												tx_int_bufptr);
			} /* else */
		break;


		case NS16450 :
			intr_ident = inportb
					(intr_array [intr_index].rx_int_bufptr->com_reg_add + 2);
			while ((intr_ident & 0x01) == 0) {
				interrupt_handler_16450 (
							intr_array [intr_index].rx_int_bufptr,
							intr_array [intr_index].tx_int_bufptr, intr_ident);
				intr_ident = inportb
					(intr_array [intr_index].rx_int_bufptr->com_reg_add + 2);
			} /* while */
		break;

		case I8274 :
			interrupt_handler_8274 (intr_array [intr_index].rx_int_bufptr,
						intr_array [intr_index].tx_int_bufptr,
						intr_array [intr_index].srx_int_bufptr,
						intr_array [intr_index].stx_int_bufptr);
		break;
	} /* switch */

	/* at this stage the character interrupt processing has been completed.
	Now have to check how many 8259 interrupt controllers we have and
	then send out the appropriate EOI instructions to them.
	*/
	switch (intr_array [intr_index].num_8259s) {
		case 0 :
		break;

		case 1 :
			outportb (intr_array [intr_index].addr_8259_mast, EOI_8259);
		break;

		case 2 :
			outportb (intr_array [intr_index].addr_8259_mast, EOI_8259);
			outportb (intr_array [intr_index].addr_8259_slave, EOI_8259);
		break;
	} /* switch */

} /* end of generic_1st_level_intr_12 */





/*-------------------------------------------------------------------------*/





/*
============================================================================
|
| generic_1st_level_intr_13
|
| This is the generic 1st level interrupt handler for all serial interrupts
|
| Parameters:   -   none
|
| Returns:      -   nothing
|
============================================================================
*/

void interrupt generic_1st_level_intr_13 (...) {

	static unsigned char intr_index = 13;
	unsigned char intr_ident;

	switch (intr_array [intr_index].uart_type) {
		case I8251A :
			/* now determine whether the interrupt is for a receive or
			transmit interrupt.
			*/
			if (intr_array [intr_index].intr_type == RX) {
				receive_interrupt_handler_8251A (intr_array [intr_index].
												rx_int_bufptr);
			} /* if */
			else {
				transmit_interrupt_handler_8251A (intr_array [intr_index].
												tx_int_bufptr);
			} /* else */
		break;


		case NS16450 :
			intr_ident = inportb
					(intr_array [intr_index].rx_int_bufptr->com_reg_add + 2);
			while ((intr_ident & 0x01) == 0) {
				interrupt_handler_16450 (
							intr_array [intr_index].rx_int_bufptr,
							intr_array [intr_index].tx_int_bufptr, intr_ident);
				intr_ident = inportb
					(intr_array [intr_index].rx_int_bufptr->com_reg_add + 2);
			} /* while */
		break;

		case I8274 :
			interrupt_handler_8274 (intr_array [intr_index].rx_int_bufptr,
						intr_array [intr_index].tx_int_bufptr,
						intr_array [intr_index].srx_int_bufptr,
						intr_array [intr_index].stx_int_bufptr);
		break;
	} /* switch */

	/* at this stage the character interrupt processing has been completed.
	Now have to check how many 8259 interrupt controllers we have and
	then send out the appropriate EOI instructions to them.
	*/
	switch (intr_array [intr_index].num_8259s) {
		case 0 :
		break;

		case 1 :
			outportb (intr_array [intr_index].addr_8259_mast, EOI_8259);
		break;

		case 2 :
			outportb (intr_array [intr_index].addr_8259_mast, EOI_8259);
			outportb (intr_array [intr_index].addr_8259_slave, EOI_8259);
		break;
	} /* switch */

} /* end of generic_1st_level_intr_13 */





/*-------------------------------------------------------------------------*/





/*
============================================================================
|
| generic_1st_level_intr_14
|
| This is the generic 1st level interrupt handler for all serial interrupts
|
| Parameters:   -   none
|
| Returns:      -   nothing
|
============================================================================
*/

void interrupt generic_1st_level_intr_14 (...) {

	static unsigned char intr_index = 14;
	unsigned char intr_ident;

	switch (intr_array [intr_index].uart_type) {
		case I8251A :
			/* now determine whether the interrupt is for a receive or
			transmit interrupt.
			*/
			if (intr_array [intr_index].intr_type == RX) {
				receive_interrupt_handler_8251A (intr_array [intr_index].
												rx_int_bufptr);
			} /* if */
			else {
				transmit_interrupt_handler_8251A (intr_array [intr_index].
												tx_int_bufptr);
			} /* else */
		break;


		case NS16450 :
			intr_ident = inportb
					(intr_array [intr_index].rx_int_bufptr->com_reg_add + 2);
			while ((intr_ident & 0x01) == 0) {
				interrupt_handler_16450 (
							intr_array [intr_index].rx_int_bufptr,
							intr_array [intr_index].tx_int_bufptr, intr_ident);
				intr_ident = inportb
					(intr_array [intr_index].rx_int_bufptr->com_reg_add + 2);
			} /* while */
		break;

		case I8274 :
			interrupt_handler_8274 (intr_array [intr_index].rx_int_bufptr,
						intr_array [intr_index].tx_int_bufptr,
						intr_array [intr_index].srx_int_bufptr,
						intr_array [intr_index].stx_int_bufptr);
		break;
	} /* switch */

	/* at this stage the character interrupt processing has been completed.
	Now have to check how many 8259 interrupt controllers we have and
	then send out the appropriate EOI instructions to them.
	*/
	switch (intr_array [intr_index].num_8259s) {
		case 0 :
		break;

		case 1 :
			outportb (intr_array [intr_index].addr_8259_mast, EOI_8259);
		break;

		case 2 :
			outportb (intr_array [intr_index].addr_8259_mast, EOI_8259);
			outportb (intr_array [intr_index].addr_8259_slave, EOI_8259);
		break;
	} /* switch */

} /* end of generic_1st_level_intr_14 */





/*-------------------------------------------------------------------------*/





/*
============================================================================
|
| generic_1st_level_intr_15
|
| This is the generic 1st level interrupt handler for all serial interrupts
|
| Parameters:   -   none
|
| Returns:      -   nothing
|
============================================================================
*/

void interrupt generic_1st_level_intr_15 (...) {

	static unsigned char intr_index = 15;
	unsigned char intr_ident;

	switch (intr_array [intr_index].uart_type) {
		case I8251A :
			/* now determine whether the interrupt is for a receive or
			transmit interrupt.
			*/
			if (intr_array [intr_index].intr_type == RX) {
				receive_interrupt_handler_8251A (intr_array [intr_index].
												rx_int_bufptr);
			} /* if */
			else {
				transmit_interrupt_handler_8251A (intr_array [intr_index].
												tx_int_bufptr);
			} /* else */
		break;


		case NS16450 :
			intr_ident = inportb
					(intr_array [intr_index].rx_int_bufptr->com_reg_add + 2);
			while ((intr_ident & 0x01) == 0) {
				interrupt_handler_16450 (
							intr_array [intr_index].rx_int_bufptr,
							intr_array [intr_index].tx_int_bufptr, intr_ident);
				intr_ident = inportb
					(intr_array [intr_index].rx_int_bufptr->com_reg_add + 2);
			} /* while */
		break;

		case I8274 :
			interrupt_handler_8274 (intr_array [intr_index].rx_int_bufptr,
						intr_array [intr_index].tx_int_bufptr,
						intr_array [intr_index].srx_int_bufptr,
						intr_array [intr_index].stx_int_bufptr);
		break;
	} /* switch */

	/* at this stage the character interrupt processing has been completed.
	Now have to check how many 8259 interrupt controllers we have and
	then send out the appropriate EOI instructions to them.
	*/
	switch (intr_array [intr_index].num_8259s) {
		case 0 :
		break;

		case 1 :
			outportb (intr_array [intr_index].addr_8259_mast, EOI_8259);
		break;

		case 2 :
			outportb (intr_array [intr_index].addr_8259_mast, EOI_8259);
			outportb (intr_array [intr_index].addr_8259_slave, EOI_8259);
		break;
	} /* switch */

} /* end of generic_1st_level_intr_15 */





/*-------------------------------------------------------------------------*/





/*
============================================================================
|
| generic_1st_level_intr_16
|
| This is the generic 1st level interrupt handler for all serial interrupts
|
| Parameters:   -   none
|
| Returns:      -   nothing
|
============================================================================
*/

void interrupt generic_1st_level_intr_16 (...) {

	static unsigned char intr_index = 16;
	unsigned char intr_ident;

	switch (intr_array [intr_index].uart_type) {
		case I8251A :
			/* now determine whether the interrupt is for a receive or
			transmit interrupt.
			*/
			if (intr_array [intr_index].intr_type == RX) {
				receive_interrupt_handler_8251A (intr_array [intr_index].
												rx_int_bufptr);
			} /* if */
			else {
				transmit_interrupt_handler_8251A (intr_array [intr_index].
												tx_int_bufptr);
			} /* else */
		break;


		case NS16450 :
			intr_ident = inportb
					(intr_array [intr_index].rx_int_bufptr->com_reg_add + 2);
			while ((intr_ident & 0x01) == 0) {
				interrupt_handler_16450 (
							intr_array [intr_index].rx_int_bufptr,
							intr_array [intr_index].tx_int_bufptr, intr_ident);
				intr_ident = inportb
					(intr_array [intr_index].rx_int_bufptr->com_reg_add + 2);
			} /* while */
		break;

		case I8274 :
			interrupt_handler_8274 (intr_array [intr_index].rx_int_bufptr,
						intr_array [intr_index].tx_int_bufptr,
						intr_array [intr_index].srx_int_bufptr,
						intr_array [intr_index].stx_int_bufptr);
		break;
	} /* switch */

	/* at this stage the character interrupt processing has been completed.
	Now have to check how many 8259 interrupt controllers we have and
	then send out the appropriate EOI instructions to them.
	*/
	switch (intr_array [intr_index].num_8259s) {
		case 0 :
		break;

		case 1 :
			outportb (intr_array [intr_index].addr_8259_mast, EOI_8259);
		break;

		case 2 :
			outportb (intr_array [intr_index].addr_8259_mast, EOI_8259);
			outportb (intr_array [intr_index].addr_8259_slave, EOI_8259);
		break;
	} /* switch */

} /* end of generic_1st_level_intr_16 */





/*-------------------------------------------------------------------------*/





/*
============================================================================
|
| generic_1st_level_intr_17
|
| This is the generic 1st level interrupt handler for all serial interrupts
|
| Parameters:   -   none
|
| Returns:      -   nothing
|
============================================================================
*/

void interrupt generic_1st_level_intr_17 (...) {

	static unsigned char intr_index = 17;
	unsigned char intr_ident;

	switch (intr_array [intr_index].uart_type) {
		case I8251A :
			/* now determine whether the interrupt is for a receive or
			transmit interrupt.
			*/
			if (intr_array [intr_index].intr_type == RX) {
				receive_interrupt_handler_8251A (intr_array [intr_index].
												rx_int_bufptr);
			} /* if */
			else {
				transmit_interrupt_handler_8251A (intr_array [intr_index].
												tx_int_bufptr);
			} /* else */
		break;


		case NS16450 :
			intr_ident = inportb
					(intr_array [intr_index].rx_int_bufptr->com_reg_add + 2);
			while ((intr_ident & 0x01) == 0) {
				interrupt_handler_16450 (
							intr_array [intr_index].rx_int_bufptr,
							intr_array [intr_index].tx_int_bufptr, intr_ident);
				intr_ident = inportb
					(intr_array [intr_index].rx_int_bufptr->com_reg_add + 2);
			} /* while */
		break;

		case I8274 :
			interrupt_handler_8274 (intr_array [intr_index].rx_int_bufptr,
						intr_array [intr_index].tx_int_bufptr,
						intr_array [intr_index].srx_int_bufptr,
						intr_array [intr_index].stx_int_bufptr);
		break;
	} /* switch */

	/* at this stage the character interrupt processing has been completed.
	Now have to check how many 8259 interrupt controllers we have and
	then send out the appropriate EOI instructions to them.
	*/
	switch (intr_array [intr_index].num_8259s) {
		case 0 :
		break;

		case 1 :
			outportb (intr_array [intr_index].addr_8259_mast, EOI_8259);
		break;

		case 2 :
			outportb (intr_array [intr_index].addr_8259_mast, EOI_8259);
			outportb (intr_array [intr_index].addr_8259_slave, EOI_8259);
		break;
	} /* switch */

} /* end of generic_1st_level_intr_17 */





/*-------------------------------------------------------------------------*/





/*
============================================================================
|
| generic_1st_level_intr_18
|
| This is the generic 1st level interrupt handler for all serial interrupts
|
| Parameters:   -   none
|
| Returns:      -   nothing
|
============================================================================
*/

void interrupt generic_1st_level_intr_18 (...) {

	static unsigned char intr_index = 18;
	unsigned char intr_ident;

	switch (intr_array [intr_index].uart_type) {
		case I8251A :
			/* now determine whether the interrupt is for a receive or
			transmit interrupt.
			*/
			if (intr_array [intr_index].intr_type == RX) {
				receive_interrupt_handler_8251A (intr_array [intr_index].
												rx_int_bufptr);
			} /* if */
			else {
				transmit_interrupt_handler_8251A (intr_array [intr_index].
												tx_int_bufptr);
			} /* else */
		break;


		case NS16450 :
			intr_ident = inportb
					(intr_array [intr_index].rx_int_bufptr->com_reg_add + 2);
			while ((intr_ident & 0x01) == 0) {
				interrupt_handler_16450 (
							intr_array [intr_index].rx_int_bufptr,
							intr_array [intr_index].tx_int_bufptr, intr_ident);
				intr_ident = inportb
					(intr_array [intr_index].rx_int_bufptr->com_reg_add + 2);
			} /* while */
		break;

		case I8274 :
			interrupt_handler_8274 (intr_array [intr_index].rx_int_bufptr,
						intr_array [intr_index].tx_int_bufptr,
						intr_array [intr_index].srx_int_bufptr,
						intr_array [intr_index].stx_int_bufptr);
		break;
	} /* switch */

	/* at this stage the character interrupt processing has been completed.
	Now have to check how many 8259 interrupt controllers we have and
	then send out the appropriate EOI instructions to them.
	*/
	switch (intr_array [intr_index].num_8259s) {
		case 0 :
		break;

		case 1 :
			outportb (intr_array [intr_index].addr_8259_mast, EOI_8259);
		break;

		case 2 :
			outportb (intr_array [intr_index].addr_8259_mast, EOI_8259);
			outportb (intr_array [intr_index].addr_8259_slave, EOI_8259);
		break;
	} /* switch */

} /* end of generic_1st_level_intr_18 */





/*-------------------------------------------------------------------------*/





/*
============================================================================
|
| generic_1st_level_intr_19
|
| This is the generic 1st level interrupt handler for all serial interrupts
|
| Parameters:   -   none
|
| Returns:      -   nothing
|
============================================================================
*/

void interrupt generic_1st_level_intr_19 (...) {

	static unsigned char intr_index = 19;
	unsigned char intr_ident;

	switch (intr_array [intr_index].uart_type) {
		case I8251A :
			/* now determine whether the interrupt is for a receive or
			transmit interrupt.
			*/
			if (intr_array [intr_index].intr_type == RX) {
				receive_interrupt_handler_8251A (intr_array [intr_index].
												rx_int_bufptr);
			} /* if */
			else {
				transmit_interrupt_handler_8251A (intr_array [intr_index].
												tx_int_bufptr);
			} /* else */
		break;


		case NS16450 :
			intr_ident = inportb
					(intr_array [intr_index].rx_int_bufptr->com_reg_add + 2);
			while ((intr_ident & 0x01) == 0) {
				interrupt_handler_16450 (
							intr_array [intr_index].rx_int_bufptr,
							intr_array [intr_index].tx_int_bufptr, intr_ident);
				intr_ident = inportb
					(intr_array [intr_index].rx_int_bufptr->com_reg_add + 2);
			} /* while */
		break;

		case I8274 :
			interrupt_handler_8274 (intr_array [intr_index].rx_int_bufptr,
						intr_array [intr_index].tx_int_bufptr,
						intr_array [intr_index].srx_int_bufptr,
						intr_array [intr_index].stx_int_bufptr);
		break;
	} /* switch */

	/* at this stage the character interrupt processing has been completed.
	Now have to check how many 8259 interrupt controllers we have and
	then send out the appropriate EOI instructions to them.
	*/
	switch (intr_array [intr_index].num_8259s) {
		case 0 :
		break;

		case 1 :
			outportb (intr_array [intr_index].addr_8259_mast, EOI_8259);
		break;

		case 2 :
			outportb (intr_array [intr_index].addr_8259_mast, EOI_8259);
			outportb (intr_array [intr_index].addr_8259_slave, EOI_8259);
		break;
	} /* switch */

} /* end of generic_1st_level_intr_19 */





/*-------------------------------------------------------------------------*/
