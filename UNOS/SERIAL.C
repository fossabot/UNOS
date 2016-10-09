/************************************************************************/
/*                                                                      */
/*                                                                      */
/*                                                                      */
/*                         SERIAL DEVICE DRIVER MODULE                  */
/*                                                                      */
/*                                   by                                 */
/*                                                                      */
/*                               Robert Betz                            */
/*            Department of Electrical and Computer Engineering         */
/*                          University of Newcastle                     */
/*                    (Copyright 1988, 1989, 1990, 1992)                */
/*                                                                      */
/*                                                                      */
/*                                                                      */
/************************************************************************/




/************************************************************************
*
*  STATUS
*
*      PL/M version working
*
*      Preliminary 'C' version - not working yet
*
*
*
*  HISTORY
*
*      27/12/87
*          Began typing in the first version of the hardware serial
*          buffer routines.
*
*      29/12/87
*          Transferred first version from PC to the VAX.
*
*      18/1/88
*          Appears to work OK except for the funny bug mentioned in the
*          UNOS history.
*
*      26/1/89
*          Added support for the 8274 dual uart.
*
*      22/4/92
*          Began translation from PL/M to C and addition of 8250 support.
*
************************************************************************/


#include "general.h"     /* Import miscellaneous defines that are
									generally throughout the system.
									*/
#include "serial.h"         /* Import the UART type definitions */


#include "unos.h"		/* Import all the operating system
									user routines.
									*/

#include "serint.h"

#include <dos.h>					/* Import the inportb and outportb
									function prototypes.
									*/

#include <conio.h>					/* Import the cprintf function */

#include <stdlib.h>					/* Import exit function and related
									definitions.
									*/



/*
DESCRIPTION

This module contains the routines required to implement serial device drivers
for UNOS (University of Newcastle Operating System). The module is designed
to provide an integrated driver for the following UARTS:-

	Intel 8251A
	Intel 8274
	National Semiconductor INS8250A, NS16450

The particular UART is selected by passing the appropriate parameter to the
serial device creation routine.

The sections of the code for the 8251 and 8274 UARTS were translated from
code written for the Pipeline Crack Investigation Vehicle (PCIV) project
which began in 1988. The code for the 8250 was added for the Cook Islands
Antenna tracking project in 1992. This was required since the hardware
platform was an IBM-PC and the 8250 is the UART used on this particular
piece of equipment.
*/





/*************************************************************************/
/*                                                                       */
/*                          MAIN TYPES AND DEFINTIONS                    */
/*                                                                       */
/*************************************************************************/

/* Error numbers */
#define RX_BUFFER_OVERRUN 	0
#define DATA_REG_OVERRUN 	1
#define FRAMING_ERROR 		2
#define PARITY_ERROR 		3
#define BREAK_RECEIVED		4


/* Defines used to define the state of a UART transmitter */
#define ENABLED 1
#define DISABLED 0


/* Defines for user ctrlS and user Ctrl Q */
#define USER_CTRL_S 0
#define USER_CTRL_Q 1



/* Define the uart types that can be handled */


/* The clock frequency for the NS16450 uart - note that this is normally
1.8432MHz.
*/

#define NS16450_IN_FREQ 1843200L


/*------ Registor offset definitions for the NS16450 ------*/

/* The following addresses are offsets relative to the command and data
register addresses. For the NS16450 the command and data registers appear
at the same base address.
*/

#define INTR_ENABLE_REG 1
					/* Note that DLAB = 0 */

#define INTR_IDENT_REG 2
					/* Note that this is read only reg */

#define LINE_CTRL_REG 3
					/* Line control register - the main control reg */

#define MODEM_CTRL_REG 4

#define LINE_STATUS_REG 5
					/* Contains the status of the uart - see masks below */

#define MODEM_STATUS_REG 6

#define SCRATCH_REG 7

#define LS_DIVISOR_LATCH 0
					/* Note DLAB has to be 1 to access */

#define MS_DIVISOR_LATCH 1
					/* Note DLAB has to be 1 to access */

/* Definitions of the bit length for the NS16450 */
#define L_5_BITS 0
#define L_6_BITS 1
#define L_7_BITS 2
#define L_8_BITS 3


/* Masking bits use to set and reset the transmitter and receiver interrupts
for the NS16450.
*/
#define ENABLE_RX_INTR 0x01
						/* OR the contents of IER register with this */

#define DISABLE_RX_INTR 0xfe
						/* AND the contents of IER register with this */

#define ENABLE_TX_INTR 0x02
						/* OR the contents of IER register with this */

#define DISABLE_TX_INTR 0xfd
						/* AND the contents of IER register with this */

#define ENABLE_LINE_STATUS_INTR 0x04
						/* OR the contents of IER register with this */

#define DISABLE_LINE_STATUS_INTR 0xfb
						/* AND the contents of IER register with this */

#define ENABLE_MODEM_STATUS_INTR 0x08
						/* OR the contents of IER register with this */

#define DISABLE_MODEM_STATUS_INTR 0xf7
						/* AND the contents of IER register with this */


#define NS16450_PENDING_INTR_MASK 0x01
						/* AND with the content of the IIR register to see
						if there is any interrupt.
						*/

#define NS16450_INTR_SRC 0x06
						/* AND with the contents of the IIR register to
						determine the source of the interrupt.
						*/


/* Interrupt Type definitions */

#define MODEM_STATUS_INTR 0
#define TX_EMPTY_INTR 2
#define RX_DATA_RDY_INTR 4
#define RX_ERROR_INTR 6


/* Now define the error masks for the NS16450 */

#define OVERRUN_MASK_16450 1
#define PARITY_ERROR_MASK_16450 2
#define FRAMING_ERROR_MASK_16450 4











/*------ Definitions for the 8274 UART ------*/

/*------ 8274 status register internal addresses ------*/

#define REG0 0x0
#define REG1 0x1
#define REG2 0x2
#define REG3 0x3
#define REG4 0x4
#define REG5 0x5
#define REG6 0x6
#define REG7 0x7



/*------  8274 control bytes ------*/

#define SUSPEND_8274_TRANSMITTER_INTS 0x28
#define I8274_EOI 0x38
#define I8274_ERROR_RESET 0x30


/*------ 8274 programming information ------*/

#define RESET_8274_UART 0x18
#define MODE_DATA_8274_1 0x44
										/* no parity, x16 clk div, 1 stop
										bit
										*/

#define INT_MODE_8274_1 4
										/* interrupt mode - uses 8259 */

#define RECEIVER_SETUP_8274_1 0xe1
										/* enable receiver & set 8 data
										bits
										*/

#define TRANSMIT_SETUP_8274_1 0xe8
										/* enable transmitter set 8 data
										bits
										*/

#define RESET_ERROR_REG_8274 0x30

#define RESET_EXT_STATUS_8274 0x0

#define ENABLE_INT_8274_CHA 0x12
#define ENABLE_INT_8274_CHB 0x16

/*--- declarations used for channel identification for the 8274 ----*/

#define CHA 0
#define CHB 1





#define SOFTWARE_RESET_8251 0x40	/* software reset for 8251 */

#define CLEAR_ERR_8251 0x10      	/* or'd into the command to clear
									uart errors */






/*******************************************************/
/*                                                     */
/*        Error detection masks for the 8251 uart      */
/*                                                     */
/*******************************************************/

#define FRAME_ERR_MASK 0x20
										/* used to seperate the bit which
										indicates framing errors */

#define PARITY_ERR_MASK 0x80
										/* used to seperate out the bit which
										indicates parity errors */

#define DATA_OVRUN_MASK 0x10
										/* used to seperate out the bit which
										indicates a data register overrun */




/************************************************************************/
/*																		*/
/*						    GLOBAL VARIABLES							*/
/*																		*/
/************************************************************************/



/* Type definition used to pass variables to reentrant serial tasks */

typedef
	struct {
		unsigned int mbx_mess_size;
		rx_int_buffer *rx_int_bufptr;
		tx_int_buffer *tx_int_bufptr;
	} local_var_type;



/************************************************************************/
/*																		*/
/*							FUNCTION PROTOTYPES							*/
/*																		*/
/************************************************************************/


void put_rx_buffer (unsigned char uart_data, rx_int_buffer* rx_buf_ptr);












/*---------------------------------------------------------------------------*/





/****************************************************************************/
/*                                                                          */
/*                       HARDWARE SERIAL BUFFER SECTION                     */
/*                                                                          */
/****************************************************************************/



/****************************************************************************/
/*                                                                          */
/*                              SECTION DESCRIPTION                         */
/*                                                                          */
/****************************************************************************/


/*

This section of UNOS is one of the major areas of user modification.  This is
mainly due to the fact that some of the routines are hardware dependent and
also that the buffer implementations may vary.

The simplest and probably conceptually most appealing way of setting up these
serial drivers is as normal tasks within UNOS.  Let us consider what would
happen on the receipt of a serial interrupt :  it causes an entry to the kernel
which activates a high priority task dependent upon the source of the
interrupt.  This high priority task is immediately scheduled by the scheduler
and is run.  The task reads the character in from the hardware interrupt
source and places it in a circular buffer.  The transmit interrupts are
handled in a similar fashion except that they are set up as a low priority
task.

The problem with this approach to the serial buffers is that the operating
system has to do a task switch each time a serial interrupt occurs.  In the
case of high baud rate channels this would cause the system to spend most of
its time in the operating system and not do much else - i.e. the system would
very quickly become I/O bound.  For this reason the current implementation
avoids going through the operating system for the actual transfer of the data
from the hardware itself.

The current implementation was developed for the Pipeline Crack Investigation
Vehicle ( PCIV ) project.  Upon receipt of a receive interrupt the following
occurs :  an interrupt procedure is vectored to directly which takes the
character from the hardware device and puts it into a buffer.  The only
interaction with the operating system is a special SIGNAL to wake up a task
to take the contents of the buffer and further process it.  Due to the fact
that the hardware buffers have to be of a special structure as compared with
the normal system fifo buffers the hardware buffers are accompanied with
special interface routines which handle special circumstances such as control
S and control Q as well as providing a device independent interface.  The
overall layout of the serial input/output system is as below :
$eject

						 --------                    --------
                         |      |                    |      |
          ----------     |      |     ----------     |      |
          |        |     |  rx  |     |        |     | fifo |     to
  input   |   rx   |     |      |     |  rx    |     |      |     normal
  ------> |  int   |---->|      |---->| inter- |---->|      |---->fifo
  char    | routine|     |      |     | face   |     |  buf |     buffer
          |        |     |  buf |     | routine|     |      |     routines
          ----------     |      |     ----------     |      |
			 |  |        |      |       |    |       |      |
			 |  |        --------       |    |       --------
             |  -------------------------    |
tx interrupt |              |                |
<------------|-------------------------------|
control      |              |                |
             |              V                |
             |           --------            |       --------
			 |           |      |            |       |      |
          ----------     |      |     ----------     |      |
          |        |     |  tx  |     |        |     | fifo |     from
  output  |   tx   |     |      |     |  tx    |     |      |     normal
  <-------|  int   |<----|      |<----| inter- |<----|      |<----fifo
  char    | routine|     |      |     | face   |     |  buf |     buffer
		  |        |     |  buf |     | routine|     |      |     routines
          ----------     |      |     ----------     |      |
                         |      |                    |      |
						 --------                    --------



Let us firstly consider the operation of the receive system.  The format of a
receive serial buffer is as follows:

buf$size -      size of the buffer in bytes

buf$low$lim -   lower limit on the number of characters stored in the buffer
                for the issue of a control Q

buf$up$lim -    upper limit on the number characters stored in the buffer for
				the issue of a control S

err$buf$index - index into the error buffer to the point where 4 consecutive
                locations are for the reporting of uart related errors.

data$reg$add -  address in the i/o space of the uart data register.

com$reg$add  -  address in i/o space of the uart control register.

uart$com$data - contains the data normally used to program the control register
                of the uart.  The bit which controls the transmit interrupts
                is set so that the interrupts are disabled.  This data is
				required so that the tx interrupt status can be manipulated
				without affecting the normal programming of the uart.

ctS$ctQ$flg -   this flag is set when the receiver has issued a control S.
				Reset when a control Q is issued.

tx$buf$ptr -    pointer to the transmit buffer structure which is associated
				with this receive channel.  If no transmit buffer associated
                then this location is set to a null.

buf$sem$index - index into the semaphore array to the semaphore which is
                used to indicate that data is available in the buffer.

buf$free -      used to store the number of free bytes in the buffer.

get$ptr -       the index into the circular buffer which is used to fetch data.

put$ptr -       the index in the buffer where data is deposited.

buffer -        buffer used to store the data.


One notices that this buffer structure is quite a bit different from the
fifo buffers defined for inter-task communication.  The reason for this is the
special support required for the hardware nature of the buffers.  As can be
seen from the diagram above of the architecture of the buffer system serial
i/o comes in through 2 buffers.  The first of these is of the above format and
is used by the interrupt routine and the interface routine.  The second buffer
which is used only by the interface routine is a normal system fifo buffer.
Therefore the interface routine serves to hide the nature of the hardware
buffer form the communicating tasks of the system.  This means that as far as
a task is concerned communicating with a hardware device is exactly the same as
communicating directly with another task.  The buffer interface routines are
run as a normal task by the system.  This task is activated by a special
signal routine known as kernel$signal which can be used from within the kernel
or an interrupt routine.  Logically the number of interface tasks is equal to
the total number of receive and transmit buffers,  however all the
receive interface tasks use the same piece of software as they are defined
reentrantly.  Similarly for the transmit interface tasks.

One of the messy details which these routines hide from the tasks is the
control Q/control S control.  The system implements bidirectional control Q
/control S control - i.e. if the receive interrupt routine detects that the
receive buffer has filled to the upper limit then it signals directly to the
transmitter that a control S must be issued.  To achieve this the receive
interrupt routine must directly manipulate the transmitter interrupts.
Similarly when the receive interface routine detects that the lower limit
has been reached after an upper limit then it signals to the transmit routine
that a control Q must be issued.  The situation is further complicated if
user initiated control S/control Q is to be supported.  This situation occurs
when the receive routine detects a control Q or S in the input stream.  Again
quick response is required so direct control of the transmitter from the
reciver is required.  Both these control links are achieved via direct
manipulation of the transmitter hardware interrupt enable/disable and the
setting of the appropriate flags in the transmit buffer.  These control
connections are indicated in the above diagram.

The transmit buffers are similar but slighly simpler than the receive buffers.
The buffer has the following format:

buf$size -      size of the buffer in bytes

rx$ctrlQ$ctrlS - used by an associated receiver to signal that a control S
				 or control Q should be sent out immediately upon the next
				 transmit interrupt.  If the value stored in here is non-zero
				 then the transmitter outputs this value on the next interrupt.

cur$tx$int$status - current interrupt status of the transmitter - only the low
					order bit is used.  1 => tx int enable; 0 => tx int disable

user$ctrlS$ctrlQ - flag used to control interrupt status.  If the user has
				   typed a control S then this location is zero, a control Q
                   then this location is 1.  The contents of this location
                   is controlled from the receive routine.

tx$interrupt -  this flag indicates the interrupt status as far as the
				interrupt buffer management routines are concerned.
				0 => tx interrupts disabled.
				1 => tx interrupts enabled.

data$reg$add -  used to store the address of the data register of the uart in
                question.

com$reg$add  -  used to store the address of the control register of the uart
                in question.

uart$com$data - used to store the command data stored in the control register
                of the uart.  Note that the transmit enable bit is set so that
                the interrupts are disabled.

buf$room$index - index into the semaphore array to the room in the tx buffer
                 semaphore.

buf$used -      the number of used locations in the buffer.

get$ptr -       index into the circular buffer to the next byte to be retrieved

put$ptr -       the index into the buffer to the next free location.

buffer -        the array of bytes used to store the data.



The operation of the transmit routine is complicated by the control S/control Q
protocol.  As mentioned previously there are two sources of this protocol -
the receive routine which issues a control S when it reaches its upper limit
and a control Q when it gets to its lower limit after just reaching its upper
limit,  and secondly a user typed control S/control Q which operate in the
normal way.  These various sources are indicated to the transmit interrupt
routine by two flags.  The rx$ctrlS$ctrlQ flag is set to the type of control
character which the receive routine wants the transmitter routine to put out on
the next transmit interrupt.  If it doesn't want a special character to be
output it puts a null into this byte.  The user$ctrlS$ctrlQ flag is used as a
mask to set the interrupt status of the uart in conjunction with the
tx$interrupt flag.  The user$ctrlS$ctrlQ is again manipulated directly from
the receive routine.  If the receive interrupt detects a ctrl S or Q as the
input character it sets this flag appropriately and then ands this flag with
the tx$interrupt flag and writes the result to the cur$tx$int$status and the
uart control register.  The tx$interrupt is a flag which indicates the status
of the transmitter as far as the transmitter interrupt routine and transmitter
interface tasks are concerned.  The "anding" of the two flags ensures that
both the transmitter interface task and the uasr$ctrlS$ctrlQ are of the same
state before the interrupts can be enabled.

It should be noted that both the receive and transmit buffers have to be setup
during the system initialisation.


*/


/*****************************************************************************/
/*                                                                           */
/*                           INTERRUPT ROUTINES                              */
/*                                                                           */
/*****************************************************************************/




/*------------------------------------------------------------------------*/





/*
============================================================================
|
| special_rx_cond_8274
|
| This routine is entered upon a special receive condition for the 8274 uart.
| Special receive conditions are defined as parity errors, framing errors,
| overrun errors.  This routine interrogates the RR1 register of the 8274 and
| then determines from the bit pattern what the source of the special receive
| condition is.  An appropriate error message to the error system is then
| generated.
|
|   Parameters : - pointer to the receive buffer for the appropriate channel
|
|   Entry via  : - interrupt_handler_8274
|
============================================================================
*/

void special_rx_cond_8274 (rx_int_buffer* rx_buf_ptr) {

	unsigned char error_code;
	unsigned char dummy_value;

	/*
	read the RR1 register of the appropriate uart to get the required bits
	the determine what the error is. Error messages can be specific to the
	particular uart because the rx buffer has loaded into it at
	initialisation a base error number for all the errors related to the
	uart.  An offset is then added to this base for the particular error and
	this then forms the error number for the error handling system.

	After the error type is determined and the error flagged then the faulty
	byte is read out of the uart and the error bit in the 8274 is reset.
	*/

	outportb (rx_buf_ptr->com_reg_add, REG1);

	error_code = ((inportb (rx_buf_ptr->com_reg_add) & 0x70 ) >> 4 );


	/*
	check for a parity error
	*/

	if ((error_code & 0x1) > 0) {
/*!@!#$     indicate_error ( rx_buf_ptr->err_buf_index + parity_err_offset );
!@#@$*/
		dummy_value = inportb (rx_buf_ptr->data_reg_add);
	} /* if */

	/*
	check for a data overrun error
	*/

	if ((error_code & 0x2) > 0) {
/*!@#$      call indicate_error ( rx_buf_ptr->err_buf_index +
												data_reg_ovrun_offset );
!@#$*/
		;
	} /* if */


	/*
	check for a framing error
	*/

	if ((error_code & 0x4) > 0) {
/*!@#$          call indicate_error
						( rx_buf_ptr->err_buf_index + frame_err_offset );
!@#$*/
		dummy_value = inportb (rx_buf_ptr->data_reg_add);
		dummy_value++;
	} /* if */

	/*
	now reset the latched error bits
	*/

	outportb (rx_buf_ptr->com_reg_add, RESET_ERROR_REG_8274);


} /* end of special_rx_cond_8274 */





/*------------------------------------------------------------------------*/





/*
===========================================================================
|
| service_tx_8274
|
| The function of this procedure is to service the interrupts from the 8274
| transmitters - i.e. for ch A and ch B.  Essentially the procedure is entered
| upon the receipt of a a transmit interrupt from the 8274.  Its function is
| to take a byte from the transmit interrupt buffer and put it out of the 8274
| transmitter.
|
| The operation of the task is complicated by the fact that it must work in
| conjunction with the receive task for the same channel to provide control Q
| / control S protocol.  These functions are carried out using three main flags
| - rx_ctrlS_ctrlQ, tx_interrupt and user_ctrlS_ctrlQ.  I shall describe the
| function of these flags :
|
| rx_ctrlS_ctrlQ : -
|
| This flag is set by the receiver if a ctrl S or ctrl Q has to be sent out on
| the current interrupt.  If the contents of this location is > 0 then the
| value is sent out of the transmitter.
|
| tx_interrupt : -
|
| This flag is used by the tx_interface_task to control the status of the
| transmitter interrupts.  If this flag is 1 then the transmitter can be
| enabled as far as the tx_interface_task is concerned.
|
| user_ctrlS_ctrlQ : -
|
| This flag is controlled by the receiver an is user to control the status of
| the transmitter interrupts as far as the user is concerned.  If this flag is
| one then the transmitter can be enabled as far as the user is concerned.  If
| it is zero then the transmitter cannot be enabled as far as the user is
| concerned.
|
| The other remaining flag is the cur_tx_int_status which should contain the
| current status of the transmitter.  The values of the tx_interrupt flag and
| the user_ctrlS_ctrlQ should be and'd and put into this flag.
|
|   Parameters : - pointer to the transmit buffer
|
|   Entry via  : - interrupt_handler_8274
|
==============================================================================
*/

void service_tx_8274 (tx_int_buffer* tx_buf_ptr) {


	 /*
    firstly check if a control S or control Q has to be sent out of the
    transmitter
    */

	if ( (tx_buf_ptr->xonxoff ) && (tx_buf_ptr->rx_ctrlS_ctrlQ > 0 ) ) {
		outportb (tx_buf_ptr->data_reg_add,
										tx_buf_ptr->rx_ctrlS_ctrlQ );
		tx_buf_ptr->rx_ctrlS_ctrlQ = FALSE;
	} /* if */
	else {
		/*
		control S or control Q do not have to be sent out.  Now check to see
		if the transmiiter should be allowed to be enabled.
		*/

        if (tx_buf_ptr->user_ctrlS_ctrlQ && tx_buf_ptr->tx_interrupt) {
            /*
				transmitter is allowed to be active so see if a character is in
			the transmitter to send.  If none then suspend the transmit
            interrupts and clear the tx_interrupt flag else take the
            character out of the buffer and write it to the appropriate uart
            data register.
            */

            if (tx_buf_ptr->buf_used > 0) {
                outportb (tx_buf_ptr->data_reg_add,
						tx_buf_ptr->buf_ptr [tx_buf_ptr->get_ptr]);

				if (tx_buf_ptr->get_ptr == (tx_buf_ptr->buf_size - 1)) {
                    tx_buf_ptr->get_ptr = 0;
				} /* if */
                else {
                    (tx_buf_ptr->get_ptr)++;
                } /* else */

					 (tx_buf_ptr->buf_used)--;

				_signal (tx_buf_ptr->buf_room_index);
			} /* if */
            else {
                /*
                no data in the buffer so suspend the interrupts and set the
				appropriate flags
				*/

				outportb (tx_buf_ptr->com_reg_add,
										SUSPEND_8274_TRANSMITTER_INTS);

                tx_buf_ptr->tx_interrupt = DISABLED;
                tx_buf_ptr->cur_tx_int_status = DISABLED;
            } /* else */
        } /* if */
        else {
				/*
			one of the interrupt enable flags is low therefore disable the
            transmitter interrupts.
			*/

            outportb (tx_buf_ptr->com_reg_add,
									SUSPEND_8274_TRANSMITTER_INTS);

            tx_buf_ptr->cur_tx_int_status = DISABLED;
        } /* else */
	} /* else */
} /* end of service_tx_8274 */





/*------------------------------------------------------------------------*/





/*
============================================================================
|
| service_rx_8274
|
| This is a generic interrupt service routine for the 8274 uart receive
| interrupt.  This routine has at the simplest level the function of taking a
| character from the uart data register and placing it into the interrupt
| receive buffer.  In addition to this basic function it also handles the
| ctrl Q/ control S operation of the receive for both user initiated ctrl S/Q
| and those generated from this routine when it senses that the receive buffer
| has filled to the top limit.  The following summarises the behaviour under
| the various conditions which can occur during its operation :
|
|
| (i)   If the character read from the uart data register is a control Q
|       then the routine checks to see if the associated transmitter is
|       suspended.  If so then if there is a character in the tx buffer it
|       takes it and puts it directly out of the transmitter.  This then
|       reenables the transmit interrupts.  The user_ctrlS_ctrlQ flag is set
|       the 1 to say that the channel is not blocked by a control S.  If
|       there is no character in the  transmit buffer then the
|       user_ctrlS_ctrlQ flag is set to 1.
|
| (ii)  If the character is a control S then the user_ctrlS_ctrlQ
|       flag in the tx buffer is set to zero.
|
| (iii) If the character is neither a control S or a control Q then
|       it is a character which should be put in the buffer.  If the
|       number of characters in the buffer is >= buf_up_lim but less
|       than buf_size then the character is placed in the buffer and
|       then the receiver must instigate the transmission of a
|       control S.  To do this it firstly checks to see if the
|       transmitter is suspended.  If so then the receiver directly
|       sends the character out of the transmitter data register.
|       This would have the effect of reenabling the transmitter
|       interrupts - none of the control flags are effected by this
|       operation so that if the tx channel is blocked by a user
|       control S then on the first interrupt this is checked and
|       the transmitter would be set back to suspended ( although
|       this would have the effect of deadlocking the system ).
|
| (iv)  If the buffer is full when an attempt is made to put a
|       character in it then an error is signalled.
|
|
|   Parameters : - pointer to the receive buffer
|
|   Entry via  : - interrupt_handler_8274
|
============================================================================
*/

void service_rx_8274 (rx_int_buffer* rx_buf_ptr ) {

	tx_int_buffer* tx_buf_ptr;
	unsigned char uart_data;
	char buf_room_sem_flag = FALSE;
	char buf_sem_index_flag = FALSE;

	uart_data = inportb (rx_buf_ptr->data_reg_add) & 0x7f;

    /*
	now check if the character is control S or a control Q
	*/

	if ((rx_buf_ptr->xonxoff) && uart_data == CTRL_S) {
		tx_buf_ptr = rx_buf_ptr->tx_buf_ptr;
		tx_buf_ptr->user_ctrlS_ctrlQ = DISABLED;  /* set to suspend tx int */
	} /* if */
	else {
		if ((rx_buf_ptr->xonxoff ) && uart_data == CTRL_Q) {
			tx_buf_ptr = rx_buf_ptr->tx_buf_ptr;
			if (tx_buf_ptr->cur_tx_int_status == DISABLED) {
				/*
				transmitter disabled so check if there is a
				character in the transmit buffer - if so then output
				it to enable the interrupts.
				*/

				if (tx_buf_ptr->buf_used > 0) {

					outportb (tx_buf_ptr->data_reg_add,
								tx_buf_ptr->buf_ptr [tx_buf_ptr->get_ptr ]);

					if (tx_buf_ptr->get_ptr == (tx_buf_ptr->buf_size - 1)) {
						tx_buf_ptr->get_ptr = 0;
					} /* if */
					else {
						tx_buf_ptr->get_ptr++;
					} /* else */

					tx_buf_ptr->buf_used--;

					/*
					removing a character from the buffer so have to do a
					signal to balance the wait done when the character was
					placed in the buffer.
					*/

					/* indicate that a signal must be done on the
					buf_room_index semaphore upon leaving this routine.
					*/
					buf_room_sem_flag = TRUE;

					tx_buf_ptr->cur_tx_int_status = ENABLED; /* int enabled */
				} /* if */
			} /* if */

			/*
			set the user_ctrlSctrlQ flag so that the transmit interrupts can
			be reenabled.
			*/

			tx_buf_ptr->user_ctrlS_ctrlQ = ENABLED;
		} /* if */
		else {
			/*
			just a normal character so attempt to place it into the receive
			buffer.
			*/

			if (rx_buf_ptr->buf_free > 0) {
				rx_buf_ptr->buf_ptr [rx_buf_ptr->put_ptr] = uart_data;

				if (rx_buf_ptr->put_ptr == (rx_buf_ptr->buf_size - 1 )) {
					rx_buf_ptr->put_ptr = 0;
				} /* if */
				else {
					rx_buf_ptr->put_ptr++;
				} /* else */

				rx_buf_ptr->buf_free--;

				/*
				now do a signal to the rx_buffer receive interface routine.
				The flag is set so that the signal is carried out at the
				end of this routine as the signal may cause a task switch.
				*/

				buf_sem_index_flag = TRUE;

				/*
				now check to see if the buffer has filled above the upper
				limit.
				*/

				if ((rx_buf_ptr->xonxoff ) && (rx_buf_ptr->buf_size - rx_buf_ptr->buf_free)
										> rx_buf_ptr->buf_up_lim ) {
					/*
					need to cause the transmitter to issue a control S.
					This must be achieved regardless of the current
					transmitter interrupt status.
					*/

					if (tx_buf_ptr->cur_tx_int_status == DISABLED) {
						/*
						transmitter suspended so send the character directly
						out of the transmitter.
						*/

						outportb (tx_buf_ptr->data_reg_add, CTRL_S);

						/*
						set the ctS_ctQ_flg so that the receive interface
						routine task knows that a control S has been issued
						*/

						rx_buf_ptr->ctS_ctQ_flg = TRUE;
						tx_buf_ptr->cur_tx_int_status = ENABLED;
					} /* if */
					else {
						/*
						transmitter is enabled so set the rx_ctrlQ_ctrlS
						flag so that the control S will be sent out on the
						next interrupt.
						*/

						tx_buf_ptr->rx_ctrlS_ctrlQ = CTRL_S;

						/*
						set the ctS_ctQ_flg so that the receive interface
						routine task knows that a control S has been issued.
						Even if the receive interface task issues a control
						Q before the control S has been issued no problem
						will occur ( only a redundant ctrl Q ) since it will
						simply overwrite the rx_ctrlS_ctrlQ.
						*/

						rx_buf_ptr->ctS_ctQ_flg = TRUE;
					} /* else */
				} /* if */
			} /* if */
			else {
				/*
				no room in the rx buffer so issue an error
				*/

/*!@#$              call indicate_error ( rx_buffer.err_buf_index );
!@#$*/
				;
			} /* else */
		} /* else */
	} /* else */

	if (buf_room_sem_flag) {
		_signal (tx_buf_ptr->buf_room_index);
	} /* if */

	if (buf_sem_index_flag) {
		_signal (rx_buf_ptr->buf_sem_index);
	} /* if */

} /* end of service_rx_8274 */





/*------------------------------------------------------------------------*/





/*
============================================================================
|
| interrupt_handler_8274
|
| This routine is the generic interrupt handling routine for all 8274 uarts in
| the  system.  The function of the routine is to determine what the source of
| the interrupt is within the uart and then branch to the appropriate handing
| routine.  The assumption made in the routine is that the 8274 has been
| programmed in non-vectored mode with status affects vector.  By using the
| status affects vector feature this routine can determine by polling what the
| source of the interurpt is and which channel it has occurred on.
|
| Once the source of the interrupt has been determined control then branches
| to a handling routine.  These routines are also generic and are designed to
| be used by all the interrupts in the system.
|
|    Parameters : - pointer to the receive buffer for channel A
|                 - pointer to the transmit buffer for channel A
|                 - pointer to the receive buffer for channel B
|                 - pointer to the transmit buffer for channel B
|
|    Entry via  : - 8274 specific interrupt routine.
|
============================================================================
*/

void interrupt_handler_8274 (rx_int_buffer* chA_rx_ptr,
                             tx_int_buffer* chA_tx_ptr,
                             rx_int_buffer* chB_rx_ptr,
                             tx_int_buffer* chB_tx_ptr ) {

	unsigned char interrupt_source;

    /*
    firstly determine the source of the interrupt.  Get the value of RR2 in
    chB as this contains the vector number.  Three of the bits in this are
    defined as follows :

            bit 4   =   1   =>  chA

			bits 3 & 2  =
                        00  =>  Tx buffer empty
                        01  =>  Ext/status change
						10  =>  Rx char available
                        11  =>  Special Rx condition

            bit 4   =   0   =>  chB

            bits 3 & 2  =
						Same as chA above
	*/


    /*
    request a read of the RR2 in chB
    */

	outportb (chB_rx_ptr->com_reg_add, REG2);

    /*
    now read the status affected interrupt vector
    */

	interrupt_source = ((inportb (chB_rx_ptr->com_reg_add) & 0x1c) >> 2 );

    /*
    now depending on the interrupt source call a number of handling routines
	*/

	if (interrupt_source <= 7) {
        switch (interrupt_source) {
            case 0 :
                service_tx_8274 (chB_tx_ptr);                   /* 0 */
                break;
			case 1 :
/*!@#$          call indicate_error ( illegal_8274_int_src );
!@#$*/
                break;
			case 2 :
                service_rx_8274 (chB_rx_ptr);                   /* 2 */
                break;
            case 3 :
                special_rx_cond_8274 (chB_rx_ptr);              /* 3 */
                break;
            case 4 :
				service_tx_8274 (chA_tx_ptr);                   /* 4 */
				break;
            case 5 :
/*!@#$          indicate_error ( illegal_8274_int_src );
!@#$*/
            case 6 :
                service_rx_8274 (chA_rx_ptr);                   /* 6 */
                break;
				case 7 :
                special_rx_cond_8274 (chA_rx_ptr);              /* 7 */
		} /* switch */
    } /* if */
    else {
        /*
        now somehow got an illegal interrupt source
        */

/*!@#$  call indicate_error ( illegal_8274_int_src );
!@#$*/
        ;
    } /* else */

} /* end of interrupt_handler_8274 */





/*------------------------------------------------------------------------*/





/*
===========================================================================
|
| receive_interrupt_handler_8251A
|
|
| This routine is entered for all the serial receive interrupts from 8251A
| uarts in the system. It is entered via an interrupt source specific routine
| which passes in a pointer to the buffer of interest.
|
| The function of the routine is to take the characters from the 8251A uart
| data register and place it in the buffer.  Several conditions can occur
| depending on the state of the buffer and the character input :
|
|
| (i)     If the character read from the uart is a control Q or control S then
|        the routine manipulates the transmit interrupts etc. to achieve the
|        function.  The character is not placed in the buffer.
|
| (ii)    If the character does not belong to (i) and if the number of characters
|       does not equal buf_up_lim then the character is placed in the
|       buffer,  pointers are updated and a rx_signal is carried out on the
|       semaphore indexed by buf_sem_index.
|
| (iii)   If the character does not belong to (i) but the number of characters
|       in the buffer is equal to or greater than buf_up_lim but less than
|       buf_size then the character is placed in the buffer and the
|       ctS_ctQ_flg is set and the transmit routine is signalled to send a
|       ctrl S or ctrl Q.
|
| (iv)    If the buffer is full then the error buffer is updated to indicate
|       that a buffer overrun has occurred.
|
| (v)     If upon reading a character a framing, parity or data overrun error
|       is detected the appropriate location in the error buffer are updated.
|
| The index into the error buffer is passed into the routine inside the buffer
| data structure.  This index is to the start of a block of 4 consecutive error
| locations which are used by the serial handlers.
|
| Parameters :
|        pointer to the receive interrupt buffer structure
|
=============================================================================
*/

void receive_interrupt_handler_8251A (rx_int_buffer* rx_buf_ptr) {

	unsigned char uart_err_data;
	tx_int_buffer* tx_buf_ptr;
	//unsigned int index_err_buf;
	unsigned char uart_data;

	/* firstly check for any errors in the received data */

	uart_err_data = inportb (rx_buf_ptr->com_reg_add);

	if ((uart_err_data & FRAME_ERR_MASK) > 0) {
		/* framing error has occurred so update error buffer and read in
		the bad character - this character is discarded.  Note the error
		buffer can be updated directly since we are updating an error number
		which can only be updated from this routine. */

		/* read out the bad character */
		put_rx_buffer (FRAMING_ERROR_BYTE, rx_buf_ptr);
		uart_data = inportb (rx_buf_ptr->data_reg_add);

		/* now clear the error - remember to maintain the current
		transmitter interrupt status */

		tx_buf_ptr = rx_buf_ptr->tx_buf_ptr;
		outportb (rx_buf_ptr->com_reg_add,
				rx_buf_ptr->uart_com_data | tx_buf_ptr->cur_tx_int_status
				| CLEAR_ERR_8251);
	} /* if */
	else {
		if ((uart_err_data & PARITY_ERR_MASK) > 0) {
			/* parity error - do as for framing error above */

			put_rx_buffer (PARITY_ERROR_BYTE, rx_buf_ptr);
			/* read out the bad character */

			uart_data = inportb (rx_buf_ptr->data_reg_add);

			/* now clear the error - remember to maintain the current
			transmitter interrupt status */

			tx_buf_ptr = rx_buf_ptr->tx_buf_ptr;

			/*
			remember to maintain the current transmitter interrupt
			status.
			*/

			outportb (rx_buf_ptr->com_reg_add,
									 rx_buf_ptr->uart_com_data
									 | tx_buf_ptr->cur_tx_int_status
									 | CLEAR_ERR_8251);
		} /* if */
		else {
			if ((uart_err_data & DATA_OVRUN_MASK ) > 0) {
				/* data register overrun error - update error
				buffer and attempt to place character in the
				buffer */

				put_rx_buffer (OVERRUN_ERROR_BYTE, rx_buf_ptr);

				uart_data = inportb (rx_buf_ptr->data_reg_add);
				put_rx_buffer (uart_data, rx_buf_ptr);

				/* now clear the error - remember to maintain the
				current transmitter interrupt status */

				tx_buf_ptr = rx_buf_ptr->tx_buf_ptr;
				outportb (rx_buf_ptr->com_reg_add,
						rx_buf_ptr->uart_com_data |
						tx_buf_ptr->cur_tx_int_status |
						CLEAR_ERR_8251);
			} /* if */
			else {
				/* no errors in uart so place data in data
				buffer */
				uart_data = inportb (rx_buf_ptr->data_reg_add);
				put_rx_buffer (uart_data, rx_buf_ptr);
			} /* else */
		} /* else */
	} /* else */
} /* end of receive_interrupt_handler_8251A */





/*---------------------------------------------------------------------------*/





/*
===========================================================================
|
| put_rx_buffer
|
| This procedure places the byte passed in into the buffer whose pointer is
| passed into the routine.  This is a special procedure since it is designed
| to operate on the special receive buffers.
|
| This procedure also performs a number of other functions.  If the character
| passed in is a control S or control Q it is not placed in the receive buffer.
| Instead the tramnsmit routine has to be signalled to set itself to the
| appropriate state.  For example if the character is a control S then the user
| is requesting that transmission be stopped.  The user_ctrlS_ctrlQ flag in the
| appropriate transmit buffer is set appropriayely and the transmit interrupts
| are directly manipulated.
|
| If upon attempting to place the character in the buffer it is found that the
| buf_up_lim is exceeded the transmit routine is signalled to issue a control S
| via the ctrlS_ctrlQ_flag and the character is placed in the buffer.  If the
| buffer is completely full then the error buffer is updated to say that a buffer
| overrun has occurred.
|
|
| Parameters
|       - data to be placed in the buffer
|       - pointer to the receive buffer data structure
|
============================================================================
*/

/* rjlov - I changed the uart_data to uart_8data, and added
** uart_data as a local variable, so that we would not
** lose the high bits. This looks like a bug in UNOS to me.
*/
void put_rx_buffer (unsigned char uart_8data, rx_int_buffer* rx_buf_ptr) {
	unsigned char uart_data;
	int putinbuf;

	tx_int_buffer* tx_buf_ptr;
	//unsigned int index_err_buf;

	tx_buf_ptr = rx_buf_ptr->tx_buf_ptr;

	/*
	make sure the high bit is not set so that correct comparisons are
	ensured even if 7 bit data has been programmed
	*/

	uart_data = uart_8data & 0x7f;

	/* firstly check if the character is a control S or control Q */

	/* rjlov - I can't believe it.
	** The thing was still doing checking for ^Q even
	** when rx_buf_ptr->xonxoff was false.
	*/
	if (rx_buf_ptr->xonxoff) {
		putinbuf = 0;
		if (uart_data == CTRL_S) {

		/* set the user_ctrlS_ctrlQ flag so that interrupts will be disabled
		upon receipt of the next transmitter interrupt */

			tx_buf_ptr->user_ctrlS_ctrlQ = DISABLED; /* set to disable tx int */
		} /* if */
		else {
			if ((uart_data == CTRL_Q) || (uart_data == _CR)) {

			/*
			set the user_ctrlS_ctrlQ flag so that interrupts will be
			enabled and then enable the tx interrupts if required
			*/

				tx_buf_ptr->user_ctrlS_ctrlQ = ENABLED; /* set to enable tx int */

				if ((tx_buf_ptr->cur_tx_int_status == DISABLED ) &&
					(tx_buf_ptr->tx_interrupt == ENABLED )) {
					outportb (rx_buf_ptr->com_reg_add +
						rx_buf_ptr->intr_ctrl_reg_offset,
						rx_buf_ptr->uart_com_data |
						rx_buf_ptr->tx_enable_disable_bit);
					tx_buf_ptr->cur_tx_int_status = ENABLED;
				} /* if */

			/* <<<<<< Special section put in for the Cook Islands Project.
			Want CR to be interpreted as a Ctrl S (as in the above few lines)
			but the CR also has to be put into the receive buffer and not
			thrown away as is the Ctrl Q.
			>>>>>>*/

				if (uart_data == _CR) {
					if (rx_buf_ptr->buf_free > 0) {
						rx_buf_ptr->buf_ptr [rx_buf_ptr->put_ptr] = uart_data;
						if (rx_buf_ptr->put_ptr == (rx_buf_ptr->buf_size - 1 )) {
							rx_buf_ptr->put_ptr = 0;
						} /* if */
						else {
							(rx_buf_ptr->put_ptr)++;
						} /* else */

						(rx_buf_ptr->buf_free)--;

					/* now check to see if the buffer is filled above
					the upper limit */

						if ((rx_buf_ptr->xonxoff) && (rx_buf_ptr->buf_size - rx_buf_ptr->buf_free)
													> rx_buf_ptr->buf_up_lim ) {
						/* cause the transmitter to issue a controlS */

							tx_buf_ptr->rx_ctrlS_ctrlQ = CTRL_S;

						/* set the ctS_ctQ_flg so that the receive
							interface task knows that a control S has
						been issued.  Even if it was possible that
						the control Q is issued before the control S
						this should cause no problem - one would
						just get a redundant control Q. */

							rx_buf_ptr->ctS_ctQ_flg = TRUE;

						/* now enable the transmitter interrupt
						regardless of the status of the other flags
						*/

							outportb (rx_buf_ptr->com_reg_add +
									rx_buf_ptr->intr_ctrl_reg_offset,
									rx_buf_ptr->uart_com_data |
									rx_buf_ptr->tx_enable_disable_bit);


							tx_buf_ptr->cur_tx_int_status = ENABLED;
						} /* if */

						_signal (rx_buf_ptr->buf_sem_index);

					} /* if */
					else {
					/* there is no room in the receive buffer so issue
					an error */

					} /* else */

				} /* if CR */

			} /* if */
			else {
			/* not control S or control Q so attempt to place the
			character in the buffer */
				putinbuf = 1;
			} /* else */
		}    /* if not xonxoff */
	} /* end if Xon Xoff is enabled */
	else
		putinbuf = 1;
		  /* if Xon Xoff is disabled, then put the character into
		  ** the buffer, no matter what it is.
		  */

	 if (putinbuf) {
			if (rx_buf_ptr->buf_free > 0) {

			/* rjlov - old code
				rx_buf_ptr->buf_ptr [rx_buf_ptr->put_ptr] = uart_data;
			*/
				rx_buf_ptr->buf_ptr [rx_buf_ptr->put_ptr] = uart_8data;


				if (rx_buf_ptr->put_ptr == (rx_buf_ptr->buf_size - 1 )) {
					rx_buf_ptr->put_ptr = 0;
				} /* if */
				else {
					(rx_buf_ptr->put_ptr)++;
				} /* else */

				(rx_buf_ptr->buf_free)--;

				/* now do a signal to the rx_buffer interface
				routine */

				/* now check to see if the buffer is filled above
				the upper limit */

				if ((rx_buf_ptr->xonxoff) && (rx_buf_ptr->buf_size - rx_buf_ptr->buf_free)
											> rx_buf_ptr->buf_up_lim ) {
					/* cause the transmitter to issue a controlS */

					tx_buf_ptr->rx_ctrlS_ctrlQ = CTRL_S;

					/* set the ctS_ctQ_flg so that the receive
					interface task knows that a control S has
					been issued.  Even if it was possible that
					the control Q is issued before the control S
					this should cause no problem - one would
					just get a redundant control Q. */

					rx_buf_ptr->ctS_ctQ_flg = TRUE;

					/* now enable the transmitter interrupt
					regardless of the status of the other flags
					*/

					outportb (rx_buf_ptr->com_reg_add +
							rx_buf_ptr->intr_ctrl_reg_offset,
							rx_buf_ptr->uart_com_data |
							rx_buf_ptr->tx_enable_disable_bit);


					tx_buf_ptr->cur_tx_int_status = ENABLED;
				} /* if buffer full and xonxoff */

				_signal (rx_buf_ptr->buf_sem_index);

			} /* if buf_free  */
			else {
				/* there is no room in the receive buffer so issue
				an error */
			} /* if buf_free ... else */
	 } /* if (putinbuf) */
} /* end of put_rx_buffer */





/*--------------------------------------------------------------------------*/







/****************************************************************************/
/*                                                                          */
/*                         RECEIVE INTERFACE TASK                           */
/*                                                                          */
/****************************************************************************/


/*
============================================================================
| rx_buf_interface_task
|
| This task takes characters from the receive buffer of a hardware channel and
| puts it into a normal system mbx buffer - i.e. the task provides a virtual
| interface between the hardware buffers and the mbx buffers.  This means that
| tasks can communicate with the serial channels in the same way as they would
| to other tasks.
|
| Support for the 8274 uart was added some time after the original 8251A
| version of this task was running.  With this addition this task now forms a
| device independent interface for both these uarts.
|
| For the Cook Islands project the NS16450 uart support and task initialsation
| was added. The initialisation was a bit of a fudge. Because the new version
| of UNOS does ot allow parameters to be passed to tasks when they are created
| it was necessary to devise the following scheme:- when the task is created
| the initialisation routine passed to the create_task function is the task
| itself. UNOS is set up so that when the initialisation task is called the
| stack being used is the runtime stack. Therefore any local variables set
| up during this first call will sit on the task's stack. The values used
| for the initialisation have to be stored in some global variables. These
| variables are task specific. A function is called from within the task
| to see whether the variables should be initlialised. This variable indicates
| whether or not the kernel is running, and therefore allows one to
| differentiate whether this is an initialisation call or a normal task
| switch.
|
| Parameters
|        - pointer to the receive buffer
|        - pointer to the system fifo buffer
|        - type of uart the receive task is attached to ( 8251A, 8274 )
|
| Entry via
|       - main system creation procedure ( main )
|
=============================================================================
*/

void rx_buf_interface_task (void *local_var_ptr) {


	rx_int_buffer* rx_buf_ptr;
	unsigned char uart_type;
	tx_int_buffer* tx_buf_ptr;
	unsigned char buf_data;
	char* task_name_ptr;
	unsigned char message [2];
	unsigned int mess_lgth;
	local_var_type *ptr_local_var;
	unsigned char temp_ctS_ctQ_flag;

	disable ();
	ptr_local_var = (local_var_type*)local_var_ptr;
	rx_buf_ptr = ptr_local_var->rx_int_bufptr;
	uart_type = rx_buf_ptr->uart_type;
	enable ();

	/* now wait here for a task to be connected to the serial handler.
	Until this happens the serial handler will not run.
	*/
	task_name_ptr = rcv_mess (message, &mess_lgth, 0);

	/* Now have a task name to connect to so store this in a connected_tasks
	structure.
	*/
	if ((rx_buf_ptr->connected_task_ptr = (connected_tasks*) umalloc (
						sizeof (connected_tasks))) == NULL) {
		/* has been a problem allocating memory so generate an error
		and do something to stop.
		*/
		cprintf ("\n\nProblem allocating memory in rx_buf_interface_task\n");
		exit (EXIT_FAILURE);
	} /* if */

	/* now fill in the connected_tasks structure appropriately */
	rx_buf_ptr->connected_task_ptr->connected_taskname_ptr = task_name_ptr;
	rx_buf_ptr->connected_task_ptr->next_taskname_ptr = NULL;


	/* begin the infinite loop of the task */

	while (TRUE) {
		/* wait for data available in the receive buffer */

		wait (rx_buf_ptr->buf_sem_index);

/*
		wait (buffer_manipulation);
*/

		disable ();

		buf_data = rx_buf_ptr->buf_ptr [rx_buf_ptr->get_ptr];
		if (rx_buf_ptr->get_ptr == (rx_buf_ptr->buf_size - 1)) {
			rx_buf_ptr->get_ptr = 0;
		} /* if */
		else {
			(rx_buf_ptr->get_ptr)++;
		} /* else */

		(rx_buf_ptr->buf_free)++;

		temp_ctS_ctQ_flag = rx_buf_ptr->ctS_ctQ_flg;

		enable ();

/*
		_signal (buffer_manipulation);
*/

		if(rx_buf_ptr->xonxoff)
		{
		 /* now check to see if a control S has been issued from the receive
		 interrupt routine.  If so then check to see if the lower buffer limit
		 has been reached and if so then signal the transmitter to send out a
		 control Q. */

		 if ((temp_ctS_ctQ_flag > 0 ) &&
			(return_semaphore_value (rx_buf_ptr->buf_sem_index)
										 < rx_buf_ptr->buf_low_lim)) {

			/* have issued a control S before - now have reached the lower
			limit  so send out a control Q */

			disable ();
			rx_buf_ptr->ctS_ctQ_flg = FALSE;   /* clear the flag */
			enable ();

			tx_buf_ptr = rx_buf_ptr->tx_buf_ptr;

			/*
			now the action changes here depending on the type of uart -
			i.e. an 8274, 8251A or NS16450.
			*/

			if (uart_type == I8274) {
				disable ();
				if (tx_buf_ptr->cur_tx_int_status == DISABLED) {
					outportb (tx_buf_ptr->data_reg_add, CTRL_Q);
					tx_buf_ptr->cur_tx_int_status = ENABLED;
				} /* if */
				else {
					/*
					make sure the control Q is sent out on the next
					interrupt
					*/
					tx_buf_ptr->rx_ctrlS_ctrlQ = CTRL_Q;
				} /* else */
				enable ();
			} /* if */
			else  {

				/*
				must be an 8251A or NS16450 uart
				*/

				/*
				set transmitter up to issue control Q
				*/
				disable ();
				tx_buf_ptr->rx_ctrlS_ctrlQ = CTRL_Q;

				/*
				enable the transmitter interrupts regardless of the
				status of the other tx interrupt flags
				*/

				outportb (rx_buf_ptr->com_reg_add +
						  rx_buf_ptr->intr_ctrl_reg_offset,
						  rx_buf_ptr->uart_com_data |
						  rx_buf_ptr->tx_enable_disable_bit);

				tx_buf_ptr->cur_tx_int_status = ENABLED;
				enable ();
			} /* else */
		 } /* if */
		} // if
		/* now write the data read from the receive buffer into the
		appropriate mbx buffer */

		send_mess ( &buf_data, 1,
				rx_buf_ptr->connected_task_ptr->connected_taskname_ptr);


	} /* while */
} /* end of rx_buf_interface_task */





/*--------------------------------------------------------------------------*/





/*
============================================================================
|
| transmit_interrupt_handler_8251A
|
| This routine is entered for all 8251A transmit interrupts in the system.
| Essentially this routine takes data from the special transmit buffer and
| puts it out of the particular hardware channel which has caused the
| interrupt.
|
| Besides this straight forward function this routine also does special
| processing depending on the status of the rx_ctrlS_ctrlQ variable in the
| transmit buffer.  If non-zero then the contents of this variable are
| transmitted on the next interrupt.  If zero then the data is fetched from the
| transmit buffer.
|
| If upon attempting a fetch from the transmit buffer it is found that the buffer
| is empty the transmit interrupts for this particular uart are disabled and the
| flag in the transmit buffer is set appropriately to indicate the status of the
| interrupt.
|
| Parameters
|       - pointer to the transmit buffer in question
|
============================================================================
*/

void transmit_interrupt_handler_8251A (tx_int_buffer* tx_buf_ptr) {

	/* first check to see if a control S or control Q has been ordered by the
	receiver */

	if ((tx_buf_ptr->xonxoff) && (tx_buf_ptr->rx_ctrlS_ctrlQ > 0)) {
			outportb (tx_buf_ptr->data_reg_add, tx_buf_ptr->rx_ctrlS_ctrlQ);
			tx_buf_ptr->rx_ctrlS_ctrlQ = 0;
	} /* if */
	else {
		/* now check if a user control S has been issued - if so then
		then disable the interrupts */

		if (tx_buf_ptr->user_ctrlS_ctrlQ == 0) {
			/*
			disable the transmitter interrupts
			*/

			outportb (tx_buf_ptr->com_reg_add, tx_buf_ptr->uart_com_data);

			/*
			set the appropriate status in the tx_buffer struc.
			*/

			tx_buf_ptr->cur_tx_int_status = DISABLED;
		} /* if */
		else {
			/* no user control S so process normally - get a byte
			from the buffer and output normally */

			if (tx_buf_ptr->buf_used > 0) {
				outportb (tx_buf_ptr->data_reg_add,
							tx_buf_ptr->buf_ptr [tx_buf_ptr->get_ptr]);
				if (tx_buf_ptr->get_ptr == ( tx_buf_ptr->buf_size - 1 )) {
					tx_buf_ptr->get_ptr = 0;
				} /* if */
				else {
					(tx_buf_ptr->get_ptr)++;
				} /* else */

				(tx_buf_ptr->buf_used)--;
				_signal (tx_buf_ptr->buf_room_index);
			} /* if */
			else {
				/*
				no used bytes in the buffer so disable the
				transmitter interrupts
				*/

				tx_buf_ptr->tx_interrupt = DISABLED;
				tx_buf_ptr->cur_tx_int_status = DISABLED;
				outportb (tx_buf_ptr->com_reg_add,
											tx_buf_ptr->uart_com_data);
			} /* else */
		} /* else */
	} /* else */
} /* end of transmit_interrupt_handler_8251A */





/*--------------------------------------------------------------------------*/






/****************************************************************************/
/*                                                                          */
/*                          TRANSMIT INTERFACE TASK                         */
/*                                                                          */
/****************************************************************************/



/*
============================================================================
|
| tx_buf_interface_task
|
| This task takes a byte of data from a standard system fifo buffer and places
| it into the tx interrupt buffer.  The routine makes the tx buffer appear to be
| a fifo buffer as far as the rest of the tasks in the system are concerned.
|
| This routine is required so that the transmit interrupts for the particular
| hardware device is manipulated appropriately - i.e. interrupts are enabled
| when data has been placed in the buffer etc.  This interrupt control is done
| in such a way that user ctrl S ctrl Q requests are respected.
|
| The task is defined reentrantly so that it may be used for all transmit buffers
| in the system.  A new task with its own stack has to be defined for each buffer
| which has to be serviced.  The setup procedure for each tx buffer task is
| identical to that for non-reentrant tasks - queue the task on the appropriate
| queue ( usually a blocked queue ) and then create a stack for the task.
| The buffers which are to be acted on by the routine are passed into the task
| at the point where it is activated in the main routine.  When the task starts
| running for the first time any initialisation is performed.  The task then
| enters an infinite loop.
|
| The section added to this task for the 8274 was added after the original
| 8251A version of it had been running successfully for approximately 12
| months. The NS16450 uart support was added in 1992 for the Cook Islands
| project.
|
| Parameters:	-	none
|
| Returns:		-	nothing
|
============================================================================
*/

void tx_buf_interface_task (void *local_var_ptr) {

	unsigned char uart_type;
	tx_int_buffer* tx_buf_ptr;
	unsigned char* buf_data_ptr;
	unsigned int mess_lgth = 0;
	local_var_type *ptr_local_var;
	int i;
	char user_ctrlS_ctrlQ;
	char cur_tx_int_status;


	disable ();
	ptr_local_var = (local_var_type*)local_var_ptr;

	tx_buf_ptr = ptr_local_var->tx_int_bufptr;
	uart_type = tx_buf_ptr->uart_type;

	if ((buf_data_ptr = (unsigned char*)umalloc
								(ptr_local_var->mbx_mess_size)) == NULL) {
		/* has been a problem allocating memory so print a message and
		quit from the program.
		*/
		cprintf ("\nError allocating memory for serial messages",
					"\n in the transmit buffer interface task\n\n");
		exit (EXIT_FAILURE);
	} /* if */

	enable ();

	while (TRUE) {
		/* get data from mail box buffer */

		rcv_mess (buf_data_ptr, &mess_lgth, 0);

		i = 0;

		/* now transfer the message to the interrupt buffer */
		while (mess_lgth > 0) {
			/* wait for room in the transmit buffer */

			wait ( tx_buf_ptr->buf_room_index );

			/*
			if the uart type for this channel is an 8274 then test to see if
			the transmit interrupts are currently enabled.  If not then the
			byte from the fifo is not put into the buffer but instead output
			directly to the uart data register.  This also enables the
			interrupts so the flags must be updated.
			*/

			if (uart_type == I8274) {
				/*
				see if the interrupts are allowed to be enabled
				*/

				disable ();
				user_ctrlS_ctrlQ = tx_buf_ptr->user_ctrlS_ctrlQ;
				cur_tx_int_status = tx_buf_ptr->cur_tx_int_status;
                enable ();

				if (user_ctrlS_ctrlQ == TRUE) {
					/*
					check if the data has to go into the buffer or whether it
					is put directly out of the uart.  This depends on the
					cur_tx_int_status.
					*/

					if (cur_tx_int_status == ENABLED) {
						/*
						put the data in the buffer as the interrupts are
						currently enabled
						*/

						tx_buf_ptr->buf_ptr [tx_buf_ptr->put_ptr] =
															buf_data_ptr[i];

						if (tx_buf_ptr->put_ptr == (tx_buf_ptr->buf_size - 1 )) {
							tx_buf_ptr->put_ptr = 0;
						} /* if */
						else {
							(tx_buf_ptr->put_ptr)++;
						} /* else */

						i++;

						disable();
						(tx_buf_ptr->buf_used)++;

						tx_buf_ptr->tx_interrupt = ENABLED;
						enable ();
					} /* if */
					else {
						/*
						the interrupts are currently disabled so to enable
						them again the first byte has to be output directly
						out of the uart.
						*/

						outportb (tx_buf_ptr->data_reg_add,
														buf_data_ptr[i]);

						/*
						since the wait at the top of this routine basically
						decrements the amount of storage available in the
						buffer then a kernel_signal must be issued here on
						the same semaphore since the byte is not being put
						into the buffer but instead is being sent directly
						to the uart.
						*/

						i++;

						disable ();
						tx_buf_ptr->tx_interrupt = ENABLED;
						tx_buf_ptr->cur_tx_int_status = ENABLED;
						enable ();
						_signal ( tx_buf_ptr->buf_room_index );

					} /* else */
				} /* if */
				else {
					/*
					the user has issued a control S so put the data in the
					buffer.
					*/

					tx_buf_ptr->buf_ptr [tx_buf_ptr->put_ptr] =
														buf_data_ptr[i];

					if (tx_buf_ptr->put_ptr == (tx_buf_ptr->buf_size - 1 )) {
						tx_buf_ptr->put_ptr = 0;
					} /* if */
					else {
						(tx_buf_ptr->put_ptr)++;
					} /* else */

					i++;

					disable ();
					(tx_buf_ptr->buf_used)++;

					tx_buf_ptr->tx_interrupt = ENABLED;
                    enable ();
				} /* else */
			} /* if */
			else {
				/*
				enter this section if the uart is an 8251A or NS16450
				*/


				/* now put the data in the buffer */

				tx_buf_ptr->buf_ptr [tx_buf_ptr->put_ptr] = buf_data_ptr[i];

				if (tx_buf_ptr->put_ptr == (tx_buf_ptr->buf_size - 1)) {
					tx_buf_ptr->put_ptr = 0;
				} /* if */
				else {
					(tx_buf_ptr->put_ptr)++;
				} /* else */

				i++;

				disable ();
				(tx_buf_ptr->buf_used)++;

				/*
				now make sure that the tx_interrupt is enabled.  Then "and"
				with user_ctrlS_ctrlQ to form the current interrupt status
				and program the uart interrupt status
				*/

				tx_buf_ptr->tx_interrupt = ENABLED;
				tx_buf_ptr->cur_tx_int_status = tx_buf_ptr->tx_interrupt
											& tx_buf_ptr->user_ctrlS_ctrlQ;


				outportb (tx_buf_ptr->com_reg_add +
							 tx_buf_ptr->intr_ctrl_reg_offset,
							 tx_buf_ptr->uart_com_data |
							 tx_buf_ptr->tx_enable_disable_bit);
				enable ();


			} /* else */

			mess_lgth--;
		} /* while */

	} /* while */


} /* end of tx_buf_interface_task */





/*-------------------------------------------------------------------------*/





/*
============================================================================
|
| service_modem_status_16450
|
| This function is entered if there has been an interrupt caused by a change
| of the modem status bits in the NS16450 uart. It simply reads the modem
| status register and then returns. There is no action currently taken on the
| modem status bits.
|
| Parameters:	-	pointer to the receive interrupt buffer
|
| Returns:		-	nothing
|
============================================================================
*/

void service_modem_status_intr_16450 (rx_int_buffer* rx_bufptr) {

	unsigned int modem_status;

	/* read the modem status register to clear the interrupt */

	modem_status = inportb (rx_bufptr->com_reg_add + MODEM_STATUS_REG);

	modem_status++;
} /* end of service_modem_status_intr_16450 */





/*--------------------------------------------------------------------------*/





/*
=============================================================================
|
| service_rxerr_16450
|
| This function services any interrupt from the NS16450 uart which is the
| result of an error condition in the uart. The error conditions result in the
| generation of an error in the error log of the system.
|
| The break bit is strictly not an error. However it is also in the same
| register as the error bits. In the current implementation I am ignoring
| it.
|
| Parameters:	-	pointer to the receive interrupt buffer
|
| Returns:		-	nothing
|
=============================================================================
*/

void service_rxerr_intr_16450 (rx_int_buffer* rx_bufptr) {

	unsigned char rx_error_type;

	/* now read the line status register */

	rx_error_type = ((inportb (rx_bufptr->com_reg_add + LINE_STATUS_REG)
								& 0x1e) >> 1);

	/* Now detect the individual errors */
	if (rx_error_type & OVERRUN_MASK_16450) {
		put_rx_buffer (OVERRUN_ERROR_BYTE, rx_bufptr);
	} /* if */

	if (rx_error_type & PARITY_ERROR_MASK_16450) {
		put_rx_buffer (PARITY_ERROR_BYTE, rx_bufptr);
	} /* if */

	if (rx_error_type & FRAMING_ERROR_MASK_16450) {
		put_rx_buffer (FRAMING_ERROR_BYTE, rx_bufptr);
	} /* if */

	/* I am currently ignoring the break bit */

	/* Flush out any funny characters that may be present */

	inportb (rx_bufptr->data_reg_add);

} /* end of service_rxerr_intr_16450 */





/*-------------------------------------------------------------------------*/





/*
============================================================================
|
| service_tx_16450
|
| This routine is entered for all NS16450 transmit interrupts in the system.
| Essentially this routine takes data from the special transmit buffer and
| puts it out of the particular hardware channel which has caused the
| interrupt.
|
| Besides this straight forward function this routine also does special
| processing depending on the status of the rx_ctrlS_ctrlQ variable in the
| transmit buffer.  If non-zero then the contents of this variable are
| transmitted on the next interrupt.  If zero then the data is fetched from
| the transmit buffer.
|
| If upon attempting a fetch from the transmit buffer it is found that the
| buffer is empty the transmit interrupts for this particular uart are
| disabled and the flag in the transmit buffer is set appropriately to
| indicate the status of the interrupt.
|
| Parameters:	-	pointer to the transmit buffer in question
|
| Returns:		-	nothing
|
============================================================================
*/

void service_tx_intr_16450 (tx_int_buffer* tx_buf_ptr) {

	/* first check to see if a control S or control Q has been ordered by the
	receiver */

	if ((tx_buf_ptr->xonxoff) && (tx_buf_ptr->rx_ctrlS_ctrlQ > 0)) {
			outportb ( tx_buf_ptr->data_reg_add, tx_buf_ptr->rx_ctrlS_ctrlQ);
			tx_buf_ptr->rx_ctrlS_ctrlQ = 0;
	} /* if */
	else {
		/* now check if a user control S has been issued - if so then
		then disable the interrupts */

		if ((tx_buf_ptr->xonxoff) &&(tx_buf_ptr->user_ctrlS_ctrlQ == DISABLED)){
			/*
			disable the transmitter interrupts
			*/

			outportb (tx_buf_ptr->com_reg_add +
					tx_buf_ptr->intr_ctrl_reg_offset,
					tx_buf_ptr->uart_com_data);

			/*
			set the appropriate status in the tx_buffer struc.
			*/

			tx_buf_ptr->cur_tx_int_status = DISABLED;
		} /* if */
		else {
			/* no user control S so process normally - get a byte
			from the buffer and output normally */

			if (tx_buf_ptr->buf_used > 0) {
				outportb (tx_buf_ptr->data_reg_add,
							tx_buf_ptr->buf_ptr [tx_buf_ptr->get_ptr]);
				if (tx_buf_ptr->get_ptr == ( tx_buf_ptr->buf_size - 1 )) {
					tx_buf_ptr->get_ptr = 0;
				} /* if */
				else {
					(tx_buf_ptr->get_ptr)++;
				} /* else */

				(tx_buf_ptr->buf_used)--;
				_signal (tx_buf_ptr->buf_room_index);
			} /* if */
			else {
				/*
				no used bytes in the buffer so disable the
				transmitter interrupts
				*/

				tx_buf_ptr->tx_interrupt = DISABLED;
				tx_buf_ptr->cur_tx_int_status = DISABLED;
				outportb (tx_buf_ptr->com_reg_add +
						tx_buf_ptr->intr_ctrl_reg_offset,
						tx_buf_ptr->uart_com_data);
			} /* else */
		} /* else */
	} /* else */
} /* end of transmit_interrupt_handler_8251A */





/*--------------------------------------------------------------------------*/




/*
=============================================================================
|
| service_rx_intr_16450
|
| This routine is entered for all the serial receive interrupts from MS16450
| uarts in the system. It is entered via an interrupt source identification
| routine which passes in a pointer to the buffer of interest.
|
| The function of the routine is to take the characters from the NS16450 uart
| data register and place it in a buffer.  Several conditions can occur
| depending on the state of the buffer and the character input :
|
|
| (i)   If the character read from the uart is a control Q or control S then
|       the routine manipulates the transmit interrupts etc. to achieve the
|       function.  The character is not placed in the buffer.
|
| (ii)  If the character does not belong to (i) and if the number of characters
|       does not equal buf_up_lim then the character is placed in the
|       buffer,  pointers are updated and a rx_signal is carried out on the
|       semaphore indexed by buf_sem_index.
|
| (iii) If the character does not belong to (i) but the number of characters
|       in the buffer is equal to or greater than buf_up_lim but less than
|       buf_size then the character is placed in the buffer and the
|       ctS_ctQ_flg is set and the transmit routine is signalled to send a
|       ctrl S or ctrl Q.
|
| (iv)  If the buffer is full then the error buffer is updated to indicate
|       that a buffer overrun has occurred.
|
| (v)   If upon reading a character a framing, parity or data overrun error
|       is detected the appropriate location in the error buffer are updated.
|
| Parameters:	-	pointer to the receive interrupt buffer structure
|
| Returns:		-	nothing
|
=============================================================================
*/

void service_rx_intr_16450 (rx_int_buffer* rx_buf_ptr) {

	unsigned char uart_data;

	uart_data = inportb (rx_buf_ptr->data_reg_add);
	put_rx_buffer (uart_data, rx_buf_ptr);

} /* end of service_rx_intr_16450 */





/*---------------------------------------------------------------------------*/





/*
=============================================================================
|
| interrupt_handler_16450
|
| This routine is the generic interrupt handler for all interrupts from the
| NS16450 uarts in the system. This function is called from the first level
| interrupt handler which is the interrupt specific interface.
|
| This function gets a byte from the uart and then uses this to distinguish
| the source of the interrupt. It then calls the appropriate handling routine
| for the particular interrupt source.
|
| Parameters:	-	pointer to the receive interrupt buffer.
|				-	pointer to the transmit interrupt buffer.
|
| Returns:		-	nothing
|
=============================================================================
*/

void interrupt_handler_16450 (rx_int_buffer* rx_bufptr,
				tx_int_buffer* tx_bufptr, unsigned char intr_source) {

	switch (intr_source) {
		case MODEM_STATUS_INTR :
			service_modem_status_intr_16450 (rx_bufptr);
		break;

		case TX_EMPTY_INTR :
			service_tx_intr_16450 (tx_bufptr);
		break;

		case RX_DATA_RDY_INTR :
			service_rx_intr_16450 (rx_bufptr);
		break;

		case RX_ERROR_INTR :
			service_rxerr_intr_16450 (rx_bufptr);
		break;

		default :
		break;
	} /* switch */
} /* end of interrupt_handler_16450 */





/*--------------------------------------------------------------------------*/


#ifdef __COMMENT__


/***************************************************************************/
/*                                                                         */
/*                       Interrupt Routines for Uarts                      */
/*                                                                         */
/***************************************************************************/

/*

This section contains the interrupt routines which are directly vectored to
via the 8259 interrupt controller or the 80186 interrupt controller.
The job of this routine is to call the appropriate interrupt handler passing
in the correct buffer address.
*/






/*-----------------------------------------------------------------------*/





/*
==========================================================================
| i8274_1_interrupt
|
| This interrupt procedure is the first level entry point for an interrupt
| from the 8274 uart in the pipeline system.  The function of the first level
| routine for the 8274 interrupts is to set up the correct buffer pointers
| before calling the generic 8274 interrupt handler.  Because the 8274 uart is
| a dual uart four buffers have to be passed to this handler.  It is then the
| job of the handler to work out by polling the 8274 what the source of the
| interrupt is.  Note that the structure of the system assumes that the 8274
| has been programmed in non-vectored/status affects vector mode.
|
|   Parameters : - none
|
|   Entry via  : - hardware interrupt from 8274 uart
|
==========================================================================
*/

void i8274_1_interrupt : procedure interrupt i8274_1_interrupt_number public;

    call interrupt_handler_8274 ( @rx_buf_0, @tx_buf_0, @rx_buf_1,
                                                            @tx_buf_1 );

    /*
    now reenable the interrupts for the 8274, the 8259 and the 80186
	interrupt controller
    */

	output ( i8274_1_comstat_cha ) = i8274_eoi;
    output ( ocw2_add ) = eoi_ctrl_8259;
	output ( eoi_reg_80186 ) = eoi_int_ctrl_80186;

end i8274_1_interrupt;

_endif





/*-----------------------------------------------------------------------*/




_if i8251A_uart

/*
rx_0_first_level_int_handler_8251A

This routine is entered directly because of a receiver 0  from an 8251A uart
interrupt.  The routine calls the receive_interrupt_handler passing in the
pointer to the correct buffer.

Upon leaving an eoi is issued to the 80186 and 8259 interrupt controllers.

    Parameters : none
    Entry via : rx_0_interrupt interrupt vector

*/

rx_0_first_level_int_handler_8251A : procedure interrupt rx_0_interrupt public;

	call receive_interrupt_handler_8251A ( @rx_buf_0 );

	/* send out the eoi's */

	outword ( eoi_reg_80186 ) = eoi_int_ctrl_80186;

    output ( ocw2_add ) = eoi_ctrl_8259;

end rx_0_first_level_int_handler_8251A;





/*--------------------------------------------------------------------------*/





/*
tx_0_first_level_int_handler_8251A

This routine is entered directly because of an 8251A transmitter 0
interrupt.  The routine calls the transmit_interrupt_handler_8251A passing
in a pointer to the buffer in question.

Upon leaving an eoi is issued to the 80186 and 8259 interrupt controllers.

    Parameters : none
    Entry via : tx_0_interrupt interrupt vector

*/

tx_0_first_level_int_handler_8251A : procedure interrupt tx_0_interrupt
																	public;

	call transmit_interrupt_handler_8251A ( @tx_buf_0 );

	/* send out the eoi to the interrupt controllers */

    outword ( eoi_reg_80186 ) = eoi_int_ctrl_80186;

    output ( ocw2_add ) = eoi_ctrl_8259;

end tx_0_first_level_int_handler_8251A;





/*--------------------------------------------------------------------------*/





/*
rx_1_first_level_int_handler_8251A

This routine is entered directly because of a receiver 1 8251A interrupt.
The routine calls the receive_interrupt_handler passing in the pointer to
the correct buffer.

Upon leaving an eoi is issued to the 80186 and 8259 interrupt controllers.

    Parameters : none
    Entry via : rx_1_interrupt interrupt vector

*/

rx_1_first_level_int_handler_8251A : procedure interrupt rx_1_interrupt
																	public;

	call receive_interrupt_handler_8251A ( @rx_buf_1 );

    /* send out the eoi's */

    outword ( eoi_reg_80186 ) = eoi_int_ctrl_80186;

    output ( ocw2_add ) = eoi_ctrl_8259;

end rx_1_first_level_int_handler_8251A;





/*--------------------------------------------------------------------------*/





/*
tx_1_first_level_int_handler_8251A

This routine is entered directly because of an 8251A transmitter 1
interrupt.  The routine calls the transmit_interrupt_handler_8251A passing
in a pointer to the buffer in question.

Upon leaving an eoi is issued to the 80186 and 8259 interrupt controllers.

    Parameters : none
    Entry via : tx_1_interrupt interrupt vector

*/

tx_1_first_level_int_handler_8251A : procedure interrupt tx_1_interrupt
																	public;

    call transmit_interrupt_handler_8251A ( @tx_buf_1 );

    /* send out the eoi to the interrupt controllers */

    outword ( eoi_reg_80186 ) = eoi_int_ctrl_80186;

	output ( ocw2_add ) = eoi_ctrl_8259;

end tx_1_first_level_int_handler_8251A;

_endif





/*--------------------------------------------------------------------------*/





/*
rx_2_first_level_int_handler_8251A

This routine is entered directly because of a receiver 2 8251A interrupt. 
The routine calls the receive_interrupt_handler passing in the pointer to
the correct buffer.

Upon leaving an eoi is issued to the 80186 and 8259 interrupt controllers.

	Parameters : none
    Entry via : rx_2_interrupt interrupt vector

*/

rx_2_first_level_int_handler_8251A : procedure interrupt rx_2_interrupt
                                                                    public;

	call receive_interrupt_handler_8251A ( @rx_buf_2 );

    /* send out the eoi's */

	outword ( eoi_reg_80186 ) = eoi_int_ctrl_80186;

    output ( ocw2_add ) = eoi_ctrl_8259;

end rx_2_first_level_int_handler_8251A;





/*--------------------------------------------------------------------------*/





/*
tx_2_first_level_int_handler_8251A

This routine is entered directly because of an 8251A transmitter 2
interrupt.  The routine calls the transmit_interrupt_handler_8251A passing
in a pointer to the buffer in question.

Upon leaving an eoi is issued to the 80186 and 8259 interrupt controllers.

	Parameters : none
	Entry via : tx_2_interrupt interrupt vector

*/

tx_2_first_level_int_handler_8251A : procedure interrupt tx_2_interrupt
                                                                    public;

	call transmit_interrupt_handler_8251A ( @tx_buf_2 );

    /* send out the eoi to the interrupt controllers */

    outword ( eoi_reg_80186 ) = eoi_int_ctrl_80186;

    output ( ocw2_add ) = eoi_ctrl_8259;

end tx_2_first_level_int_handler_8251A;





/*--------------------------------------------------------------------------*/





/*
rx_3_first_level_int_handler_8251A

This routine is entered directly because of a receiver 3 8251A interrupt.
The routine calls the receive_interrupt_handler passing in the pointer to
the correct buffer.

Upon leaving an eoi is issued to the 80186 and 8259 interrupt controllers.

    Parameters : none
    Entry via : rx_3_interrupt interrupt vector

*/

rx_3_first_level_int_handler_8251A : procedure interrupt rx_3_interrupt
                                                                    public;

    call receive_interrupt_handler_8251A ( @rx_buf_3 );

    /* send out the eoi's */

    outword ( eoi_reg_80186 ) = eoi_int_ctrl_80186;

    output ( ocw2_add ) = eoi_ctrl_8259;

end rx_3_first_level_int_handler_8251A;





/*--------------------------------------------------------------------------*/





/*
tx_3_first_level_int_handler_8251A

This routine is entered directly because of an 8251A transmitter 3
interrupt.  The routine calls the transmit_interrupt_handler_8251A passing
in a pointer to the buffer in question.

Upon leaving an eoi is issued to the 80186 and 8259 interrupt controllers.

	Parameters : none
    Entry via : tx_3_interrupt interrupt vector

*/

tx_3_first_level_int_handler_8251A : procedure interrupt tx_3_interrupt
                                                                    public;

    call transmit_interrupt_handler_8251A ( @tx_buf_3 );

    /* send out the eoi to the interrupt controllers */

    outword ( eoi_reg_80186 ) = eoi_int_ctrl_80186;

    output ( ocw2_add ) = eoi_ctrl_8259;

end tx_3_first_level_int_handler_8251A;





/*--------------------------------------------------------------------------*/






/****************************************************************************/
/*                                                                          */
/*              SOFTWARE AND HARDWARE INITIALISATION SECTION                */
/*                                                                          */
/****************************************************************************/





/*!
initialise_interrupt_vectors

This routine sets up the addresses of the interrupt vectors in the low part of
memory.  This is required in a system which is being loaded via the monitor
because it destroys the interrupt vectors produced by the compiler.  In a
ROM environment this routine is not needed.

    Parameters : none
    Entry via : software_initialise

*/

initialise_interrupt_vectors : procedure;

    /*------ set up interrupt vectors because of ram load ------*/

    call set_interrupt ( time_slice_int, time_slice );
    call set_interrupt ( kernel_entry, kernel );

_if i8251A_uart
    call set_interrupt ( rx_0_interrupt, rx_0_first_level_int_handler_8251A );
    call set_interrupt ( tx_0_interrupt, tx_0_first_level_int_handler_8251A );
    call set_interrupt ( rx_1_interrupt, rx_1_first_level_int_handler_8251A );
    call set_interrupt ( tx_1_interrupt, tx_1_first_level_int_handler_8251A );
_endif

_if i8274_uart
	call set_interrupt ( i8274_1_interrupt_number, i8274_1_interrupt );
_endif

    call set_interrupt ( rx_2_interrupt, rx_2_first_level_int_handler_8251A );
    call set_interrupt ( tx_2_interrupt, tx_2_first_level_int_handler_8251A );
    call set_interrupt ( rx_3_interrupt, rx_3_first_level_int_handler_8251A );
    call set_interrupt ( tx_3_interrupt, tx_3_first_level_int_handler_8251A );
    
end initialise_interrupt_vectors;





/*--------------------------------------------------------------------------*/










/*!
initialise_transmit_buffers

This routine initialise all the hardware transmit serial buffers in the system.
For details on how this is done refer to the description in the HARDWARE SERIAL
BUFFER SECTION.

	Parameters : none
    Entry via : software_initialise

*/

initialise_transmit_buffers : procedure;

    /*------ transmit buffer init ------*/

_if i8251A_uart

    /*------ transmit buffer 0 ------*/

    tx_buf_0.buf_size = tx_buffer_0_size;
    tx_buf_0.rx_ctrlS_ctrlQ = 0;
	tx_buf_0.cur_tx_int_status = 0;   /* tx int disabled */
    tx_buf_0.user_ctrlS_ctrlQ = 1;    /* channel control Q */
    tx_buf_0.tx_interrupt = 0;        /* tx int disabled */
    tx_buf_0.data_reg_add = data_reg_8251_0;
    tx_buf_0.com_reg_add = com_reg_8251_0;
    tx_buf_0.uart_com_data = com_val_8251_0;
	tx_buf_0.buf_room_index = tx_buf_0_room_avail;
    tx_buf_0.buf_used = 0;
	tx_buf_0.get_ptr = 0;
    tx_buf_0.put_ptr = 0;

    /*------ transmit buffer 1 ------*/

	tx_buf_1.buf_size = tx_buffer_1_size;
    tx_buf_1.rx_ctrlS_ctrlQ = 0;
    tx_buf_1.cur_tx_int_status = 0;   /* tx int disabled */
    tx_buf_1.user_ctrlS_ctrlQ = 1;    /* channel control Q */
    tx_buf_1.tx_interrupt = 0;        /* tx int disabled */
    tx_buf_1.data_reg_add = data_reg_8251_1;
    tx_buf_1.com_reg_add = com_reg_8251_1;
	tx_buf_1.uart_com_data = com_val_8251_1;
	tx_buf_1.buf_room_index = tx_buf_1_room_avail;
	tx_buf_1.buf_used = 0;
	tx_buf_1.get_ptr = 0;
    tx_buf_1.put_ptr = 0;

_endif

_if i8274_uart

    /*------ transmit buffer 0 for chA ------*/

    tx_buf_0.buf_size = tx_buffer_0_size;
    tx_buf_0.rx_ctrlS_ctrlQ = 0;
    tx_buf_0.cur_tx_int_status = 0;   /* tx int disabled */
    tx_buf_0.user_ctrlS_ctrlQ = 1;    /* channel control Q */
    tx_buf_0.tx_interrupt = 0;        /* tx int disabled */
    tx_buf_0.data_reg_add = i8274_1_data_cha;
    tx_buf_0.com_reg_add = i8274_1_comstat_cha;
    tx_buf_0.uart_com_data = NULL;
    tx_buf_0.buf_room_index = tx_buf_0_room_avail;
	tx_buf_0.buf_used = 0;
    tx_buf_0.get_ptr = 0;
    tx_buf_0.put_ptr = 0;

	/*------ transmit buffer 1 for chB ------*/

    tx_buf_1.buf_size = tx_buffer_1_size;
    tx_buf_1.rx_ctrlS_ctrlQ = 0;
    tx_buf_1.cur_tx_int_status = 0;   /* tx int disabled */
    tx_buf_1.user_ctrlS_ctrlQ = 1;    /* channel control Q */
    tx_buf_1.tx_interrupt = 0;        /* tx int disabled */
    tx_buf_1.data_reg_add = i8274_1_data_chb;
	tx_buf_1.com_reg_add = i8274_1_comstat_chb;
    tx_buf_1.uart_com_data = NULL;
	tx_buf_1.buf_room_index = tx_buf_1_room_avail;
    tx_buf_1.buf_used = 0;
	tx_buf_1.get_ptr = 0;
	tx_buf_1.put_ptr = 0;

_endif


    /*------ transmit buffer 2 ------*/

	tx_buf_2.buf_size = tx_buffer_2_size;
	tx_buf_2.rx_ctrlS_ctrlQ = 0;
    tx_buf_2.cur_tx_int_status = 0;   /* tx int disabled */
    tx_buf_2.user_ctrlS_ctrlQ = 1;    /* channel control Q */
    tx_buf_2.tx_interrupt = 0;        /* tx int disabled */
    tx_buf_2.data_reg_add = data_reg_8251_2;
    tx_buf_2.com_reg_add = com_reg_8251_2;
    tx_buf_2.uart_com_data = com_val_8251_2;
    tx_buf_2.buf_room_index = tx_buf_2_room_avail;
    tx_buf_2.buf_used = 0;
    tx_buf_2.get_ptr = 0;
	tx_buf_2.put_ptr = 0;

    /*------ transmit buffer 3 ------*/

    tx_buf_3.buf_size = tx_buffer_3_size;
    tx_buf_3.rx_ctrlS_ctrlQ = 0;
    tx_buf_3.cur_tx_int_status = 0;   /* tx int disabled */
    tx_buf_3.user_ctrlS_ctrlQ = 1;    /* channel control Q */
    tx_buf_3.tx_interrupt = 0;        /* tx int disabled */
    tx_buf_3.data_reg_add = data_reg_8251_3;
    tx_buf_3.com_reg_add = com_reg_8251_3;
    tx_buf_3.uart_com_data = com_val_8251_3;
    tx_buf_3.buf_room_index = tx_buf_3_room_avail;
    tx_buf_3.buf_used = 0;
	tx_buf_3.get_ptr = 0;
    tx_buf_3.put_ptr = 0;

end initialise_transmit_buffers;





/*--------------------------------------------------------------------------*/





/*!
initialise_receive_buffers

This routine initialises the receive buffers which are acted upon by the
receive interrupts and the receive interrupt interface routines.  For details
on how these buffers should be initialised refer to the description in the
HARDWARE SERIAL BUFFER SECTION further up in this module.

    Parameters : none
    Entry via : software initialise

*/

initialise_receive_buffers : procedure;

    /*------ receive buffer init ------*/

_if i8251A_uart

    /*------ receive buffer 0 ------*/

    rx_buf_0.buf_size = rx_buffer_0_size;
	rx_buf_0.buf_low_lim = rx_buf_0_low_lim;
	rx_buf_0.buf_up_lim = rx_buf_0_up_lim;
    rx_buf_0.err_buf_index = 0;
    rx_buf_0.data_reg_add = data_reg_8251_0;
    rx_buf_0.com_reg_add = com_reg_8251_0;
    rx_buf_0.uart_com_data = com_val_8251_0;
    rx_buf_0.ctS_ctQ_flg = 0;
    rx_buf_0.tx_buf_ptr = @tx_buf_0;
	rx_buf_0.buf_sem_index = rx_buf_0_data_avail;
    rx_buf_0.buf_free = rx_buffer_0_size;
    rx_buf_0.get_ptr = 0;
    rx_buf_0.put_ptr = 0;

    /*------ receive buffer 1 ------*/
    
    rx_buf_1.buf_size = rx_buffer_1_size;
	rx_buf_1.buf_low_lim = rx_buf_1_low_lim;
    rx_buf_1.buf_up_lim = rx_buf_1_up_lim;
    rx_buf_1.err_buf_index = 4;
	rx_buf_1.data_reg_add = data_reg_8251_1;
	rx_buf_1.com_reg_add = com_reg_8251_1;
	rx_buf_1.uart_com_data = com_val_8251_1;
    rx_buf_1.ctS_ctQ_flg = 0;
    rx_buf_1.tx_buf_ptr = @tx_buf_1;
    rx_buf_1.buf_sem_index = rx_buf_1_data_avail;
	rx_buf_1.buf_free = rx_buffer_1_size;
    rx_buf_1.get_ptr = 0;
    rx_buf_1.put_ptr = 0;

_endif

_if i8274_uart

	/*------ receive buffer channel A ------*/

	rx_buf_0.buf_size = rx_buffer_0_size;
	rx_buf_0.buf_low_lim = rx_buf_0_low_lim;
    rx_buf_0.buf_up_lim = rx_buf_0_up_lim;
    rx_buf_0.err_buf_index = 0;
    rx_buf_0.data_reg_add = i8274_1_data_cha;
    rx_buf_0.com_reg_add = i8274_1_comstat_cha;
    rx_buf_0.uart_com_data = NULL;
    rx_buf_0.ctS_ctQ_flg = 0;
    rx_buf_0.tx_buf_ptr = @tx_buf_0;
    rx_buf_0.buf_sem_index = rx_buf_0_data_avail;
    rx_buf_0.buf_free = rx_buffer_0_size;
    rx_buf_0.get_ptr = 0;
    rx_buf_0.put_ptr = 0;

    /*------ receive buffer channel B ------*/
    
    rx_buf_1.buf_size = rx_buffer_1_size;
    rx_buf_1.buf_low_lim = rx_buf_1_low_lim;
	rx_buf_1.buf_up_lim = rx_buf_1_up_lim;
	rx_buf_1.err_buf_index = 4;
    rx_buf_1.data_reg_add = i8274_1_data_chb;
    rx_buf_1.com_reg_add = i8274_1_comstat_chb;
    rx_buf_1.uart_com_data = NULL;
    rx_buf_1.ctS_ctQ_flg = 0;
    rx_buf_1.tx_buf_ptr = @tx_buf_1;
    rx_buf_1.buf_sem_index = rx_buf_1_data_avail;
	rx_buf_1.buf_free = rx_buffer_1_size;
    rx_buf_1.get_ptr = 0;
    rx_buf_1.put_ptr = 0;

_endif

    /*------ receive buffer 2 ------*/

    rx_buf_2.buf_size = rx_buffer_2_size;
	rx_buf_2.buf_low_lim = rx_buf_2_low_lim;
	rx_buf_2.buf_up_lim = rx_buf_2_up_lim;
    rx_buf_2.err_buf_index = 8;
    rx_buf_2.data_reg_add = data_reg_8251_2;
	rx_buf_2.com_reg_add = com_reg_8251_2;
    rx_buf_2.uart_com_data = com_val_8251_2;
    rx_buf_2.ctS_ctQ_flg = 0;
    rx_buf_2.tx_buf_ptr = @tx_buf_2;
    rx_buf_2.buf_sem_index = rx_buf_2_data_avail;
    rx_buf_2.buf_free = rx_buffer_2_size;
    rx_buf_2.get_ptr = 0;
    rx_buf_2.put_ptr = 0;

    /*------ receive buffer 3 ------*/
    
    rx_buf_3.buf_size = rx_buffer_3_size;
    rx_buf_3.buf_low_lim = rx_buf_3_low_lim;
    rx_buf_3.buf_up_lim = rx_buf_3_up_lim;
    rx_buf_3.err_buf_index = 12;
	rx_buf_3.data_reg_add = data_reg_8251_3;
    rx_buf_3.com_reg_add = com_reg_8251_3;
    rx_buf_3.uart_com_data = com_val_8251_3;
    rx_buf_3.ctS_ctQ_flg = 0;
    rx_buf_3.tx_buf_ptr = @tx_buf_3;
    rx_buf_3.buf_sem_index = rx_buf_3_data_avail;
    rx_buf_3.buf_free = rx_buffer_3_size;
    rx_buf_3.get_ptr = 0;
    rx_buf_3.put_ptr = 0;

end initialise_receive_buffers;





/*--------------------------------------------------------------------------*/





/*
initialise_8274_uarts

This routine programs all the 8274 uarts in the system.  They are programmed 
in asynchronous mode.

    Parameters : - none

	Entry via  : - hardware_initialise

*/

initialise_8274_uarts : procedure;

    dcl ( i, transmitter_not_rdy ) byte;
    
    /* Channel reset */

    output(i8274_1_comstat_chA) = reset_8274_uart;
    i = i + 1;      /* delay to allow for the reset */
    i = i + 1;
    i = i + 1;
    i = i + 1;
    i = i + 1;
    
    output(i8274_1_comstat_chB) = reset_8274_uart;
    i = i + 1;      /* delay to allow for the reset */
	i = i + 1;
    i = i + 1;
	i = i + 1;
	i = i + 1;

	/* 8274-1  parity, clock division, stop bits - written to WR4 */

	output(i8274_1_comstat_chA) = REG4;
    output(i8274_1_comstat_chA) = mode_data_8274_1;
	output(i8274_1_comstat_chB) = REG4;
    output(i8274_1_comstat_chB) = mode_data_8274_1;


	/* Now set up the interrupt mode for both channels in each UART */

	output(i8274_1_comstat_chA) = REG2;
    output(i8274_1_comstat_chA) = int_mode_8274_1;


    /* Enable the receiver and set the number of data bits to 8 */

	output(i8274_1_comstat_chA) = REG3;
    output(i8274_1_comstat_chA) = receiver_setup_8274_1;
	output(i8274_1_comstat_chB) = REG3;
    output(i8274_1_comstat_chB) = receiver_setup_8274_1;


    /* Enable the transmitter and set the number of data bits to 8 */

	output(i8274_1_comstat_chA) = REG5;
    output(i8274_1_comstat_chA) = transmit_setup_8274_1;
	output(i8274_1_comstat_chB) = REG5;
    output(i8274_1_comstat_chB) = transmit_setup_8274_1;



    /*
	Polled operation initialisation
	*/
    
/*  output ( i8274_1_comstat_chA ) = REG4;
    output ( i8274_1_comstat_chA ) = mode_data_8274_1;
	output ( i8274_1_comstat_chB ) = REG4;
    output ( i8274_1_comstat_chB ) = mode_data_8274_1;
    
	output ( i8274_1_comstat_chA ) = REG3;
    output ( i8274_1_comstat_chA ) = 0c1h;
	output ( i8274_1_comstat_chB ) = REG3;
    output ( i8274_1_comstat_chB ) = 0c1h;
    
	output ( i8274_1_comstat_chA ) = REG5;
    output ( i8274_1_comstat_chA ) = 068h;
	output ( i8274_1_comstat_chA ) = REG5;
    output ( i8274_1_comstat_chA ) = 068h;
*/


    /* Reset the error register and the external status register to get
	rid of any funnies caused by power up */

    output(i8274_1_comstat_cha) = reset_error_reg_8274;
    output(i8274_1_comstat_cha) = reset_ext_status_8274;
	output(i8274_1_comstat_cha) = reset_ext_status_8274;
	output(i8274_1_comstat_chb) = reset_error_reg_8274;
    output(i8274_1_comstat_chb) = reset_ext_status_8274;
    output(i8274_1_comstat_chb) = reset_ext_status_8274;


    /* Final enabling of the interrupts */

	output(i8274_1_comstat_chA) = REG1;
	output(i8274_1_comstat_chA) = enable_int_8274_chA;
	output(i8274_1_comstat_chB) = REG1;
	output(i8274_1_comstat_chB) = enable_int_8274_chB;



    /*
    now enter the infinite test polling loop
    */
    
/*  do while 1;
        transmitter_not_rdy = true;
        do while transmitter_not_rdy;
            if ( input ( i8274_1_comstat_chA ) and 04h ) > 0 then
                transmitter_not_rdy = false;
        end;
        
        output ( i8274_1_data_chA ) = 'A';
    end;
*/


  
end initialise_8274_uarts;





/*-------------------------------------------------------------------------*/





/*
initialise_8251A_uarts

This procedure programs all 8251A uarts in the system, assuming that tha
program may have been entered without a reset being issued.  This implies
that a worst case software reset must be issued for the 8251A uarts.  See
the control word declarations for a complete description of the current
programming of the uarts.

    Parameters : none
	Entry via : hardware_initialise

*/

initialise_8251A_uarts : procedure;

    dcl ( udata, i ) byte;


    /* assume system may not be hardware reset - issue three consecutive nulls
	with a delay between them */

_if i8251A_uart

    /* for uart 0 first */

    output ( com_reg_8251_0 ) = 0;
    i = i + 1;
    i = i + 1;
    i = i + 1;
    i = i + 1;
    i = i + 1;
    output ( com_reg_8251_0 ) = 0;
    i = i + 1;
	i = i + 1;
    i = i + 1;
	i = i + 1;
	i = i + 1;
    output ( com_reg_8251_0 ) = 0;

    /* now for uart 1 */

	output ( com_reg_8251_1 ) = 0;
    i = i + 1;
    i = i + 1;
    i = i + 1;
	i = i + 1;
	i = i + 1;
    output ( com_reg_8251_1 ) = 0;
    i = i + 1;
    i = i + 1;
	i = i + 1;
    i = i + 1;
    i = i + 1;
    output ( com_reg_8251_1 ) = 0;

_endif

    /* now for uart 2 */

    output ( com_reg_8251_2 ) = 0;
    i = i + 1;
    i = i + 1;
    i = i + 1;
    i = i + 1;
    i = i + 1;
    output ( com_reg_8251_2 ) = 0;
    i = i + 1;
    i = i + 1;
	i = i + 1;
    i = i + 1;
	i = i + 1;
	output ( com_reg_8251_2 ) = 0;

    /* now for uart 3 */

    output ( com_reg_8251_3 ) = 0;
    i = i + 1;
    i = i + 1;
	i = i + 1;
    i = i + 1;
    i = i + 1;
    output ( com_reg_8251_3 ) = 0;
    i = i + 1;
    i = i + 1;
    i = i + 1;
    i = i + 1;
	i = i + 1;
    output ( com_reg_8251_3 ) = 0;
    i = i + 1;
	i = i + 1;
	i = i + 1;
	i = i + 1;
    i = i + 1;

    /* now issue the software reset to all the uarts */

_if i8251A_uart

    output ( com_reg_8251_0 ) = software_reset_8251;
	output ( com_reg_8251_1 ) = software_reset_8251;

_endif

	output ( com_reg_8251_2 ) = software_reset_8251;
    output ( com_reg_8251_3 ) = software_reset_8251;


    /* wait for a time before reprogramming */

    i = 0;
    do while i < 30;
        i = i + 1;
    end;
    

    /* now program all the uarts */

    /* firstly uart 0 */
_if i8251A_uart

    output ( com_reg_8251_0 ) = mode_val_8251_0;

    /* insert delay before command */

	i = i + 1;
    i = i + 1;
    i = i + 1;
    i = i + 1;
    i = i + 1;
    
    output ( com_reg_8251_0 ) = com_val_8251_0 or clear_err_8251;


    /* uart 1 */

	output ( com_reg_8251_1 ) = mode_val_8251_1;

    /* insert delay before command */

    i = i + 1;
	i = i + 1;
	i = i + 1;
    i = i + 1;
    i = i + 1;

    output ( com_reg_8251_1 ) = com_val_8251_1 or clear_err_8251;

_endif


    /* uart 2 */

    output ( com_reg_8251_2 ) = mode_val_8251_2;

    /* insert delay before command */

    i = i + 1;
    i = i + 1;
    i = i + 1;
	i = i + 1;
    i = i + 1;
    
    output ( com_reg_8251_2 ) = com_val_8251_2 or clear_err_8251;


    /* uart 3 */

    output ( com_reg_8251_3 ) = mode_val_8251_3;

    /* insert delay before command */

    i = i + 1;
    i = i + 1;
	i = i + 1;
    i = i + 1;
	i = i + 1;

    output ( com_reg_8251_3 ) = com_val_8251_3 or clear_err_8251;

end initialise_8251A_uarts;





/*-------------------------------------------------------------------------*/





/*
initialise_8259

This routine initialises the Intel 8259 interrupt controller present on the
pciv processor board.  The controller is set up in fully nested interrupt
mode with level sensitive interrupts.  It does not issue auto eoi.

    Parameters : none
	Entry via : hardware_initialise

*/

initialise_8259 : procedure;

	/* write out the initial sequence of 3 control words - see the declarations
	section for a description of the mode into which the controller is being
	programmed */

	output ( icw1_add ) = icw1_val;

	output ( icw2_add ) = icw2_val;

	output ( icw4_add ) = icw4_val;

end initialise_8259;


#endif


/*-------------------------------------------------------------------------*/





/*
=============================================================================
|
| alloc_intr_buffers
|
| The purpose of this function is to allocate the interrupt buffers for use
| by the particular uart. After the allocation is carried out the routine
| then fills in the data structures with the appropriate data which is passed
| into the routine.
|
| Parameters:	-	uart_type - can be I8274, I8251A, NS16450
|					command_reg - address of the uart command register
|					data_reg - address of the uart data register
|					rx_int_buf_size - size of the receive interrupt buffer
|					tx_int_buf_size - size of the transmit interrupt buffer
|					buf_low_lim - lower limit to be reached before a control
|					Q is sent to a remote.
|					buf_upper_lim - the upper limit on the receive buffer
|					before a control S is issued.
|					rx_int_buffer - pointer to the allocated receive interrupt
|					buffer.
|					tx_int_buffer - pointer to the transmit interrupt buffer.
|					srx_int_buffer - pointer to the secondary interrupt
|					buffer.
|					stx_int_buffer - pointer to the secondary transmit
|					interrupt buffer.
|                   xonxoff - enable/disable (=0) software control
| Returns:		-	TRUE if the allocation process is successful, else it
|					returns a FALSE.
|
============================================================================
*/

int alloc_intr_buffers (unsigned char uart_type, unsigned int command_reg,
				unsigned int data_reg, unsigned char command_data,
				unsigned char tx_enable_disable_bit,
				unsigned char intr_ctrl_reg_offset,
				unsigned int rx_int_buf_size, unsigned int tx_int_buf_size,
				unsigned int buf_low_lim, unsigned int buf_upper_lim,
				rx_int_buffer** rx_int_bufptr,
				tx_int_buffer** tx_int_bufptr, rx_int_buffer** srx_int_bufptr,
				tx_int_buffer** stx_int_bufptr,
				unsigned char xonxoff) {

	unsigned int temp;

	/* firstly allocate the memory for the interrupt buffers */

	switch (uart_type) {
		case I8251A :
		case NS16450 :
			if ((*rx_int_bufptr =
				(rx_int_buffer*)(umalloc (sizeof (rx_int_buffer)))) == NULL) {
				/* there has been a problem allocating memory */
				return (FALSE);
			} /* if */

			if ((*tx_int_bufptr =
				(tx_int_buffer*)(umalloc (sizeof (tx_int_buffer)))) == NULL) {
				/* there has been a problem allocating memory */
				return (FALSE);
			} /* if */
		break;

		case I8274 :
			if ((*rx_int_bufptr =
				(rx_int_buffer*)(umalloc (sizeof (rx_int_buffer)))) == NULL) {
				/* there has been a problem allocating memory */
				return (FALSE);
			} /* if */

			if ((*tx_int_bufptr =
				(tx_int_buffer*)(umalloc (sizeof (tx_int_buffer)))) == NULL) {
				/* there has been a problem allocating memory */
				return (FALSE);
			} /* if */

			if ((*srx_int_bufptr =
				(rx_int_buffer*)(umalloc (sizeof (rx_int_buffer)))) == NULL) {
				/* there has been a problem allocating memory */
				return (FALSE);
			} /* if */

			if ((*stx_int_bufptr =
				(tx_int_buffer*)(umalloc (sizeof (tx_int_buffer)))) == NULL) {
				/* there has been a problem allocating memory */
				return (FALSE);
			} /* if */
		break;
	} /* switch */

	/* To get to this point then all the memory must have been allocated
	correctly. Now create the semaphores that will be used by the buffers.
	*/

	switch (uart_type) {
		case I8251A :
		case NS16450 :
			if ((temp = create_semaphore ()) == 0xffff) {
				/* has been a probelm creating the semaphore */
				return (FALSE);
			} /* if */

			(*rx_int_bufptr)->buf_sem_index = temp;
			init_semaphore (temp, 0, 1);

			if ((temp = create_semaphore ()) == 0xffff) {
				/* has been a probelm creating the semaphore */
				return (FALSE);
			} /* if */

			(*tx_int_bufptr)->buf_room_index = temp;
			init_semaphore (temp, tx_int_buf_size, 1);

		break;

		case I8274 :
			if ((temp = create_semaphore ()) == 0xffff) {
				/* has been a probelm creating the semaphore */
				return (FALSE);
			} /* if */

			(*rx_int_bufptr)->buf_sem_index = temp;
			init_semaphore (temp, 0, 1);

			if ((temp = create_semaphore ()) == 0xffff) {
				/* has been a probelm creating the semaphore */
				return (FALSE);
			} /* if */

			(*tx_int_bufptr)->buf_room_index = temp;
			init_semaphore (temp, tx_int_buf_size, 1);

			if ((temp = create_semaphore ()) == 0xffff) {
				/* has been a problem creating the semaphore */
				return (FALSE);
			} /* if */

			(*srx_int_bufptr)->buf_sem_index = temp;
			init_semaphore (temp, 0, 1);

			if ((temp = create_semaphore ()) == 0xffff) {
				/* has been a probelm creating the semaphore */
				return (FALSE);
			} /* if */

			(*stx_int_bufptr)->buf_room_index = temp;
			init_semaphore (temp, tx_int_buf_size, 1);

		break;
	} /* switch */

	/* at this stage have also created the semaphores. Now start to fill out
	the rest of the interrupt data structures.
	*/

	switch (uart_type) {
		case I8251A :
		case NS16450 :
			(*rx_int_bufptr)->buf_size = rx_int_buf_size;
			(*rx_int_bufptr)->buf_low_lim = buf_low_lim;
			(*rx_int_bufptr)->buf_up_lim = buf_upper_lim;
			(*rx_int_bufptr)->data_reg_add = data_reg;
			(*rx_int_bufptr)->com_reg_add = command_reg;
			(*rx_int_bufptr)->uart_com_data = command_data;
			(*rx_int_bufptr)->uart_type = uart_type;
			(*rx_int_bufptr)->uart_number = 1;
			(*rx_int_bufptr)->tx_enable_disable_bit = tx_enable_disable_bit;
			(*rx_int_bufptr)->intr_ctrl_reg_offset = intr_ctrl_reg_offset;
			(*rx_int_bufptr)->ctS_ctQ_flg = FALSE;
			(*rx_int_bufptr)->tx_buf_ptr = *tx_int_bufptr;
			(*rx_int_bufptr)->connected_task_ptr = NULL;
			(*rx_int_bufptr)->buf_free = rx_int_buf_size;
			(*rx_int_bufptr)->get_ptr = 0;
			(*rx_int_bufptr)->put_ptr = 0;
			(*rx_int_bufptr)->xonxoff = xonxoff;

			/* now allocate the storage for the circular character buffer
			itself.
			*/
			if (((*rx_int_bufptr)->buf_ptr
							= (char*)(ucalloc (rx_int_buf_size,
											sizeof (char)))) == NULL) {
				/* has been a problem allocating memory */
				return (FALSE);
			} /* if */

			(*tx_int_bufptr)->buf_size = tx_int_buf_size;
			(*tx_int_bufptr)->rx_ctrlS_ctrlQ = 0;	/* not outputting anything */
			(*tx_int_bufptr)->cur_tx_int_status = DISABLED;
			(*tx_int_bufptr)->user_ctrlS_ctrlQ = USER_CTRL_Q;
			(*tx_int_bufptr)->tx_interrupt = DISABLED;
			(*tx_int_bufptr)->data_reg_add = data_reg;
			(*tx_int_bufptr)->com_reg_add = command_reg;
			(*tx_int_bufptr)->uart_com_data = command_data;
			(*tx_int_bufptr)->tx_enable_disable_bit = tx_enable_disable_bit;
			(*tx_int_bufptr)->intr_ctrl_reg_offset = intr_ctrl_reg_offset;
			(*tx_int_bufptr)->uart_type = uart_type;
			(*tx_int_bufptr)->uart_number = 1;
			(*tx_int_bufptr)->buf_used = 0;
			(*tx_int_bufptr)->get_ptr = 0;
			(*tx_int_bufptr)->put_ptr = 0;
			(*tx_int_bufptr)->xonxoff = xonxoff;

			/* now allocate the memory required for the character buffer. */
			if (((*tx_int_bufptr)->buf_ptr
							= (char*)(ucalloc (tx_int_buf_size,
										sizeof (char)))) == NULL) {
				/* has been a problem allocating memory */
				return (FALSE);
			} /* if */
		break;

		case I8274 :
			;
		break;
	} /* switch */

	return (TRUE);
} /* end of alloc_intr_buffers */





/*-------------------------------------------------------------------------*/





/*
============================================================================
|
| prog_16450_uart
|
| This function programs the hardware of the National Semiconductor NS16450
| uasrt (the standard uart used in the IBM-PC). The interrupts of the
| uart are set up so that the transmitter is disabled and the receiver
| is enabled.
|
| Parameters:   -   baud_rate - the baud rate that the uart is to be
|                   programmed to.
|               -   stop_bits - the number of stop bits to program the
|                   uart to. The legal values are 1, 1.5 and 2.
|               -   parity - legal values EVEN_PARITY, ODD_PARITY, NO_PARITY.
|               -   command_reg - address of the command register for the
|                   uart.
|               -   data_reg - the address of the data register for the uart.
|
| Returns:      -   the data loaded into the command register to set the
|                   uart up with the transmit interrupts disabled.
|
============================================================================
*/

unsigned char prog_16450_uart (unsigned int baud_rate, float stop_bits,
			char parity, char char_lgth, unsigned int command_reg,
												unsigned int data_reg) {

	unsigned int divisor_value;
	unsigned char command_data;
	unsigned char line_ctrl_value = 0;
	unsigned int i;
	unsigned int waste_time = 0;

	/* Clear the uart line control register */
	outportb (command_reg + LINE_CTRL_REG, 0x00);

	/* firstly work out the divisor required to generate the baud rate */
	divisor_value = (unsigned int)(NS16450_IN_FREQ / (16 * (long)baud_rate));

	/* now program the line control register and set the DLAB bit so that the
	baud rate can be programmed in the next phase of the set up. Firstly
	work out what the word has to be to send out to the register.
	*/

	switch (char_lgth) {
		case 5 :
			line_ctrl_value = L_5_BITS;
		break;

		case 6 :
			line_ctrl_value = L_6_BITS;
		break;

		case 7 :
			line_ctrl_value = L_7_BITS;
		break;

		case 8 :
			line_ctrl_value = L_8_BITS;
		break;
	} /* switch */

	/* The number of stop bits in the NS16450 is dependent on the number
	of bits in a character - this is mainly to do with setting 1.5 stop
	bits. I have left it to the user to figure this out. If the user
	nominates 1.5 or 2 stop bits I program that responsible bit in the uart
	the same and do not check the character bits.
	*/
	if ((stop_bits == 1.5) || (stop_bits == 2)) {
		line_ctrl_value |= 0x04;
	} /* if */

	/* now for the parity */
	switch (parity) {
		case NO_PARITY :
		break;

		case EVEN_PARITY :
			line_ctrl_value |= 0x18;
		break;

		case ODD_PARITY :
			line_ctrl_value |= 0x08;
		break;
	} /* switch */


	outportb (command_reg + LINE_CTRL_REG, line_ctrl_value & 0x7f);

	/* can now program the baud rate */
	outportb (command_reg + LINE_CTRL_REG, (inportb(command_reg +
									LINE_CTRL_REG) | 0x80));
	outportb (command_reg + LS_DIVISOR_LATCH, (unsigned char)divisor_value);
	outportb (command_reg + MS_DIVISOR_LATCH,
							(unsigned char)((divisor_value & 0xff00) >> 8));
	for (i = 0; i < 32000; i++) {
		waste_time++;
		waste_time = waste_time;
	} /* for */

	/* Now reset the DLAB bit so that the normal registers can again be
	accessed.
	*/
	outportb (command_reg + LINE_CTRL_REG, (inportb(command_reg +
									LINE_CTRL_REG) & 0x7f));

	/* now set up the modem control bits */
	outportb (command_reg + MODEM_CTRL_REG, 0x0b);

	/*<<<<<<< The following line has been added for debugging purposes.
	It causes the uart to be placed into loop back mode.
	>>>>>>>>*/

	/* now clear any pending interrupts */
	inportb (data_reg);
	inportb (data_reg + LINE_STATUS_REG);

	/* now set the initial interrupt status so that transmitter is disabled
	and the receiver is enabled and the receive line status interrupt is
	enabled. This register becomes the command_data value.
	*/

	outportb (command_reg + INTR_ENABLE_REG, 0x05);
	command_data = 0x05;

	return (command_data);
} /* end of prog_16450_uart */





/*--------------------------------------------------------------------------*/





/*
============================================================================
|
| create_serial_channel
|
| This function creates a serial channel which runs with UNOS. This creation
| process is fairly complete and includes the programming of the hardware
| of the UART, creation of the interrupt buffers and any other data
| structures required and finally the creation and initialisation of the
| transmit and receive tasks that control the serial channel.
|
| The serial hardware currently supported is the Intel 8251A, the Intel
| 8274 and the National Semiconductor 8250A/16450 (ie the standard IBM-PC
| uart).
|
| Limitations of the routine are that if the serial hardware requires an
| external timer to generate the baud rate then this has to be set up
| external to this module. In addition programming of the 8259 interrupt
| controllers has to be done externally. Other restrictions relate to the
| modes that the uarts can be set up in - basically the driver only
| supports async communication with software handshaking. In the case of
| the 8274 dual uart both uarts are programmed identically.
|
| Parameters:   -   uart_type - the type of uart being used. The allowed
|                   types are i8274, i8251A, NS16450.
|               -   baud_rate - the baud rate of the uart. This parameter
|                   is mainly of use with uarts that have internal baud
|                   rate generators. The allowed baud rates depend on the
|                   uart.
|               -   stop_bits - the number of stop bits (1, 1.5, 2)
|				-	char_lgth - character length - allowed values 5, 6, 7, 8
|					bits.
|               -   parity - ODD_PARITY, EVEN_PARITY, NO_PARITY
|               -   command_reg - the I/O address of the command register.
|               -   data_reg - the I/O address of the data register.
|               -   divisor_ratio - the internal division ration required
|                   for uarts such as the 8251.
|               -   alarm_num_base - the alarm number base address for
|                   reporting errors to the alarm handler.
|               -   rx_intr_num - the interrupt number to be used by the
|                   uart receiver.
|               -   tx_intr_num - the interrupt number to be used by the
|                   uart transmitter.
|				-	tx_enable_disable_bit - the bit which is used to enable
|					and disable the transmitter interrupts.
|				-	intr_ctrl_reg_offset - the offset of the interrupt
|					control register from the base command register.
|               -   rx_int_buf_size - the size of the interrupt routine
|                   circular buffer in bytes.
|               -   tx_int_buf_size - the size of the transmit interrupt
|                   buffer in bytes.
|               -   buf_low_lim - the lower limit on the receive buffer when
|                   a control Q is issued to recommence transmission of
|                   data from an external source.
|               -   buf_upper_lim the upper limit on the amount of data
|                   in the receive buffer before a control S is issued to
|                   the external serial source.
|               -   mbx_q_size - the size of the queue of messages for the
|                   transmit task mail box.
|               -   mbx_mess_size - the maximum size of each message that can
|                   be placed in the transmit task mail box.
|               -   num_8259s - the number of 8259s connected to the uart
|                   concerned. The maximum number is 2 (ie a cascaded system
|                   of 8259s), the minimum is 0.
|               -   addr_8259_mast - the address of the master 8259.
|               -   addr_8259_slave - the address of the slave 8259.
|               -   ch1_rx_name_ptr - a pointer to the name of the receive
|                   task for the receive channel on uart 1 of a dual uart.
|               -   ch1_tx_name_ptr - pointer to the name of the transmit task
|                   for the transmit channel on uart 1 of a dual uart.
|               -   ch2_rx_name_ptr - a pointer to the name of the receive
|                   task for the receive channel on uart 2 of a dual uart.
|               -   ch2_tx_name_ptr - pointer to the name of the transmit task
|                   for the transmit channel on uart 2 of a dual uart.
|               -   rx_task_pri - the priority of the receive task.
|               -   tx_task_pri - the priority of the transmit task.
|               -   xonxoff - enable (=1) /disbale (=0) software control
| Returns:      -   TRUE if the channel creation has been successful.
|                   FALSE if the channel creation has not been successful.
|
============================================================================
*/

int create_serial_channel (char uart_type, unsigned int baud_rate,
			float stop_bits, char char_lgth, char parity,
			unsigned int command_reg,
			unsigned int data_reg, unsigned char division_ratio,
			unsigned char rx_intr_num,
			unsigned char tx_intr_num, unsigned char tx_enable_disable_bit,
			unsigned char intr_ctrl_reg_offset,
			unsigned int rx_int_buf_size,
			unsigned int tx_int_buf_size, unsigned int buf_low_lim,
			unsigned int buf_upper_lim, unsigned int mbx_q_size,
			unsigned int mbx_mess_size, char num_8259s,
			unsigned int addr_8259_mast, unsigned int addr_8259_slave,
			char* ch1_rx_name_ptr, char* ch1_tx_name_ptr,
			char* ch2_rx_name_ptr, char* ch2_tx_name,
			unsigned char rx_task_pri, unsigned char tx_task_pri,
			unsigned char xonxoff ) {


	unsigned char command_data;
	rx_int_buffer *rx_int_bufptr, *srx_int_bufptr;
	tx_int_buffer *tx_int_bufptr, *stx_int_bufptr;
	local_var_type* local_var_ptr;

	division_ratio = division_ratio;

	/* the first step in this routine is to program up the hardware of the
	particular uart. The hardware programming of each uart is carried out
	individually for each type since there is very little commonality
	between them.
	*/
	switch (uart_type) {
		case I8251A :
			/* prog_8251_uart */
			break;

		case I8274 :
			/* prog_8274_uart */
			break;

		case NS16450 :
			command_data = prog_16450_uart (baud_rate, stop_bits, parity,
								char_lgth, command_reg, data_reg);
			break;
	} /* switch */

	/* now allocate the memory required for the interrupt buffer data
	structures. For the 8251 and 16450 two buffers are allocated - one
	for the receive interrupt and the other for the transmit. In the
	case of the 8274 two receive interrupt structures are created and
	two transmit structures. After the memory is allocated this
	routine then fills out the buffer with the appropriate information.
	If the operation is successful then a TRUE is returned, else a FALSE
	is returned. The semaphores are also set up in this routine, and the
	numbers are put into the appropriate sections of the data structure.
	*/

	if ((alloc_intr_buffers (uart_type, command_reg, data_reg, command_data,
			tx_enable_disable_bit, intr_ctrl_reg_offset,
			rx_int_buf_size, tx_int_buf_size,
			buf_low_lim, buf_upper_lim,
			&rx_int_bufptr, &tx_int_bufptr,
			&srx_int_bufptr, &stx_int_bufptr, xonxoff )) == FALSE) {
		/* There has been a problem allocating the memory for the interrupt
		buffers, therefore abort the rest of the serial channel set up and
		return a FALSE.
		*/
		return (FALSE);
	} /* if */


	/* At this point the uart's hardware has been programmed and the
	interrupt buffers have been allocated and initialised. Now set up the
	interrupt routine/s and vector/s for the uart. Again this has to be
	handled more or less individually since the behaviour of the 8274 in
	relation to interrupts is different to the i8251 and ns16450.
	*/
	if (setup_interrupt_routine (uart_type, num_8259s, addr_8259_mast,
			addr_8259_slave, rx_intr_num, tx_intr_num, rx_int_bufptr,
			tx_int_bufptr, srx_int_bufptr, stx_int_bufptr) == FALSE) {
		/* Has been a problem allocation the interrupt routine so return
		with a FALSE value.
		*/
		return (FALSE);
	} /* if */

	/* now create the tasks for the channel. In the case of an 8274 uart
	4 tasks are created, two for receiving data and two for transmitting
	data. For the 8251 and the 8274 only two tasks are created - one for
	receiving and one for transmitting. One of the tricky aspects of the
	task creation is the mechanism of connecting the interrupt buffers to
	the tasks. This is difficult because UNOS does not allow parameters
	to be passed to the task itself when it is created. However UNOS does
	allow an initialisation routine to be executed when a task is created
	and the trick to make the task know the interrupt buffers that it has
	to use is to call the task itself as the initialisation routine.
	When the task runs it then picks up the values of the interrupt buffers
	from the global variables that at this point of the program already
	contain the interrupt buffer pointers. These values are then stored
	in local static variables within the task. Obviously ones does wish the
	task proper to begin execution during this initialisation process,
	therefore the first invocation of the task is treated as a special
	initialisation call and the task does not begin to execute.

	The 8274 is a more difficult case because we wish to associate a
	separate transmit task with each channel in the uart. Therefore
	tasks wishing to transmit data would have a distinct mail box for
	each of the channels. Similarly for received data we wish to be able
	to associate on or more tasks mail boxes with each serial channel in
	the 8274.
	*/

	switch (uart_type) {
		case I8274 :
			ch2_rx_name_ptr = ch2_rx_name_ptr;
			ch2_tx_name = ch2_tx_name;
		break;

		case I8251A :
		case NS16450 :

			/* Now allocate the memory for the local variables to be passed
			to the tasks.
			*/
			if ((local_var_ptr =
					(local_var_type*)umalloc (sizeof (local_var_type)))
																== NULL) {
				return (FALSE);
			} /* if */

			local_var_ptr->mbx_mess_size = mbx_mess_size;
			local_var_ptr->rx_int_bufptr = rx_int_bufptr;
			local_var_ptr->tx_int_bufptr = tx_int_bufptr;

			cprintf ("Creating Serial Task Receive...\n\r");
			/* firstly create the receive task */
			if (create_task (ch1_rx_name_ptr, rx_task_pri, 0, TASK_RUNNABLE,
					PRIORITY_Q_TYPE, 0, 0x300, mbx_q_size, mbx_mess_size,
					NULL, rx_buf_interface_task,
										(void*)local_var_ptr) == FALSE) {
				/* has been a problem creating the task so get out with
				a FALSE.
				*/
				return (FALSE);
			} /* if */

			cprintf("Creating Serial Task Transmit...\n\r");
			/* now create the transmit task */
			if (create_task (ch1_tx_name_ptr, tx_task_pri, 0, TASK_RUNNABLE,
					PRIORITY_Q_TYPE, 0, 0x300, mbx_q_size, mbx_mess_size,
					NULL, tx_buf_interface_task,
										(void*)local_var_ptr) == FALSE ) {
				/* there has been a problem creating the task so get out with
				a FALSE.
				*/
				return (FALSE);
			} /* if */
		break;
	} /* switch */

	return (TRUE);
} /* end of create_serial_channel */