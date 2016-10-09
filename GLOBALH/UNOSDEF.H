
/***************************************************************************

			Buffers, mailboxes and other UNOS-related definitions
			This file must always be present. It is used by UNOS.

****************************************************************************
*/
#ifndef __UNOSDEF_H
#define __UNOSDEF_H

/*========================================================================*/
/* Mailbox Definitions */

						/* ***** WARNING **** */

/* BE VERY CAREFUL WHEN SETTING MAILBOX SIZES. REMEMBER THAT IF MESSAGES
ARE BEING PASSED THRU SEVERAL TASKS e.g. parser->protocol->tx->interr

THAT THE MBXS AND INTERRUPT FIFOS WILL BE EXPECTED TO HANDLE THE LARGEST
EXPECTED BUFFER TRANSFER.
*/

			/* Task mailboxes */
#define SLP_MESS_Q_SIZE             100
#define SLP_MESS_SIZE				2

#define	RX_0_PROTOCOL_MESS_Q_SIZE	100
#define	RX_0_PROTOCOL_MESS_SIZE		50

#define	RX_1_PROTOCOL_MESS_Q_SIZE	100
#define	RX_1_PROTOCOL_MESS_SIZE		50

#define	RX_2_PROTOCOL_MESS_Q_SIZE	100
#define	RX_2_PROTOCOL_MESS_SIZE		50

#define	RX_3_PROTOCOL_MESS_Q_SIZE	100
#define	RX_3_PROTOCOL_MESS_SIZE		50

#define	ORBIT_TRACK_MESS_Q_SIZE		2
#define	ORBIT_TRACK_MESS_SIZE		2

#define	BEACON_SIMULATOR_TASK_MESS_Q_SIZE	2
#define	BEACON_SIMULATOR_TASK_MESS_SIZE		2

#define	DECODER_MESS_Q_SIZE		100
#define	DECODER_MESS_SIZE		10

#define	CMU_MESS_Q_SIZE				20    /* Not a task - used for PARSER */
#define	CMU_MESS_SIZE				350   /*  task defineitions.          */

#define LCMU_PARSER_MESS_Q_SIZE     CMU_MESS_Q_SIZE
#define LCMU_PARSER_MESS_SIZE       CMU_MESS_SIZE

#define RCMU_PARSER_MESS_Q_SIZE     CMU_MESS_Q_SIZE
#define RCMU_PARSER_MESS_SIZE       CMU_MESS_SIZE

#define SEQUENCER_TASK_Q_SIZE		2             
#define SEQUENCER_MESS_SIZE			2

#define SCREEN_TASK_Q_SIZE		2
#define SCREEN_TASK_MESS_SIZE			2

#define	NULL_MESS_Q_SIZE		2
#define	NULL_MESS_SIZE			2

#define MESS_Q_SIZE_KEYBD_TASK 1
#define MESS_SIZE_KEYBD_TASK   1

#define KEYBOARD_PARSER_MESS_Q_SIZE 2
#define KEYBOARD_PARSER_MESS_SIZE	2


/*========================================================================*/
/* Project Size */


	/* Define the maximum number of semaphores required.
	Note that the semaphore numbers start from 0. The system requires
	a minimum of num_tasks * 2 semaphores to support the mailbox
	mechanism. The additional semaphores that the user creates are
	stored as a pool of semaphores which can be obtained by tasks that
	require semaphores.
	*/
#define MAX_NUM_SEMAPHORES 60

	// size of the memory pool
#define MEMORY_POOL_SIZE 0x40000L   /* was 30000L   -sto 30/8/95 */

	// interrupt number for kernel entry
#define KERNEL_ENTRY 96

	// define the maximum number of timers in the system
#define MAX_NUM_TIMERS 50

#define ENTER_KERNEL_VALUE 2


/*========================================================================*/
#endif /* __UNOSDEF_H */