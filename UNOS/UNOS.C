#include <stdio.h>
#include <dos.h>
#include <conio.h>
#include <math.h>
#include <alloc.h>
#include "unos.h"
#include "general.h"
#include "fpx.h"
#include "unosasm.h"

/************************************************************************/
/*                                                                      */
/*                                                                      */
/*              University of Newcastle Operating System                */
/*                                                                      */
/*                              ( UNOS )                                */
/*                                                                      */
/*                             Version 1.5                              */
/*                                                                      */
/*                            KERNEL MODULE                             */
/*                                                                      */
/*                                 by                                   */
/*                                                                      */
/*                             Robert Betz                              */
/*             Department of Electrical and Computer Engineering        */
/*                        University of Newcastle                       */
/*                                                                      */
/*                          ( Copyright 1989 )                          */
/*                                                                      */
/*                                                                      */
/************************************************************************/





/*==========================================================================*/






/**********************************************************************

	HISTORY

This version of UNOS was originally created by carrying out a hand
translation of the PL/M version written for the Pipeline Project.  A
functionally identical C version was tested.  Some changes had to be made to
the task creation to account for the new language.

The first enhancement carried out was the rewriting of the task creation
section to make it considerably simpler - i.e. just call the create_task
routine passing in the correct parameters.  In addition the user modified
section of UNOS were put into a seperate module so that the operation system
itself does not normally have to be recompiled.

After this initial enhancement then the development of a Version 1.5 of UNOS
began in earnest.  The Version 1.5 features added are listed below :

    - greatly improved task creation procedure - dynamic allocation of most 
	  operating system data structures.
	- operating system need not be recompiled to include changes to task
	  number, priority number etc.
    - seperation of the time slice period from the clock period.
    - made the operating system pre-emptive
    - allow an arbitrary number of priorities
	- include general purpose timers which can be used by user tasks and
	  operating system functions.
	- implementation of dynamic priorities for tasks so that semaphore
      blockage will be fair.
    - addition of mailbox intertask communications.
    - dynamic memory management module.


16/12/93 - fixed the remaining memory bug in the ufree routine.





***********************************************************************/



/*
****************************************************************************
*
*	FUDGE :- IN ORDER TO STOP THE SEQUENCER OVERRUN PROBLEM TWO PROCEDURES
*	HAVE BEEN ADDED TO UNOS - NAMELY disable_task_switch AND
* 	enable_task_switch.  A FLAG HAS BEEN INTRODUCED - task_switch_flag. THE
*	ABOVE MENTIONED PROCEDURES MANIPULATE THIS FLAG.  THIS FLAG IS THEN
*	USED TO PREVENT TASK SWITCHING IN ALL CASES EXCEPT FOR A "wait".
*
*	THE ADDED STATEMENTS CAN BE FOUND BY SEARCHING FOR task_switch_flag.  IN
*	ADDITION THERE ARE TWO PROTOTYPES IN UNOS.H FOR THE ABOVE TWO FUNCTIONS.
*
****************************************************************************
*/





/*

DESCRIPTION

For a complete description of the UNOS including how to use the  operating
system refer to the UNOS.DOC ( Microsoft Word5 format file ) or  the
UNOS.ASC ( ascii format ) files.  The information available in these files
is also in Postscript format and can be printed out to form a manual. These
files are not included in the program as comments since they are quite long
and only serve to slow down the compilation times.  The description is
included in two seperate file formats so that they can be viewed and edited
if necessary with a conventional editor ( using the ascii file ) or the
manual generation file can be edited to form a new manual ( with the Word5
file ).

*/


/*

OTHER REQUIREMENTS

In order to form a complete UNOS there are a number of modules.  These
extra modules are "included" into this module so that altogether they
form one 'C' module.  This was done so that none of the operating system
specific routines need to be public.  They can all be static and therefore
hidden from the user.

*/








/************************************************************************/
/*                                                                      */
/*                                                                      */
/*                          GLOBAL VARIABLES                            */
/*                                                                      */
/*                                                                      */
/************************************************************************/


/* THIS IS SPECIAL VARIABLE REQUIRED TO OVERCOME THE SEQUENCER OVERRUN
PROBLEM PRESENT IN THE COOK ISLANDS PROJECT.

WHEN THIS PATCH IS REMOVED LOOK FOR task_switch_flag IN THE CODE TO FIND
WHERE IT HAS BEEN USED.
*/

static char task_switch_flag = TRUE;





extern unsigned int uinAlarmCritSecSema;

/*------ 8259 INTERRUPT CONTROLLER RELATED DEFINITIONS ------*/

#define I8259_INT_CTRL_ADD 0x20
#define I8259_EOI 0x20





/*
definitions of indexes into the central table
*/

#define BGN_PRIORITY_Q_1 1
#define CURRENT_TASK 0

/*
number of the highest priority in the system - currently always 1
*/

#define HIGH_PRIORITY 1


/*------ Mail box hash table size ------*/
/* Note that this number is set up to be a prime number */
#define HASH_TABLE_SIZE 241



/*------ MAXIMUM NUMBER OF TASKS IN THE SYSTEM ------*/
/* The variable defined below contains the maximum number of tasks that can be
created in the system. This value is initialised from a define in the
main program. The maximum number of tasks is significant since it defines
the amount of resources allocated to some crucial system tables.
*/
unsigned int max_num_of_tasks = 0;


/*------ NUMBER OF TASKS IN THE SYSTEM ------*/
/* This location is used to store the number of tasks in the system. This
location is incremented each time a task is created in the system, so that
after all the tasks have been created it will contain the total number of
system tasks. This total number of tasks has to be less than the value
contained in the max_num_of_tasks variable contained in the operating system.
*/
static unsigned int num_of_tasks = 0;


/*------ NUMBER OF SEMAPHORES IN THE SYSTEM ------*/
/* This location is used to store the total number of semaphores in the
system.  As with the num_of_tasks above this has been implemented as
a variable so that the UNOS module does not have to recompiled if
the number of semaphores changes.
*/

static unsigned int num_semaphores;

/* the following variable is the index into the semaphore array
indicating the next semaphore number to allocate. If there are no
semaphores to allocate then the semaphore number is set to zero. This
number cannot be one for a semaphore which can be allocated because
there must always be at least one task in a operational system,
therefore semaphores 0 and 1 must be allocated to the mailbox
associated with that task.
*/
unsigned int next_semaphore = 0;


/*------ NUMBER OF PRIORITY LEVELS ------*/
/* This location is used to store the number of priority levels in the
system. this is stored in a variable so that UNOS does not have to be
recomplied if the number of priorities is changed.  This number
is fundamental in that is establishes the basic structure of the
central table of the system.  The maximum number of priorities is 256,
the minimum is 1 ( the null task ).
*/

static unsigned char num_of_priorities = 1;



/*------ START SEMAPHORE CENTRAL TABLE ENTRIES ------*/
/* This location stores the central table index where the semaphore central
table entries begin.  This number is calculated from the number
of priorities that the system supports.
*/

static unsigned int bgn_semaphore_central_table;


/*------ BASE NUMBER OF TICKS PER TIME SLICE ------*/
/* This location contains the number of clock ticks that should occur
before a time slice entry to the kernel should occur if the sub-priority
is zero
*/

static unsigned int base_ticks_per_time_slice;


/*------ VARIABLES TO STORE TIME QUEUE INDEXES INTO CENT TABLE ------*/

static unsigned char bgn_active_time_q;
static unsigned char end_active_time_q;
static unsigned char bgn_inactive_time_q;
static unsigned char end_inactive_time_q;



/*------ INTERRUPT NUMBER FOR KERNEL ENTRY ------*/

unsigned char kernel_entry;


/*------ TASK START FLAG ------*/
/* This flag is used to indicate that the tasks are all ready to start.
The flag is tested inside the create_task routine when the first
schedules of the starting tasks occur.  If set then a call to each
particular task is carried out in the create_task routine.
*/

static unsigned char start_flag;



/*------ REAL TIME CLOCK VARIABLE ------*/
/* The real time clock variable is used by the operating system to keep
track of time.  Upon every entry to the tick routine the value stored in
this variable is incremented.  The value stored in this variable finds a
number of uses in UNOS.  The kernel itself uses the times stored here to
check whether certain critical tasks are executed within user specified
time bins.  User tasks can directly acces the time stored here to carry
out user defined timing functions.

Note that the value linearly increases until it wraps around.  The
effects of wrap around must be handled by the user tasks which use the
timer.
*/

static unsigned long rt_clock = 0;


/*------ TIME SLICE FLAG ------*/
/* The time slice flag is used by the tick routine to determine whether to
allow a time slice entry into the kernel.  If true then a time slice
entry is allowed, if FALSE then a time slice entry is not allowed.  The
value of the flag is manipulated by the start_time_slice and
stop_time_slice routines.
*/

static unsigned char time_slice_flag = FALSE;



/*------ SIGNAL ENTRY FROM TIMERS FLAG ------*/
/* This flag is set if a timer has expired and is going to execute a
timeout handler.  One of the most common uses of a timeout handler is
to signal another task that it is supposed to do something.  This is
usually achieved by the handling routine doing a signal to the
appropriate semaphore that the responding task is currently blocked
on.  A signal normally could cause a pre-emptive task switch.  However
for timing reasons it is undesirable for this to occur from the
dec_timers routine (this is where the timeout fuction handler is called).
Therefore the signal routines need to be modified if called from the
dec_timers routine.  This is the purpose of this flag and another defined
below - possible_preemption.  This flag is checked in the signal routine.
If set then the signal routine will put any tcb to be moved onto the
appropriate priority queue but will not call the scheduler.  The signal
handler sets a flag called possible_preemption if a tcb is moved.
*/

static char sig_entry_from_dec_timers = FALSE;

/*------ POSSIBLE PRE-EMPTION FLAG ------*/
/* This flag is related to that defined above so read the comments for
it.  This flag is checked in the tick routine.  If set then it indicates
that there is the possibility of a pre-emptive task switch due to a
timeout of one of the timers.
*/

static char possible_preemption = FALSE;




/*------ MAIL BOX TABLE POINTER ------*/
/* This pointer points to a table of pointers. There are num_of_tasks
pointers in the table which when initialise by the create_mbx routine
point to the mail box structures. The index into the array of pointers
pointed to by this pointer is the task number with which the mail box is
associated. This table is required to allow the OS to find the mail box
memeory address given the task number with which communication is
required.
*/

static mbx **mbx_table;



/*------ MAIL EXCHANGE HASH TABLE ------*/
/* this table contains pointers to taskname_map structures. The array of
pointers is indexed into by a hash value derived from the address of the
array containing the name of the task concerned.
*/

static taskname_map **hash_table;




/*------ MEMORY ALLOCATION RELATED STRUCTURES AND LOCATIONS ------*/
/* The following two locations have to be initialised by the user to define
the system memory pool. This pool of memory is the memory from which all
dynamically allocated data structures will be allocated.
*/
/* the following pointer points to the beginning of the memory pool */

char huge *mem_pool_ptr = NULL;

/* the following location is used to store the size of the memory pool
which can be dynamically allocated at run time.
*/

unsigned long mem_pool_size = 0;

/* the following location is used to store the remaining total memory in the
memory pool. Note that this does not mean that any one memory block will
be large enough for an allocation - if the memory fragmentation is bad then
memory may not be able to be allocated even though the total memory is large
enough.
NB. The remaining memory is stored in header sized units, where the header
is that used by the memory allocation routines. To get the actual memory
in bytes then multiply the value stored here by the sizeof ( blk_header ).
*/

unsigned long rem_memory = 0;




/*------ Initial Empty Free Block Header ------*/
/* The following is the initial memory header required to initiate the
formation of the free list.
*/

static blk_header start_header;

/* the last_blk_alloc is used to keep track of the last header block
accessed in the free list of memory blocks. This position is the starting
point for searches through the queue of free memory blocks.
*/
static blk_header huge *last_blk_alloc = NULL;       /* last allocated block */





/****************************************************************/
/*                                                              */
/*        Task Control Block ( tcb ) Related Definitions        */
/*                                                              */
/****************************************************************/

/*------ pointer to the array of pointers to the task control blocks ------*/

static task_control_block **tcb;





/*------ Task status masks ------*/

/*   Note that the bits in the tcb are defined as follows:
		  bit 0 - 1 => task runnable
				  0 => task blocked on a semaphore.
		  bit 1 -> 7 - reserved
*/








/*==========================================================================*/



/***********************************************/
/*                                             */
/*        Semaphore Related Definitions        */
/*                                             */
/***********************************************/

/*------ Semaphore type definitions ------*/

/*------ Queueing types used in the semaphore queue manipulation ------*/
/*------ routines                                                ------*/

#define DEQUEUE 0        /* Remove tcb from semaphore queue */
#define QUEUE 1          /* Add tcb to semaphore queue */

/* The following defines the two diffeent semaphore types. These are required
to ensure that when a semaphore is claimed that it is not on a synchronization
type of semaphore. If these semaphore types are not present then a problem
occurs with mail boxes for example. In the case when the rcv_mess routine is
called when there is a message already in the mail box then the data
available semaphore would be calimed by the calling task (since it would
have a value of 1. However this is erroneous since the calling task does
not control the semaphore value - the task that calls send_mess controls the
semaphore.

A semaphore type is normally set to be CRITICAL_SECTION_TYPE. However if the
value of the semaphore is set to a value then the type of the semaphore is
set to SYNC_SEMA, since this semaphore is almost always used for resource
counting or synchronization applications.
*/
#define SYNC_SEMA 0
#define CRITICAL_SECTION_SEMA 1

/*------ Semaphore data structure array ------*/


static semaphore_struc **semaphore;



/*========================================================================*/


/******************************************************************/
/*                                                                */
/*        Kernel Control Block ( kcb ) Related Definitions        */
/*                                                                */
/******************************************************************/

/*------ Kernel Control Block (kcb) definition ------*/

static struct {
	unsigned char entry_type;
	unsigned int semaphore_index;
	unsigned int task_number;
	unsigned char new_priority;
} kcb;

/*------ Kernel entry type definitions ------*/

#define MAX_ENTRY_NUM 5
#define WAIT_ENTRY 0
#define RESCHEDULE_ENTRY 1
#define TIME_SLICE_ENTRY 2
#define CREATE_TASK_STK_ENTRY 3
#define PREEMPTIVE_ENTRY 4
#define CHGE_TASK_PRI_ENTRY 5




/************************************************************************/
/*                                                                      */
/*						     UNOS DEBUGGING STUFF						*/
/*																		*/
/************************************************************************/

/* Below is defined a structure which is used to store the previous task
execution history.  This structure is filled out during each kernel
entry.  Currently the structure can only be interrogated by using a debugger,
however the longer term plan is to allow the future UNOS monitor to
interrogate the structure.

The components of the structure are:-

char *taskname_ptr		contains the name of the pointer to the previous
						task that was executing.

int static_priority		static priority of the task.

int dynamic_priority	dynamic priority of the task.

unsigned long clock_cnt	contains the value of the UNOS real-time clock at the
						time of the task switch.

char *reason_ptr		contains a pointer to ashort message that indicates
						the reason for the last task switch.

unsigned int semaphore_num The semaphore number upon which a wait or signal
						is occurring.  Only valid number if this is the
						entry condition.

char *nxt_taskname_ptr	contains the name of the task that executed after
						the stored task.

int nxt_static_priority	the static priority of the next task.

int nxt_dynamic_priority the dynamic priority of the next task.

unsigned long count_num	a simple counter that allows one to see the order of
						the task switches stored in the circular array of
						these structures.

Upon entry to the kernel a structure of the above form is filled out for
temporary storage.  If upon exit from the kernel it is still the same task
then nothing is stored in the circular buffer of structures.  If on the other
hand the task is different then the nxt_taskname_ptr part of the structure
is updated and the structure is stored in the circular array.
*/

typedef
	struct {
		char *taskname_ptr;
		int static_priority;
		int dynamic_priority;
		unsigned long clock_cnt;
		char *reason_ptr;
		unsigned int semaphore_num;
		char *nxt_taskname_ptr;
		int nxt_static_priority;
		int nxt_dynamic_priority;
		unsigned long count_num;
	} kernel_debug;


/* Now define the temporary structure used to hold the info upon kernel
entry
*/
static kernel_debug temp_kernel_store;


/* Now define the array of kernel_debug structures which holds a dozen or so
of the previous tasks that have executed in the system.
*/
#define SIZE_DEBUG_BUFFER 12
static kernel_debug kernel_debug_info [SIZE_DEBUG_BUFFER];


/* The following variable is used to point to the next location in the
array of kernel_debug structures that information is to be stored.
*/
static int kernel_debug_head = 0;


/* Task switch count value */
static unsigned long count_num = 0;


/* The messages that are used for the task switch reasons */
static char time_slice_entry [] = "Time slice entry";
static char wait_entry [] = "Wait entry";
static char reschedule_entry [] = "Reschedule entry";
static char preemptive_entry [] = "Preemptive task switch";
static char change_priority_entry [] = "Change task priority entry";
static char create_task_entry [] = "Create task entry";




/****************************************************************************/
/*                                                                          */
/*                          CENTRAL TABLE DEFINITION                        */
/*                                                                          */
/****************************************************************************/

static void **central_table;



/*------ FUNCTION PROTOTYPE DEFINITIONS ------*/


static void add_queue ( task_control_block *tcb_ptr,
				 unsigned int cent_tab_index );


static void remove_top_queue (  unsigned int cent_tab_index );

static void remove_queue ( task_control_block *tcb_ptr,
					unsigned int cent_tab_index );

static void place_priority_queue ( task_control_block *tcb_ptr );

static void semaphore_queue_handler ( int queue_op );

static void place_semaphore_queue ( unsigned int semaphore_num );

static void scheduler ( void );

static void wait_handler ( void );

static void reschedule_handler ( void );

static void time_slice_handler ( void );

static void add_timer_active_q ( timer_struc* timer_ptr,
                                 unsigned char timer_type,
                                 unsigned long init_time_count,
                                 void ( *timeout_handler )(),
                                 void *data_ptr );

static void add_timer_inactive_q ( timer_struc *timer_ptr );

static timer_struc* remove_timer_inactive_q ( void );

static timer_struc* remove_timer_active_q ( timer_struc* timer_ptr );

static void dec_timers ( void );

static void timed_wait_handler ( void* handler_ptr );

static mbx** alloc_mbx_table ( unsigned int num_of_tasks );

static mbx* create_mbx ( unsigned int task_num, char mbx_type, unsigned int
				 num_mess, unsigned int mess_size, unsigned int
                 spce_avail_sema, unsigned int mess_avail_sema );

static  blk_header huge* morecore ( void );

static void preemptive_schedule_handler ( void );

static void chge_pri_q_manip ( unsigned int tsk_num, unsigned char
                                                        new_priority );

static void chge_task_pri_handler ( void );

static unsigned int mail_exchange ( char *mess_addr_ptr );

static unsigned int hash_taskname_ptr ( char *task_name_ptr );

static taskname_map **alloc_mail_exch_table ( unsigned int size_of_table );




/************************************************************************/
/*																		*/
/*						  COOK ISLANDS MODIFICATION						*/
/*								  REB - 7/9/92							*/
/*                                            							*/
/************************************************************************/







/*
=========================================================================
|
| disable_task_switch
|
| This function clears the flag which is tested before doing any task
| switches.  If the flag is clear then a task switch does not occur.
| This routine is a temporary measure to overcome the sequencer overrun
| problem present in the Cook Islands software.
|
| Parameters	:	- none
|
| Returns		:	- nothing
|
=========================================================================
*/

void disable_task_switch (void) {

	char int_status;

	int_status = return_interrupt_status ();
	disable ();
	task_switch_flag = FALSE;

	if ( int_status ) {
		enable ();
	} /* if */
} /* end of disable_task_switch */





/*-----------------------------------------------------------------------*/





/*
==========================================================================
|
| enable_task_switch
|
| This function sets the task switch flag.  If this flag is set then task
| switching is enabled.  This routine is here to fix the sequencer overrun
| problem in the Cook Islands software.
|
| Parameters	:	- none
|
| Returns		:	- nothing
|
==========================================================================
*/

void enable_task_switch (void) {

	char int_status;

	int_status = return_interrupt_status ();
	disable ();

	task_switch_flag = TRUE;

	if (int_status) {
		enable ();
	} /* if */
} /* end of enable_task_switch */




/****************************************************************************/
/*                                                                          */
/*                         OPERATING SYSTEM ATOMS                           */
/*                                                                          */
/****************************************************************************/





/*
==========================================================================
|
| add_queue
|
| This is a general purpose queue maintenance routine which will add a tcb to
| the end of the appropriate queue.
|
| The parameters passed to the routine are:
|   (i)  a pointer to the tcb to be placed in a queue.
|   (ii) the index into the central_table which contains the pointer to the
|        beginning of the particular queue.
|
==========================================================================
*/

static void add_queue ( task_control_block* tcb_ptr,
                 unsigned int cent_tab_index ) {

	task_control_block* end_q_ptr;

    /* update the queue information in the tcb */
    tcb_ptr->cent_tab_index = cent_tab_index;

    cent_tab_index++;
    end_q_ptr = ( task_control_block* ) central_table [ cent_tab_index ];

    if ( end_q_ptr == NULL ) {
        /* The queue is at the moment empty.  This implies that the
        bgn_q_ptr is also a null.  Therefore the bgn_q_ptr and the
        end_q_ptr should be made equal.
		*/
        ( task_control_block* ) central_table [ cent_tab_index - 1 ] = tcb_ptr;
        ( task_control_block* ) central_table [ cent_tab_index ] = tcb_ptr;
        tcb_ptr->prev_task_ptr = NULL;
        tcb_ptr->next_task_ptr = NULL;
    } /* if */
    else {
        /* already other tcb in the queue link the previous end of the queue
        to the new tcb
        */
        end_q_ptr->next_task_ptr = tcb_ptr;
        tcb_ptr->prev_task_ptr = end_q_ptr;
        tcb_ptr->next_task_ptr = NULL;
        /* update the central_table end of queue pointer */
		( task_control_block* ) central_table [ cent_tab_index ] = tcb_ptr;
    } /* else */
}   /* end of add_queue */





/*--------------------------------------------------------------------------*/





/*
==========================================================================
|
| remove_top_queue
|
| This is a simpler queue maintenance routine which only allows tcb's to be
| removed from the top of a queue.  This is included for speed reasons since
| on some occassions the more complex remove_queue routine is not needed.
|
|   Parameters: - index into the central table location which contains
|                 the pointer to the top of the queue in question.
|   Entry via : - multiple places in the kernel
|
==========================================================================
*/

static void remove_top_queue (  unsigned int cent_tab_index ) {

    task_control_block* tcb_ptr;

    tcb_ptr = ( task_control_block* ) central_table [ cent_tab_index ];

    if ( tcb_ptr != NULL ) {
        /* something to remove so do it */
		tcb_ptr->q_type = DONOT_Q_TYPE;
        tcb_ptr->cent_tab_index = 0;
        ( task_control_block* ) central_table [ cent_tab_index ] =
                                                    tcb_ptr->next_task_ptr;
        if ( central_table [ cent_tab_index ] == NULL )
            central_table [ cent_tab_index + 1 ] = NULL;   /* queue empty */
		else {
            tcb_ptr =
                 ( task_control_block* ) central_table [ cent_tab_index ];
            tcb_ptr->prev_task_ptr = NULL;
        } /* else */
    } /* if */

}   /* end of remove_top_queue */






/*---------------------------------------------------------------------------*/





/*
============================================================================
|
| remove_queue
|
| This is a general purpose queue maintenance routine.  It removes the tcb
| passed in from the queue passed in.  The tcb does not have to be at the
| beginning or end of a queue, but can also be removed from the middle.
|
| The parameters passed in are defined as follows:
|   (i) pointer to the task control block to be removed from the queue.
|   (ii)index into the central_table to the position which contains the
|        pointer to the top of the particular queue.
|
============================================================================
*/

static void remove_queue ( task_control_block* tcb_ptr,
					unsigned int cent_tab_index ) {

    task_control_block* prev_tcb_ptr;

    /* Note: prev_tcb_ptr = 0 if the tcb to be removed is at the beginning
    of a queue.  This routine is tricky because one must account for the
    the case where the queue only contains one entry - therefore the
    central_table begin and end pointers should end up with zero in them.
    */
    prev_tcb_ptr = tcb_ptr->prev_task_ptr;

    if ( tcb_ptr ==
            ( task_control_block* ) central_table [ cent_tab_index ] ) {
        /* removing first entry in the queue - update beginning of queue
        pointer
        */
        ( task_control_block* ) central_table [ cent_tab_index ] =
                                                tcb_ptr->next_task_ptr;
		if ( central_table [ cent_tab_index ] != NULL )
            ( ( task_control_block* )central_table [ cent_tab_index ] )
                                                    ->prev_task_ptr = NULL;
    } /* if */

	if ( tcb_ptr ==
            ( task_control_block* ) central_table [ cent_tab_index + 1 ] ) {
        /* removing last entry in queue. */
		( task_control_block* ) central_table [ cent_tab_index + 1 ] =
                                                            prev_tcb_ptr;
        if ( prev_tcb_ptr != NULL )
            /*
            not the only entry in the queue.
            */
			prev_tcb_ptr->next_task_ptr = NULL;
    } /* if */
    else {
		/* removing an entry somewhere in the middle of a queue so update
        the pointers in the tcbs appropriately.
        */
        if ( prev_tcb_ptr != NULL ) {
            prev_tcb_ptr->next_task_ptr = tcb_ptr->next_task_ptr;
            tcb_ptr = tcb_ptr->next_task_ptr;
            tcb_ptr->prev_task_ptr = prev_tcb_ptr;
        } /* if */

    } /* else */

    /* update the queue related components of the tcb */
    tcb_ptr->q_type = DONOT_Q_TYPE;
    tcb_ptr->cent_tab_index = 0;
}   /* end of remove_queue */





/*------------------------------------------------------------------------*/





/*
============================================================================
|
| place_priority_queue
|
| This procedure accepts a parameter of type pointer which points to the tcb
| which is to be placed on the appropriate priority queue of runnable tasks.
| The task is placed on the end of the priority queue.  The status in the
| task control block is made runnable.  Note that in the case of a task coming
| from the time queue it should always be runnable, however tasks from the
| blocked queues have to be made runnable.
|
| Parameters :- pointer to the tcb to be queued.
|
============================================================================
*/

static void place_priority_queue ( task_control_block* tcb_ptr ) {

    /* firstly change the status of the task to runnable. */
    tcb_ptr->task_status = TASK_RUNNABLE;

    /* update the queue related information in the tcb */
    tcb_ptr->q_type = PRIORITY_Q_TYPE;

    /* now link the tcb into the correct priority queue. */
    add_queue ( tcb_ptr, ( ( ( tcb_ptr->dynamic_priority ) - 1 ) << 1 ) + 1 );

}   /* end of place_priority_queue */




/*-------------------------------------------------------------------------*/






/*
============================================================================
|
| timed_wait_handler
|
| This function is entered if a timed wait has timed out. The function of this
| function is to remove the timed out tcb from the queue that it is currently
| and place it in the appropriate priority queue. The timeout_flag is set in
| the offending tcb so that the timed_wait routine can determine if the tcb
| has been released due to a signal or a time out. Since a time out is a timer
| initiated signal this routine does the same thing as the signal handler -
| it sets a flag if a pre-emptive task switch is required.
|
| Parameters : - pointer to the task control block to be remove froma queue
|
| Entry via  : - dec_timers
|
============================================================================
*/

static void timed_wait_handler ( void* handler_ptr ) {

    task_control_block* tcb_ptr;

    tcb_ptr = ( task_control_block* )handler_ptr;

    /* set the timeout_flag to indicate that the tcb has been put onto the
    priority queue
    */
    tcb_ptr->timeout_flag = TRUE;

    /* remove the tcb from the queue it is on and put it on the
    correct priority queue. Then set the possible_preemption flag so that
    the scheduler will be called from the tick routine.
    */
    remove_queue ( tcb_ptr, tcb_ptr->cent_tab_index );
    place_priority_queue ( tcb_ptr );
    possible_preemption = TRUE;
} /* end of timed_wait_handler */





/*-------------------------------------------------------------------------*/




/*
=============================================================================
|
| semaphore_queue_handler
|
| This routine handles the queueing and dequeueing operations for all the
| semaphore queues in the system.  This is the default routine which is stored
| in the semaph_queue_handler pointer in the semaphore data structure during
| system initialization.
|
| The queue/dequeue routines work in the following way.
|
| If a task is to be added to a queue it is added to the queue so that the
| whole queue is ordered in dynamic priority order. That is the highest
| dynamic priority tcb is put onto the beginning of the queue and the lowest
| dynamic priority tcb would be on the end of the queue. This is done to
| that the highest priority tasks are blocked for the smallest amount of time.
| If a task is added to the head of the queue then this means that the task is
| a higher priority task than any of the previous tasks on the queue. This in
| turn implies that the dynamic priority component of the semaphore structure
| has to be adjusted so that the dynamic priority of the task which currently
| has the semaphore is the dynamic priority of this newest highest priority
| task. The system should carry out a schedule after the current task has been
| queued which will give the highest priority task the opportunity to run.
|
| If a task is to be removed from the queue it is removed from the top.  If a
| task is removed from the top of the queue then a preemptive_schedule must
| occur since the current running task may now not be the highest priority one.
| If so a preemptive task switch occurs.  There are two other variations to
| this basic behaviour. The first is related to entering this routine from a
| signal instigated by the expiring of a timer in the system.  In this case
| one does not want a pre_emptive task switch to occur since this could cause
| a lengthy task to run when the entry to this point has occurred from the
| dec_timers routine in the tick function. This is a problem because it would
| upset the precision of the expiry of other timers if multiple timers were
| expiring at the same time as the first timer. The other is the dequeueing
| when the multiplier component of the semaphore is non zero.  This is
| usually the case for a semaphore which is being used for an event.  In this
| case one wants multiple tasks to be started from the one signal.  Therefore
| the dequeue routine will attempt the move multiple tasks from the semaphore
| queue to the appropriate priority queues.  If the number of tasks specified
| by the num_of_tasks_to_remove cannot be removed then the remainder is
| stored on the semaphore_value component of the semaphore structure.
|
| If on a semaphore dequeue operation the semaphore value is zero (for both
| single or multiple value semaphores) then the tcb which has just been
| dequeued is the new tcb which has the semaphore. Therefore the task related
| components of the semaphore structure have to be updated.
|
| A further complication occurs in this routine due to the presence of timed
| waits on semaphores. A check is made to see if the task removed from the
| queue is a timed wait task and if so then the timer associated with the
| task is stopped.
|
| The parameters passed in is the required operation.  The queue to be acted on
| is pointed to by the semaphore contained in the kernel control block.
|
| Parameters : - operation to be performed on the queue.
|
| Entry via  : - _signal function.
|
=============================================================================
*/

static void semaphore_queue_handler ( int queue_op ) {

	task_control_block* tcb_ptr, * tcb_ptr_1, * bgn_sema_q_ptr;
	unsigned int num_of_tasks_to_remove;
	unsigned int bgn_q_index, end_q_index;
	char sync_semaphore;

	if ( queue_op == QUEUE ) {
		bgn_q_index = semaphore [ kcb.semaphore_index ]->semaph_bgn_q_index;
		bgn_sema_q_ptr = ( task_control_block* )central_table [ bgn_q_index ];
		tcb_ptr = ( task_control_block* )central_table [ CURRENT_TASK ];
        /* update the information on the queue type stored in the tcb */
        tcb_ptr->q_type = SEMAPHORE_Q_TYPE;
		tcb_ptr->task_status = TASK_BLOCKED;
        tcb_ptr->cent_tab_index = bgn_q_index;
        tcb_ptr->sema_blocked_on = kcb.semaphore_index;

        /* now check to see if the semaphore queue is currently empty. If so
        then place the current task at the beginning of the queue.
        */
        if ( bgn_sema_q_ptr == NULL ) {
			( task_control_block* )central_table [ bgn_q_index ] = tcb_ptr;
			( task_control_block* )central_table [ bgn_q_index + 1 ] =
                                                                    tcb_ptr;
            tcb_ptr->prev_task_ptr = NULL;
            tcb_ptr->next_task_ptr = NULL;
        } /* if */
        else {
            /* there are already other tcbs on the semaphore queue therefore
            find the correct spot on the queue to place the current task
            */
			tcb_ptr_1 = bgn_sema_q_ptr;

            /* now search through the queue to find the correct spot to put
			the current task. N.B. the highest priority is priority 1.
            */
            while ( ( tcb_ptr->dynamic_priority >=
                                        tcb_ptr_1->dynamic_priority ) &&
                                        ( tcb_ptr_1 != NULL ) )
                tcb_ptr_1 = tcb_ptr_1->next_task_ptr;

            /* at this stage tcb_ptr_1 is either pointing to the tcb which
            should be the one after the tcb to be placed, or alternatively
            it is pointing to the end of the queue.
            */
            if ( tcb_ptr_1 != NULL ) {
                /* the tcb has to be placed somewhere in the middle of a
				semaphore queue.
                */
                if ( tcb_ptr_1 != bgn_sema_q_ptr )
                    tcb_ptr_1->prev_task_ptr->next_task_ptr = tcb_ptr;
                else
                    ( task_control_block* )central_table [ bgn_q_index ] =
                                                                tcb_ptr;
                tcb_ptr->prev_task_ptr = tcb_ptr_1->prev_task_ptr;
                tcb_ptr->next_task_ptr = tcb_ptr_1;
                tcb_ptr_1->prev_task_ptr = tcb_ptr;
			} /* if */
            else {
                /* the tcb has to be placed on the end of the queue */
				end_q_index = bgn_q_index + 1;
                ( ( task_control_block* )central_table [ end_q_index ] )->
                                        next_task_ptr = tcb_ptr;
                tcb_ptr->prev_task_ptr = ( task_control_block* )central_table [
                                            end_q_index ];
                tcb_ptr->next_task_ptr = NULL;
				( task_control_block* )central_table [ end_q_index ] =
                                                                    tcb_ptr;
            } /* else */
        } /* else */

        /* now see if the dynamic priority of the task which currently
        has the semaphore has to be updated.
		*/
        tcb_ptr = ( task_control_block* )central_table [ bgn_q_index ];
        tcb_ptr_1 = semaphore [ kcb.semaphore_index ]->tcb_ptr;
        if ( ( tcb_ptr_1 != NULL ) &&
            ( tcb_ptr->dynamic_priority < tcb_ptr_1->dynamic_priority ) ) {
            /* now physically change the dynamic priority of the task
            which currently has the semaphore
            */
            tcb_ptr_1->dynamic_priority = tcb_ptr->dynamic_priority;
            /* now remove the task from the queue that it is currently on and
            put it onto the new priority queue. The task is only removed from
            the queue if it is on a priority queue. It is not moved if it is
            on a semaphore queue.
			*/
            if ( tcb_ptr_1->q_type == PRIORITY_Q_TYPE ) {
                remove_queue ( tcb_ptr_1, tcb_ptr_1->cent_tab_index );
                place_priority_queue ( tcb_ptr_1 );
            } /* if */
        } /* if */
	} /* if */
    else {  /* must be a dequeue operation that is required */
        bgn_q_index = semaphore [
                            kcb.semaphore_index ]->semaph_bgn_q_index;
        
        /* if a signal has occurred on a semaphore which has a value at this 
        point of zero and the tcb_ptr component of the semaphore structure 
		is NULL then this means that the semaphore is an intertask
        synchronization semaphore - ie. a task has been blocked on a 
		semaphore which had an initial value of zero and the signal to
        unblock it has come from another task. In this case the unblocked 
        task should not claim the semaphore since it is not being used as a 
        critical section protection primitive. If it were to claim the 
        semaphore then as soon as a rcv_mess operation is carried out by a 
        task it would always be claiming a semaphore.
        
        To overcome this problem set a flag if this condition is detected. 
        This flag is used subsequently to see if the calling task should 
        claim the semaphore.
        */
		if ( ( semaphore [ kcb.semaphore_index ]->semaphore_value == 0 ) &&
            ( semaphore [ kcb.semaphore_index ]->tcb_ptr == NULL ) )
            sync_semaphore = TRUE;
        else
            sync_semaphore = FALSE;

        /* to enter this routine the semaphore value must be zero (or else 
        one cannot have blocked tasks). Therefore try to remove the 
        semaphore_multipler number of tasks.
        */
        num_of_tasks_to_remove = semaphore [ kcb.semaphore_index ]->
                                                semaphore_multiplier;

		/* now enter the remove tcb from queue loop.  This loop attempts to
        remove num_of_tasks_to_remove from the semaphore queue.  If it
		cannot remove this number of tasks then the remainder is stored in
        the semaphore_value location of the semaphore structure.
        */
        do {
            tcb_ptr = ( task_control_block* )central_table [ bgn_q_index ];

            /* now check to see if this task is on a timed wait. If so then
            stop the timer associated with the task. The timer pointer is
            contained within the tcb structure.
			*/
            if ( tcb_ptr->timer_ptr != NULL ) {
				stop_timer ( tcb_ptr->timer_ptr );
                tcb_ptr->timer_ptr = NULL;
            } /* if */
            remove_top_queue ( bgn_q_index );
            place_priority_queue ( tcb_ptr );
            num_of_tasks_to_remove--;
        } while ( ( num_of_tasks_to_remove ) &&
                            central_table [ bgn_q_index ] != NULL );

        semaphore [ kcb.semaphore_index ]->semaphore_value =
                                            num_of_tasks_to_remove;

        /* now make the semaphore structure point to the last tcb removed
        from the semaphore queue and update the num_semas_claimed component 
        of the tcbs. If not a synchronization semaphore.
        */
        if ( ( num_of_tasks_to_remove == 0 ) && ( !sync_semaphore ) ) {
            semaphore [ kcb.semaphore_index ]->tcb_ptr = tcb_ptr;
            tcb_ptr->num_semas_claimed++;
        } /* if */
        
        /* now check to see whether the signal has occurred from a function 
        which has been activated from a timer expiration
        */
		if ( sig_entry_from_dec_timers )
            possible_preemption = TRUE;
    } /* else */
}   /* end of semaphore_queue_handler */





/*--------------------------------------------------------------------------*/




/*
=============================================================================
|
| place_semaphore_queue
|
| This routine places the tcb pointed to by the central_table(CURRENT_TASK)
| on the blocked queue indicated by the semaphore passed in.
|
| Parameter
|       - semaphore array index to semaphore in question
=============================================================================
*/

static void place_semaphore_queue ( unsigned int semaphore_num ) {

    void ( *handler_semaphore_q ) ( int queue_op );

	handler_semaphore_q = (void (*)(int))semaphore [ semaphore_num ]->semaph_queue_handler;
    ( *handler_semaphore_q ) ( QUEUE );
} /* end place_semaphore_queue */




/*--------------------------------------------------------------------------*/





/*
============================================================================
|
| init_tcb_and_q_it
|
| The function of this procedure is to initialize all of the elements of the
| tcb for a task except the stack pointer and to place the tcb in the correct
| queue.  Note that to allow for setting up of the null task ( which is set
| up as the current task and is not queued ) there is a special q_type called
| DONOT_Q_TYPE.  This is simply a return statement.  Hence all the routine
| has done is to initialise the tcb for the task.
|
| In order to carry out the initialisation of the tcb memory firstly must
| be allocated for it. This is allocated from the UNOS heap. The pointer to
| the allocated memory is then placed in the tcb pointer array in the
| location corresponding to the task number.
|
| The routine returns the pointer to the tcb memory area allocated. If the
| memory allocation has been unsuccessful then the pointer returned is a
| NULL.
|
|
| The parameters passed into the routine are:
|
|   task_name - a pointer to a character array containing the name of the
|               task.
|
|   priority_of_task - the priority of the newly created task.
|
|   tick_delta - the amount that the ticks_per_time_slice value is altered
|                to change the micro priority of tasks at the same priority
|                level.
|
|   status_of_task - contains the desired contents of the tcb status.  Must
|                    be consistent with the queue that the task is placed on.
|
|   q_type - the type of queue the tcb is to be placed in - priority queue,
|            semaphore queue, or the time queue.  This information is required
|            to invoke the correct routine for queue addition.
|            Queue types -
|                    PRIORITY_Q_TYPE
|                    SEMAPHORE_Q_TYPE
|                    DONOT_Q_TYPE
|
|   semaphore - index into the semaphore array if tcb is to be added to the
|               blocked queue of a particular semaphore.
|
==============================================================================
*/

static task_control_block *init_tcb_and_q_it ( char *task_name,
                           unsigned char priority_of_task,
                           int tick_delta,
                           unsigned char status_of_task,
                           unsigned char q_type,
                           unsigned int semaphore_num
                          ) {

    /* firstly allocate the memory for the task control block */
	if ( ( tcb [ num_of_tasks ] = ( task_control_block * )umalloc
                        ( sizeof ( task_control_block ) ) ) == NULL ) {
		return tcb [ num_of_tasks ];
    } /* if */

    tcb [ num_of_tasks ]->task_num = num_of_tasks;
    tcb [ num_of_tasks ]->task_name_ptr = task_name;
    tcb [ num_of_tasks ]->static_priority = priority_of_task;
    tcb [ num_of_tasks ]->dynamic_priority = priority_of_task;

    /* now set up the number of ticks before this particular task will
	enter the kernel. The number of ticks is set up as an absolute value.
    */
    if ( base_ticks_per_time_slice + tick_delta <= 0 ) {
        tcb [ num_of_tasks ]->ticks_before_kernel_entry = 1;
    } /* if */
    else {
        tcb [ num_of_tasks ]->ticks_before_kernel_entry =
                        base_ticks_per_time_slice + tick_delta;
    } /* else */

    tcb [ num_of_tasks ]->ticks_to_go =
                        tcb [ num_of_tasks ]->ticks_before_kernel_entry;

    tcb [ num_of_tasks ]->q_type = q_type;
    tcb [ num_of_tasks ]->task_status = status_of_task;
    tcb [ num_of_tasks ]->timeout_flag = FALSE;
    tcb [ num_of_tasks ]->cent_tab_index = 0;
    tcb [ num_of_tasks ]->num_semas_claimed = 0;
    tcb [ num_of_tasks ]->timer_ptr = NULL;

    /* Set up these initializations for the following routines */
    ( task_control_block* ) central_table [ CURRENT_TASK ] =
                                                    tcb [ num_of_tasks ];
    /* setup for semaphore q routine */
    kcb.semaphore_index = semaphore_num;

	/* Now place the tcb in the appropriate queue */
    switch ( q_type) {
        case 0 :
            place_priority_queue ( tcb [ num_of_tasks ] );
            break;
        case 1 :
            place_semaphore_queue ( semaphore_num );
            break;
        case 2 :
			tcb [ num_of_tasks ]->cent_tab_index = 0;
            break;
        default :
            cprintf ( "Illegal queue type\n\r" );
    } /* switch */

    return tcb [ num_of_tasks ];

}   /* end of init_tcb_and_q_it */




/*--------------------------------------------------------------------------*/





/***************************************************************************/
/*                                                                         */
/*                          OPERATING SYSTEM MOLECULES                     */
/*                                                                         */
/***************************************************************************/





/*
=============================================================================
|
| scheduler
|
| This procedure searches through the priority queues to find the next task
| to run.  The search begins with the high priority queue - the task at the
| top of the queue is selected to run.  If the queue is empty or contains
| no active task then the search continues in the low priority queue.  If
| there is not a task here to run then the search continues in the null queue.
| The null task is always ready to run.  The task to next run is placed in the
| central_table CURRENT_TASK location.
|
|   Parameters : - none
|   Entry via  : - multiple paces in kernel
|
============================================================================
*/

static void scheduler ( void ) {

	unsigned int central_table_index = BGN_PRIORITY_Q_1;

	while ( central_table_index < ( num_of_priorities * 2 ) ) {
		if ( central_table [ central_table_index ] != NULL ) {
			/* task in priority queue so make it the next task to run. */
			central_table [ CURRENT_TASK ] = central_table [
													central_table_index ];
			remove_top_queue ( central_table_index );
			central_table_index = ( num_of_priorities * 2 );
		} /* if */
		else {
			/* either no task in the queue so go to the next queue */
			central_table_index = central_table_index + 2;
		} /* else */
	} /* while */

	/* at this point the CURRENT_TASK entry in the central table should contain
	the next task to be dispatched.
	*/

}   /* end of scheduler */





/*--------------------------------------------------------------------------*/







/************************************************************************/
/*                                                                      */
/*                                                                      */
/*                                                                      */
/*                           UNOS_2 MODULE                              */
/*                                                                      */
/*                                by                                    */
/*                                                                      */
/*                            Robert Betz                               */
/*      Department of Electrical Engineering and Computer Science       */
/*                      University of Newcastle                         */
/*                             Australia                                */
/*                                                                      */
/*                         ( Copyright 1989 )                           */
/*                                                                      */
/*                                                                      */
/*                                                                      */
/************************************************************************/



/*

HISTORY

This module was input on the 22/11/89

*/



/*

DESCRIPTION

This module contains the lattice routines of the operating system kernel and
the operating functions related to the timers in UNOS.  The user interface
routines for the timers reside in the UNOS_3 include module.

For a complete description of the timer mechanism refer to the UNOS.DOC file.

*/




/****************************************************************************/
/*                                                                          */
/*                          THE LATTICE ROUTINES                            */
/*                                                                          */
/*                      I.E. KERNEL HANDLER ROUTINES                        */
/*                                                                          */
/****************************************************************************/


/* These routines are called directly from the kernel to handle the source of
kernel entry.
*/


/*
=============================================================================
|
| wait_handler
|
| This handler is entered if when a semaphore value is decremented in the wait
| routine the value goes to zero.  This means that the task has become blocked
| therefore another has to be scheduled.  This routine puts the current task
| onto the appropriate semaphore queue and then calls the scheduler.
|
|   Parameters : - pointer to the current task control block
|
|   Entry via  : - kernel routine
|
=============================================================================
*/

static void wait_handler ( void ) {

	void ( *handler_semaphore_q ) ( int queue_op );

	handler_semaphore_q = (void (*)(int))semaphore [
							kcb.semaphore_index ]->semaph_queue_handler;
	( *handler_semaphore_q ) ( QUEUE );
	scheduler ();

}   /* end of wait_handler */





/*----------------------------------------------------------------------------*/




/*
=============================================================================
|
| reschedule_handler
|
| This procedure is entered if the current task has requested a reschedule.
| This routine differs from the pre-emptive handler in that the current
| task is not placed on the priority queue before the scheduler is called.
| Therefore the current task is not the next task to run. After the scheduler
| has got the next task to run the previous current task is placed on the
| appropriate priority queue. This implies that the next task to run could
| be lower priority then the current task.
|
|    Parameters : - none
|
|    Entry via  : - multiple places
|
=============================================================================
*/

static void reschedule_handler ( void ) {

	task_control_block* tcb_ptr;

	tcb_ptr = ( task_control_block* )central_table [ CURRENT_TASK ];

	/* THE FOLLOWING IF STATEMENT PLACED IN THE CODE TO PREVENT TO OVERCOME
	THE OVERRUN PROBLEM WITH THE COOK ISLANDS CODE.
	*/
	if (task_switch_flag) {
	scheduler ();
	place_priority_queue ( tcb_ptr );
	} /* if */
}   /* end of reschedule_handler */





/*----------------------------------------------------------------------------*/





/*
==============================================================================
|
| time_slice_handler
|
| This routine is entered if a time slice interrupt has been received.  It
| simply calls the scheduler to get the next routine to run.
|
|   Parameters : - none
|
|   Entry via  : - kernel
|
=============================================================================
*/

static void time_slice_handler ( void ) {

	/* THE FOLLOWING IF STATEMENT PLACED IN THE CODE TO PREVENT TO OVERCOME
	THE OVERRUN PROBLEM WITH THE COOK ISLANDS CODE.
	*/
	if (task_switch_flag) {
	place_priority_queue (
			   ( task_control_block* ) central_table [ CURRENT_TASK ] );
	scheduler ( );
	} /* if */
}   /* end of time_slice_handler */





/*-----------------------------------------------------------------------*/







/*
===========================================================================
|
| preemptive_schedule_handler
|
| This routine is entered if there is the possibility of a preemptive
| schedule occurring.  This routine differs from the normal schedule
| routine in that it checks to see if there is a task of higher priority
| than the current task.  This is achieved by checking the beginning of
| each of the queues for higher priority tasks than the current task to
| see if the first tcb pointer is not a NULL.  If not a NULL then there is a
| task on the queue ready to run.  The current task is then saved in the
| appropriate priority queue and the higher priority task is then made
| the current task.
|
| Note that even if there is a task of the same priority then this task will
| not be run - i.e. with a pre-emptive task switch round robin scheduling
| does not occur.
|
|   Parameters : - none
|   Entry via  : - kernel routine
|
============================================================================
*/

static void preemptive_schedule_handler ( void ) {

    unsigned int central_table_index = BGN_PRIORITY_Q_1;
    unsigned int central_table_fence;
    task_control_block* tcb_ptr;


	/* THE FOLLOWING IF STATEMENT IS A FUDGE PUT INTO UNOS TO GET AROUND
	THE SEQUENCER OVERRUN PROBLEM IN THE COOK ISLANDS PROJECT.
    */
	if (task_switch_flag) {

	/* see if the priority of the current task is less then the highest
	priority.  If not then leave since no other task can be of higher
	priority.
	*/
	tcb_ptr = ( task_control_block* ) central_table [ CURRENT_TASK ];
	if ( tcb_ptr->dynamic_priority > 1 ) {
		central_table_fence = ( ( tcb_ptr->dynamic_priority - 1 ) << 1 ) + 1;
		while ( central_table_index < central_table_fence ) {
			if ( central_table [ central_table_index ] != NULL ) {
				/* there is a task on this queue so place the current task on
				the appropriate priority queue and then make this task the
				current task and remove it from its priority queue.
				*/
				place_priority_queue ( tcb_ptr );
				central_table [ CURRENT_TASK ] =
									central_table [ central_table_index ];
				remove_top_queue ( central_table_index );
				/* change the central_table_index so that the while loop will
				terminate.
				*/
				central_table_index = central_table_fence;
			} /* if */
			else
				/* the current queue has no tcbs in it so increment the index
				to point to the next queue.
				*/
				central_table_index += 2;
		} /* while */
	} /* if */

	} /* if */

} /* end preemptive_schedule_handler */







/*--------------------------------------------------------------------------*/






/*
===========================================================================
|
| chge_pri_q_manip
|
| This function carries out the function of determining what queue a task is
| on which has had its priority changed and then moving that task either
| onto a different queue (if the task was on a priority queue) or moving the
| task within a queue (if task was on a semaphore queue). If a task is moved
| within a semaphore queue then a check is made to see if the task claiming
| the semaphore has to moved onto a different priority queue (note that this
| occurs in the normal semaphore queue maintenance routines).
|
| Note that routine assumes that the task whose priority is being changed
| is not the current task. Therefore the task has to be either on a priority
| queue or a semaphore queue.
|
| Parameters :- pointer to the name of the task.
|
| Entry via  :- chge_task_pri_handler
|
===========================================================================
*/

static void chge_pri_q_manip ( unsigned int tsk_num, unsigned char
                                                        new_priority ) {

    void ( *handler_semaphore_q ) ( int queue_op );
    task_control_block* temp_task_ptr;

    if ( tcb [ tsk_num ]->q_type == PRIORITY_Q_TYPE ) {
        remove_queue ( tcb [ tsk_num ], tcb [ tsk_num ]->cent_tab_index );
        tcb [ tsk_num ]->dynamic_priority = new_priority;
        place_priority_queue ( tcb [ tsk_num ] );
    } /* if */
    else {
		/* must be a semaphore queue - remove task from queue, change its
        priority and then place it back onto the semaphore queue.
        */

        remove_queue ( tcb [ tsk_num ], tcb [ tsk_num ]->cent_tab_index );

        /* now change the dynamic priority of the task */
        tcb [ tsk_num ]->dynamic_priority = new_priority;

        /* now put back into the semaphore queue at the correct position.
        To do this we need to make the tcb be the current task since the
		semaphore queue handler assumes that a queuing entry is from the
        wait_handler. To allow this to happen temporarily store the current
        task pointer and then restore it.
        */
        /* temporarily save the current task number */
        temp_task_ptr = ( task_control_block* )central_table [
                                    CURRENT_TASK ];
        ( task_control_block* )central_table [ CURRENT_TASK ] =
                                                    tcb [ tsk_num ];
		handler_semaphore_q = (void (*)(int))semaphore [ tcb [ tsk_num ]->sema_blocked_on ]
                                                   ->semaph_queue_handler;
        ( *handler_semaphore_q ) ( QUEUE );

        /* restore the current task pointer */
        ( task_control_block* )central_table [ CURRENT_TASK ] =
                                                        temp_task_ptr;
    } /* else */
} /* end of chge_pri_q_manip */





/*--------------------------------------------------------------------------*/





/*
=========================================================================
|
| chge_task_pri_handler
|
| This function is the handler called when a priority change is to be made
| to a task. The operation of this procedure is described in general detail
| in the interface routine to this kernel call - change_task_priority which
| resides in the UNOS_3.INC file. I will not repeat the same description in
| total here. Suffice to say that the process in complicated because of the
| presence of dynamic priorities. Generally when a dynamic priority is
| changed then a task has to be moved either onto a new queue or moved
| within the same queue.
|
| Most of the data required for this function is contained in the kernel
| control block (kcb).
|
| The logic of the routine should not be too difficult to follow.
|
| Parameters : - none
|
| Entry via  : - kernel function switch statement
|
==========================================================================
*/

static void chge_task_pri_handler ( void ) {

	unsigned int tsk_num = kcb.task_number;
	unsigned char new_priority = kcb.new_priority;
	task_control_block* cur_tcb_ptr =
						( task_control_block* )central_table [ CURRENT_TASK ];

	/* now check to see if the task whose priority is to be changed is the
	current task. If so then change the dynamic priority only if it is claiming
	no semaphores, or if it is claiming semaphores only if the new dynamic
	priority is greater than or equal to the dynamic priority of the task on
	the top of the semaphore queue.
	*/
	if ( tsk_num == cur_tcb_ptr->task_num ) {
		if ( cur_tcb_ptr->num_semas_claimed == 0 )
			cur_tcb_ptr->dynamic_priority = new_priority;
		else {
			/* must be claiming semaphores therefore check the priority of the
			top task blocked on the semaphore.
			*/
			if ( central_table [ cur_tcb_ptr->cent_tab_index ] != NULL ) {
				/* must be something blocked on the semaphore that the current
				task is claiming.
				*/
				if ( new_priority <= ( ( task_control_block* )central_table [
						cur_tcb_ptr->cent_tab_index ] )->dynamic_priority )
					cur_tcb_ptr->dynamic_priority = new_priority;
			} /* if */
			else
				/* no task being blocked so update the priority */
				cur_tcb_ptr->dynamic_priority = new_priority;
		} /* else */
	} /* if */
	else {
		/* mustn't be the current task which is having its priority changed
		*/

		/* check to see if the new priority is > than the current dynamic of the
		task. If so then the dynamic priority is changed. Note the > is in the
		priority sense which means that numerically it is less than.
		*/
		if ( new_priority < tcb [ tsk_num ]->dynamic_priority ) {
			/* now call the routine which will put the task on the correct
			priority queue or will reorder the semaphore queue and then reorder
			the claiming task on the priority queue if necessary.
			*/
			chge_pri_q_manip ( tsk_num, new_priority );
		} /* if */
		else {
			/* the new priority numerically > current dynamic priority */
			if ( tcb [ tsk_num ]->num_semas_claimed == 0 ) {
				chge_pri_q_manip ( tsk_num, new_priority );
			} /* if */
			else {
				/* task is claiming a semaphore - if the number of semaphores
				claimed is only one and the new priority is greater than the
				priority of the task on the top of the semaphore queue than
				change the priority of the task and reorder the system queues.
				*/
				if ( tcb [ tsk_num ]->num_semas_claimed == 1 ) {
					if ( ( central_table [ tcb [ tsk_num ]->cent_tab_index ]
							!= NULL ) &&
					  ( ( ( task_control_block* )central_table
					  [ tcb[ tsk_num ]->cent_tab_index ] )->dynamic_priority
										> new_priority ) ) {
						chge_pri_q_manip ( tsk_num, new_priority );
					} /* if */
				} /* if */
			} /* else */
		} /* else */
	} /* else */

	/* now see if a task switch is necessary. */
	preemptive_schedule_handler ( );
} /* end of chge_task_pri_handler */






/*--------------------------------------------------------------------------*/





/*
=============================================================================
|
| kernel
|
| This procedure is usually entered as a result of a software interrupt.  The
| routine is set up as an interrupt procedure and this ensures that all the
| current task data is saved on the stack.  The routine assumes that the kcb
| has been set up by some prologue routine before entering this routine - see
| the kcb definition and the description at the begining of the program
| for the details on this.
|
| The first function of the routine is to save the current stack offset and
| frame in the tcb of the current task.  The procedure then enters a case
| statement whose selector is a component of the kernel control block.  It is
| assumed by the kernel that the kernel control block has been set up by one
| of the prologue routines before entering the kernel proper.
|
| Exit from the kernel is via the dispatcher section, except for the
| create_task_stack entry which exits via a goto.  The dispatcher always
| reinstates the task whose tcb is pointed to by the central_table(CURRENT_TASK)
| value.
|
===========================================================================
*/

void interrupt kernel ( void  ) {

	static task_control_block* cur_tcb_ptr;
	unsigned int stkbase, stkptr, baseptr;
//	char strg1 [20];
//	char strg2 [20];
//	unsigned char static_priority;
//	unsigned char dynamic_priority;
//	char* base_video_ptr;
//	int i;

	/* Firstly save the stackbase and stack pointer for the task pointed to
	by the central_table.  For a TurboC compiler the base pointer must also
	be saved since this is used to reference the variables defined in this
	routine - i.e. the local variables.
	!!!!! Note this section of the code is not portable and would most likely
	have to be implemented as an assembly language routine in most 'C'
	implementations.
	*/

	stkbase = _SS;
	stkptr = _SP;
	baseptr = _BP;

	cur_tcb_ptr = ( task_control_block* ) central_table [ CURRENT_TASK ];
	cur_tcb_ptr->task_stkbase = stkbase;
	cur_tcb_ptr->task_stkptr = stkptr;
	cur_tcb_ptr->task_baseptr = baseptr;

	/* save all the floating point registers in the system */
	save_fp_regs ( cur_tcb_ptr->fp_save_area );

	/* Now save the task information in the kernel debugging structure.
	Note that this code can be commented out (along with similar code at
	the end of the kernel) with no ill effects in terms of kernel operation.
	In fact it will speed up the task switching.
	*/

	/* The task information is saved in a temporary location, and a check
	is made at the end of the kernel to see if there has been a task switch.
	If so then this temporary information is stored in the circular buffer
	of these structures.
	*/
	temp_kernel_store.taskname_ptr = cur_tcb_ptr->task_name_ptr;
	temp_kernel_store.static_priority = cur_tcb_ptr->static_priority;
	temp_kernel_store.dynamic_priority = cur_tcb_ptr->dynamic_priority;
	temp_kernel_store.clock_cnt = rt_clock;


	/* Now enter a case statement which uses the entry_type component of the
	kcb as the selector to begin execution of the correct handling routine.
	*/

	switch ( kcb.entry_type ) {
		case WAIT_ENTRY :
			temp_kernel_store.reason_ptr = wait_entry;
			temp_kernel_store.semaphore_num = kcb.semaphore_index;
			wait_handler ( );
			break;
		case RESCHEDULE_ENTRY :
			temp_kernel_store.reason_ptr = reschedule_entry;
			reschedule_handler ( );
			break;
		case TIME_SLICE_ENTRY :
			temp_kernel_store.reason_ptr = time_slice_entry;
			time_slice_handler ( );
			break;
		case CREATE_TASK_STK_ENTRY :
			temp_kernel_store.reason_ptr = create_task_entry;
			break;
		case PREEMPTIVE_ENTRY :
			temp_kernel_store.reason_ptr = preemptive_entry;
			temp_kernel_store.semaphore_num = kcb.semaphore_index;
			preemptive_schedule_handler ( );
			break;
		case CHGE_TASK_PRI_ENTRY :
			temp_kernel_store.reason_ptr = change_priority_entry;
			chge_task_pri_handler ( );
			break;
	} /* switch */

	/* After returning from one of the kernel entry handlers the task pointed
	to by the central_table(CURRENT_TASK) is the next task to run.  The
	dispatcher simply reinstates the stored stack pointer and stack base by
	executing the normal exit from an interrupt routine.
	*/


	/* The following bit is for debugging purposes only. Allows data to be
	written directly out onto the video screen.
	*/

//	dynamic_priority = tcb [1]->dynamic_priority;
//	static_priority = tcb [1]->static_priority;

//	base_video_ptr = MK_FP(0xb800, 0x0000);

	/* now format the info */
//	sprintf (strg1, "D%d", dynamic_priority);
//	sprintf (strg2, "S%d", semaphore [uinAlarmCritSecSema]->semaphore_value);


	/* Now print onto the screen */
//	i = 0;
//	while (strg2 [i] != NULL) {
//		*base_video_ptr = strg2 [i];
//		i++;
//		base_video_ptr++;
//		*base_video_ptr = 07;
//		base_video_ptr++;
//	} /* while */


//	base_video_ptr = MK_FP(0xb800, 0x00a0);
//	i = 0;
//	while (strg1 [i] != NULL) {
//		*base_video_ptr = strg1 [i];
//		i++;
//		base_video_ptr++;
//		*base_video_ptr = 07;
//		base_video_ptr++;
//	} /* while */

	/* Dispatcher */

	cur_tcb_ptr = ( task_control_block* ) central_table [ CURRENT_TASK ];

	/* Now check to see if there has been a task switch - if so then
	write the extra information into the temp_kernel_store structure and then
	save it into the circular buffer of structures, else do nothing.
	*/
	if (temp_kernel_store.taskname_ptr != cur_tcb_ptr->task_name_ptr) {
		/* Has been a task switch so save the information */
		temp_kernel_store.nxt_taskname_ptr = cur_tcb_ptr->task_name_ptr;
		temp_kernel_store.nxt_static_priority = cur_tcb_ptr->static_priority;
		temp_kernel_store.nxt_dynamic_priority = cur_tcb_ptr->dynamic_priority;
		temp_kernel_store.count_num = ++count_num;

		/* Now copy the data into the circular buffer structure */
		kernel_debug_info [kernel_debug_head++] = temp_kernel_store;

		if (kernel_debug_head >= SIZE_DEBUG_BUFFER) {
			kernel_debug_head = 0;
		} /* if */
	} /* if */



	/* now restore the floating point registers */
	restore_fp_regs ( cur_tcb_ptr->fp_save_area );

	_SP = cur_tcb_ptr->task_stkptr;
	_BP = cur_tcb_ptr->task_baseptr;
	_SS = cur_tcb_ptr->task_stkbase;

}   /* end of kernel */





/*------------------------------------------------------------------------*/




/*
===========================================================================
|
| add_timer_active_q
|
| This function places a timer on the queue of active timers. During this
| process the timing related contents of the timer structure have to be
| initialised with the variables passed in.
|
| The active time queue is a delta queue in relation to the time count.  This
| means that the period before timeout for any timer is the accumulation of
| all the time counts for the timers preceding the timer in question plus the
| time count for the timer in question.  The initial time count is the total
| timeout time for the timer.  Therefore when a timer is placed on the time
| queue it is placed in the correct location so that the accumulated deltas
| will give the correct total timeout time.  Depending on the position that
| the timer is placed this may involve altering the next timer delta count as
| well.
|
| Parameters : - timer type - single shot or repetitive.
|              - initial time count
|              - pointer to timeout handler
|              - pointer to timeout handler data
|
| Entry via  : - task$to$sleep procedure and alarm set routines
|
==============================================================================
*/

static void add_timer_active_q ( timer_struc* timer,
									unsigned char timer_type,
									unsigned long init_time_count,
									void ( *timeout_handler ) ( ),
									void* data_ptr ) {

	timer_struc* temp_timer;
	timer_struc* otemp_timer = NULL;
	unsigned long accumulated_time;
    char placed_timer = FALSE;

    if ( timer != NULL ) {
        timer->status = ACTIVE;
        timer->type = timer_type;
        timer->init_time_cnt = init_time_count;
		timer->timer_handler = (void (*)(void *))timeout_handler;
        timer->timer_handler_data = data_ptr;

        /* now link the timer into the correct part of the active queue. Start
        searching from the top of the queue to find the correct spot.
        */
        temp_timer = ( timer_struc* ) central_table [ bgn_active_time_q ];

        if ( temp_timer == NULL ) {
            /* active queue is empty so put on top of queue */
            timer->prev_timer_ptr = NULL;
            timer->next_timer_ptr = NULL;
            timer->delta_time_cnt = init_time_count;
            ( timer_struc* )central_table [ bgn_active_time_q ] = timer;
            ( timer_struc* )central_table [ end_active_time_q ] = timer;
        } /* if */
        else {
            /* have to search queue to find correct spot */
            accumulated_time = temp_timer->delta_time_cnt;

            while ( ( temp_timer != NULL ) && ( !placed_timer ) ) {
                if ( accumulated_time >= timer->init_time_cnt ) {
                    /* place timer in current position */
                    timer->delta_time_cnt = accumulated_time -
                                        temp_timer->delta_time_cnt;
                    timer->delta_time_cnt = timer->init_time_cnt -
                                            timer->delta_time_cnt;
                    timer->prev_timer_ptr = temp_timer->prev_timer_ptr;
					timer->next_timer_ptr = temp_timer;
                    if ( otemp_timer != NULL )
                        otemp_timer->next_timer_ptr = timer;
                    temp_timer->prev_timer_ptr = timer;
                    temp_timer->delta_time_cnt = accumulated_time -
                                                timer->init_time_cnt;
                    placed_timer = TRUE;

                    /* see if at the beginning of the active queue */
                    if ( timer->prev_timer_ptr == NULL )
                        ( timer_struc* )central_table [ bgn_active_time_q ]
                                                                = timer;
                } /* if */
                else {
                    /* update temp_timer and increment accumulated_time */
                    otemp_timer = temp_timer;
                    temp_timer = temp_timer->next_timer_ptr;
                    if ( temp_timer != NULL )
                        accumulated_time += temp_timer->delta_time_cnt;
                } /* else */
            } /* while */

            if ( !placed_timer ) {
                /* have reached the end of the queue so add timer here */
                timer->delta_time_cnt = timer->init_time_cnt -
                                                    accumulated_time;
                timer->prev_timer_ptr = otemp_timer;
                timer->next_timer_ptr = NULL;
                otemp_timer->next_timer_ptr = timer;
                ( timer_struc* ) central_table [ end_active_time_q ] = timer;
            } /* if */
        } /* else */
    } /* if */

} /* end of add_timer_active_q */





/*------------------------------------------------------------------------*/





/*
============================================================================
|
| add_timer_inactive_q
|
| This function adds a timer to the queue of inactive timers.  Timers are
| generic structures therefore a timer is simply added to the end of the
| queue.
|
| Parameters : - pointer to the timer to be added to inactive queue
|
| Entry via  : -
|
============================================================================
*/

static void add_timer_inactive_q ( timer_struc* timer_ptr ) {

    timer_struc* end_q_ptr;

    end_q_ptr = ( timer_struc* )central_table [ end_inactive_time_q ];

    if ( end_q_ptr == NULL ) {
        /* the inactive time queue is currently empty */
		( timer_struc* )central_table [ bgn_inactive_time_q ] = timer_ptr;
        ( timer_struc* )central_table [ end_inactive_time_q ] = timer_ptr;
        timer_ptr->prev_timer_ptr = NULL;
        timer_ptr->next_timer_ptr = NULL;
        timer_ptr->status = INACTIVE;
    } /* if */
    else {
        /* link onto the end of exisiting queue */
        end_q_ptr->next_timer_ptr = timer_ptr;
        timer_ptr->prev_timer_ptr = end_q_ptr;
        timer_ptr->next_timer_ptr = NULL;
        timer_ptr->status = INACTIVE;
        ( timer_struc* )central_table [ end_inactive_time_q ] = timer_ptr;
    } /* else */
} /* end add_timer_inactive_q */





/*-------------------------------------------------------------------------*/





/*
============================================================================
|
| remove_timer_inactive_q
|
| This function simply removes a timer from the top of the inactive timer
| queue.  If there is no timer in the queue then a NULL pointer is returned.
|
| Parameters : - none
|
| Entry via  : -
|
============================================================================
*/

static timer_struc* remove_timer_inactive_q ( void ) {

    timer_struc* temp_timer = NULL;

    if ( central_table [ bgn_inactive_time_q ] != NULL ) {
        temp_timer = ( timer_struc* ) central_table [ bgn_inactive_time_q ];
        ( timer_struc* ) central_table [ bgn_inactive_time_q ] =
                                                temp_timer->next_timer_ptr;
        if ( central_table [ bgn_inactive_time_q ] == NULL ) {
            central_table [ end_inactive_time_q ] = NULL;
        } /* if */
        else {
            /*--- queue is not empty so make sure that the first timer
            structure in the queue has its prev_timer_ptr set to NULL.
            ---*/
            temp_timer->next_timer_ptr->prev_timer_ptr = NULL;
        } /* else */
    } /* if */

    return temp_timer;
} /* end of remove_timer_inactive_q */





/*-------------------------------------------------------------------------*/





/*
============================================================================
|
| remove_timer_active_q
|
| This is a function which allows one to remove a timer from the queue of
| active timers.  The timer can be removed from the queue at any position
| in the queue.  This routine is used generally to remove timers from the
| queue before they have timed out.  When a timer is removed the link
| pointers have to be updated and the delta_time_cnt of the following timer
| altered appropriately.  A pointer to the timer removed is returned.
|
| Parameters : - pointer to a timer structure
|
| Entry via  : - multiple places
|
============================================================================
*/

static timer_struc* remove_timer_active_q ( timer_struc* timer_ptr ) {

    if ( ( central_table [ bgn_active_time_q ] ==
            central_table [ end_active_time_q ] ) &&
            ( timer_ptr->status == ACTIVE ) ) {
        /* only one timer on the queue */
        timer_ptr = ( timer_struc* ) central_table [ bgn_active_time_q ];
        central_table [ bgn_active_time_q ] = NULL;
        central_table [ end_active_time_q ] = NULL;
        return timer_ptr;
    } /* if */

    if ( ( timer_struc* ) central_table [ bgn_active_time_q ] == timer_ptr ) {
        /* timer is on the top of the queue and there is more than one entry
        in the queue
        */
        ( timer_struc* ) central_table [ bgn_active_time_q ] =
                                                    timer_ptr->next_timer_ptr;
        timer_ptr->next_timer_ptr->delta_time_cnt += timer_ptr->delta_time_cnt;
        timer_ptr->next_timer_ptr->prev_timer_ptr = NULL;
        return timer_ptr;
    } /* if */

    if ( ( timer_struc* ) central_table [ end_active_time_q ] == timer_ptr ) {
        /* timer is on the end of the queue */
        ( timer_struc* ) central_table [ end_active_time_q ] =
                                                    timer_ptr->prev_timer_ptr;
        timer_ptr->prev_timer_ptr->next_timer_ptr = NULL;
        return timer_ptr;
    } /* if */

    /* timer should be somewhere in the middle of the active time queue. Check
    to see if the timer is active as this will ensure that it is on the active time
    queue and not on the inactive time queue.
    */
    if ( timer_ptr->status == ACTIVE ) {
        timer_ptr->prev_timer_ptr->next_timer_ptr = timer_ptr->next_timer_ptr;
        timer_ptr->next_timer_ptr->prev_timer_ptr = timer_ptr->prev_timer_ptr;
        timer_ptr->next_timer_ptr->delta_time_cnt +=
                                               timer_ptr->delta_time_cnt;
        return timer_ptr;
    } /* if */

    return NULL;

} /* end of remove_timer_active_q */





/*---------------------------------------------------------------------------*/





/*
=============================================================================
|
| dec_timers
|
| This function is called on every tick entry to the operating system.  The
| function firstly checks to see if the delta_time_cnt value of the time on
| the top of the active time queue is zero.  If so then :
|   - remove the timer from the time queue
|   - if timer is repetitive then place back at the appropriate place in
|     in the time queue else put it onto the inactive time queue
|   - call the timeout handler
|   - check if the next timer on the queue has got a delta of zero.  If so
|   - then go back to the first point on this list and repeat.
|
|   - enter at this point if all timed out timers have been removed from the
|     time queue.  If there is a timer on the time queue then decrement the
|     timer and return.
|
| Parameters : - none
|
| Entry via  : - tick interrupt routine
|
==============================================================================
*/

static void dec_timers ( ) {

    timer_struc* timer_ptr;
    void ( * func ) ( void* ptr );

    /* check to see if the timer at the top of the queue has timed out. */
    while ( ( ( ( timer_struc* ) central_table [ bgn_active_time_q ] )
                    ->delta_time_cnt == 0 ) &&
                       ( central_table [ bgn_active_time_q ] != NULL ) ) {

        /* timer has timed out so remove timer from the top of the queue.
        Check to see if the timer is a repetitive timer or a single shot timer.
        If reprtitive timer then place the timer back into the active queue,
        else place the timer in the inactive queue if single shot.
        */
        timer_ptr = ( timer_struc* ) central_table [ bgn_active_time_q ];
        timer_ptr = remove_timer_active_q ( timer_ptr );
        if ( timer_ptr->type == REPETITIVE )
            add_timer_active_q ( timer_ptr, timer_ptr->type,
                        timer_ptr->init_time_cnt, (void(*)())timer_ptr->timer_handler,
                        timer_ptr->timer_handler_data );
        else /* single shot timer */
            add_timer_inactive_q ( timer_ptr );

        /* set the sig_entry_from_dec_timers in case the handler routine is
        going to do a signal.  This flag prevents the signal handler from
        doing a preemptive task switch at this stage.
        */
        sig_entry_from_dec_timers = TRUE;

        /* now execute the handling routine */
        func = timer_ptr->timer_handler;
        ( * func ) ( timer_ptr->timer_handler_data );
    } /* while */

    /* decrement the current delta time count if there is a timer on the
    queue
    */
    if ( central_table [ bgn_active_time_q ] != NULL )
        ( ( timer_struc* )( central_table [ bgn_active_time_q ] ) )->
                                                        delta_time_cnt--;

    /* clear the sig_entry_from_dec_timers flag as it will have already
    done its job at this stage.
    */
    sig_entry_from_dec_timers = FALSE;
} /* end of dec_timers */





/*-------------------------------------------------------------------------*/





/****************************************************************************/
/*                                                                          */
/*                                                                          */
/*                                                                          */
/*                              UNOS_3 MODULE                               */
/*                                                                          */
/*                                   by                                     */
/*                                                                          */
/*                              Robert Betz                                 */
/*        Department of Electrical Engineering and Computer Science         */
/*                         University of Newcastle                          */
/*                               Australia                                  */
/*                                                                          */
/*                           ( Copyright 1989 )                             */
/*                                                                          */
/*                                                                          */
/*                                                                          */
/****************************************************************************/



/*

HISTORY

This module was formed from the initial single module UNOS to allow the
interactive environment of Turbo C to be used.  At a later stage this module
and the others which comprise UNOS can be formed into a single module.

*/


/*

DESCRIPTION

This module is a sub-part of UNOS.  It is "included" into the UNOS_1
module to form a complete system. The module primarily contains the user 
interface routines to the kernel.

*/




/****************************************************************************/
/*                                                                          */
/*                           USER INTERFACE TO KERNEL                       */
/*                                                                          */
/****************************************************************************/





/*
=============================================================================
|
| start_timer
|
| This routine is the user interface to start a timer for whatever purpose.
| This procedure preserves the interrupt status of the calling routine meaning
| that it can be called from an interrupt routine. The routine gets an existing
| timer from the inactive time queue.
|
| Returns a pointer to the timer structure.
|
| Parameters : - timer_type - repetitive or single shot
|              - initial timer count
|              - pointer to the timeout handler
|              - pointer to the timeout handler data structure
|
| Entry via  : - multiple places
|
==============================================================================
*/

timer_struc* start_timer ( unsigned char timer_type,
                           unsigned long init_time_count,
						   void ( * timeout_handler ) ( int * ),
						   void * data_ptr ) {

    char int_status;
    timer_struc* timer_ptr;

    int_status = return_interrupt_status ();

    disable ();
    timer_ptr = remove_timer_inactive_q ();
    if ( timer_ptr != NULL )
        add_timer_active_q ( timer_ptr, timer_type,
            init_time_count,( void (*)())timeout_handler, data_ptr );
    if ( int_status )
        enable ();

        return timer_ptr;
} /* end of start_timer */





/*--------------------------------------------------------------------------*/





/*
============================================================================
|
| reset_timer
|
| Takes a timer currently on the time queue and resets its count to the
| initial value.  This involves firstly taking the timer out of the active
| timer queue and then putting it back into the time queue.  This is required
| because the reset timer will in all probability sit at a different position
| on the time queue.
|
| If the reset is successful then a pointer to the timer is returned.
|
| Parameters : - pointer to the timer to be reset
|
| Entry via  : - multiple places
|
=============================================================================
*/

timer_struc* reset_timer ( timer_struc* timer_ptr ) {

    char int_status;
    timer_struc* temp_ptr;

    int_status = return_interrupt_status ();
    disable ();
    temp_ptr = remove_timer_active_q ( timer_ptr );
    if ( temp_ptr != NULL )
        add_timer_active_q ( timer_ptr, timer_ptr->type,
                             timer_ptr->init_time_cnt,
							 (void(*)())timer_ptr->timer_handler,
                             timer_ptr->timer_handler_data );
    if ( int_status )
        enable ();

    return temp_ptr;
} /* end of reset_timer */





/*--------------------------------------------------------------------------*/





/*
=============================================================================
|
| stop_timer
|
| This function takes a timer off the active time queue and places it onto
| the inactive timer queue.  The status of the timer is changed to inactive.
|
| Parameters : - pointer to the timer of interest
|
| Entry via  : - multiple places
|
=============================================================================
*/

timer_struc* stop_timer ( timer_struc* timer_ptr ) {

    char int_status;
    timer_struc* temp_ptr;

    int_status = return_interrupt_status ();
    disable ();
    temp_ptr = remove_timer_active_q ( timer_ptr );
    if ( temp_ptr != NULL )
        add_timer_inactive_q ( timer_ptr );
    if ( int_status )
        enable ();

    return temp_ptr;
} /* end of stop_timer */





/*-----------------------------------------------------------------------*/





/*
===========================================================================
|
| create_timer
|
| The purpose of this function is to allocate storage for a timer structure
| on the heap.  If the allocate is successful then a pointer to the
| structure is returned, else a NULL pointer is returned.
|
| If the timer is created successfully then it is added to the inactive
| timer queue.
|
| Parameters : - none
|
| Entry via : - UNMAIN.C or user tasks.
|
===========================================================================
*/

timer_struc* create_timer ( void ) {

    timer_struc* timer;
    char int_status;

    int_status = return_interrupt_status ( );
    disable ( );
	timer = ( timer_struc * )umalloc ( sizeof ( timer_struc ) );
    if ( timer != NULL )
        add_timer_inactive_q ( timer );
    if ( int_status )
        enable ( );
    return timer;
} /* end of create_timer */





/*--------------------------------------------------------------------------*/





/*
============================================================================
|
| timed_wait
|
| The purpose of this routine is to implement a timed wait.  This is
| different from the normal wait in that if the task becomes blocked it will
| only be blocked for a maximum time as determined by the wait time. The
| task can obviously become unblocked earlier than this due to a signal
| on the semaphore from another task. The means by which the wait has been
| terminated can be determined by the return code received.  If the return
| code is zero then the wait has not timed out and has been terminated by
| a signal. However if the return code is not zero then some problem has
| occurred. The nature of the problem can be determined by the number of the
| return code. If the number is 1 then the terminate has occurred due to
| a timeout on the wait. If the number is 2 then the wait has been
| terminated due to the absence of a timer being available to implement the
| timed wait. It is up to the user routine to do something about these
| problems.
|
| Parameters : - semaphore number to wait on
|              - time to wait in clock ticks.
|
| Entry via  : - multiple places in the user code.
|
===========================================================================
*/

int timed_wait ( unsigned int sema_num, unsigned long timeout_value ) {

    timer_struc* timer_ptr;
    task_control_block* tcb_ptr;
    char int_status;

    int_status = return_interrupt_status ( );
    disable ( );
    tcb_ptr = ( task_control_block* )central_table [ CURRENT_TASK ];

    /* the first step is to get a timer from the inactive time queue and
    start it. It is assumed that the timer has already been created.
    */
	timer_ptr = start_timer ( SINGLE_SHOT, timeout_value,(void (*)(int *)) timed_wait_handler,
							( void* )tcb_ptr );

	if ( timer_ptr == NULL ) {
        if ( int_status )
            enable ( );
        return 2;   /* no timer available */
    } /* if */

    /* set up the components in the tcb with the correct data */
    tcb_ptr->timer_ptr = timer_ptr;
    tcb_ptr->timeout_flag = FALSE;

    /* now carry out the actual wait operation by doing a normal wait */
	wait ( sema_num );

    /* have returned from the wait so now check to see what caused the
    return. This information is contained in the timeout_flag contained
    in the tcb
    */
    tcb_ptr->timer_ptr = NULL;

    if ( int_status )
        enable ( );
    if ( tcb_ptr->timeout_flag )
        return 1;   /* semaphore timed out */
    else
        return 0;   /* return due to a signal */
} /* end of timed_wait */





/*--------------------------------------------------------------------------*/





/*
=============================================================================
|
| wait
|
| This procedure implements the wait synchronization primitive.  The value passed
| in is the index into the semaphore array.  The routine checks to see if the
| semaphore value has reached zero.  If so then the kernel is entered proper to
| put the task on the appropriate semaphore queue and a new task is scheduled.
| If the semaphore value has not reached zero then the value is simply
| decremented and the routine is exited.   This serves to keep the overhead of
| entering the kernel proper to a minimum.
| Note that the routine preserves the interrupt status upon entry.
|
|   Parameters : - semaphore number
|   Entry via  : - multiple places
|
=============================================================================
*/

void wait ( unsigned int semaphore_num ) {

	char int_status;
	task_control_block* cur_tcb_ptr;
	timer_struc *timer_ptr;

	int_status = return_interrupt_status ( );
	disable ( );

	if ( semaphore [ semaphore_num ]->semaphore_value > 1 ) {
		/* semaphore value not zero or one so decrement. */
		semaphore [ semaphore_num ]->semaphore_value--;

		/* now make sure that the routine is not being called from the
		timed_wait routine. If it has been then the timer for the task
		has to be removed from the active timer queue and placed back in the
		inactive timer queue. The tcb_ptr must be reset for the situation
		where the task is no longer claiming a timer.
		*/
		cur_tcb_ptr = ( task_control_block* )central_table [ CURRENT_TASK ];
		if ( cur_tcb_ptr->timer_ptr != NULL ) {
			/* task claiming a timer */
			timer_ptr = remove_timer_active_q ( cur_tcb_ptr->timer_ptr );
			add_timer_inactive_q ( timer_ptr );
			/* indicate that timer has not timed out */
			cur_tcb_ptr->timeout_flag = FALSE;
		}  /* if */
	} /* if */
	else {
		if ( semaphore [ semaphore_num ]->semaphore_value == 1 ) {
			cur_tcb_ptr = ( task_control_block* )central_table [ CURRENT_TASK ];

			/* now check to see if the semaphore is a SYNC_SEMA or a
			CRITICAL_SECTION_SEMA. If a SYNC_SEMA then the calling task
			should not claim the semaphore, but if CRITICAL_SECTION_SEMA then
			claim the semaphore.
			*/
			if (semaphore [ semaphore_num ]->semaphore_type ==
											CRITICAL_SECTION_SEMA) {
				cur_tcb_ptr->num_semas_claimed++;
				semaphore [ semaphore_num ]->tcb_ptr = cur_tcb_ptr;
			} /* if */

			semaphore [ semaphore_num ]->semaphore_value--;
			/* now make sure that the routine is not being called from the
			timed_wait routine. If it has been then the timer for the task
			has to be removed from the active timer queue and placed back in
			the inactive timer queue. The tcb_ptr must be reset for the
			situation where the task is no longer claiming a timer.
			*/
			if ( cur_tcb_ptr->timer_ptr != NULL ) {
				/* task claiming a timer */
				timer_ptr = remove_timer_active_q ( cur_tcb_ptr->timer_ptr );
				add_timer_inactive_q ( timer_ptr );
				/* indicate that timer has not timed out */
				cur_tcb_ptr->timeout_flag = FALSE;
			}  /* if */
		} /* if */
		else {
			/* semaphore value is equal to zero so enter the kernel to put
			the task to sleep and reschedule another task
			*/

			/* Set up the entry type. */
			kcb.entry_type = WAIT_ENTRY;

			/* Index into semaphore table. */
			kcb.semaphore_index = semaphore_num;

			/* Process number is irrelevant in this case. */
			geninterrupt ( kernel_entry );
		} /* else */
	} /* else */

	if ( int_status )
		enable ( );

}   /* end of wait */





/*----------------------------------------------------------------------------*/





/*
==============================================================================
|
| _signal
|
| This procedure implements the signal ( or send ) synchronization primitive.
| If no task is waiting or has claimed the semaphore then regardless of the
| value of the semaphore its value is simply incremented by the value in
| the semaphore_multiplier. The other situations which must be handled are:-
|
|   (i)     a task is claiming the semaphore but there are no tasks blocked
|   (ii)    a task is claiming the semaphore and tasks are blocked
|   (iii)   no task is claiming the semaphore but tasks are blocked.
|
| Only when necessary is the kernel proper entered - i.e., when a task 
| switch is required. Note the use of the possible_premption flag which is
| used to delay a preemptive kernel call in the case of the signal routine
| being entered from the dec_timers routine.
|
| Note that the state of the interrupts is preserved by this routine.
|
| The name of this routine is preceded with an underscore so that it will
| not conflict with the signal name used in standard C.
|
|   Parameters : - semaphore number
|   Entry via  : - multiple places
|
=============================================================================
*/

void _signal ( unsigned int semaphore_num ) {

	void ( *handler_semaphore_q ) ( int queue_op );
	char int_status;
	char kernel_entry_flag = FALSE;

	int_status = return_interrupt_status ( );
	disable ( );

	if ( semaphore [ semaphore_num ]->tcb_ptr != NULL ) {
		/* clear the possible_preemption flag so that the state is correct
		if this entry is not from the dec_timers routine
		*/
		possible_preemption = FALSE;

		/* have entered this section of the code if a task is currently
		claiming the semaphore. Note that this does not necessarily mean
		that a task is blocked on the semaphore though. If the number of
		semaphores claimed by the claiming task is one (i.e., the task is
		only claiming this semaphore) then reset the components of the task
		control block related to claiming the semaphore. Always decrement
		the number of semaphores claimed.
		*/

		if ( semaphore [ semaphore_num ]->tcb_ptr->num_semas_claimed == 1 ) {
			semaphore [ semaphore_num ]->tcb_ptr->dynamic_priority =
					semaphore [ semaphore_num ]->tcb_ptr->static_priority;
			semaphore [ semaphore_num ]->tcb_ptr->num_semas_claimed--;
		} /* if */
		else
			semaphore [ semaphore_num ]->tcb_ptr->num_semas_claimed--;


		/* now check to see if there is a task blocked on the queue. If
		there isn't then simply increment the semaphore value by the
		multiplier value and then set a flag to indicate that a kernel entry
		is not necessary. Set the tcb_ptr component of the semaphore
		structure to zero to indicate that the semaphore is not being
		claimed. Note that the latter has to be done within this if
		statement because the combination of a semaphore value = 0 and
		semaphore [ ? ].tcb_ptr = NULL means that the semaphore is a
		synchroniztion semaphore. This is specially treated in the semaphore
		queue handler.
		*/
		if ( ( task_control_block* )central_table [ semaphore [ semaphore_num ]
									->semaph_bgn_q_index ] == NULL ) {
			semaphore [ semaphore_num ]->semaphore_value +=
						semaphore [ semaphore_num ]->semaphore_multiplier;

			/* now make sure that the semaphore structure does not think that it
			is being claimed any more by the calling task.
			*/
			semaphore [ semaphore_num ]->tcb_ptr = NULL;

			kernel_entry_flag = FALSE;
		} /* if */
		else {
			/* must be a task blocked on the queue so dequeue it and put it
			on the appropriate priority queue. The task which causes the
			semaphore value to go to zero is the task which will be claiming
			the semaphore. The tcb_ptr part of the semaphore structure is
			set up appropriately.
			*/
			kernel_entry_flag = TRUE;
			kcb.semaphore_index  = semaphore_num;
			handler_semaphore_q = (void(*)(int))semaphore [ semaphore_num
												]->semaph_queue_handler;
			( *handler_semaphore_q ) ( DEQUEUE );
		} /* else */

		/* if the kernel_entry_flag is set and the signal entry has not
		occurred from a timer expiration then do a preemptive kernel entry
		to ascertain which is the highest priority task to run.
		*/
		if ( kernel_entry_flag && ( !sig_entry_from_dec_timers ) )
			preemptive_schedule ( );
	}
	else {
		/* enter here if the tcb_ptr component of the semaphore structure is
		a NULL.  Note that this does not mean that the semaphore does not
		contain any blocked tasks.
		*/
		possible_preemption = FALSE;

		if ( ( task_control_block* )central_table [ semaphore [
						semaphore_num ]->semaph_bgn_q_index ] != NULL ) {
			/* must be a task blocked on the queue so dequeue it and put it
			on the appropriate priority queue. The task which causes the
			semaphore value to go to zero is the task which will be claiming
			the semaphore. The tcb_ptr part of the semaphore structure is
			set up appropriately.
			*/
			kcb.semaphore_index  = semaphore_num;
			handler_semaphore_q = (void (*)(int))semaphore [ semaphore_num
											   ]->semaph_queue_handler;
			( *handler_semaphore_q ) ( DEQUEUE );

			/* if the kernel_entry_flag is set and the signal entry has not
			occurred from a timer expiration then do a preemptive kernel entry
			to ascertain which is the highest priority task to run.
			*/
			if ( !sig_entry_from_dec_timers )
				preemptive_schedule ( );
		} /* if */
		else {
			/* no blocked or claiming tasks so increment the semaphore */
			semaphore [ semaphore_num ]->semaphore_value +=
						semaphore [ semaphore_num ]->semaphore_multiplier;
		} /* else */
	} /* else */

	if ( int_status )
		enable ( );

}   /* end of _signal */





/*-------------------------------------------------------------------------*/





/*
============================================================================
|
| rtn_current_task_num
|
| This function can be called from any task and it returns the task number of the
| currently executing task.
|
|   Parameters : - none
|
|   Entry via :  - any currently executing task
|
============================================================================
*/

 unsigned int rtn_current_task_num ( void ) {

    unsigned int cur_tsk_num;
    task_control_block* cur_tcb_ptr;
    char int_status;

    int_status = return_interrupt_status ( );
    disable ( );
    cur_tcb_ptr = (tsk_ctrl_blk *)central_table [ CURRENT_TASK ];
    cur_tsk_num = cur_tcb_ptr->task_num;
    if ( int_status )
        enable ( );

    return cur_tsk_num;

}   /* end of rtn_current_task_num */





/*-------------------------------------------------------------------------*/





/*
============================================================================
|
| rtn_current_task_name_ptr
|
| This function can be called from any task and it returns the task name pointer
| of the currently executing task.
|
|   Parameters : - none
|
|   Entry via :  - any currently executing task
|
============================================================================
*/

char *rtn_current_task_name_ptr ( void ) {

	char *cur_tsk_name_ptr;
	task_control_block* cur_tcb_ptr;
	char int_status;

	int_status = return_interrupt_status ( );
	disable ( );
	cur_tcb_ptr = (tsk_ctrl_blk *)central_table [ CURRENT_TASK ];
	cur_tsk_name_ptr = cur_tcb_ptr->task_name_ptr;
    if ( int_status )
        enable ( );

	return cur_tsk_name_ptr;

}   /* end of rtn_current_task_name_ptr */





/*-------------------------------------------------------------------------*/





/*
============================================================================
|
| reschedule
|
| This routine sets up the kcb for an entry to the kernel which will cause a
| software reschedule to another task.  This software reschedule is exactly
| the same as if a time slice induced reschedule had occurred except for one 
| difference. The task which requested the reschedule will not run after
| leaving the kernel even if it is still the highest priority task in the
| system. If this is not the desired behaviour then see preemptive_schedule
| below. 
| 
| Note that the interrupt status of the calling task is preserved.
|
|   Parameters : - none
|   Entry via  : - multiple places
|
============================================================================
*/

void reschedule ( void ) {

    char int_status;

    int_status = return_interrupt_status ( );
    disable ( );
    kcb.entry_type = RESCHEDULE_ENTRY;
    geninterrupt ( kernel_entry );
    if ( int_status )
        enable ( );

}   /* end of reschedule */





/*------------------------------------------------------------------------*/





/*
==========================================================================
|
| preemptive_schedule
|
| This routine is entered if the user wishes to carry out a preemptive
| schedule.  It sets up the kcb for this type of entry and then enters
| the kernel.
|
| A premptive_schedule differs from a reschedule in that it may or may
| not cause a reschedule, whereas a reschedule always runs the scheduler
| and will never return with the same task running even if it is the only
| task in the system which is runnable ( except for the nulltsk ).  A
| preemptive reschedule only occurs if there is a higher priority task
| than the current task running.
|
| Note that the interrupt status of the calling task is preserved allowing
| this routine to be called from within interrupt routines
|
|   Parameters : - none
|   Entry via  : - multiple places
|
===========================================================================
*/

void preemptive_schedule ( void ) {

    char int_status;

    int_status = return_interrupt_status ( );
    disable ( );
    kcb.entry_type = PREEMPTIVE_ENTRY;
    geninterrupt ( kernel_entry );
    if ( int_status )
        enable ( );
} /* end of preemptive_schedule */





/*--------------------------------------------------------------------------*/





/*
=============================================================================
|
| stop_time_slice
|
| This routine disables the time slice entry. This is achieved by setting a
| flag so that the operating system proper is not entered after the time slice
| number of clock ticks.  The clock tick routine still runs.
|
|   Parameters : - none
|
|   Entry via :  - multiple places
|
=============================================================================
*/

void stop_time_slice ( void ) {

    char int_status;

    int_status = return_interrupt_status ( );
    disable ( );
    time_slice_flag = FALSE;
    if ( int_status )
        enable ( );
}   /* end of stop_time_slice */





/*--------------------------------------------------------------------------*/





/*
=============================================================================
|
| start_time_slice
|
| This routine enables the time slice interrupt.  This is done by setting
| the time_slice_flag to true.  This flag is checked in the tick routine to
| see whether a time slice shall be carried out after the enter_kernel_value
| is reached.
|
|   Parameters : - none
|
|   Entry via :  - multiple places
|
=============================================================================
*/

void start_time_slice ( void ) {

    char int_status;

    int_status = return_interrupt_status ( );
    disable ( );
    time_slice_flag = TRUE;
    if ( int_status )
        enable ( );

}   /* end of start_time_slice */





/*--------------------------------------------------------------------------*/





/*
============================================================================
|
| tick
|
| The tick routine provides the basic timing for UNOS.  This routine is
| entered upon an interrupt from the main system timer.  It should be noted
| that the tick routine entry can be at a different frequency to the kernel
| time slice entry.  This is done so that the time resolution of the system
| timers is independent of the time slice frequency. For example, it is
| feasible to have a time resolution of say 1 msec for sleeping tasks and a
| time slice of 20 msec.  There the user has independent control of timing
| accuracy and the operating system overhead.
|
| Upon the tick interrupt the tick routine is entered.  The first function
| carried out by the routine is to send an end of interrupt to the 8259
| interrupt controller.  This is done here so that the exit path from the
| routine becomes irrelevant.  The next task is to increment the UNOS real
| time clock variable ( rt_clock ) which is implemented as a unsigned double
| word binary number.  This variable is used by the kernel for task time bin
| testing and can be used by user tasks to determine elapsed time.  Next the
| time slice counter is incremented.  If this counter reaches the time slice
| value then a reschedule is required on this entry to the interrupt routine.
| However  before a time slice entry is to occur two other functions must be
| carried out : -
|
|   - the dec_timers routine is called.  This routine decrements the time
|   values in the timer data structures.  If an alarm goes to zero then the
|   timer handler ( whose address is stored in the timer structure ) is
|   executed.  If the timer type is repetitive then it is also immediately
|   put back onto the timer queue.  The timer handler routine would
|   generally execute a signal to cause make a task become runnable.
|   If the timer handler is urgent then it can be handled directly in the
|   interrupt routine.
|
| Assuming that control passes through the above mentioned routines without
| any entry to the kernel then conditional upon the time slice entry being
| enabled then the kcb is set up for a normal time slice entry.  The time
| slice counter is always reset inside the scheduler routine for a time slice
| entry ( see below for discussion of this process ).
|
| From the aboove discussion it can be deduced that a timer handler which
| does a signal could cause a pre-emptive tasks switch. This would be the
| situation if measures were not taken to prevent this from occurring. Two
| flags are used to prevent a pre-emptive taks switch from occurring from
| the timer handler level - the sig_entry_from_dec_timers flag indicates
| to the signal routines that they are being entered from a timer handler.
| This causes the signal routine to move any tcb which must be moved from
| a semaphore queue to a priority queue, but does not call the scheduler
| at this stage. This flag is required since several tasks could be
| signalled from a number of timers which have timed out, and to ensure
| that they all get onto the priority queues at the same time there should
| be no rescheduling. The other flag used is the possible_preemption flag.
| This flag is set in the signal handler to indicate that a tcb has been
| moved onto a priority queue. The flag is checked back in the tick
| function and if set then the kernel is entered from around the point that
| it would normally be entered in tick if there was a time slice entry.
|
| WARNING :
| Although the user kernel interface routines can be called from interrupt
| routines with the interrupt status preserved, it is not sensible to call
| some of them.  For example, if a 'wait' is executed from within an interrupt
| routine then the task which was being executed at the time that the
| interrupt occurred would become blocked, even though it most probably has
| nothing to do with the wait semaphore causing the blockage.  Furthermore, if
| the routine executing at the time of the interrupt is the routine which
| 'signals' on the semaphore that the interrupt routine waits on then a
| deadlock situation occurs.
|
|   Parameters : - none
|
|   Entry via  : - hardware timer interrupt
|
============================================================================
*/

void interrupt tick ( void ) {

    task_control_block *cur_tcb_ptr;

	/* end of interrupt for the 8259 interrupt controller */
    outportb ( I8259_INT_CTRL_ADD, I8259_EOI );

    /* increment the real time clock counter */
    rt_clock++;

    /* decrement the counts for system timers and execute handler if a
    timeout has occurred */
    dec_timers ( );

    /* now check whether a preemptive entry to the kernel should be made
    due to a timeout of one of the timers.
    */
    if ( possible_preemption ) {
        possible_preemption = FALSE;
        kcb.entry_type = PREEMPTIVE_ENTRY;
        geninterrupt ( kernel_entry );
    } /* if */

    /* now check whether a time slice entry to the kernel should be made.  This
    conditioned on time slicing being enabled and the ticks_to_go being
    equal to zero.
    */
    cur_tcb_ptr = ( task_control_block* )central_table [ CURRENT_TASK ];
    if ( ! (--( cur_tcb_ptr->ticks_to_go ) ) ) {
        cur_tcb_ptr->ticks_to_go = cur_tcb_ptr->ticks_before_kernel_entry;
        if ( time_slice_flag ) {
            kcb.entry_type = TIME_SLICE_ENTRY;
            geninterrupt ( kernel_entry );
        } /* if */
    } /* if */

} /* ticks */





/*-----------------------------------------------------------------------*/





/*
===========================================================================
|
| rtn_task_priority
|
| This function returns the priority of the task whose name is passed into
| the routine. If the task name is illegal then the routine returns 0xffff
| as the task priority.
|
| Parameters : - pointer to the task name
|
| Entry via  : - any user task
|
===========================================================================
*/

unsigned int rtn_task_priority ( char *task_name_ptr ) {

    unsigned int task_num, temp_task_priority;
    char int_status;

    /* firstly corrolate the task name with the task number */
    task_num = mail_exchange ( task_name_ptr );

    /* now check to see if the task name was a legal one */
    if ( task_num >= num_of_tasks ) {
        /* task name was illegal so return an illegal priority */
        return 0xffff;
    } /* if */

    int_status = return_interrupt_status ( );
    disable ( );

    /* now get the task priority */
    temp_task_priority = tcb [ task_num ]->static_priority;

    if ( int_status ) {
        enable ( );
    } /* if */

    return temp_task_priority;
} /* rtn_task_priority */





/*-------------------------------------------------------------------------*/





/*
=============================================================================
|
| change_task_priority
|
| This procedure allows a task to change the priority of any task (including
| itself). This is a seemingly simple procedure, however it is made more
| complicated in this instance by the presence of dynamic priorities.
|
| Dynamic priorities were included in UNOS so that problems associated with
| tasks of varying priority being blocked on a semaphore would be overcome.
| However it leads to a rather messy priority change procedure. Normally if
| the priority is changed it would involve modifying the priority component
| of the tcb and moving the task to the correct queue if it was on a priority
| queue. In the current implementation there are two components to the
| tcb related to priority - a static priority (which comforms to the priority
| in the normal case mentioned above) and the dynamic priority, which normally
| equals the static priority but under semaphore manipulation conditions may
| have a value greater than the static priority. All scheduling is done based
| on the dynamic priority.
|
| The problem arises with priority changes when a task has claimed a semaphore.
| In this case the dynamic priority could be greater than the static priority
| (meaning that a higher priority task has become blocked on the semaphore).
| If the priority of the task is to be lowered then the claiming task static
| priority is set to the new priority. However the dynamic priority change
| is not straight forward when the priority is being changed to a lower
| value and a semaphore is being claimed. Checks must be made to see how many
| semaphores the task is claiming - if > 1 then do not change the dynamic
| priority because the priority required by all but the last semaphore claimed
| cannot be determined (in the current implementation anyway). If number of
| semaphores claimed is equal to one then the priority of the task on the top
| of the semaphore queue needs to be checked - if < new priority then change
| the priority of the task.
|
| Whenever the dynamic priority is changed it means that a task must be moved
| on whatever queue it is on. If on a priority queue then it is placed in the
| priority queue associated with its new dynamic priority. If on a semaphore
| queue then it is repositioned in the semaphore queue. A check also has to
| be made to see if the priority of the claiming task has to be changed
| under this condition. If so then the change is made and the claiming task
| is repositioned in the priority queue.
|
| Because this routine can result in a task switch then it is not safe to
| call this routine from an anywhere in an interrupt routine except at the
| end.
|
| It should be noted that this routine caries out a kernel entry, and in
| most instances the scheduler will be called. Therefore the calling task
| may not be running after exit from the kernel.
|
| If the change in the task priority is successful then a TRUE is returned,
| else a FALSE is returned.
|
|   Parameters : - task number, new priority
|
|   Entry via  : - multiple places
|
============================================================================
*/

int change_task_priority ( char *task_name_ptr,
                                        unsigned char new_priority ) {

    char int_status;
    unsigned char task_num;

    /* now make the connection between the task name and the task number. */
    task_num = mail_exchange ( task_name_ptr );

    /* firstly check to see if the task number and the priority are legal
    values.
    */
    if ( ( task_num >= num_of_tasks ) || ( new_priority < 1 ) ||
                    ( new_priority > num_of_priorities ) ) {
        /* normally call an error handler here */
        return FALSE;
    } /* if */
    

    int_status = return_interrupt_status ( );
    disable ( );

    /* always change the static priority to the new priority */
    tcb [ task_num ]->static_priority = new_priority;
    
    /* now check to see if the kernel needs to be entered at all */
    if ( new_priority == tcb [ task_num ]->dynamic_priority ) {
        /* no need to enter the kernel since the task is already on the
        correct queue for its new priority.
        */
        if ( int_status )
            enable ( );
        return TRUE;
    } /* if */

    /* now set up the kernel control block for an entry to the kernel */
    kcb.entry_type = CHGE_TASK_PRI_ENTRY;
    kcb.task_number = task_num;
    kcb.new_priority = new_priority;

    geninterrupt ( kernel_entry );

    if ( int_status )
        enable ( );

    return TRUE;

}   /* end of change_task_priority */





/*-----------------------------------------------------------------------*/





/*
============================================================================
|
| alloc_tcb_table
|
| This function returns a pointer to an array of pointers to the tcb's. The
| array allocated is of a size to accommodate the maximum number of tasks
| allowed in the system. The values of the pointers are all initialised to
| have NULL values.
|
|   Parameters : - maximum number of tasks
|
|   Entry via  : - setup_os_data_structures
|
============================================================================
*/

static task_control_block **alloc_tcb_table ( unsigned int max_num_of_tsks ) {

    int i;

    tcb = ( task_control_block ** )ucalloc ( max_num_of_tsks,
                                        sizeof ( task_control_block * ) );
    if ( tcb != NULL ) {
        for ( i = 0; i < max_num_of_tsks; i++ ) {
            tcb [ i ] = NULL;
        } /* for */
    } /* if */

    return tcb;
} /* end of alloc_tcb_table */





/*--------------------------------------------------------------------------*/





/*
=============================================================================
|
| alloc_semaphore_array
|
| This function allocates an array of pointers to the semaphore array. This
| array of pointers is used to store the addresses of the semaphore
| structures themselves. Initially the array is initialised with NULL
| values in all the locations.
|
| The size of this array determines the maximum number of semaphores that can
| be allocated by user tasks. The advantage of having a table to store the
| location of all semaphores is that a operating system performance monitor
| can easily locate all the semaphores.
|
| The function returns a pointer to the semaphore array. If the pointer
| returned is a NULL then the required memory could not be allocated.
|
|   Parameters : - number of semaphores to allocate.
|
|   Entry via  : - setup_os_data_structures
|
=============================================================================
*/

static semaphore_struc **alloc_semaphore_array ( unsigned int max_num_semaph ) {

    unsigned int i;

	num_semaphores = max_num_semaph;
    semaphore = ( semaphore_struc ** )ucalloc ( max_num_semaph,
                                        sizeof ( semaphore_struc * ) );

    if ( semaphore == NULL ) {
        return semaphore;
    } /* if */

	/* now initialise the semaphore pointer array */
	for ( i = 0; i < num_semaphores; i++ ) {
        semaphore [ i ] = NULL;
    } /* if */

    return semaphore;
} /* end of alloc_semaphore array */






/*-------------------------------------------------------------------------*/





/*
============================================================================
|
| init_semaphore
|
| This function allows any semaphore to be custom initialised.  The values that
| it is to be initialised to are passed into the function.
|
|   Parameters : - number of semaphore to be initialised
|                - initial semaphore value
|                - semaphore multiplier value for the semaphore.
|
==========================================================================
*/

void init_semaphore ( unsigned int sema_num, unsigned int sema_value,
						unsigned int multiplier_value ) {

    char int_status;

	int_status = return_interrupt_status ( );
	disable ( );
	if (sema_value != 1) {
		/* then must be a synchronization or resource counting semaphore */
		semaphore [ sema_num ]->semaphore_type = SYNC_SEMA;
	} /* if */
	semaphore [ sema_num ]->semaphore_value = sema_value;
	semaphore [ sema_num ]->semaphore_multiplier = multiplier_value;
	if (int_status) {
		enable ( );
	}
} /* end of init_semaphore */





/*------------------------------------------------------------------------*/





/*
==========================================================================
|
| create_semaphore
|
| This function as the name implies creates a semaphore for the caller. What
| in actual fact that is returned is an index number into the semaphore
| pointer array for the new semaphore. A pool of semaphores is created during
| system initialisation, and if this pool is exhausted then an illegal
| semaphore number is returned - namely a 0xffff.
|
| The semaphores whose index is returned have been initialised at the time
| they were created to have an initial value of 1 and the multiplier is
| also 1. If these values are not suitable then they can be modified in the
| users task to the appropriate values using the init_semaphore function.
|
|   Parameters : - none
|
|   Entry via  : - user tasks requiring semaphores.
|
==========================================================================
*/

unsigned int create_semaphore ( void ) {

	char int_status;
	unsigned int return_semaphore_num;
	semaphore_struc *sema_ptr;

	int_status = return_interrupt_status ( );
	disable ( );

	/* check to see if the total number of semaphores has not been violated */
	if ( next_semaphore != 0xffff ) {
		/* allocate the memory for the semaphore structure */
		sema_ptr = ( semaphore_struc * )umalloc (
										sizeof ( semaphore_struc ) );
		if ( sema_ptr != NULL ) {
			return_semaphore_num = next_semaphore;

			/* assign the pointer to the semaphore pointer array */
			semaphore [ next_semaphore ] = sema_ptr;

			/* initialise the semaphore structure */
			sema_ptr->creating_taskname_ptr = tcb [
							rtn_current_task_num ( ) ]->task_name_ptr;
			sema_ptr->semaphore_value = 1;
			sema_ptr->semaphore_multiplier = 1;
			sema_ptr->semaphore_type = CRITICAL_SECTION_SEMA;
			sema_ptr->tcb_ptr = NULL;
			sema_ptr->semaph_bgn_q_index = bgn_semaphore_central_table;
			sema_ptr->semaph_queue_handler = (void (*) ())semaphore_queue_handler;
			bgn_semaphore_central_table += 2;


            next_semaphore++;
			if ( next_semaphore >= num_semaphores ) {
                /* have run out of semaphores */
                next_semaphore = 0xffff;
            } /* if */
        } /* if */
        else {
            /* cannot allocate the memory for the semaphore */
            return_semaphore_num = next_semaphore = 0xffff;
        } /* else */
	} /* if */
    else {
        /* have run out of semaphores */
        return_semaphore_num = next_semaphore;
    } /* else */

    if (int_status) {
        enable ( );
    } /* if */

    return return_semaphore_num;
} /* end of create_semaphore */





/*------------------------------------------------------------------------*/




/*
===========================================================================
|
| alloc_central_table
|
| This function allocates memory from the heap for the central table.  If the
| allocation is successful then the function passes back a pointer to the
| table, else it passes back a NULL.  If the central table can be allocated then
| it is initialised to zero.
|
| In addition to the central table storage being allocated the indexes into
| the central table are also defined.
|
|   Parameters : - number of tasks
|                - number of semaphores
|
|   Entry via  : - setup_os_data_structures
|
==========================================================================
*/

static void** alloc_central_table ( unsigned char number_priorities,
                                        unsigned int num_semaph ) {

    int i;

	num_of_priorities = number_priorities;

    /* calculate the beginning point for the semaphore central table entries.
    Every entry in the central table consists of two pointers ( except for the
    current task pointer ). The first pointer points to the beginning of a
	queue and the second to the end of a queue. The pointers appear in the
    central table from index zero as :
        the pointer to the current task
        sets of pointers to the various priority queues
        sets of pointers to the time queue for task control blocks
        sets of pointers to the active timer queue
        sets of pointers to the inactive timer queue


        In the following calculation for the index to the beginning of the
        active time queue based on adding 1 for the current task pointer
        and then 2 times the number of semaphores, since these occur in
        pairs.

    */

    bgn_active_time_q = 1 + num_of_priorities * 2;
	end_active_time_q = bgn_active_time_q + 1;
    bgn_inactive_time_q = 3 + num_of_priorities * 2;
	end_inactive_time_q = bgn_inactive_time_q + 1;
    bgn_semaphore_central_table = 5 + num_of_priorities * 2;

    ( void** ) central_table = ( void ** )ucalloc (
                    bgn_semaphore_central_table + 2 * num_semaph,
                    sizeof ( void * ) );
    if ( central_table == NULL )
        return ( void ** ) central_table;

    /* initialise the central table to NULL */
    for ( i = 0; i < ( bgn_semaphore_central_table + num_semaph * 2 ); i++ )
        central_table [ i ] = NULL;

    return ( void** ) central_table;
} /* end of alloc_central_table */





/*-----------------------------------------------------------------------*/





/*
==========================================================================
|
| return_start_flag
|
| This function returns the status of the start_flag. If this flag is
| clear then the operating system has not started. If the flag is set
| then the operating system has started. The function is used mainly to
| aid in the initialisation of reentrant tasks. When the task is entered
| for the first time and the flag is not set then it is known that this
| entry is for initialisation purposes. Therefore local variables can be
| initialised (since the normal runtime stack is being used). Upon the
| second entry into the routine the flag is again checked. This time it
| will be set inducating that entry has occurred as a consequence of a normal
| task switch. In this case the local variables are not initialised.
|
| Parameters:	-	none
|
| Returns:		-	nothing.
|
==========================================================================
*/

char return_start_flag (void) {

	return (start_flag);

} /* end of return_start_flag */





/*-----------------------------------------------------------------------*/





/*
===========================================================================
|
| create_task
|
| This function creates and initialises the tcb and creates a ready to run
| stack for the task name passed in. The stack is dynamically allocated on the
| UNOS heap.  After the task is created the flag start_flag determines whether
| the task is entered or not.  This flag is set by the start processes routine.
|
| The routine also provides a mechanism to allow the global variables for
| a task to be initialised before any tasks in the system start to run. This
| mechanism is ideal to set up any semaphores that are to be used by a task.
|
| When the task is created the routine also creates the mailbox associated
| with the task. Since the size of the various mailbox components are passed
| into the routine then the mail box for individual tasks can be
| customized. Implementing the mail boxes in this section of code also
| allows the name of the task to be placed into the mail exchange so that
| the address for mail box messages is effectively the task name as far as
| other user tasks are concerned.
|
|   Parameters : - pointer to the name of the task
|                - priority of task created
|                - tick delta for the task
|                - status of the task - runnable, blocked, suspended, active.
|                - queue type which task is to placed on
|                - semaphore number if task to be placed on a semaphore queue.
|                - size of the task stacks
|                - mail box message queue size
|                - maximum size of mail box message
|                - address of the task initialisation function
|                - address of the task function
|				 - pointer to an unknown variable that is used to pass
|				   initialisation data to a task.  The primary use is for
|				   setting up local variables in reentrant tasks.
|
|   Entry via  : - main module
|
============================================================================
*/

int create_task (  char *task_name_ptr, unsigned char priority_of_task,
					int task_tick_delta, unsigned char status_of_task,
					unsigned char q_type, unsigned int semaphore_num,
					unsigned int task_stk_size, unsigned int mess_q_size,
					unsigned int mess_size, void ( *init_task ) ( void ),
					void ( *task )( void* ), void* local_var_ptr ) {

	static char huge* stack_location;
	static void ( *task_ptr ) ( void* );
	static void* temp_local_var_ptr;
	static unsigned int stackbase;
	static unsigned int stackptr;
	static unsigned int baseptr;
	unsigned int sema1, sema2;
	mbx *mbx_ptr;
	taskname_map *taskname_map_ptr;
	taskname_map *nxt_ptr;
	taskname_map *old_nxt_ptr;
	unsigned int hash_table_index;

	/* upon entry to this routine the num_of_tasks variable can be used as
	an index into the tcb table to the position of the next task to be
	created.
	*/

	/* check to see that we are not attempting to create too many tasks. */
	if ( num_of_tasks >= max_num_of_tasks ) {
		return FALSE;
	} /* if */

	if ( init_tcb_and_q_it ( task_name_ptr, priority_of_task, task_tick_delta,
						status_of_task, q_type, semaphore_num ) == NULL ) {
		return FALSE;
	} /* if */

	/* if the task stack size is zero then we don't want to form a stack
	for the task. This condition only occurs if the task is the null task -
	this task uses the stack setup by the compiler. Note that the initialisation
	is also bypassed since the null task usually does not use any resources
	or variables. Note that a mail box is not created for the null task.
	*/
	if ( task_stk_size == 0 ) {
		central_table [ CURRENT_TASK ] = ( void *)tcb [ num_of_tasks ];
		num_of_tasks++;
		return TRUE;
	} /* if */

	/* now create a mail box for the task */
	/* firstly get the two semaphores required */
	sema1 = create_semaphore ( );
	sema2 = create_semaphore ( );


	if ( ( sema1 == 0xffff ) || ( sema2 == 0xffff ) ) {
		/* have run out of semaphores so exit */
		return FALSE;
	} /* if */

	/* customize the semaphores for the mail boxes */

	init_semaphore ( sema1, mess_q_size, 1 );
	init_semaphore ( sema2, 0, 1 );

	mbx_ptr = create_mbx ( num_of_tasks, FIFO_TYPE, mess_q_size, mess_size,
							sema1, sema2 );

	if ( mbx_ptr == NULL ) {
		/* cannot allocate the mail box */
		return FALSE;
	} /* if */

	/* place the mail box pointer into the mail box table */
	mbx_table [ num_of_tasks ] = mbx_ptr;

	/* now allocate the memory for the taskname_map structure associated with
	this task.
	*/
	taskname_map_ptr = ( taskname_map * )umalloc
									( sizeof ( taskname_map ) );
	if ( taskname_map_ptr == NULL ) {
		return FALSE;
	} /* if */

	/* set up the taskname_map structure with the appropriate data */
	taskname_map_ptr->task_name_ptr = task_name_ptr;
	taskname_map_ptr->task_number = num_of_tasks;
	taskname_map_ptr->nxt_taskname_map = NULL;

	/* now set up the mail exchange table so that this mail box is associated
	with the currently created task. In order to do this the task_name pointer
	has to be hashed to index into the hash_table.
	*/
	hash_table_index = hash_taskname_ptr ( task_name_ptr );

	/* now place the taskname_map into the hash table at the index
	position.
	*/
	nxt_ptr = hash_table [ hash_table_index ];

	if ( nxt_ptr == NULL ) {
		/* the index position is empty so place the taskname_map structure
		address into the hash table itself.
		*/
		hash_table [ hash_table_index ] = taskname_map_ptr;
	} /* if */
	else {
		/* already a taskname_map structure in the has table so have to chain
		the new entry onto the end of the taskname_map structures which hash
		to this location.
		*/
		while ( nxt_ptr != NULL ) {
			old_nxt_ptr = nxt_ptr;
			nxt_ptr = nxt_ptr->nxt_taskname_map;
		} /* while */

		/* have now found the end of the chained list so store the new
		taskname_map structure into the table.
		*/
		old_nxt_ptr->nxt_taskname_map = taskname_map_ptr;
	} /* else */


	/* now create the stack for this task. Firstly save the stack base and
	stack pointer for the working stack.
	*/
	stackbase = _SS;
	stackptr = _SP;
	baseptr = _BP;

	/* store the task to be executed address in a global variable.  This is done
	because the variables passed into the routine are stored on the working
	stack and referenced by the BP register.  When the new stack is set up
	a new BP register value will be established.  Therefore upon start up
	of the tasks control will return to the line after the geninterrupt in
	this function with the BP register containing the value for the new stack.
	Therefore the task variable used in the call to the task to be started
	will be incorrect since the software will be indexing into the new stack
	which does not contain the task address.  If one decides not to create a
	BP register value for the next stack then upon start of a task when
	control enters this routine just after the geninterrupt the BP value will
	be that of the working stack.  Therefore the value of task picked up
	for all tasks will be that left on the stack from the last task created -
	i.e. the null task.  Hence one gets multiple versions of the null task
	running.

	The approach taken to get around these problems is to store the task
	address in a global and then transfer it to the appropriate place on
	the new stack so that a new BP register value will find it correctly.
	This approach ensures that each new stack carries with it the address
	of the task to be executed.
	*/
	task_ptr = task;
	temp_local_var_ptr = local_var_ptr;

	kcb.entry_type = CREATE_TASK_STK_ENTRY;
	central_table [ CURRENT_TASK ] = tcb [ num_of_tasks ];

	/* now create the new stack.  The stack is created with an offset of 0x1b
	so that the task address can be placed on the new stack in the same
	relative location to the base pointer as the working stack.  Currently
	this relative location is bp+0x18. The stack is allocated from the
	dynamic memory heap.
	*/
	if ( (stack_location = umalloc ( task_stk_size ) ) == NULL ) {
		return FALSE;
	} /* if */


	/* !!!!!! NOTE !!!!!!!

	THE FOLLOWING STACK OFFSET CALCULATIONS ARE COMPILER DEPENDENT. THE
	PRECISE NUMBERS CHANGE DEPENDING ON THE VERSION OF TURBOc COMPILER USED.
	*/
	_SP = FP_OFF ( stack_location ) + task_stk_size - 1 - 0x21;
	_SS = FP_SEG ( stack_location );

	/* now create the new task BP register value and store the task address in
	the correct relative location.
	*/
	_BP = _SP;
	_SP = _BP - 0x14;
	task = task_ptr;    /* place task address on new stack */
	local_var_ptr = temp_local_var_ptr;

	geninterrupt ( kernel_entry );

	/* now check if the task is to be started */
	if ( start_flag ) {
		enable ();
		( *task ) (local_var_ptr);
	} /* if */


	/* restore the working stack */
	_SP = stackptr;
	_SS = stackbase;
	_BP = baseptr;

	/* Now check if there is a task initialisation function that has to be
	run.
	*/
	if (init_task != NULL) {
		(init_task)();
	} /* if */


	/* Task creation successful so increment the number of tasks created
	in the system.
	*/
	num_of_tasks++;

	return TRUE;
} /* end of create_task */





/*----------------------------------------------------------------------*/





/*
============================================================================
|
| start_tasks
|
| This function starts the processes in the system running.  This is achieved
| by setting the start_flag and starting the time slice interrupts.  The
| last function of the routine is to call the null task. Note that the
| routine uses the task setup by the compiler.
|
|   Parameters : - address of the null task
|
|   Entry via  : - main routine
|
============================================================================
*/

void start_tasks (  void ( *null_task ) ( void *) ) {

    start_flag = 1;
	( *null_task ) ( NULL );

} /* end of start_tasks */





/*----------------------------------------------------------------------*/





/*
=========================================================================
|
| set_kernel_interrupt_num
|
| Sets the interrupt number for the software interrupt to enter the kernel
|
|   Parameters : - kernel interrupt number
|
|   Entry via  : - unmain
|
==========================================================================
*/

void set_kernel_interrupt_num ( unsigned char kernel_interrupt_num ) {

    kernel_entry = kernel_interrupt_num;
} /* end of set_kernel_interrupt_num */





/*----------------------------------------------------------------------*/





/*
==========================================================================
|
| chg_task_tick_delta
|
| The purpose of this function is to change the tick delta for the particular
| task whose address is passed into the routine. The tick delta is then
| added to the base tick time slice frequency to decide how many ticks the
| particular task will run for if not preempted. Therefore the tick delta
| mechanism is effectively a secondary priority mechanism, in that tasks of
| the same priority can be allocated different amounts of the processor
| time.
|
| If the change has been successfully completed then the routine returns
| a TRUE else it returns a FALSE.
|
| Parameters : - pointer to the tasks name
|              - the new tick delta value
|
| Entry via  : - any user task
|
===========================================================================
*/

int chg_task_tick_delta ( char *task_name_ptr, int new_tick_delta ) {

    unsigned int task_num, int_status;

    /* now make the connection between the task name and the task number. */
    task_num = mail_exchange ( task_name_ptr );

    /* firstly check to see if the task number is a legal one */
    if ( task_num >= num_of_tasks ) {
        /* normally call an error handler here */
        return FALSE;
    } /* if */
    

    int_status = return_interrupt_status ( );
    disable ( );

    /* now set up the number of ticks before this particular task will
    enter the kernel. The number of ticks is set up as an absolute value.
    */
    if ( base_ticks_per_time_slice + new_tick_delta <= 0 ) {
        tcb [ task_num ]->ticks_before_kernel_entry = 1;
    } /* if */
    else {
        tcb [ task_num ]->ticks_before_kernel_entry =
                        base_ticks_per_time_slice + new_tick_delta;
    } /* else */

    tcb [ task_num ]->ticks_to_go =
                        tcb [ task_num ]->ticks_before_kernel_entry;

    if ( int_status ) {
        enable ( );
    } /* if */

    return TRUE;
} /* chg_task_tick_delta */





/*----------------------------------------------------------------------*/




/*
=========================================================================
|
| chg_base_ticks_per_time_slice
|
| As this functions name implies its purpose is to initialise the base
| time slice frequency used in the operating system.  This time slice
| frequency is expressed in terms of the number of clock ticks which
| occur before a time slice entry to the operating system occurs. Note that
| this base time slice frequency is not necessarily the time slice frequency
| of a particular task, since the tick_delta can alter this base frequency
| on an individual task basis.
|
|   Parameters : - unsigned int containing the number of ticks
|
|   Entry via  : - multiple places
|
=========================================================================
*/

void chg_base_ticks_per_time_slice ( int new_base_ticks_per_time_slice ) {

    int i;
    int tick_delta;
    char int_status;

    int_status = return_interrupt_status ( );
    disable ( );

    /* have to go through the tcb array and alter all the
    ticks_before_kernel_entry values to conform to the new base
    tick frequency. If the altered value becomes less than or equal to
    zero then the value is set to 1.
    The current value of the tick_delta for a particular task is calculated
    before the base tick value is changed to its new value.
    */
    new_base_ticks_per_time_slice = abs ( new_base_ticks_per_time_slice );

    for ( i = 0; i < num_of_tasks; i++ ) {
        tick_delta = tcb [ i ]->ticks_before_kernel_entry -
                                            base_ticks_per_time_slice;
        if ( new_base_ticks_per_time_slice + tick_delta <= 0 ) {
            tcb [ i ]->ticks_before_kernel_entry = 1;
        } /* if */
        else {
            tcb [ i ]->ticks_before_kernel_entry =
                        new_base_ticks_per_time_slice + tick_delta;
        } /* else */
    } /* for */

    base_ticks_per_time_slice = ( unsigned int )new_base_ticks_per_time_slice;

    if (int_status) {
        enable ( );
    } /* if */

} /* end of chg_base_ticks_per_time_slice */





/*----------------------------------------------------------------------*/





/*
==========================================================================
|
| init_start_flag
|
| This routine initialises the start flag to zero.
|
==========================================================================
*/

void init_start_flag ( void ) {
    start_flag = 0;
} /* end of init_start_flag */





/*----------------------------------------------------------------------*/






/*
========================================================================
|
| return_semaphore_value
|
| This function returns the current value of the semaphore whose number is
| passed to the function. The interrupt status of the calling task is
| preserved by the routine.
|
| Function returns the semaphore value. If the semaphore number passed in
| is illegal then the returned value is 0xffff.
|
| Parameters : - semaphore number whose value is to be returned.
|
| Entry via  : - multiple places
|
========================================================================
*/

unsigned int return_semaphore_value ( unsigned int sema_num ) {

    int int_status;
    unsigned int temp = 0xffff;

    int_status = return_interrupt_status ( );
    disable ( );

    /* check if the semaphore number is legal */
    if (sema_num < next_semaphore) {
        temp = semaphore [ sema_num ]->semaphore_value;
    } /* if */

    if ( int_status )
        enable ( );

    return ( temp );
} /* end of return_semaphore_value */





/*----------------------------------------------------------------------*/





/*
==========================================================================
|
| setup_os_data_structures
|
| This function sets up all the major data structures in the UNOS system.
| All these data structures are created dynamically on the heap. This will
| help leave the option open to extend UNOS so that tasks can be dynamically
| created (and destroyed) at run time. Obviously this option would only be 
| of use in a system where tasks can be loaded from disk. Since the current
| version of UNOS is targeted from ROM based embedded applications these
| features are currently not implemented.
|
| The data structures created in this routine are:-
|
|   central table
|   tcbs
|   semaphores
|   mail box table
|   mail box structures
|
| If there are no problems in the creation of these structures then the 
| routine returns TRUE, else it returns FALSE.
|
| Parameters : - interrupt number for the kernel entry
|              - number of ticks before time slice entry of kernel
|              - number of priority levels
|              - maximum number of semaphores
|              - number of tasks
|              - pointer to the memory pool
|              - size of the memory pool
|
| Entry via : - software_initialise in the main module
|
===========================================================================
*/

char setup_os_data_structures ( unsigned char kernel_interrupt_num,
                                int clock_ticks_kernel_ent,
                                unsigned int num_of_priorities,
								unsigned int max_num_semaphores,
                                unsigned int maximum_num_of_tasks,
                                char* ptr_to_mem_pool,
                                unsigned long memory_pool_size ) {

    semaphore_struc **sema_ptr;

    mem_pool_size = memory_pool_size;
    mem_pool_ptr = ptr_to_mem_pool;
    max_num_of_tasks = maximum_num_of_tasks;

    /* make sure that the clock ticks to enter kernel value is a
    positive number, and assign it to the global base value. This value
    is then used to set up the values in the tcb.
    */
    base_ticks_per_time_slice = abs ( clock_ticks_kernel_ent );

    set_kernel_interrupt_num ( kernel_interrupt_num );
    init_start_flag ( );
	alloc_central_table ( num_of_priorities, max_num_semaphores );
    tcb = alloc_tcb_table ( max_num_of_tasks );
	sema_ptr = alloc_semaphore_array ( max_num_semaphores );
    mbx_table = alloc_mbx_table ( max_num_of_tasks );
    hash_table = alloc_mail_exch_table ( HASH_TABLE_SIZE );

    if ( (mbx_table != NULL) && (tcb != NULL) && (hash_table != NULL) &&
                                                    (sema_ptr != NULL) ) {
        return TRUE;
    } /* if */
    else {
        return FALSE;
    } /* else */
} /* end of setup_os_data_structures */





/*-------------------------------------------------------------------------*/





/****************************************************************************/
/*                                                                          */
/*                                                                          */
/*                                                                          */
/*                              UNOS_4 MODULE                               */
/*                                                                          */
/*                                   by                                     */
/*                                                                          */
/*                              Robert Betz                                 */
/*        Department of Electrical Engineering and Computer Science         */
/*                         University of Newcastle                          */
/*                               Australia                                  */
/*                                                                          */
/*                       ( Copyright 1989, 1990 )                           */
/*                                                                          */
/*                                                                          */
/*                                                                          */
/****************************************************************************/


/*
HISTORY

Began writing this module on the 18th January 1990.

*/


/*

BRIEF DESCRIPTION

This module is included into the UNOS operating system. Its function is to
provide mail box communications for the operating system. A full description
of the operation of the mail box facility can be found in the UNOS.ASC (ascii
documentation file for UNOS) or in the printed documentation (which is a
laser printer output of this file).
*/


/*
DATA STRUCTURES

The main data structures used by the mail box system are defined at the top of
the UNOS_1.C module of the operating system.
*/





/*-----------------------------------------------------------------------*/





/*
==========================================================================
|
| alloc_mail_exch_table
|
| The purpose of this function is to allocate an array of pointers to the
| the tskname_struc structures. This table forms the centre of the mail
| exchange mechanism used in UNOS. This mechanism allows messages to be
| sent to a task simply by using the array name of the array containing the
| tasks name. This name is then "hashed" to form an index into this table
| which contains pointers to tskname_map structures which contain the
| information to connect the task name to the task number. This task number
| is then used in the lower levels of the mail system to index into the
| mail box array. The function returns a pointer to the array of pointers.
| If the allocation fails then a NULL pointer is returned. If the
| allocation occurs correctly then the table is initialised with NULL's
| throughout.
|
| Parameters    :-  size of the hash table
|
| Entry via     :-  setup_os_data_structures
|
==========================================================================
*/

static taskname_map **alloc_mail_exch_table ( unsigned int size_of_table ) {

    taskname_map **mail_exch_ptr;
    int i;

    mail_exch_ptr = ( taskname_map ** )ucalloc ( size_of_table,
                                        sizeof ( taskname_map * ) );
    if ( mail_exch_ptr != NULL ) {
        for ( i = 0; i < size_of_table; i++ ) {
            mail_exch_ptr [ i ] = NULL;
        } /* for */
    } /* if */
    return mail_exch_ptr;
} /* end of alloc_mail_exch_table */





/*-----------------------------------------------------------------------*/





/*
===========================================================================
|
| hash_taskname_ptr
|
| This function accepts a pointer to an array of characters and hashes the
| pointer address to an index within the range of zero to hash_table_size-1.
| The hashing is carried out by converting the 8086 segmented address into
| a linear address (this is obviously implementation specific) and then
| carrying out a mod hash function on this address, with the size of the
| table being a prime number.
|
| Parameters    :- pointer to the character array defining the name of the
|                  task.
|
| Entry         :- create_task function.
|
===========================================================================
*/

static unsigned int hash_taskname_ptr ( char *task_name_ptr ) {


    unsigned long int offset;
    unsigned long int segment;
    unsigned long int lin_addr;

    offset = FP_OFF ( task_name_ptr );
    segment = FP_SEG ( task_name_ptr );

    /* now convert to a linear address */
    lin_addr = segment * 16 + offset;

    /* now carry out the hash operation */

	return ( (unsigned int)(lin_addr % HASH_TABLE_SIZE) );

} /* end of hash_taskname_ptr */





/*----------------------------------------------------------------------*/





/*
==========================================================================
|
| alloc_mbx_table
|
| The function of this routine is to allocate storage for an array of pointers
| to the mail boxes in the system. This array is created on the heap at
| initialisation time. The array is initialised with NULL pointers which are
| set up to the correct values in the create_mbx routine which itself is
| called from the create_task routine. The array components point to the mail
| boxes themselves. The purpose of the table is to allow the kernel to
| physically locate where a mail box is in memory using only the task number
| associated with the mail box. The task number effectively forms the index
| into this table.
|
| The function returns a pointer to the area of memory which has been
| allocated for the table. If the pointer is a NULL then the allocation has
| not been successful.
|
| Parameters : - number of tasks in the system.
|
| Entry via : - create_unos_data_struc function
|
==========================================================================
*/

static mbx** alloc_mbx_table ( unsigned int num_of_tasks ) {

    mbx** mbx_table;
    int i;

    mbx_table = ( mbx** )ucalloc ( num_of_tasks, sizeof ( mbx* ) );
    if ( mbx_table != NULL ) {
        for ( i = 0; i < num_of_tasks; i++ )
            mbx_table [ i ] = NULL;
    } /* if */

    return mbx_table;

} /* end of alloc_mbx_table */





/*-------------------------------------------------------------------------*/





/*
============================================================================
|
| create_mbx
|
| The function of this routine is to allocate and initialise a mail box.
| The routine allocates storage on the heap for a mail box and then
| initialises its contents based on the parameters passed in.
|
| The function returns the address of the memory allocated for the mail box.
| If there has been a problem allocating memory then a NULL pointer is
| returned.
|
| Parameters : - task number for which the mbx is to be created
|              - type of mail box - fifo or priority (only fifo currently
|                                   implemented)
|              - number of messages in the mail box
|              - size of each message
|              - space available semaphore
|              - message available semaphore
|
| Entry via : - create_task routine
|
============================================================================
*/

static mbx* create_mbx ( unsigned int task_num, char mbx_type, unsigned int
                 num_mess, unsigned int mess_size, unsigned int
                 spce_avail_sema, unsigned int mess_avail_sema ) {

    mbx* mbx_ptr;
    char allocation_problem = FALSE;
    unsigned int i;

	mbx_ptr = ( mbx* )umalloc ( sizeof ( mbx ) );
    if ( mbx_ptr != NULL ) {
        mbx_ptr->mess_addr = task_num;
        mbx_ptr->mbx_type = mbx_type;
        mbx_ptr->q_size = num_mess;
        mbx_ptr->mess_size = mess_size;
        mbx_ptr->spce_avail_sema = spce_avail_sema;
        mbx_ptr->mess_avail_sema = mess_avail_sema;
        mbx_ptr->free = num_mess;
        mbx_ptr->used = 0;
        mbx_ptr->get_ptr = 0;
        mbx_ptr->put_ptr = 0;
        mbx_ptr->qik_mess_flag = FALSE;
        /* now allocate the quick message envelope */
		mbx_ptr->qik_mess_ptr = ( envelope* )umalloc ( sizeof ( envelope ) );
        /* now allocate the message queue envelopes */
        mbx_ptr->mess_q_ptr =( envelope* )ucalloc ( num_mess,
                                                    sizeof ( envelope ) );

        /* check to see if the allocation for the envelopes has been
        successful
        */
        if ( ( mbx_ptr->qik_mess_ptr == NULL ) ||
            ( mbx_ptr->mess_q_ptr == NULL ) )
            return ( mbx* )NULL;

        /* now allocate the message buffers for each of the envelope
        structures
        */
        mbx_ptr->qik_mess_ptr->message_ptr = ( char* )ucalloc ( mess_size,
                                                        sizeof ( char ) );
        for ( i = 0; i < num_mess; i++ ) {
            mbx_ptr->mess_q_ptr [ i ].message_ptr = ( char* )ucalloc (
                                                mess_size, sizeof ( char ) );
            if ( mbx_ptr->mess_q_ptr [ i ].message_ptr == NULL )
                allocation_problem = TRUE;
        } /* for */

        /* now check if the allocation of the message buffers for each of
        the envelopes has been successful
        */
        if ( ( mbx_ptr->qik_mess_ptr->message_ptr == NULL ) ||
            allocation_problem )
            return ( mbx* )NULL;
        else
            return mbx_ptr;
    } /* if */
    else {
		cprintf ( "Mail Box allocation problem\n" );
        return ( mbx* )NULL;
    } /* else */
} /* end create_mbx */





/*-------------------------------------------------------------------------*/





/*
============================================================================
|
| mail_exchange
|
| The purpose of this function is to match the pointer to a task name to the
| task number. This matching operation is required in the mail system to
| allow the address of a task to be independent of its task number. The
| mapping from the pointer value to the task number is implemented using
| a chained hashing algorithm. The hash table is initialised during the
| task creation process. The hash table itself contains pointers to linked
| lists of taskname_map structures. The index into the table is calculated
| using a simple (but effective) modulus based hashing scheme.
|
| If the pointer address hashes to a table entry which has no taskname_map
| structure pointer then one is attempting to send data to an undefined
| task. This is indicated by a hash_table entry being NULL. The routine
| in this case returns the task number as the contents of num_of_tasks
| (which is actually one more than the number of the last task number).
|
| Parameter :- pointer to a character string.
|
| Entry via :- send_mess function
|
============================================================================
*/

static unsigned int mail_exchange ( char *mess_addr_ptr ) {

    unsigned int hash_table_index;
    taskname_map *taskname_struc_ptr;
    unsigned int task_number = 0xffff;

    /* firstly hash the pointer */
    hash_table_index = hash_taskname_ptr ( mess_addr_ptr );

    /* now look into the hash table */

    if ( hash_table [ hash_table_index ] != NULL ) {
        taskname_struc_ptr = hash_table [ hash_table_index ];
        do {
            if ( taskname_struc_ptr->task_name_ptr ==
                                                    mess_addr_ptr ) {
                task_number = taskname_struc_ptr->task_number;
                taskname_struc_ptr = NULL;
            } /* if */
            else {
                taskname_struc_ptr = taskname_struc_ptr->nxt_taskname_map;
            } /* else */
        } while ( taskname_struc_ptr != NULL );
    } /* if */

    /* now check to see if a legal task number has been found */
    if ( task_number == 0xffff ) {
        /* illegal task number found */
        return num_of_tasks;
    } /* if */
    else {
        return task_number;
    } /* else */
} /* end of mail_exchange */





/*-------------------------------------------------------------------------*/





/*
==============================================================================
|
| send_mess
|
| This function puts a message into a mail box for a particular task. If there
| is room for the message then control will return almost immediately, else
| the calling task will become blocked. If the message size is such that it
| is too large for the message size in the mail box then the routine will
| return a FALSE, else it will return a TRUE. A FALSE is also returned if
| the message is sent a non-existent task.
|
| In order to make the address of the message independent of task numbering
| message addresses consist of a pointer to a string describing the task. The
| connection between this pointer and the task number is created at task
| creation time. In order to make it fast to obtain the task number from
| the string pointer a hash table is used. This mapping is carried out by the
| mail exchange which implements a chained hashing algorithm.
|
| NOTE : this routine currently does not handle priority mail boxes.
|
| Note that interrupts are disabled for a large part of this routine in order
| to make the message indivisible.
|
| Parameters : - pointer to the message to be sent
|              - length of the message to be sent
|              - address of the receiver of the message. This is a pointer to
|                a character string that contains the name of the task. The
|                pointer value is actually used as address of the task and
|                the contents of the character string.
|
| Entry via : - multiple places
|
==============================================================================
*/

int send_mess ( unsigned char* mess_ptr, unsigned int mess_lgth,
                char *mess_addr_ptr ) {

    unsigned int mess_addr, sender_task_num;
    char *sender_addr_ptr;
    char int_status;
    unsigned int i;

    /* firstly call the mail exchange to establish which task number is
    being addressed, and then check to see if the task that the message
    is being sent to actually exists. If it doesn't then return with a FALSE.
    */
    if ( ( ( mess_addr = mail_exchange ( mess_addr_ptr ) ) >= num_of_tasks)) {
       return FALSE;    /* non-existent task being addressed */
    } /* if */

    /* now check whether the message will fit in the message buffer */
    if ( mess_lgth > mbx_table [ mess_addr ]->mess_size )
        return FALSE;   /* does not fit in the message buffer */

    int_status = return_interrupt_status ( );

    wait ( mbx_table [ mess_addr ]->spce_avail_sema );
    disable ( );

    sender_task_num = rtn_current_task_num ( );

    /* now get the name of the sending task by looking in the tcb of
    the task.
    */
    sender_addr_ptr = tcb [ sender_task_num ]->task_name_ptr;

    /* copy the message into the correct mail box envelope */
    for ( i = 0; i < mess_lgth; i++ )
        mbx_table [ mess_addr ]->mess_q_ptr [
           mbx_table [ mess_addr ]->put_ptr ].message_ptr [ i ] =
                                                            mess_ptr [ i ];

    /* now complete the rest of the envelope structure */
    mbx_table [ mess_addr ]->mess_q_ptr [ mbx_table [ mess_addr ]->put_ptr ]
                                                .mess_lgth = mess_lgth;
    mbx_table [ mess_addr ]->mess_q_ptr [ mbx_table [ mess_addr ]->put_ptr ]
                                                .rtn_addr_ptr = sender_addr_ptr;
    mbx_table [ mess_addr ]->mess_q_ptr [ mbx_table [ mess_addr ]->put_ptr ]
                        .sender_pri = tcb [ sender_task_num ]->static_priority;

    /* now update the mail box accounting locations */
    mbx_table [ mess_addr ]->put_ptr++;
    if ( mbx_table [ mess_addr ]->put_ptr >= mbx_table [ mess_addr ]->q_size )
        mbx_table [ mess_addr ]->put_ptr = 0;
    mbx_table [ mess_addr ]->free--;
    mbx_table [ mess_addr ]->used++;

    if ( int_status )
        enable ( );

    _signal ( mbx_table [ mess_addr ]->mess_avail_sema );

    return TRUE;    /* indicate that message successfully sent */

} /* end of send_mess */





/*--------------------------------------------------------------------------*/





/*
==========================================================================
|
| send_qik_mess
|
| This function sends a quick message to a mail box for a particular task.
| A quick or express message is different from a normal message in that
| it should be the next message read from the mail box regardless of the
| other messages that may be stored in the mail box. This is generally
| achieved by putting the quick message just prior to the next message to
| be read from the buffer and then adjusting the get_ptr appropriately.
| If the message queue is full however this cannot be done, and in fact the
| task generating the message could become blocked (depending on how this
| situation was handled). For this reason the mail box contains a special
| envelope to store quick messages. If the situation arises where the special
| mail box is full also then the routine returns immediately with a return
| code equal to 2 which indicates this condition. A return code of 0 (usually
| corresponding to a FALSE) indicates that the message is too big for the
| size of the message buffers. A return code of 1 (usually corresponding to
| a TRUE) indicates a successful message sent.
|
| Parameters : - pointer to the message to be sent
|              - message length
|              - address where message is to be sent - a pointer to the
|                task name.
|
| Entry via : - multiple places
|
===========================================================================
*/

int send_qik_mess ( unsigned char* mess_ptr, unsigned int mess_lgth,
                    char * mess_addr_ptr ) {

    char int_status;
    unsigned int mess_addr, i;

    /* firstly call the mail exchange to establish which task number is
    being addressed, and then check to see if the task that the message
    is being sent to actually exists. If it doesn't then return with a FALSE.
    */
    if ( ( ( mess_addr = mail_exchange ( mess_addr_ptr ) ) >= num_of_tasks)) {
       return FALSE;    /* non-existent task being addressed */
    } /* if */

    /* check if the message will fit in the message buffer */
    if ( mess_lgth > mbx_table [ mess_addr ]->mess_size ) {
        return FALSE;   /* message too big for the buffer */
    } /* if */

    int_status = return_interrupt_status ( );
    disable ( );
    /* now check to see if there is room in the normal message queue for the
    express message
    */
    if ( mbx_table [ mess_addr ]->free == 0 ) {
        /* enter here if there is no room in the normal message buffer so
        now check to see if there is room in the special qik message buffer.
        If there is no room here then return with the appropriate return
        code.
        */
        if ( mbx_table [ mess_addr ]->qik_mess_flag ) {
            /* qik message envelope full so reset interrupts and return to
            the calling program
            */
            if ( int_status )
                enable ( );
            return 2;
        } /* if */
        else {
            /* there is room in the qik message envelope */
            for ( i  = 0; i < mess_lgth; i++ )
                mbx_table [ mess_addr ]->qik_mess_ptr->message_ptr [ i ] =
                                                            mess_ptr [ i ];
            mbx_table [ mess_addr ]->qik_mess_ptr->rtn_addr_ptr =
                    tcb [ rtn_current_task_num ( )]->task_name_ptr;
            mbx_table [ mess_addr ]->qik_mess_ptr->sender_pri =
                                        tcb [ mess_addr ]->static_priority;
            mbx_table [ mess_addr ]->qik_mess_ptr->mess_lgth = mess_lgth;
            mbx_table [ mess_addr ]->qik_mess_flag = TRUE;
        } /* else */
    } /* if */
    else {
        /* there are free locations in the message queue so put the qik
        message at the head if this queue. If the message queue is empty
        then put the message in it at the put_ptr location
        */

        if ( mbx_table [ mess_addr ]->used == 0 ) {
            /* now write the message into the envelope in the envelope queue */
            for ( i = 0; i < mess_lgth; i++ )
                mbx_table [ mess_addr ]->mess_q_ptr [ mbx_table [
                  mess_addr ]->put_ptr ].message_ptr [ i ] = mess_ptr [ i ];

            /* now update the other components of the envelope */
            mbx_table [ mess_addr ]->mess_q_ptr [
                mbx_table [ mess_addr ]->put_ptr ].rtn_addr_ptr =
                        tcb [ rtn_current_task_num ( ) ]->task_name_ptr;
            mbx_table [ mess_addr ]->mess_q_ptr [
                mbx_table [ mess_addr ]->put_ptr ].sender_pri =
                                            tcb [ mess_addr ]->static_priority;
            mbx_table [ mess_addr ]->mess_q_ptr [
                mbx_table [ mess_addr ]->put_ptr ].mess_lgth = mess_lgth;

            mbx_table [ mess_addr ]->put_ptr++;
            if ( mbx_table [ mess_addr ]->put_ptr >= mbx_table [
                                                    mess_addr ]->q_size )
                mbx_table [ mess_addr ]->put_ptr = 0;

        } /* if */
        else {
             if ( mbx_table [ mess_addr ]->get_ptr == 0 )
                mbx_table [ mess_addr ]->get_ptr = mbx_table [ mess_addr ]->
                                                q_size - 1;
            else
                mbx_table [ mess_addr ]->get_ptr--;

            /* now write the message into the envelope in the envelope queue */
            for ( i = 0; i < mess_lgth; i++ )
                mbx_table [ mess_addr ]->mess_q_ptr [ mbx_table [ mess_addr ]->
                            get_ptr ].message_ptr [ i ] = mess_ptr [ i ];

            /* now update the other components of the envelope */
            mbx_table [ mess_addr ]->mess_q_ptr [
                mbx_table [ mess_addr ]->get_ptr ].rtn_addr_ptr =
                        tcb [ rtn_current_task_num ( ) ]->task_name_ptr;
            mbx_table [ mess_addr ]->mess_q_ptr [
                mbx_table [ mess_addr ]->get_ptr ].sender_pri =
                                            tcb [ mess_addr ]->static_priority;
            mbx_table [ mess_addr ]->mess_q_ptr [
                mbx_table [ mess_addr ]->get_ptr ].mess_lgth = mess_lgth;
        } /* else */

        /* now update the used and free locations */
        mbx_table [ mess_addr ]->free--;
        mbx_table [ mess_addr ]->used++;
    } /* else */

    if ( int_status )
        enable ( );

    /* now signal that data is available in the mail box */
    _signal ( mbx_table [ mess_addr ]->mess_avail_sema );

    return TRUE;
} /* end of send_qik_mess */





/*-------------------------------------------------------------------------*/





/*
=============================================================================
|
| rcv_mess
|
| This routine receives a message from a mail box associated with a particular
| task. There is no need to specify the mail box since the mail box from
| which the message is read from is that associated with the task.
| One important feature of this routine is that the receiving task can
| indicate how long it will wait for the message by passing a time limit to the
| routine. If a value of zero is passed for the time limit then the time limit
| is deemed to be infinite.
|
| The function returns the address of the sending task if the timeout period
| has not expired. Note that this address is returned as a pointer to the
| character string which names the task. If the timeout period has expired
| without a message being received it will return NULL. If the return is
| due to the fact that a timer is not available for the timed_wait operation
| then the return code is 0xffff:000f.
|
| Parameters : - pointer to the memory area where the message will be stored
|              - pointer to the integer where the message length will be
|                stored.
|              - time limit value.
|
| Entry via : - multiple places
|
=============================================================================
*/

char *rcv_mess ( unsigned char* mess_ptr, unsigned int* mess_lgth,
                        unsigned long time_limit ) {

    unsigned int mbx_addr;
    char *rtn_addr_ptr;
    int wait_result = 0;
    char int_status;
	unsigned int i;
	unsigned int *mess_lgth_ptr;
	char *qik_mess_ptr;
	char *mbx_mess_ptr;

    mbx_addr = rtn_current_task_num ( );

    /* firstly check what type of wait has to be carried out */
    if ( time_limit == 0 )
        wait ( mbx_table [ mbx_addr ]->mess_avail_sema );
    else
        wait_result = timed_wait ( mbx_table [ mbx_addr ]->
                                                mess_avail_sema, time_limit );

    /* now check the value of the wait_result. If zero then the wait has
    been terminated by a signal condition or no wait has occurred. If the
    wait result is non-zero then some error condition has occurred. the type
    of error condition is determined by the value:- 1=>wait terminated due
    to a timeout on the semaphore; 2=>a timer was not available for the
    timed_wait.
    */
    if ( wait_result ) {
        if ( wait_result == 1 )
            return NULL;  /* timeout occurred */
        return ( (char *)MK_FP ( 0xffff, 0x000f ) );      /* no timer available */
    } /* if */

    int_status = return_interrupt_status ( );
    disable ( );

    /* enter this section of the code if there is a message to receive. Now
    check to see if the message is in the qik message envelope. If so then
    retrieve it from there, else get the message from the message envelope
    queue.
    */
    if ( mbx_table [ mbx_addr ]->qik_mess_flag ) {
		/* message in the qik envelope */
		/* now setup pointers to the appropriate variables so that the
		compiler will be forced to produce efficient code.
		*/
		mess_lgth_ptr = &mbx_table [ mbx_addr ]->qik_mess_ptr->mess_lgth;
		qik_mess_ptr = mbx_table [ mbx_addr ]->qik_mess_ptr->message_ptr;

		for ( i = 0; i < *mess_lgth; i++ ) {
			mess_ptr [ i ] = *(qik_mess_ptr + i);
		} /* for */

		*mess_lgth = *mess_lgth_ptr;
		mbx_table [ mbx_addr ]->qik_mess_flag = FALSE;
		rtn_addr_ptr = mbx_table [ mbx_addr ]->qik_mess_ptr->rtn_addr_ptr;
	} /* if */
	else {
		/* message must be in the normal message queue so retrieve from here
		*/
		/* firstly assign the key variables to pointers to force the compiler
		to produce efficient code.
		*/
		mess_lgth_ptr = &mbx_table [ mbx_addr ]->mess_q_ptr [
							mbx_table [ mbx_addr ]->get_ptr ].mess_lgth;
		mbx_mess_ptr = mbx_table [ mbx_addr ]->mess_q_ptr [
							mbx_table [ mbx_addr ]->get_ptr ].message_ptr;
		for ( i = 0; i < *mess_lgth_ptr; i++ ) {
			mess_ptr [ i ] = *(mbx_mess_ptr + i);
		} /* for */

        *mess_lgth = mbx_table [ mbx_addr ]->mess_q_ptr [
                    mbx_table [ mbx_addr ]->get_ptr ].mess_lgth;
        rtn_addr_ptr = mbx_table [ mbx_addr ]->mess_q_ptr [
                    mbx_table [ mbx_addr ]->get_ptr ].rtn_addr_ptr;
        mbx_table [ mbx_addr ]->get_ptr++;
        if ( mbx_table [ mbx_addr ]->get_ptr >= mbx_table [ mbx_addr ]->
																q_size ) {
			mbx_table [ mbx_addr ]->get_ptr = 0;
		} /* if */

        mbx_table [ mbx_addr ]->free++;
        mbx_table [ mbx_addr ]->used--;
    } /* else */

    /* now signal that space is available in the mail box */
    _signal ( mbx_table [ mbx_addr ]->spce_avail_sema );

    if ( int_status )
        enable ( );

    return rtn_addr_ptr;

} /* end of rcv_mess */





/*--------------------------------------------------------------------------*/





/*
=============================================================================
|
| size_mbx
|
| This function returns the size of the message queue for a particular mail
| box. If one tries to look at an illegal mail box then a zero is returned,
| else the size of the mail box is returned.
|
| Parameters : - mail box address - which is the task name with which it is
|                associated.
|
| Entry via : - multiple places
|
=============================================================================
*/

unsigned int size_mbx ( char *mbx_addr_ptr ) {

    unsigned int mbx_num;
    unsigned int mbx_size;

    /* carry out the mapping between the mail box name (which is the task
    name) and the mail box number.
    */
    if ( ( mbx_num = mail_exchange ( mbx_addr_ptr ) ) >= num_of_tasks ) {
        /* trying to look at an illegal mail box */
        mbx_size = 0;
    } /* if */
    else {
        /* address OK */
        mbx_size = mbx_table [ mbx_num ]->q_size;
    } /* else */
    return mbx_size;

} /* end of size_mbx */





/*--------------------------------------------------------------------------*/





/*
=============================================================================
|
| size_mbx_mess
|
| This function returns the maximum size of a message which can be sent to a
| mail box (i.e. the size of each of the message slots in the message queue.
| If the mail box address is illegal then the routine returns a value of
| 0, else the length of the messages is returned.
|
| Parameters : - address of the mail box (i.e. task name with which it is
|                associated) whose size is to be determined.
|
| Entry via : - multiple places
|
=============================================================================
*/

unsigned int size_mbx_mess ( char *mbx_addr_ptr ) {

    unsigned int mbx_num;
    unsigned int size_mbx_mess;

    /* carry out the mapping between the mail box name (which is the task
    name) and the mail box number.
    */
    if ( ( mbx_num = mail_exchange ( mbx_addr_ptr ) ) >= num_of_tasks ) {
        /* trying to look at an illegal mail box */
        size_mbx_mess = 0;
    } /* if */
    else {
        /* address OK */
        size_mbx_mess = mbx_table [ mbx_num ]->mess_size;
    } /* else */
    return size_mbx_mess;

} /* end of size_mbx_mess */





/*---------------------------------------------------------------------------*/





/*
==============================================================================
|
| free_mbx
|
| This function returns the number of free message slots in a mail box. If
| the mail box number is illegal then the routine returns a 0xffff value, else
| it returns the number of fre8e locations.
|
| Parameters : - address of the mail box (i.e. task name pointer with which
|                it is associated) whose free space is to be determined.
|
| Entry via : - multiple places
|
=============================================================================
*/

unsigned int free_mbx ( char *mbx_addr_ptr ) {

    char int_status;
    unsigned int mbx_num;
    unsigned int free;

    int_status = return_interrupt_status ( );
    disable ( );
    /* carry out the mapping between the mail box name (which is the task
    name) and the mail box number.
    */
    if ( ( mbx_num = mail_exchange ( mbx_addr_ptr ) ) >= num_of_tasks ) {
        /* trying to look at an illegal mail box */
         free = 0xffff;
    } /* if */
    else {
        /* address OK */
        free = mbx_table [ mbx_num ]->free;
    } /* else */
    if ( int_status )
        enable ( );
    return free;

} /* end of free_mbx */





/*---------------------------------------------------------------------------*/





/*
==============================================================================
|
| used_mbx
|
| This function returns the number of used message slots in a mail box. If the
| mail box number is illegal then the number returned is 0xffff.
|
| Parameters : - address of the mail box (i.e. the task nmae pointer with
|                which the mail box is associated) whose used space is to
|                be determined.
|
| Entry via : - multiple places
|
==============================================================================
*/

unsigned int used_mbx ( char *mbx_addr_ptr ) {

    char int_status;
    unsigned int mbx_num;
    unsigned int used;

    int_status = return_interrupt_status ( );
    disable ( );
    /* carry out the mapping between the mail box name (which is the task
    name) and the mail box number.
    */
    if ( ( mbx_num = mail_exchange ( mbx_addr_ptr ) ) >= num_of_tasks ) {
        /* trying to look at an illegal mail box */
         used = 0xffff;
    } /* if */
    else {
        /* address OK */
        used = mbx_table [ mbx_num ]->used;
    } /* else */
    if ( int_status )
        enable ( );
    return used;

} /* end of used_mbx */





/*------------------------------------------------------------------------*/





/*
============================================================================
|
| flush_mbx
|
| This function as the name implies flushes the contents of the mail box
| which is owned by the calling function. Flushing involves setting the
| get and put pointers back to zero, resetting the used and free locations,
| and clearing the qik_mess_flag to the states which exist when the mail
| box was first created. If the mail box is full at the time that this
| command is executed then it is possible that there are blocked tasks
| waiting to access the mail box. Therefore if these are not released
| then these tasks would remain blocked (even if the semaphore value has
| been reset). Therefore under this condition any blocked tasks are removed
| from the appropriate semaphore queue. The semaphore values for the mail
| box are reset to the initial values.
|
| The flush command can only be executed from the task with which the mail
| box is associated to prevent the situation where another task could
| flush the mail box whilst the task to which it belongs is left blocked
| on a semaphore waiting to receive a message.
|
| Parameters : - none
|
| Entry via : - multiple places
|
==========================================================================
*/

void flush_mbx ( void ) {

    unsigned int mbx_addr;
    char int_status;

    mbx_addr = rtn_current_task_num ( );

    int_status = return_interrupt_status ( );
    disable ( );
    mbx_table [ mbx_addr ]->get_ptr = 0;
    mbx_table [ mbx_addr ]->put_ptr = 0;
    mbx_table [ mbx_addr ]->free = mbx_table [ mbx_addr ]->q_size;
    mbx_table [ mbx_addr ]->used = 0;
    mbx_table [ mbx_addr ]->qik_mess_flag = FALSE;

    /* now check to see if there are any tasks blocked on the spce_avail_sema
    for the mail box.
    */
    while ( semaphore [ mbx_table [ mbx_addr ]->spce_avail_sema ]
              ->semaphore_value  != mbx_table [ mbx_addr ]->q_size ) {
        /* must be tasks blocked on the spce_avail_sema so remove the task
        from the semaphore queue and place on the appropriate priority
        queue.
        */
        _signal ( mbx_table [ mbx_addr ]->spce_avail_sema );
    } /* while */

    /* now reset the mess_avail_sema value */
    semaphore [ mbx_table [ mbx_addr ]->mess_avail_sema ]->semaphore_value = 0;


    if ( int_status )
        enable ( );
} /* end of flush_mbx */





/*-------------------------------------------------------------------------*/






/************************************************************************/
/*                                                                      */
/*                                                                      */
/*                              UNOS_5 MODULE                           */
/*                                                                      */
/*                                  by                                  */
/*                                                                      */
/*                              Robert Betz                             */
/*      Department of Electrical Engineering and Computer Science       */
/*                         University of Newcastle                      */
/*                              Australia                               */
/*                                                                      */
/*                        ( Copyright 1989, 1990 )                      */
/*                                                                      */
/*                                                                      */
/************************************************************************/


/*
HISTORY

Began typing this module on the 30th January 1990.

*/

/*
DESCRIPTION

The function of this module is to allow the allocation and freeing of memory
from a pseudo heap in the UNOS system. The ualloc and ufree routines
essentially follow those routines described in Kernighan and Ritchie pages
173 - 177. The main change is that the morecore routine does not call UNIX
operating system memory allocation routines but allocates initially the whole
pool size to the free block. The pool size and location have to be set up
by the user and depend on the target system. For example if UNOS is being
executed on a MS-DOS platform then the pool can be formed by gaining an
allocation from MS-DOS at system initialisation. In more embedded
environments the pool size and location have to be explicitly set variables.

For a more complete description of the allocation system refer to the UNOS
documentation or the ascii version of this documentation distributed with
this software.

Changes which need to be made to this routine in order to make it work
correctly with a segmented architecture such as the 8086 are outline in the
accompanying documentation.
*/





/*-----------------------------------------------------------------------*/


/*
==========================================================================
|
| umalloc
|
| This routine allocates the number of bytes requested from the free memory
| block list. The routine is basically a copy of the routine in the
| Kernighan and Ritchie book pages 175. This routine takes the requested
| number of bytes and rounds it to a proper number of header-sized units.
| The actual block that will allocated contains one more unit which is
| reserved for the header itself. The pointer returned from the routine is
| to the free memory area and not to the header. Functionally this routine
| behaves the same as the malloc routine in the 'C' language - hence the
| name.
|
| Initially when the routine a is executed a free list of memory blocks will not exist.
| In this circumstance the morecore routine is called which returns a
| pointer to more free memory. In an UNIX environment this routine would
| call the operating system to be allocated more memory. This is the way
| that the Kernighan and Ritchie implementation works. However in an
| embedded application this does not make sense and the morecore routine
| simply allocates the entire memory pool size to the initial free block
| of memory. I have retained the original Kernighan/Ritchie model in case
| UNOS is extended at a future stage to be a full flegged disk base os in
| its own right.
|
| If a block of free memory is found and it is exactly the correct size then
| the block is removed from the linked list of free memory blocks. If the
| block of free memory is larger than the requested block then the remainder
| of the free block after the requested amount of memory is removed is
| returned to the list of free memory blocks.
|
| If the memory allocation has been successful then a pointer to the memory
| block is returned. If unsuccessful then a NULL pointer is returned.
|
| Parameters : - number of bytes to be allocated
|
| Entry via  : - multiple places
|
============================================================================
*/

char huge* umalloc ( unsigned long num_bytes ) {

	blk_header huge *ptr1, huge *ptr2;
	char int_status;
	unsigned long blksize_units;
	unsigned int cur_seg, cur_offset, norm_seg, norm_offset;
	blk_header huge *norm_start_header_ptr;

	int_status = return_interrupt_status ( );
	disable ( );

	/* round the number of bytes so that it is an integral number of
	header sized blocks. This is done to maintain the correct byte
	alignment as forced by the union header structure. Note that the
	basic allocation unit then becomes sizeof ( blk_header ) units.
	It is this value which is stored in the blk_size variable of the
	header structure.
	*/
	blksize_units = 1 + ( num_bytes + sizeof ( blk_header ) - 1 ) /
                                    sizeof ( blk_header );

    if ( ( ptr1 = last_blk_alloc ) == NULL ) {
        /* enter here if currently no free list so set up a dummy start
        header known as start_header
		*/
        cur_seg = FP_SEG ( &start_header );
        cur_offset = FP_OFF ( &start_header );
        norm_seg = (unsigned int) (( ( unsigned long )cur_seg * 16 + cur_offset ) / 16);
        norm_offset = (unsigned int) (( ( unsigned long )cur_seg * 16 + cur_offset )
                                    - norm_seg * 16 );
        norm_start_header_ptr = (header_struc huge *)MK_FP ( norm_seg, norm_offset );
        start_header.header.nxt_blk_ptr = last_blk_alloc = ptr1 =
                                                norm_start_header_ptr;
        start_header.header.blk_size = 0;
    } /* if */

    /* now start searching through the linked list of block headers
	searching for a block of free RAM >= the rounded number of bytes
    requested.
    */
    for ( ptr2 = ptr1->header.nxt_blk_ptr; ; ptr1 = ptr2, ptr2 = ptr2->
                                                header.nxt_blk_ptr ) {
        if ( ptr2->header.blk_size >= blksize_units ) {
            /* the free block currently pointed to is big enough. Now
            check to see if it is exactly the correct size or not.
            */
            if ( ptr2->header.blk_size == blksize_units )
                /* exactly the right size so update the pointer from the
                previous free block to point to the block after the current
                block
				*/
                ptr1->header.nxt_blk_ptr = ptr2->header.nxt_blk_ptr;
            else {
                /* the block to be allocated is not the exact size of the
                free area therefore take the required storage from the end
                of the current free block.
                */
				/* change the size of the free memory block to reflect the
                new size
                */
                ptr2->header.blk_size -= blksize_units;

                /* now update the pointer ptr2 to point to the top blksize_units
				which have been taken from the top of the free area. This
                is the pointer which will be used in the return statement.
                */
                ptr2 += ptr2->header.blk_size;

                /* now update the header information for the allocated
                block of memory
                */
                ptr2->header.blk_size = blksize_units;
            } /* if */

            last_blk_alloc = ptr1;
            rem_memory -= blksize_units;

            if ( int_status )
                enable ( );

            /* now return the pointer to the memory in the allocated block */
            return ( ( char huge * ) ( ptr2 + 1 ) );
        } /* if */

        /* enter if the area in the currently pointed to free block is not
        large enough. If ptr2 is pointing to the last allocated location
        then the end of the free list has been reached and more memory
        has to be allocated. In the UNOS case this routine should only
        be called when the free list to be firstly created. The morecore
		routine passes back the entire memory pool to the free list.
		*/
		if ( ptr2 == last_blk_alloc )
			/* wrapped around free list */
			if ( ( ptr2 = ( blk_header* )morecore ( ) ) == NULL ) {
                if ( int_status )
                    enable ( );
                return NULL;    /* no free core */
            } /* if */
    } /* for */
} /* end of umalloc */





/*------------------------------------------------------------------------*/





/*
===========================================================================
|
| ret_free_mem ( )
| This function returns the amount of free memory in the heap maintained
| by this memory management software. The value returned is in bytes.
|
| Parameters    : - none
|
| Entry via     : - Multiple places in user code.
|
===========================================================================
*/

unsigned long ret_free_mem ( void ) {

    return ( rem_memory * sizeof ( blk_header ) );

} /* end of ret_free_mem() */


/*------------------------------------------------------------------------*/





/*
============================================================================
|
| morecore
|
| In the original Kernighan and Ritchie book this routine had the job of
| returning some more memory from the UNIX operating system to be dynamically
| allocated by the allocate routine. In UNOS it obviously does not do this
| since it is not operating in a UNIX environment. This routine on the first
| call returns the total pool size. On subsequent calls it will return a NULL
| to indicate that no more memory is available. In order to put the free
| memory into the free memory block list the ufree function is called with
| a pointer to the new memory area.
|
| Parameters : - none
|
| Entry via  : - umalloc function in this module
|
============================================================================
*/

static blk_header huge * morecore ( void ) {

	blk_header huge * new_core;

	if ( mem_pool_size > 0 ) {
		new_core = ( blk_header huge *) mem_pool_ptr;
		new_core->header.blk_size = mem_pool_size / sizeof ( blk_header )
										- 1;
		mem_pool_size = 0;
		ufree ( ( char huge * ) ( new_core + 1 ) );

		/* now place the new core onto the list of free memory blocks. In this
		process the last_blk_alloc is set
		*/

		return ( last_blk_alloc );
	} /* if */

	else
		return NULL;        /* no memory available */
} /* end of morecore */





/*------------------------------------------------------------------------*/





/*
===========================================================================
|
| ufree
|
| This function scans the free list starting at last_alloc_blk looking for
| the place to insert the block which is to be added to the free list. The
| location to insert the free block is determined based on the fact that the
| blocks are stored in order from the low address to the high address. This
| ordering is maintained in order to make it simple to concatenate the block
| to be added with consecutive adjacent blocks. This is designed to help
| prevent memory fragmentation.
|
| Parameters : - pointer to the memory area to be added to the free list
|
| Entry via  : - multiple places
|
============================================================================
*/

void ufree ( char huge* blk_ptr ) {

    blk_header huge *ptr1, huge *ptr2;
    unsigned long blk_size;
    char int_status;

    int_status = return_interrupt_status ( );
    disable ( );

    /* firstly make sure that the pointer is pointing to the block header
    of the free block to be added to the list
    */
    ptr2 = ( blk_header huge * )blk_ptr - 1;

    /* Now store the size of the block
    */
    blk_size = ptr2->header.blk_size;

    /* now search for the correct place to put the block in the free list.
    The following 'for' statement searches through the free list starting at
    the last_alloc_blk for the correct position to put the block
    */
    for ( ptr1 = last_blk_alloc; !( ptr2 > ptr1 && ptr2 <
            ptr1->header.nxt_blk_ptr ); ptr1 = ptr1->header.nxt_blk_ptr )
        /* now check to see if the new block has to be placed at the beginning
        or the end of the list. If so then exit this for loop, else continue to
        find the correct location.
        */
        if ( ptr1 >= ptr1->header.nxt_blk_ptr && ( ptr2 > ptr1 ||
                                            ptr2 < ptr1->header.nxt_blk_ptr ) )
            break;      /* has to be placed at one end or the other */

    /* now check to see if the block should be merged with the block
    above the block to be added.
    */
    if ( ( ptr2 + ptr2->header.blk_size ) == ptr1->header.nxt_blk_ptr ) {
        /* join to the block above and ammend the pointers in the added
        block to point to the free block next above the block previously
        above the block added.
        */
        ptr2->header.blk_size += ptr1->header.nxt_blk_ptr->header.blk_size;
        ptr2->header.nxt_blk_ptr = ptr1->header.nxt_blk_ptr->
                                                    header.nxt_blk_ptr;
    } /* if */
    else
        /* the new block is being put between two existing blocks and
        cannot be concatenated with the block above it.
        */
        ptr2->header.nxt_blk_ptr = ptr1->header.nxt_blk_ptr;

    /* now check to see if the new block has to be merged with the block
    immediately below it.
    */
    if ( ( ptr1 + ptr1->header.blk_size ) == ptr2 ) {
        /* join with the block below and then update the pointers of the
        block below to reflect the size of the new super block.
        */
        ptr1->header.blk_size += ptr2->header.blk_size;
        ptr1->header.nxt_blk_ptr = ptr2->header.nxt_blk_ptr;
    } /* if */
    else
        /* block is not adjacent so simply update the pointer in the block
        below to point to the new block.
        */
        ptr1->header.nxt_blk_ptr = ptr2;

    /* update the last_blk_alloc to ptr1 since this will cover all the
    situations of catenation ( or lack of ) above.
    */
    last_blk_alloc = ptr1;

    /* now increment the remaining memory appropriately */
    rem_memory += blk_size;

    if ( int_status )
        enable ( );
} /* ufree */





/*--------------------------------------------------------------------------*/





/*
=============================================================================
|
| ucalloc
|
| This is the UNOS version of the 'C' calloc routine. As with the 'C' version
| it allows a number of n byte sized blocks to be allocated from the heap.
| The function works by working out how many bytes this area is and then calls
| the umalloc routine to allocate the block. If the routine returns a NULL then
| the requested memory cannot be allocated, else it returns a pointer to the
| area allocated.
|
| Parameters : - number of data objects to allocate
|              - size of each data object to be allocated
|
| Entry via  : - multiple places
|
=============================================================================
*/

char huge* ucalloc ( unsigned long num_obj, unsigned long size_of_obj ) {

    unsigned long num_bytes;

    /* calculate the number of bytes required */
    num_bytes = num_obj * size_of_obj;

    return ( umalloc ( num_bytes ) );
} /* end of ucalloc */




