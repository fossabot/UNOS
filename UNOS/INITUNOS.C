/****************************************************************************
* POLYTRON VERSION CONTROL SOFTWARE _ PVCS (tm) Information
*
*      $Revision:   1.6  $
*      $Date:   16 Sep 1992 15:26:04  $
* 
*****************************************************************************
*****************************************************************************
* Tracksat Project.
*****************************************************************************
* Copyright (c) 1991. The University of Newcastle Research Associates Ltd.
*****************************************************************************
*
* PROGRAMMER :-     
*                   Industrial Electronics Division,
*                   TUNRA Ltd.,
*                   University of Newcastle, NSW 2308.
*
*****************************************************************************
*****************************************************************************
* MODULE :-  UNOS                 File : INITUNOS.C
****************************************************************************/

/************************************************************************/
/*                                                                      */
/*                                                                      */
/*              University of Newcastle Operating System                */
/*                                                                      */
/*                              ( UNOS )                                */
/*                                                                      */
/*                             Version 1.5                              */
/*                                                                      */
/*                         MAIN PROGRAM MODULE                          */
/*                                                                      */
/*                                 by                                   */
/*                                                                      */
/*                             Robert Betz                              */
/*             Department of Electrical and Computer Engineering        */
/*                        University of Newcastle                       */
/*                                                                      */
/*                          ( Copyright 1989 )                          */
/*                                                                      */
/*									*/
/*		Last edited: 15 June 1992				*/
/*									*/
/************************************************************************/


/*
HISTORY

27/3/89
Began typing in the preliminary version of a test system for the 'C' version
of UNOS.  This initially consists of two tasks with no time slice interrupts
running.  The task swiches are to occur via direct operating system calls to
reschedule.
*/

/************************************************************************/
/*									*/
/*		TAILORING FOR COOK ISLANDS PROJECT			*/
/*									*/
/* PJM 25/2/92: eliminated task1, task2, task3 from the demo program	*/
/* as a first step towards building a version suitable for the Cook	*/
/* Islands project.  In future should also eliminate task0, but I'm	*/
/* leaving it in for now in order to have something interesting		*/
/* happening on the screen until I have some further tasks running.	*/
/*									*/
/* PJM 26/2/92: added InitialiseModules()				*/
/*									*/
/* PJM 28/2/92: Split up the main procedure into two procedures called	*/
/* InitUNOS and GoUNOS, and renamed this module INITUNOS.C; that is,	*/
/* the main program is no longer contained in this module.		*/
/*									*/
/* PJM 19/3/92: Eliminated the test task (task 0 or task 1, depending	*/
/*		on which bit you read).					*/
/*									*/
/* RHM 15/6/92: Add an _setcursortype(_NORMALCURSOR) to the exit	*/
/*		exit procedures.					*/
/*									*/
/************************************************************************/

/*
DESCRIPTION

This module contains the main program and the associated initialisation
routines.  This would be the main module modified by a user as more tasks,
semaphores and communication buffers are added to the system.

*/

#include <general.h>
#include <stdio.h>
#include <stdlib.h>	/* FROM stdlib IMPORT exit, atexit, EXIT_FAILURE */
#include <dos.h>
#include <conio.h>
#include <alloc.h>
#include <math.h>
#include "unos.h"
#include "hwpc.h"
#include "kbddrv.h"
#include "screen.h"
#include "..\globalh\time.h"

/* QUERY PJM 26/2/92: do the following really need to be exported?	*/

void initialisation (void);
void software_initialise (void);
void hardware_initialise (void);
void fvdRestore (void);

/************************************************************************/
/*                                                                      */
/*                          EXTERNAL DECLARATIONS                       */
/*                                                                      */
/************************************************************************/

/*
extern void task0 ( void );
extern void init_task0 ( void );
*/
extern void null_task ( void* );

	/***********************************************/
	/*                                             */
	/*          Commonly Changed Variables         */
	/*                                             */
	/***********************************************/


    /*------ define the maximum number of tasks in the system ------*/
    /* note that task numbers start from zero. Also note that is
    always one task in the system - the null task. The variable
    max_num_user_tasks below is the maximum number of USER tasks,
    and therefore excludes the null task
	*/
	#define MAX_NUM_USER_TASKS 32


	/*------ Define the user task names ------*/
	/* each task in the system is given a unique name which is used by
	other tasks as the mailing address when using the mailbox mechanism.
	The name can be of any length up to the limit set by the 'C' compiler
	for string variables. NOTE: it is the string name which is used as the
	address by the mail box routines.
	*/
/*
	char name_task0[] = "Task 0 - test task";
*/
	char pchKeyboardTask[] = "Keyboard Task";
	char pchScreenTask[] = "Screen Task";

	/* null task name always defined */
	char name_null_task[] = "Null task";


	/* the total number of tasks in the system - user tasks
	plus the null task
	*/
	#define MAX_NUM_OF_TASKS ( MAX_NUM_USER_TASKS + 1 )


	/* now define the size of the mail boxes to be used by the tasks in the
	system.
	*/
	#define MESS_Q_SIZE_KEYBD_TASK 1
	#define MESS_Q_SIZE_SCRN_TASK 8

/*
	#define MESS_Q_SIZE_TEST1_TASK 8
	#define MESS_SIZE_TEST1_TASK 80
*/
	#define MESS_SIZE_KEYBD_TASK 1
	#define MESS_SIZE_SCRN_TASK 3

	/*------ define the number of semaphores in the system ------*/
	/* Note that the semaphore numbers start from 0. The system requires
	a minimum of num_tasks * 2 semaphores to support the mailbox
	mechanism. The additional semaphores that the user creates are
	stored as a pool of semaphores which can be obtained by tasks that
	require semaphores.
	*/

	#define MAX_NUM_SEMAPHORES 200


	/*------ Task stack specifications ------*/

	#define TASK_STACK_SIZE  0x1000              /* task stack size */


	/*------ interrput number for kernel entry ------*/

	#define KERNEL_ENTRY 96

	/*------ number of clock ticks before time slice kernel entry ------*/
	/*------         (now defined in TIME.H: RHM 3/7/92.)         ------*/


	/*------ Memory pool declarations ------*/

	/* start of the memory pool */
	char* ptr_to_memory_pool = NULL;

	/* size of the memory pool */
	#define MEMORY_POOL_SIZE 0x30000L


    /* define the maximum number of timers in the system */
    #define MAX_NUM_TIMERS 50


/************************************************************************/
/*                                                                      */
/*                            GLOBAL DECLARATIONS                       */
/*                                                                      */
/************************************************************************/


void interrupt (*pfvdOldKernelPositionVector)();
void interrupt (*pfvdOldTickInterruptRoutine)();



/*==========================================================================*/


/**************************************************************************/
/*                                                                        */
/*                      HARDWARE RELATED DEFINTIONS                       */
/*                                                                        */
/**************************************************************************/





/*
============================================================================
|
| fvdRestore
|
| This is the exit fucntion which is called to restore the interrupts related
| to UNOS operation - namely the kernel entry interrupt and the tick interrupt.
| In addition the screen is reset to a standard screen.
|
| Parameters	:	none
|
| Returns		:	nothing
|
============================================================================
*/

void fvdRestore (void) {

	/* restore the timer back to its original mode */
	disable ();
	outportb (I8254_CTRL_REG, I8254_SQUARE_WAVE_MODE_COUNTER_0);
	outportb (I8254_COUNTER_0, 0xff);
	outportb (I8254_COUNTER_0, 0xff);

	/* Now restore the interrupt vector back */
	fpvdSetVector (KERNEL_ENTRY, pfvdOldKernelPositionVector);
	fpvdSetVector (8, pfvdOldTickInterruptRoutine);
    window (1, 1, 80, 25);
    textcolor (LIGHTGRAY);
    textbackground (BLACK);
    clrscr ();
    _setcursortype(_NORMALCURSOR);

} /* end of fvdRestore */





/*-----------------------------------------------------------------------*/





/*
===========================================================================
|
| hardware_initialise
|
| The function of this routine is to initialise all the hardware related
| items in the system.
|
===========================================================================
*/

void hardware_initialise ( void )
{
	unsigned int uin8254CountValue;

	/* now set up the 8254 timer so that the desired tick rate is obtained
	for the system.
	*/
	uin8254CountValue = (unsigned int)ceil(1 / (1 /
					(double)(I8254_INPUT_FREQ) *
						(double)(TICK_INTERRUPTS_PER_SEC)));

	outportb (I8254_CTRL_REG, I8254_RATE_MODE_COUNTER_0);
	outportb (I8254_COUNTER_0, (unsigned char)(uin8254CountValue));
	outportb (I8254_COUNTER_0, (unsigned char)(uin8254CountValue >> 8));

}





/*------------------------------------------------------------------------*/

/*
===========================================================================
|
| software_initialise
|
| The function of this procedure is to initialise all the software entities in
| the operating system.
|
|   Parameters : - none
|
|   Entry via  : - initialisation
|
===========================================================================
*/

void software_initialise ( void ) {

	int inI;

	/* set up all the OS main data structures */
	if ( !setup_os_data_structures ( KERNEL_ENTRY, ENTER_KERNEL_VALUE,
			NUM_OF_PRIORITIES, MAX_NUM_SEMAPHORES, MAX_NUM_OF_TASKS,
			ptr_to_memory_pool, MEMORY_POOL_SIZE ) ) {
		cprintf ("\n Problem setting up the OS data structures\n" );
		exit (EXIT_FAILURE);
	} /* if */


    /* now create the timers */
    for (inI = 0; inI < MAX_NUM_TIMERS; inI++) {
    	if (create_timer () == NULL) {
        	cprintf ("\n\rProblem creating timers");
            exit (EXIT_FAILURE);
        } /* if */
    } /* for */


	/*
	initialise the interrupt vector for the kernel entry
	*/

	disable ( );

	/*
	set up the kernel entry interrupt
	*/

	pfvdOldKernelPositionVector = fpvdSetVector (KERNEL_ENTRY, kernel);

	/*
	now set up the tick routine interrupt vector
	*/
	pfvdOldTickInterruptRoutine = fpvdSetVector (8, tick);

	/* now set up an exit routine */

	atexit (fvdRestore);

	clrscr ( );
	gotoxy ( 30, 1 );
	cprintf ( "UNOS OPERATING SYSTEM" );
} /* end of software_initialise */





/*-----------------------------------------------------------------------*/





/*
==========================================================================
|
| initialisation
|
| The function of this procedure is to initialise all the software and
| hardware entities in the system related to the operation of the OS software.
| Note that the initialisation of the software data structures related to all
| the user tasks is carried out in the user tasks.
|
|   Parameters : - none
|
|   Entry via  : - main routine
|
==========================================================================
*/

void initialisation ( void )

    {
    software_initialise ( );
    hardware_initialise ( );
    }

/************************************************************************/
/*                                                                      */
/*                              MAIN PROGRAM                            */
/*                                                                      */
/************************************************************************/


void InitUNOS ( void ) {

    /* Carries out all of the UNOS initialisation EXCEPT for creating	*/
    /* the null task and running the system.				*/

    /* have to get some memory from DOS before doing anything */

    ptr_to_memory_pool = farmalloc ( MEMORY_POOL_SIZE );

    disable ();

    /* Firstly initialise all the software data structures and the	*/
    /* necessary hardware in the system.				*/

    initialisation ();

    /********************************************************************/
    /* now begin the task creation process				*/
    /* NOTE ADDED BY PJM 28/2/92: These are the tasks internal to UNOS.	*/
    /* The application-level tasks can be created after this function	*/
    /* returns but before GoUNOS is called.				*/
    /********************************************************************/

    /* Set up the keyboard task */

    create_task ( pchKeyboardTask, PRIORITY_1, 0, TASK_RUNNABLE,
					PRIORITY_Q_TYPE, 0,	TASK_STACK_SIZE,
					MESS_Q_SIZE_KEYBD_TASK, MESS_SIZE_KEYBD_TASK,
					fvdInitScanCodeTranslatorTask,
					fvdScanCodeTranslatorTask, NULL );
    /* Set up the screen task */
    /* Needs a slightly bigger stack size then others. */
    /* 1400Hex seems to work fine. RHM 7/6/92 */
    create_task ( pchScreenTask, PRIORITY_7, 0, TASK_RUNNABLE,
				PRIORITY_Q_TYPE, 0,	0x1400,
				MESS_Q_SIZE_SCRN_TASK, MESS_SIZE_SCRN_TASK,
				fvdInitScreenTask, fvdScreenTask, NULL );

    /* Test task 1 */
/*
    create_task ( name_task0, PRIORITY_7, 0, TASK_RUNNABLE, PRIORITY_Q_TYPE,
					0, TASK_STACK_SIZE, MESS_Q_SIZE_TEST1_TASK,
					MESS_SIZE_TEST1_TASK, init_task0, task0 );
*/
    }

/************************************************************************/

void GoUNOS (void)

    /* Creates the null task, and starts UNOS.  It is assumed that	*/
    /* InitUNOS has already executed successfully.			*/

    {
    /* Set up the null task	*/

    create_task ( name_null_task, NULL_PRIORITY,
    			0, TASK_RUNNABLE, DONOT_Q_TYPE, 0, 0, 0, 0,
			NULL, null_task, NULL );

    /* Now start the null task and make all the other tasks start up. */

    start_tasks ( null_task );

    }		/* end of GoUNOS */
