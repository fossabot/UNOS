/*
********************************************************************************
PLC.C

Task to communicate to the PLC.

********************************************************************************
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <dos.h>

#include <unos.h>

#include <unosdef.h>

static timer_struc* plc_cycle_timer;
static unsigned int plc_sem;

static void plc_timer ( int * temp );

/*
****************************************************************************
plc_task ( )


****************************************************************************
*/
void plc_task ( void * Dummy ) {

	int temp;
	unsigned int return_status;

	enable ( );
	Dummy = Dummy;

	plc_sem = create_semaphore();
	if(plc_sem != 0xffff)
		init_semaphore (plc_sem, 0, 1 );

	/*---- Set up any task dependent variables, incl timers */
	/* Need to run every 2 time ticks if tick = 30Hz */
	plc_cycle_timer = start_timer ( REPETITIVE, 10, plc_timer, &temp );

	while ( 1 ) {

		wait ( sequencer_sem );

	} /* end of infinite while loop */

} /* End of plc_task */



/*
****************************************************************************
plc_timer

	Routine called on the time-out of the timer created in PLC task.
This routine merely signals the sequencer semaphore. The sequencer will then
be scheduled as the next task as it should be the highest priority.

****************************************************************************
*/
void plc_timer ( int * temp ) {

	_signal ( plc_sem );
	*temp = 1;		/* avoids warning on compilation only */

} /* End of sequencer_timer */



