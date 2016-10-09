
#include <dos.h>
#include <stdio.h>

#include <general.h>

#include <unos.h>
#include <kbtask.h>		/* contains prototype for return_os_termination*/
#include <main_ext.h>	/* contains prototype for get_fileptr	*/

#include <hwpc.h>
#include <pcscr.h>
#include <serial.h>		/* prototypes for restore_serial_driver */
#include <wdog.h>		/* imports disable_watch_dog () */

extern void restore_keyboard ( void );

void null_task ( void * Dummy ) {

	int temp = 1;

	start_time_slice ( );
	enable ( );
	Dummy = Dummy;

	while ( temp )	{

		temp = return_os_termination ( );

		} /* while */

	/*---- Close the data-logging file */
	fclose ( get_fileptr() );

	/*---- Put DOS clock Back */
	stop_time_slice ( );

	/*---- Disable watch Dog */
	disable_watch_dog ( );

	/*---- Restore serial drivers */
	//restore_serial_drivers ( );

	/*---- Clean up interrupt drivers */
	restore_dos_mode ( );
	//restore_keyboard ( );

	/*---- Wipe the screen and return a cursor */
	/* Note that graphics should be removed and normal mode restored. */
	restore_pc_screen ( );

	enable ( );

} /* end of null_task */


