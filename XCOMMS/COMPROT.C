/* date 10 May 1993
	 Source file for crc.c version 1.0
	 by Steven Siew

	 OUTDATED!!! see slp.c for error checking protocol

	 This implement the send_protocol_task which is in charge of sending
	 the data through the serial port. To send a packet to the port
	 send an internal Unos mail to name_send_protocol_task
*/

#include <stdio.h>
#include <conio.h>
#include <dos.h>

#include "general.h"
#include "unos.h"
#include "pcscr.h"
#include "queue.h"
#include "taskname.h"


void init_server_protocol_task(void) {
;
}

void server_protocol_task (void* local_var_ptr) {

	unsigned char mess;
	unsigned int mess_lgth;
	extern queueclass fromserialqueue;

	local_var_ptr=local_var_ptr;

	/* Attach to serial channel 0 receiver task. */
	send_mess( (unsigned char *)"?",1, ch_0_rx );

	enable();

	while ( 1 ) {

		// get bytes from serial driver
		rcv_mess( &mess, &mess_lgth, 0);

		// Put byte into the queue
		fromserialqueue.enqueue(mess);

		} /* while */

} // end of server_protocol_task
