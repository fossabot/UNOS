/**************************************************************************/
/*                                                                        */
/*                                                                        */
/*                        TUNE KEYBOARD MODULE                            */
/*                                                                        */
/*                                     by                                 */
/*                                                                        */
/*                                 L. J. Sciacca                          */
/*         Department of Electrical Engineering and Computer Science      */
/*                            University of Newcastle					  */
/*									Austraila                             */
/*                                                                        */
/*                         ( Copyright 92 )                               */
/*                                                                        */
/*                                                                        */
/**************************************************************************/

/*
	HISTORY

	Latest: 10 march 1992

*/

/*
MODULE DESCRIPTION
------------------

	Routines to be called from the keyboard task (kbtask.c). This way static 
variables need not clutter the kbtask module and users amy change their own 
kb entry routine without intervering with others.

	Called within kbtask when the ^P selection has selected the sequencer 
page.

*/

#include <stdio.h>
#include <conio.h>
#include <dos.h>
#include <math.h>

#include "unos.h"

#include "general.h"

//#include "taskname.h"
#include "tunekbd.h"
extern unsigned char ScreenTaskName[];

/*
****************************************************************************
decode_tune_key

	Routine to decode the keys meant for the ATCU module.

Parameters:
	key_pressed - key received from the keyboard

****************************************************************************
*/
void decode_tune_key ( unsigned char key_pressed ) {

	/* Merely send the character to the screen task where the
	tune screen task is waiting to decode the character.
	*/

	send_mess ( &key_pressed, 1, ScreenTaskName );

} /* end of decode_tune_key */

