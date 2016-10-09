/********************************************************************/
/*																	*/
/*																	*/
/*						 	HEADER FILE UNMAIN.C					*/
/*																	*/
/*									by								*/
/*								Robert Betz							*/
/*			  Department of Electrical and Computer Engineering		*/
/*						  University of Newcastle					*/
/*							 (Copyright 1992)						*/
/*																	*/
/*																	*/
/********************************************************************/




/* TASK NAMES

each task in the system is given a unique name which is used by
other tasks as the mailing address when using the mailbox mechanism.
The name can be of any length up to the limit set by the 'C' compiler
for string variables. NOTE: it is the string name which is used as the
address by the mail box routines.
*/

/* Keyboard Task */
extern char pchKeyboardTask[];

/* Screen Task */
extern char pchScreenTask[];

/* Testing tasks */
extern char name_task0[];
extern char name_task1[];
extern char name_task2[];
extern char name_task3[];
