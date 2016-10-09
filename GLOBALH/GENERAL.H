#ifndef __GENERAL_H
#define __GENERAL_H
/************************************************************************/
/*									*/
/*				GENERAL.H				*/
/*									*/
/*		Collection of some commonly used definitions		*/
/*			for use by all modules				*/
/*									*/
/*	Programmer:	P. Moylan					*/
/*	Last modified:	24 April 1992					*/
/*	Function:	This file contains definitions which		*/
/*			are needed in almost all modules.		*/
/*	Status:		OK						*/
/*									*/
/************************************************************************/

/************************************************************************/
/*				DATA TYPES				*/
/************************************************************************/

#define BOOLEAN unsigned char
#define TRUE 1
#define FALSE 0

#define true 1
#define false 0

#define BYTE unsigned char

#define	NULL 0L			/* null pointer	*/

/************************************************************************/
/*			USEFUL CHARACTER CODES				*/
/************************************************************************/

#define CTRL_P 0x10
#define CTRL_Q 0x11
#define CTRL_S 0x13
//#define LF '\x0A'
//#define CR '\x0D'

/************************************************************************/
/*		   REMEDIES FOR COMMON C CODING PROBLEMS		*/
/************************************************************************/

#define private static
#define AND &&
#define OR ||
#define EQ ==
#define NE !=

/************************************************************************/
/*			CONSTANTS USED BY UNOS				*/
/************************************************************************/

    /*------ Task status bit definitions ------*/


    #define TASK_RUNNABLE 0x1
    #define TASK_BLOCKED 0x0

    /*------ timer related definitions ------*/

    #define INACTIVE 0
    #define ACTIVE 1
    #define SINGLE_SHOT 0
    #define REPETITIVE 1

    /*------ PRIORITY RELATED DEFINITIONS ------*/

    /* The following number is the number of priorities including the	*/
    /* null priority.  The priorities are numbered from 1 to		*/
    /* NUM_OF_PRIORITIES with the lowest number being the highest	*/
    /* priority (i.e. 1 is the highest priority )			*/

    #define NUM_OF_PRIORITIES 8

    /*------ Task priority definitions ------*/

    #define PRIORITY_1 1
    #define PRIORITY_2 2
    #define PRIORITY_3 3
    #define PRIORITY_4 4
    #define PRIORITY_5 5
    #define PRIORITY_6 6
    #define PRIORITY_7 7
    #define NULL_PRIORITY 8

    /*------ Task stack specifications ------*/

    #define TASK_STACK_SIZE  0x1000


#endif /* __GENERAL_H */
