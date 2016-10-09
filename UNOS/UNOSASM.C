/* #pragma inline */

/********************************************************************/
/*                                                                  */
/*                                                                  */
/*                   UNOS ASSEMBLY LANGUAGE MODULE                  */
/*                                                                  */
/*                                by                                */
/*                                                                  */
/*                            Robert Betz                           */
/*      Department of Electrical Engineering and Computer Science   */
/*                      University of Newcastle                     */
/*                                                                  */
/*                        ( Copyright 1989 )                        */
/*                                                                  */
/*                                                                  */
/********************************************************************/


/*
HISTORY

13/4/89

This module was developed for the 'C' versions of Unos to provide access
to various low level features of the host machine not directly available
from 'C'.

*/


char return_interrupt_status ( void );

/*
========================================================================
|
| return_interrupt_status
|
| This function returns the status of the interrupt flag in 8086 series
| microprocessors.  An assembly language routine is required since this
| flag is not directly available with most 'C' compilers.
|
| The flags are obtained by using inline assembly to push then directly
| on the stack and then pop them into the AX register.  From here they are
| placed into a variable where thay are masked and shifted so that the
| required interrupt status is in the lsb bit of the returned value.
|
|   N.B. '1' => interrupts enabled
|        '0' => interrupts disabled
|
|   Parameters : - none
|   Entry via  : - kernel routines
|
==========================================================================
*/

char return_interrupt_status ( void )
{
    asm pushf;
    asm pop ax;

    return ( ( _AX & 0x0200 ) >> 9 );
}
