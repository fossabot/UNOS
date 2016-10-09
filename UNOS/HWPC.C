
/*
****************************************************************************

						utilpc.c
					
					Utility Routines to provide compatibility
					with platform dependent procedures.

Author:	L. J. Sciacca

****************************************************************************
*/
//#include "config.h"
#include "unosasm.h"

#include <dos.h>

void SetVector ( unsigned int intlev, void* routine );
void Enable_IRQX ( unsigned char IRQ );
void Disable_IRQX ( unsigned char IRQ );
void setcursor ( unsigned int shape );


/*
****************************************************************************
fpvdsetvector

	This routine loads the interrupt vector directly. This was deemed necessary as
the setvect routine provided by TurboC uses BIOS which reenables interrupts. This is not desirable when setting up drivers
etc.

****************************************************************************
*/
void *fpvdSetVector ( unsigned int intlev, char* routine ) {
	void **fn_ptr;
	void *oldisr;

	fn_ptr = (void **) MK_FP ( 0, (4*intlev) );
	oldisr = *fn_ptr;
	*fn_ptr = routine;
	return (oldisr);
} /* end of setvector */

/*
****************************************************************************
setvector

	This routine loads the interrupt vector directly. This was deemed necessary as
the setvect routine provided by TurboC uses BIOS which reenables interrupts. This is not desirable when setting up drivers
etc.

****************************************************************************
*/
void SetVector ( unsigned int intlev, void* routine ) {
	void **fn_ptr;

	fn_ptr =(void**) MK_FP ( 0, (4*intlev) );
	*fn_ptr = routine;

} /* end of setvector */

/*
***********************************************************************
Enable_IRX

	Routine to enable interrupt masks on the 8259 interrupt controller for 
the PC.

A number from 0 to 7 will determine which interrupt will be unmasked.
NOTE. Bits 0,1 and 6 must all ways be active as they are used by the system.
Bit 0 - System timer/clock 
    1 - The keyboard   
	6 - Disk controller

************************************************************************
*/
void Enable_IRQX (unsigned char IRQ ) {

	unsigned char	mask = 1;

    mask = ~( mask << IRQ );

    outportb( 0x21, (inportb( 0x21 ) & mask) );

}  /*end of Enable_IRQX */

/*
**************************************************************************
Disable_IRX

	Routine to disable interrupts on the 8259 interrupt controller.

****************************************************************************
*/
void Disable_IRQX ( unsigned char IRQ ) {

    unsigned char	mask = 1;

    mask = ( mask << IRQ );

    outportb( 0x21, inportb( 0x21 ) | mask );

} /* end of Disable_IRX */

/*
****************************************************************************
setcursor

	Routine to change cursor characteristics.

Parameters:
	Input:
		shape - integer value. 0x2000 turns cursor off.
							   _NORMALCURSOR - normal cursor
****************************************************************************
*/
void setcursor ( unsigned int shape ) {
 union REGS reg;

 reg.h.ah = 1;
 reg.x.cx = shape;
 int86(0X10, &reg, &reg);

} /* setcursor */



