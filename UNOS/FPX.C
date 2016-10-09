
//#pragma inline

/************************************************************************/
/*  WARNING: This copy of FPX.C is private to PJM - I've removed an	*/
/*  fnstcw because the emulator won't handle it - and should not be	*/
/*  incorporated into the final version of the ATCU software.		*/
/************************************************************************/

/********************************************************************/
/*																	*/
/*																	*/
/*					 UNOS FP COPROCESSOR ROUTINES MODULE			*/
/*																	*/
/*								  by								*/
/*																	*/
/*							  L. Sciacca							*/
/*	    Department of Electrical Engineering and Computer Science	*/
/*						University of Newcastle						*/
/*																	*/
/*						  ( Copyright 1989 )						*/
/*																	*/
/*																	*/
/********************************************************************/


/*
HISTORY

14/5/90

Floating point coprocessor ( x87 ) save and restore routines. These routines
have been borrowed from an 86 assembly module by P. Stepien.

16/2/92
Slightly modified by Robert Betz for the Cook Islands project.

*/



/*
========================================================================
|
| save_fp_regs
|
| This function saves the floating point registers. The area to pass the
| registers to is  passed to this routine as a pointer in the argument list.
|
|	Parameters : - address of area to save FP registers
|	Entry via  : - kernel routines
|
==========================================================================
*/

void save_fp_regs ( unsigned char *save_fp_area ){

/*asm	lds bx, [bp+6];*/		/* get the address of save area into ds:bx */

asm pushf;
asm push ds;
asm push bx;
asm push ax;

asm lds bx, save_fp_area;

asm	fnstcw [bx];        /* Save IEM bit status. */
asm	fnop;				/* Delay while 80187 saves control register. */
asm	fndisi;				/* Disable 80187 busy signal. */
asm	mov ax, [bx];		/* Get original control word. */

asm	fsave [bx];			/* Save the NPX state. */
						/* Save NPX context, the host can be safely */
						/* interrupted while waiting for the 80187 */
						/*to finish. Deadlock is not possible since */
						/* IEM = 1. */
asm	fwait;				/* Wait for save to finish. */
asm	mov [bx], ax;		/* Put original control word into NPX save area. */

asm pop ax;
asm pop bx;
asm pop ds;
asm popf;

} /* end of save_fp_regs */





/*-----------------------------------------------------------------------*/






/*
========================================================================
|
| restore_fp_regs
|
| This function restore the floating point registers. The area to get the
| registers from is passed to this routine as a pointer.
|
|	Parameters : - address of area with FP registers
|	Entry via  : - kernel routines
|
==========================================================================
*/

void restore_fp_regs ( unsigned char *save_fp_area ){

/* asm	lds bx, [bp+6];*/		/* get the address of the save area into ds:bx */
asm pushf;
asm push ds;
asm push bx;


asm lds bx, save_fp_area;
asm	frstor [bx];		        /* Restore the NPX state */

asm pop bx;
asm pop ds;
asm popf;

} /* end of restore_fp_regs */

