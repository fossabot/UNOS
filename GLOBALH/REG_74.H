/*	File:	reg_74.h
****************************************************************************
*                                                                          *
*  Copyright (C) 1988      A.D. McGuffog.                                  *
*                                                                          *
*  The information in this document is subject to change  without  notice  *
*  and  should  not  be  construed  as a  commitment.                      *
*  No responsibility is assumed for any errors that may appear in          *
*  this document or for the use  or  reliability  of any portion of this   *
*  document or the described software.                                     *
*                                                                          *
*  General permission to copy or modify, but not for  profit,  is  hereby  *
*  granted,  provided  that  this entire copyright notice (the first 62	   *
*  lines of this file) is included in unmodified form and information on   *
*  the nature of any changes is included in the space below.               *
*                                                                          *
****************************************************************************

****************************************************************************
**************************************************************************** 

				MODULE FUNCTION :

        Register definition module of the PC-74 driver system. 

**************************************************************************** 

	Written : A.D.McGuffog			March 1988.

****************************************************************************

	Languge : Microsoft C V4.0./Turbo C V1.0

**************************************************************************** 

	Other Software Required : None.

**************************************************************************** 

	Hardware required : IBM PC/XT/AT or compatible with 256K of memory, 
			    and a PC-74.

**************************************************************************** 

	Restrictions : None.

**************************************************************************** 

				Module description :

        This module provides register definitions for both the PC-74 and
		  certain PC hardware.        

		  This module can be compiled by almost all C compilers, and is
		  suitable for use on UNIX/XENIX systems.

****************************************************************************
**************************************************************************** 

       Modifications : 

**************************************************************************** 
****************************************************************************/ 

/* Firstly, some def's for the 8259 on the system board. */
#define	int_cr		0x20		 			/* Interrupt control register. */
#define	int_eoi		0x20		 			/* Non specific EOI. */

#define	int_mr		0x21		 			/* Interrupt mask register. */

#define	masked		1						/* Interrupt masked (also DMA). */ 	
#define	enabled		0						/* Interrupt enabled. */

/* Now we define a bit field for the IMR. */

typedef union {

	int by;
	struct {
	
		unsigned irq0 : 1;  			 /* Timer mask. */
		unsigned irq1 : 1;  			 /* Keyboard/RTC/Pointing device. */
		unsigned irq2 : 1;  			 /* Video. */
		unsigned irq3 : 1;  			 /* Serial. */
		unsigned irq4 : 1;  			 /* Serial. */
		unsigned irq5 : 1;  			 /* Fixed disk. */
		unsigned irq6 : 1;  			 /* Diskette. */
		unsigned irq7 : 1;  			 /* Parallel port. */

		} bi;
} imr_bits;

/* Then we do the defs for the 8237 DMA controller. */ 

#define	dma_md		0xb		 			/* DMA mode register. */

#define	single_xfr	0x11					/* Single transfer. */ 	
#define	demand_xfr	0x01					/* Demand mode transfer. */ 	

/* Now we define a bit field for the mode register. */

typedef union {

	int by;
	struct {
	
		unsigned chan : 2;  			 /* Channel select bits */
		unsigned xfr_t : 6; 			 /* Type of transfer. */

		} bi;
} dma_md_bits;

#define	dma_mask		0xa		 			/* DMA single bit mask register. */

/* Now we define a bit field for the mask register. */

typedef union {

	int by;
	struct {
	
		unsigned chan : 2;  			 /* Channel select bits */
		unsigned mask : 1; 			 /* Mask bit. */

		} bi;
} dma_msk_bits;


#define	dma_ff		0xc		 			/* DMA byte pointer reset. */

#define	dma_sts		0x8		 			/* DMA status register. */
#define	dma_base		0x0		 			/* DMA base address register. */
#define	dma_count	0x1		 			/* DMA count register. */
#define	dma_page_1	0x83		 			/* DMA CH 1 page register. */
#define	dma_page_3	0x82		 			/* DMA CH3 page register. */


/* Now the def's for the PC-74 itself. */ 


#define	adcsr		base_74		 			/* Control/status register */

/* Now we define a bit field for the control/status register. */
/* Some of these bits are read, some write and some read/write. */

typedef union {

	int by;
	struct {
	
		unsigned mode : 2;  			 /* Clock mode bits. R/W. */
		unsigned	i_en : 1;			 /* Interrupt enable. R/W. */
		unsigned	d_en : 1;			 /* DMA enable. R/W. W only in PC-74 mode. */
		unsigned	clr_err : 1;		 /* Clear error condition. W. */
		unsigned	busy : 1;			 /* A/D busy. R. */
		unsigned	error : 1;			 /* Error. R. */
		unsigned	done : 1;			 /* A/D done. R. */
        } bi;
} adcsr_bits;


#define	adgcr		(base_74	+ 1)			/* Gain/channel register */

/* Now we define a bit field for the gain/channel register. */

typedef union {

	int by;

 	struct {

		unsigned chan : 4;			 /* Channel address. R/W. */
		unsigned  : 2;			 		 /* Not used. */
		unsigned gain : 2;			 /* Gain bits. R/W. */

	} bi;
} adgcr_bits;

#define	addatl	(base_74	+ 2)			/* A/D low byte. R. */

#define	dadatl0	(base_74	+ 2)		   /* D/A 0 low byte. W. */

#define	addath	(base_74	+ 3)			/* A/D high byte. R. */

#define	dadath0	(base_74	+ 3)		   /* D/A 0 high byte. W. */

#define	dadatl1	(base_74	+ 4)		   /* D/A 1 low byte. W. */

#define	dadath1	(base_74	+ 5)		   /* D/A 1 high byte. W. */

#define	diop0		(base_74	+ 6)		   /* Digital input port. R. */

#define	diop1		(base_74	+ 6)		   /* Digital output port. W. */

#define	tmrctr	(base_74	+ 7)		   /* Timer/counter. R/W. */

#define	w_done	while (!(inp(adcsr)&0x80))	 /* Macro to check for A/D done. */

#define	w_busy	while ((inp(adcsr)&0x20))	 /* Macro to wait until current conversion done. */

