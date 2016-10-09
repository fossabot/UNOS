/*
**********************************************************************
							utilpc.h

Product: ATAC Tracking systems

Copyright:
Australian Systems Research (NSW) Pty. Ltd.

Programmer:
L. J. Sciacca

Last Modified: 9 April 1994

**********************************************************************
*/
/* Define the hardware addresses for the 8254 counter in the IBM-PC */

#define I8254_CTRL_REG 0x43
#define I8254_COUNTER_0 0x40

/* Define the data that has to be written to the control register in
order to set the counter 0 up into rate mode.
*/
#define I8254_RATE_MODE_COUNTER_0 0x34

/* Define the data to set the timer into square wave mode */
#define I8254_SQUARE_WAVE_MODE_COUNTER_0 0x36

/* The frequency of the clock feeding the counter */
#define I8254_INPUT_FREQ 1193180L

/* Address of the 8259 Chip */
#define I8259_ADDR      0x20

/* End of Interrupt byte for the 8259 */
#define EOI_8259        0x20

extern void *fpvdSetVector ( unsigned int intlev, char* routine );

extern void SetVector ( unsigned int interlev, void* rout );

extern void Enable_IRQX ( unsigned char IRQ );

extern void Disable_IRQX ( unsigned char IRQ );

extern void setcursor ( unsigned int shape );

extern void set_new_tick ( float herz_rent_a_tick, void interrupt * tick_int );

extern void restore_old_mode ( void );

extern void restore_dos_mode ( void );

