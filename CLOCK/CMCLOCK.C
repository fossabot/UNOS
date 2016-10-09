
/*****************************************************************************/
/*                                                                      	*/
/* DISCUSSION :                                                         	*/
/*			CMCLOCK.C - Time-of-day Module									*/
/*																			*/
/*  This module keeps a time-of-day record, to the nearest 16th of a		*/
/*		second, by using the CMOS clock hardware.							*/
/*																			*/
/*  Programmer:			P. Moylan											*/
/*  Last edited:		19 December 1991									*/
/*  Status:			OK														*/
/*	Working in its present form.  We still have to give some more			*/
/*	thought as to whether the present form is the one desired.				*/
/*																			*/
/*	Present version correctly compensates for BCD/binary					*/
/*	change on startup and shutdown.											*/
/*																			*/
/*	Change 19/12/91: we now interpret BINARY_CLOCK as specifying			*/
/*	the mode (binary or BCD) in which the clock hardware should				*/
/*	run, as before, but we assume that the caller is always working			*/
/*	in binary.  Thus procedures SetClock and ReadClock now have				*/
/*	appropriate BCD/binary conversions in them.								*/
/*																			*/
/*	Note on critical section protection: the three externally				*/
/*	callable functions are now protected by disabling processor				*/
/*	interrupts, to prevent a task switch which might, for example,			*/
/*	cause trouble when one task sets the clock while another is				*/
/*	reading it.  This leads to some redundant protection, since				*/
/*	there is already code present to disable clock interrupts.				*/
/*	The redundancy has been allowed to stand, on the grounds that			*/
/*	there is no good reason for tampering with code which was				*/
/*	already working.														*/
/*	The disable/enable approach might more logically be replaced			*/
/*	by semaphore protection; but the introduction of a new					*/
/*	semaphore would mean alterations in at least one other module,			*/
/*	and at this stage in the software development it seems best to			*/
/*	avoid the risk of "ripple-through" effects.								*/
/*																			*/
/*  Warning: the leap year adjustment will be wrong in the year 2100.		*/
/*  It is believed to be correct from 1990 to 2099.							*/
/*																			*/
/****************************************************************************/
/* Modification: DJLB ?/4/94.
	Reading of the IRIG board's time moved from whenever time is asked for
	to the inerrupt routine, where it is used to update MasterTime instead of
	the CMOS clock. This is more efficient than the original, as the clock is
	read by users many times more frequently than the interrupt occurrence.
*/
/* Modification: By DJLB 28/4/94.
	All references to dayofweek removed - discussed with PM. It used the CMOS
	clock's day counter, which is not set other than by a manual intervention
	for which there is no facility, and it is not corrected automatically. As
	it was only used to display the day name by ShowTime(), it was decided
	to do away with it as an error could confuse.
*/
#include <dos.h>		/* FROM DOS IMPORT setvect, getvect, inportb,	*/
						/*		outportb, disable, enable				*/
#include <stdlib.h>		/* FROM StdLib IMPORT atexit					*/
#include "unosasm.h"	// imports return_interrupt_status()
#include "cmclock.h"	/* Import my own prototypes						*/
#include "err_ex1.h"
#include "err_nums.h"

#define private static
#define Boolean unsigned char
#define TRUE 	1
#define FALSE 	0

#define AddressPort 	0x70	/* I/O ports by which CMOS can				*/
#define DataPort 		0x71	/*	be accessed								*/
#define InterruptNo 	0x70	/* hardware interrupt number				*/
#define StatusA 	10			/* status register A						*/
#define StatusB 	11			/* status register B						*/
#define StatusC 	12			/* status register C						*/
#define UF 		  0x010			/* update-ended flag in Status Register C 	*/

#define IRIG_TIMEREQ	0X190          		/* I/O address of bc620 pcb.	*/
#define IRIG_TIME0      IRIG_TIMEREQ + 1
#define IRIG_TIME1      IRIG_TIME0 + 1
#define IRIG_TIME2      IRIG_TIME1 + 1
#define IRIG_TIME3      IRIG_TIME2 + 1
#define IRIG_TIME4      IRIG_TIME3 + 1
#define IRIG_TIME5      IRIG_TIME4 + 1
#define IRIG_TIME6      IRIG_TIME5 + 1
#define IRIG_TIME_PAGE	IRIG_TIMEREQ + 0XF

/************************************************************************/
/*		    VARIABLES and FUNCTIONS GLOBAL TO THIS MODULE				*/
/************************************************************************/

/* MasterTime holds our internal record of the date and time.  A caller	*/
/* who requests the current date/time receives a copy of MasterTime.	*/
/* Note: When the CMOS clock is being used, the "hundredths" field of   */
/* MasterTime holds 16ths of a second.  The conversion to 100ths occurs */
/* at the time a copy is made for a caller.								*/

private TimeRecord MasterTime;

/* Procedure oldhandler is the interrupt routine which was in use at	*/
/* system startup.  We restore it on shutdown.							*/

private void interrupt (*oldhandler)(...);

private irig_time_is_available = FALSE;

private unsigned int days_in_month[12] 
				= { 31,28,31,30,31,30,31,31,30,31,30,31 };

private void Set_CMOSClock (TimeRecord *newtimeptr);

/************************************************************************/
/*			BCD/BINARY CONVERSIONS										*/
/************************************************************************/

private BYTE BCDtoBinary (BYTE value)

	/* Converts BCD to Binary.	*/

	{
    return 10*(value >> 4) + (value & 0x0F);
    }

/*----------------------------------------------------------------------*/

private BYTE BinarytoBCD (BYTE value)

    /* Converts Binary to BCD.	*/

    {
    return ((value/10) << 4) + (value % 10);
    }

/*----------------------------------------------------------------------*/

private void ConvertTimeRecordFromBCD (TimeRecord* p)

	/* Converts an entire time record, except the hundredths field,		*/
    /* from BCD to binary.  The hundredths field is left alone, since	*/
	/* its format is independent of whether the hardware is working		*/
	/* in BCD or binary mode.											*/

    {
    p->seconds = BCDtoBinary (p->seconds);
    p->minutes = BCDtoBinary (p->minutes);
    p->hours = BCDtoBinary (p->hours);
	p->dayofmonth = BCDtoBinary (p->dayofmonth);
    p->month = BCDtoBinary (p->month);
    p->year = BCDtoBinary (p->year);
    p->century = BCDtoBinary (p->century);
    }

/*----------------------------------------------------------------------*/

private void ConvertTimeRecordToBCD (TimeRecord* p)

	/* Converts an entire time record, except the hundredths field,		*/
    /* from binary to BCD.  The hundredths field is left alone, since	*/
	/* its format is independent of whether the hardware is working		*/
	/* in BCD or binary mode.											*/

    {
    p->seconds = BinarytoBCD (p->seconds);
    p->minutes = BinarytoBCD (p->minutes);
    p->hours = BinarytoBCD (p->hours);
	p->dayofmonth = BinarytoBCD (p->dayofmonth);
    p->month = BinarytoBCD (p->month);
    p->year = BinarytoBCD (p->year);
    p->century = BinarytoBCD (p->century);
	}

/************************************************************************/
/*			PROCEDURES TO ACCESS CMOS and IRIG							*/
/************************************************************************/

private BYTE ReadCMOS (unsigned int location)

    /* Returns the value at the given CMOS location.	*/

    {
    outportb (AddressPort, location);
    return inportb (DataPort);
    }

/*----------------------------------------------------------------------*/

private void WriteCMOS (unsigned int location, BYTE value)

	/* Stores a value at the given CMOS location.		*/

    {
    outportb (AddressPort, location);
    outportb (DataPort, value);
    }

/*----------------------------------------------------------------------*/

private void Get_IRIGTime ( BYTE time0 )
{

unsigned int 	day_of_year,
				cmosclock_month, cmosclock_year,
				days_left;

	if ( (time0 & 0X10) == 0 )
		MasterTime.irig_time_status = TRUE;
	else
		MasterTime.irig_time_status = FALSE;

	/* Get CMOS clock's month. */
	cmosclock_month = MasterTime.month;

	cmosclock_year = BCDtoBinary( MasterTime.year );
	if( (cmosclock_year % 4) == 0 )   /* Leap year? */
	days_in_month[1] = 29;
	else
	days_in_month[1] = 28;

    /* Set the bc620 page register to 0. */
	outportb( IRIG_TIME_PAGE, 0 );

	/* Load the time structure with bc620 time information. */
	day_of_year = BCDtoBinary ( (BYTE)(time0 & 0X0F) ) * 100
		  + BCDtoBinary ( inportb(IRIG_TIME1) );

	MasterTime.hours 	  = BCDtoBinary( inportb(IRIG_TIME2) );
	MasterTime.minutes 	  = BCDtoBinary( inportb(IRIG_TIME3) );
	MasterTime.seconds    = BCDtoBinary( inportb(IRIG_TIME4) );
	MasterTime.hundredths = BCDtoBinary( inportb(IRIG_TIME5) );

	/* Convert day-of-the-year to day-of-the-month and month. */
	days_left = day_of_year;
	MasterTime.month = 0;

	while( days_left > days_in_month[ MasterTime.month ] )
	{
		days_left -= days_in_month[ MasterTime.month ];
		MasterTime.month ++;
	}
	MasterTime.month ++;

	MasterTime.dayofmonth = days_left;

	/* There is a possibility of skew between CMOS clock time and bc620
	time. This will only be a problem at the end/start of the
	year, and can be allowed for by correcting the year, which has been
	obtained from the CMOS clock. */

	if( MasterTime.month == 1 && cmosclock_month == 0X12 )/* cmosclock_month*/
	{   /* The CMOS clock is behind IRIG time. */         /* is in BCD.     */
		if( cmosclock_year != 99 )                        /* cmosclock_year */
			cmosclock_year ++;                            /* is in binary.  */
		else
		{   /* Turn of the century! */
			cmosclock_year = 0;
			MasterTime.century = 0x20;
			WriteCMOS (50, MasterTime.century);
		}
	}

	if( MasterTime.month == 12 && cmosclock_month == 1 )
	{   /* The CMOS clock is ahead of IRIG time. */
		if( cmosclock_year != 0 )
			cmosclock_year --;
		else
		{
			cmosclock_year = 99;
			MasterTime.century = 0x19;
			WriteCMOS (50, MasterTime.century);
		}
	}

	MasterTime.year = cmosclock_year;
	MasterTime.century = BCDtoBinary( MasterTime.century );
}

/************************************************************************/
/*			THE INTERRUPT ROUTINE										*/
/************************************************************************/

private void interrupt InterruptHandler (...)

    /* This interrupt routine is entered once per clock tick.  A clock	*/
	/* tick occurs 16 times per second.									*/

{
BYTE 	flags, time0;
static 	BOOLEAN boMidnightPending = FALSE, WrapPending = FALSE;

	flags = ReadCMOS (StatusC);
	outportb (0xA0, 0x20);	/* EOI to controller 2 */
    outportb (0x20, 0x20);	/* EOI to controller 1 */

	/* Every second, update MasterTime from the CMOS clock or the
		IRIG time source.	*/
	if (flags & UF)
	{
		MasterTime.month		= ReadCMOS (8);
		MasterTime.year 		= ReadCMOS (9);
		MasterTime.century 		= ReadCMOS (50);

		if (MasterTime.year == 0x99)
			WrapPending = TRUE;
		else if (WrapPending && MasterTime.year == 0)
		{
			WrapPending = FALSE;
			MasterTime.century++;
			if ((MasterTime.century & 0x0F) > 9)
				MasterTime.century += 6;
			WriteCMOS (50, MasterTime.century);
		}

		/* Read bc620 TIMEREQ register to freeze the bc620 status. 	*/
		inportb ( IRIG_TIMEREQ );

		/* Bit 4 of IRIG_TIME0 is 0 if bc620 is locked
			to an external IRIG signal, 1 if it is flywheeling.

			Note that if the bc620 is not present,
			inportb( IRIG_TIME0 ) will return 0XFF, thus
			irig_time_is_available will never be true.		*/
		time0 = inportb( IRIG_TIME0 );
		if ( (time0 & 0X10) == 0 || irig_time_is_available )
		{	/* bc620 is present and IS, OR HAS BEEN, locked to external IRIG
				signal.
				irig_time_is_available stays true until power-down or
				a hard reset, even if bc620 free-wheels.					*/
			irig_time_is_available = TRUE;

			/*  Transfer the IRIG time to MasterTime.					*/
			Get_IRIGTime ( time0 );
			if( MasterTime.hours == 23 )
				boMidnightPending = TRUE;
			if( MasterTime.hours == 0 && boMidnightPending )
			{
				boMidnightPending = FALSE;
				Set_CMOSClock( &MasterTime );
            }
		}
		else
		{	/* Use CMOS clock, as the bc620 has not been locked to an
			external IRIG signal.  										*/
			MasterTime.irig_time_status = FALSE;
			MasterTime.hundredths 	= 0;
			MasterTime.seconds 		= ReadCMOS (0);
			MasterTime.minutes 		= ReadCMOS (2);
			MasterTime.hours 		= ReadCMOS (4);
			MasterTime.dayofmonth 	= ReadCMOS (7);

			ConvertTimeRecordFromBCD( &MasterTime );
        }
	}
	else
		/* At all calls except on the second:- 				*/
		MasterTime.hundredths++;
}


/************************************************************************/
/*			PROGRAM TERMINATION HANDLER									*/
/************************************************************************/

private void Cleanup (void)

	/* Disables CMOS interrupts, and sets the CMOS clock back to its	*/
	/* standard MS-DOS setup condition.									*/

    {
    WriteCMOS (StatusB, ReadCMOS(StatusB) & 0x8F);
    WriteCMOS (StatusA, 0x26);
    setvect (InterruptNo, oldhandler);
    }

/************************************************************************/
/*			     INITIALISATION											*/
/************************************************************************/

private void HardwareSetup (void)

    {

	#define PIenable	0x40		/* periodic interrupt enable		*/
	#define UIenable	0x10		/* update-ended interrupt enable 	*/
	#define mode24	2				/* 24-hour mode 					*/

	/* Select a periodic clock rate of 16 Hz.							*/

    WriteCMOS (StatusA, 0x2C);

	/* Set the desired interrupt options in status register B. 			*/

    WriteCMOS (StatusB, PIenable+mode24);

	/* Clear a bit in the interrupt mask register.  The mask			*/
    /* register address is 0x21 for the master interrupt controller,	*/
	/* 0xA1 for the slave.												*/

    outportb (0xA1, inportb(0xA1) & 0xFE);
    outportb (0x21, inportb(0x21) & 0xFB);
    }

/************************************************************************/
/*			    SETTING THE CLOCK										*/
/************************************************************************/

private void Set_CMOSClock (TimeRecord *newtimeptr)

    /* Modifies the current time and date.  Note that the hundredths	*/
    /* field of newtime is ignored, since there appears to be no way in	*/
	/* which the hardware will allow us to set a new "fraction of		*/
	/* second" time origin.  For future thought:						*/
    /*	   (a)	Does this restriction exist on all AT clones, or have	*/
	/*		we just been unlucky in the machines we tested?				*/
    /*	   (b)	Should we strive for greater accuracy in setting the	*/
	/*		clock, for example by maintaining a record of the			*/
	/*		difference between the desired setting and what the			*/
	/*		hardware holds?  At present, we judge that the cost and		*/
	/*		complexity of this modification is not justified.			*/

{
int	inIntStatus;
TimeRecord stNewTime;

		inIntStatus = return_interrupt_status();
		disable();		/* for critical section protection */

		/* Disable clock interrupts by setting a bit in the mask register	*/
		/* of the slave interrupt controller.  Also prevent any clock		*/
		/* updates by setting the high-order bit of status register B.		*/

		outportb (0xA1, inportb(0xA1) | 1);
		WriteCMOS (StatusB, ReadCMOS(StatusB) | 0x80);

		stNewTime = *newtimeptr;  /* Take a copy of the new time. */
		stNewTime.hundredths = 0;

		/* The caller-supplied time record is always in binary.				*/
		/* Convert this to BCD for CMOS clock setting.						*/

		ConvertTimeRecordToBCD (&stNewTime);

		WriteCMOS (0, stNewTime.seconds);
		WriteCMOS (2, stNewTime.minutes);
		WriteCMOS (4, stNewTime.hours);
		WriteCMOS (7, stNewTime.dayofmonth);
		WriteCMOS (8, stNewTime.month);
		WriteCMOS (9, stNewTime.year);
		WriteCMOS (50,stNewTime.century);

		/* Re-enable the clock.	*/

		outportb (0xA1, (inportb(0xA1) & 0xFE));
		WriteCMOS (StatusB, (ReadCMOS(StatusB) & 0x7F));
		/* Set MasterTime to the new time. */
		MasterTime = *newtimeptr;

		if( inIntStatus )
			enable();		/* end of critical section */
}

/************************************************************************/
/*			CALLER INTERFACE TO THE CLOCK								*/
/************************************************************************/

void ReadClock (TimeRecord *resultptr)
    {
	int temp;

	disable();					/* for critical section protection 	*/
    outportb (0xA1, inportb(0xA1) | 1);		/* disable interrupts	*/
	*resultptr = MasterTime;

	outportb (0xA1, inportb(0xA1) & 0xFE);	/* enable interrupts	*/
	enable();					/* end of critical section 			*/

    /* Adjust the "hundredths" field of the result, to account for the	*/
	/* fact that internally we have actually been counting 16ths.		*/

	temp = 25*(resultptr->hundredths) + 2;
	resultptr->hundredths = temp/4;
    }


/*----------------------------------------------------------------------*/

void SetClock( TimeRecord *newtimeptr )
{
	if( !irig_time_is_available )
		Set_CMOSClock( newtimeptr );

}

/************************************************************************/
/*			MODULE INITIALISATION										*/
/************************************************************************/

void InitialiseClock (void)

    /* Must be called at program startup.	*/

    {
    atexit (Cleanup);
    oldhandler = getvect (InterruptNo);
    setvect (InterruptNo, InterruptHandler);
    HardwareSetup();
    }

/************************************************************************/

	