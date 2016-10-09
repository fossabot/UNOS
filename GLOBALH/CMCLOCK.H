/****************************************************************************/
/*                                                                          */
/* DISCUSSION :                                                             */
/*																			*/
/*	CMCLOCK.H - PUBLIC DEFINITIONS FOR CLOCK INTERFACE MODULE				*/
/*																			*/
/*	This module keeps a time-of-day record, to the nearest 16th				*/
/*		of a second, by using the CMOS clock hardware.						*/
/*																			*/
/*	Programmer:	P. Moylan													*/
/*	Last modified:	19 December 1991										*/
/*	Status:		OK															*/
/*		This module has already been tested, and it works,					*/
/*		but there are still some policy questions to settle:				*/
/*		1.  What sort of functions to provide								*/
/*		2.  date/time formats: a structure as at present,					*/
/*		    a "seconds-since-origin", a different type of					*/
/*		    structure, all of the above.									*/
/*		This discussion might by now have converged, i.e. it				*/
/*		looks as if the present arrangement will be adopted					*/
/*		unchanged.															*/
/*																			*/
/*	Policy change 19/12/91: it has now been decided to run the				*/
/*	clock hardware in BCD mode, and to supply appropriate					*/
/*	conversions (in ReadClock and SetClock) so that the caller				*/
/*	still sees date/time records in binary.  This should solve				*/
/*	the problem where, on booting up after a power failure, the				*/
/*	initial time looks like rubbish.  The source of the problem				*/
/*	is that MS-DOS does not check the clock mode - assuming that			*/
/*	it is always BCD - and messes up the values on startup.					*/
/*                                                                      	*/
/*      5/6/92. By DJLB. Structure MasterTime modified to include			*/
/*	irig_time_status. This element is TRUE if an external IRIG      		*/
/*	timecode signal is, or has been, used to determine time & date.			*/
/*																			*/
/*		22/2/94 By DJLB Major modifications - MasterTime is now kept in     */
/*				binary, and irig time is read at the cmos clock interrupt   */
/*				rather than whenever time is requested.                     */
/*																			*/
/*																			*/
/*																			*/
/*																			*/
/****************************************************************************/

#ifndef __CMCLOCK_H
#define __CMCLOCK_H

#define BYTE unsigned char
#define BOOLEAN unsigned char
		// changed this from char to unsigned char to suprress warning

/* In the present version of this module, hundredths are binary, other	*/
/* time/date fields are binary if BINARY_CLOCK is nonzero and BCD	*/
/* encoded otherwise.  The most recent design decision has been to use	*/
/* binary for this project, with a switch back to BCD on shutdown for	*/
/* compatilibity with MS-DOS.						*/
/* Alteration 19/12/91: we now use BCD internally, but the callers	*/
/* still use binary.  This is to solve a problem caused by MS-DOS when	*/
/* starting up after an abnormal termination (i.e. a termination in	*/
/* which the shutdown routines are not called).				*/

typedef struct	{
		BOOLEAN irig_time_status;
		BYTE hundredths;
		BYTE seconds;
		BYTE minutes;
		BYTE hours;
		BYTE dayofmonth;
		BYTE month;
		BYTE year;
		BYTE century;

				} TimeRecord;

void InitialiseClock (void);

    /* Must be called at program startup.	*/

void ReadClock (TimeRecord* resultptr);

    /* Returns the current time and date.	*/

void SetClock (TimeRecord* newtimeptr);

    /* Modifies the current time and date.  Note that the hundredths	*/
    /* field of newtime is ignored, since there appears to be no way in	*/
    /* which the hardware will allow us to set a new "fraction of	*/
    /* second" time origin.						*/

#endif /* __CMCLOCK_H */
