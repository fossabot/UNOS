#include <stdio.h>
#include "general.h"
#include "unos.h"
#include "pcscr.h"
#include "cmclock.h"

struct timestruct
{ int year;  // 91,92,93
	int month;
	int day;
	int hour;
	int minute;
	int second;
	int hund;
	float fsec;
	double djsec;   // double julian seconds
} currtime;

/*
struct	TimeRecord {
		BYTE hundredths;
		BYTE seconds;
		BYTE minutes;
		BYTE hours;
		BYTE dayofweek;
		BYTE dayofmonth;
		BYTE month;
		BYTE year;     // year 91,92,93
		BYTE century;
				};
*/

int daysofmonth[13]={0,31,28,31,30,31,30,31,31,30,31,30,31};

extern long julday(int day, int month, int year);

double julsec(int day, int month, int year,
							int hour, int minute, double second)
		/*
		Subroutine to return the epoch in Julian seconds corresponding to
		the time (hour, minute, second) on date (day, month, year).
		Call:
					jul_epoch = juldat(day, month, year, hour, minute, second)
		where
			 day          is the day of the month
			 month        is the month of the year
			 year         is the year in full (i.e. 1991 rather than 91)
			 hour         is the hour of the day, Universal time
			 minute       is the minute of the hour
			 second       is the second of the minute
		Note that second could be changed to a double precision quantity
		to allow for fractions of a second. No change in the body of the
		routine would be required.
		*/
{
		double tempdays;
		tempdays =  (double) julday(day, month, year)   -  0.5;
		tempdays =	tempdays*86400UL + second + minute*60.0 + hour*3600;
		return (tempdays);
}

extern void ReadClock (TimeRecord* resultptr);

	/* Returns the current time and date.	*/

TimeRecord  *MasterTime;

unsigned int clock_semaphore;

void init_time_task(void)
{
	clock_semaphore = create_semaphore ( );
	init_semaphore ( clock_semaphore, 0, 0 );
	MasterTime=(TimeRecord *)ucalloc(1,sizeof(TimeRecord));
}

void time_task(void* local_var_ptr)
{ unsigned char oldhund;
	int yearnum;
	local_var_ptr=local_var_ptr;

	ReadClock(MasterTime);
	currtime.hour=(int)MasterTime->hours;
	currtime.minute=(int)MasterTime->minutes;
	currtime.second=(int)MasterTime->seconds;
	currtime.hund=(int)MasterTime->hundredths;
	currtime.year=(int)MasterTime->year;   // 91,92,93
	currtime.day=(int)MasterTime->dayofmonth;
	currtime.month=(int)MasterTime->month;
	oldhund=currtime.hund;
	while(1)
	{ timed_wait ( clock_semaphore, 1 );
	ReadClock(MasterTime);
	currtime.hund=(int)MasterTime->hundredths;
	if (currtime.hund<oldhund) currtime.second++;
//	currtime.second++;  // speedup 18 times
//	currtime.minute=currtime.minute+1; // speedup 100 times;
	oldhund=currtime.hund;
	// renormalise time
	if (currtime.second>59)
		{ currtime.minute++; currtime.second=currtime.second-60; }
	if (currtime.minute>59)
		{ currtime.hour++; currtime.minute=currtime.minute-60; }
	if (currtime.hour>23)
		{ currtime.hour=currtime.hour-24; currtime.day++; }

	// A leap year is a year which is divisible by 4 but not by 400
	if (currtime.year > 50 ) { yearnum=1900+currtime.year; }
	else { yearnum=2000+currtime.year; }

	if ( (currtime.day==29) && (currtime.month==2)
			 && !(yearnum % 4) && (yearnum % 400) )
	{ /* if 29 FEB on a leap year do nothing */ }
	else
	{ if ( currtime.day > daysofmonth[currtime.month] )
		{ currtime.day=1; currtime.month++; }
	}

	if ( currtime.month > 12 )
		{ currtime.month=1; currtime.year++; }

	currtime.fsec=(float) currtime.second + (float) currtime.hund / 100.00;
	currtime.djsec=
					julsec(currtime.day, currtime.month,currtime.year,
								 currtime.hour, currtime.minute,
								 (double) currtime.fsec );
	}
}