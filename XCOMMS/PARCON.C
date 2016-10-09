/* FILE parcon.c */
#include <stdio.h>
#include "general.h"
#include "unos.h"
#include "pcscr.h"
#include <string.h>

extern unsigned char current_screen; // in main.c
extern float old_track_elevation_desired; // in sequence.c
extern float old_elevation_desired;       // in sequence.c
extern float track_elevation_desired;     // in sequence.c
extern float requested_elevation_desired; // in sequence.c
extern float track_azimuth_desired;       // in sequence.c
extern float old_azimuth_desired;         // in sequence.c
extern double  star_ra;      // in sequence.c
extern double  star_dec;     // in sequence.c
extern int QUITING;          // in sequence.c
extern BOOLEAN CONTINUE;     // in nulltsk.c
extern BOOLEAN STAR;         // in main.c
extern BOOLEAN MODE_TRACK;   // in main.c
extern SEQUENCER_STATE;      // in main.c
extern DAC_STATE;            // in main.c
extern int currentsatnum;    // in main.c
extern float   min_el;       // in sequence.c
extern float   max_el;       // in sequence.c
extern int el_desired_error_flag; // in sequence.c

extern struct timestruct    // in rclock.c
{
	 int year;
	 int month;
	 int day;
	 int hour;
	 int minute;
	 int second;
	 int hund;
	 float fsec;
} currtime;

extern double RiseTime ( double *azinitial, double *elptr, double *azfinal); // in orbit.c
extern void GetDate(long DayNum,int *Year,int *Month,int *Day); // in orbit.c


// local global variables
double SatRiseTime=0.0, SatAzinitial=0.0,SatElptr=0.0,SatAzfinal=0.0;
int TrueMinute,RiseMinute,RiseHour,RiseDay,RiseMonth,RiseYear;
/* Unos Simple Parser version 1.0 By Steven Siew (copyright) 24-may-1993
	 Adapted from Simple Parser version 7.0 by Steven Siew

	This parser is simple to set up and simple to use.
	The main functions are

	void init_parser_table(errfuncptr)

		 This set up the empty parser tree, the errfuncptr is a pointer to a
		 function which is called is the parser is unable to parse the command.


	int  addcommand(char *commandstr,commandfuncptr)

		 This enter each command into parser tree,it returns the numoftableentry

	Special characters for the commandstr
	'#' is the numeric data ie. 1.233 and 45.67 in "move 1.233,45.67[RETURN]"
	'$' is at least one space ie. " " or "   " or "                 "
	examples: "move$#,#" or "set$speed$#" or "set$valve$num$#$to$#"


	int  charparser(char charac,int *state)

	This function excepts one character at the time and update the state or
	position on the parser tree. Initially the state must be 0, and each
	character of the string to be parser should be enter one by one including
	the [RETURN] char.


	int  parser(char *string)
								This is another parser that you can use.
								It parsers the entire string. It returns
								the same info as charparser()

	UPDATE:

	1) Include number handling ability ie "Wait 45[RETURN]"
	2) Include multiple space handling ability
	3) Include comments
	4) Include backspace function
	5) Include second number handling ability ie. "Move 182,67[RETURN]"
	6) Got rid of dummy char in parser tree. Improve efficiency by 99%
	7) Got rid of numbers in numberstring when unable to parse
	8) Include ability to handle leading spaces before command
	9) Got rid of '%' as second number indicator
 10) Include function call support
 11) Include ability to handle spaces at the end of command
*/
// -------------------------------------------------------------------------
// Parser code starts here. Be very careful if you change it
const unsigned char unabletoparse=0;
const unsigned char ctrlM='\r';   // [RETURN]
const unsigned char numbertoken='#';
const unsigned char spacetoken='$';

struct ParserTableStruct
{ unsigned char key;
	int next;     // Note: max table array size for
	int link;     //       1) compact model is 9300
								//       2) using huge keyword is 18600
	void (* command) (char *,char *); // pointer to function
} parsertable[400];
				// WARNING!! You must provide enough space for the entries
				// array size = total num of chars in commands + 2
				//              + num of commands

int buildtree(char *commandstr,void (*func) (char *,char *) );

void init_parser_table(void (*errfunc) (char *,char *))
{ parsertable[0].key=' ';      // This handles leading spaces before command
	parsertable[0].next=1;
	parsertable[0].link=0;
	parsertable[0].command=NULL;
	parsertable[1].key=unabletoparse;
	parsertable[1].next=1;
	parsertable[1].link=1;
	parsertable[1].command=errfunc;
}

int isnumber(char c) // this return 1 if '0' to '9' or '+' or '-' or '.'
{
	return( ((c>='0') && (c<='9')) || (c=='+') || (c=='-') || (c=='.') );
}

int addcommand(char *commandstr,void (*func) (char *,char *) )
{ char temp[80];
	int tableentry;
	strcpy(temp,commandstr);
	strcat(temp,"$");            // handle spaces at the end of command
	buildtree(commandstr,func);
	tableentry=buildtree(temp,func);
	return(tableentry);
}

int buildtree(char *commandstr,void (*func) (char *,char *) )
{ static int freestate=2;
	int i,currstate,oldstate,newstate,newstate2,horizontal;

	i=0; currstate=parsertable[0].next; oldstate=0; horizontal=0;
	while(1)
	{ if (commandstr[i]==NULL)
	{ if (!horizontal)
		{ newstate=freestate;       // insert a new node here before null node
		freestate++;
		parsertable[oldstate].next=newstate;
		parsertable[newstate].next=currstate;
		parsertable[newstate].key=ctrlM;
		parsertable[newstate].link=newstate;
		parsertable[newstate].command=func;
		return (freestate);
		}
		else // if horizontal
		{ newstate=freestate;      // insert a new node, move null node to next
		freestate++;
		parsertable[oldstate].link=newstate;
		parsertable[newstate].next=currstate;
		parsertable[newstate].key=ctrlM;
		parsertable[newstate].link=newstate;
		parsertable[newstate].command=func;
		return (freestate);
		}
	}
	if (parsertable[currstate].key==unabletoparse)
	{ if (!horizontal)
		{ newstate=freestate;  // insert current character here
		freestate++;
		parsertable[oldstate].next=newstate;
		parsertable[newstate].next=currstate;
		parsertable[newstate].key=commandstr[i];
		i++;
		parsertable[newstate].link=currstate;
		parsertable[newstate].command=NULL;

		horizontal=1;
		oldstate=newstate;
		currstate=parsertable[newstate].link;
		}
		else // if horizontal
		{ newstate=freestate;   // insert current character here
		freestate++;
		parsertable[oldstate].link=newstate;
		parsertable[newstate].next=currstate;
		parsertable[newstate].key=commandstr[i];
		i++;
		parsertable[newstate].link=currstate;
		parsertable[newstate].command=NULL;

		horizontal=1;
		oldstate=newstate;
		currstate=parsertable[newstate].link;

		}
	}
	else if (parsertable[currstate].key==commandstr[i])
		 { i++;                   // if found same character go horizontal
			 oldstate=currstate;
			 currstate=parsertable[currstate].link;
		 }
		 else
		 { oldstate=currstate;   // other wise go vertical (down) the tree
			 currstate=parsertable[currstate].next;
			 horizontal=0;
		 }
	} // end while
}

int charparser(char charac,int *state)
{ int commandnum;
	static int inNumflag;
	static int inspaceflag;
	static int numberstr1pos;
	static int numberstr2pos;
	static int memory[101]; // state of the last 100 keystrokes
	static int memorypos;
	static int firstnumst;
	static char numberstr1[80];
	static char numberstr2[80];

	if (*state==0) // if just started, initialise static variables
	{ numberstr1pos=0; inNumflag=0;
	numberstr2pos=0; inspaceflag=0; memorypos=0;
	firstnumst=1; numberstr1[0]='\0';
		numberstr2[0]='\0';
	}
	memory[0]=0; // first state is always state 0;
	if (charac=='\b')  // '\b' is ctrl-H or backspace
	{ memorypos--;
	if (memorypos<0) memorypos=0;
	*state=memory[memorypos];
	return 0;
	}
	else  // if not backspace
	{ memorypos++;
	if (memorypos>100) memorypos=100;
	}
	while(1)
	{ // handle numerical inputs
	if ( isnumber(charac) && parsertable[*state].key==numbertoken )
	{ if (firstnumst)
		{ inNumflag=1; // set inside '#' flag
		numberstr1[numberstr1pos]=charac;
		numberstr1pos++;
		numberstr1[numberstr1pos]='\0'; // terminate string with NULL
		memory[memorypos]=*state; // keep state in memory
		return 0;
		}
		else // if not firstnumst
		{ inNumflag=1; // set inside '%' flag
		numberstr2[numberstr2pos]=charac;
		numberstr2pos++;
		numberstr2[numberstr2pos]='\0'; // terminate string with NULL
		memory[memorypos]=*state; // keep state in memory
		return 0;
		}
	}
				// handle multiple spaces
	else if ( charac==' ' && parsertable[*state].key==spacetoken )
	{ inspaceflag=1; // set inside '$' flag
		memory[memorypos]=*state; // keep state in memory
		return 0;
	}
	else if (parsertable[*state].key==charac) // if found then goto next level
	{
		*state=parsertable[*state].link;
		memory[memorypos]=*state; // keep state in memory
		if (charac==ctrlM) parsertable[*state].command(numberstr1,numberstr2);
		return 0;
	}
	else if (parsertable[*state].key==unabletoparse)
		 { memory[memorypos]=*state; // keep state in memory
			 numberstr1[0]='\0'; // reset first  num string
			 numberstr2[0]='\0'; // reset second num string
			 if (charac==ctrlM)
				parsertable[*state].command(numberstr1,numberstr2);
			 return 0;
		 }    // return unable to parse
	else if (inNumflag)  // if inside the '#' state but not number
	{ inNumflag=0;       // we are no longer in numbers state
		*state=parsertable[*state].link; // goto next level
		if (firstnumst) firstnumst=0; // if in first numeric then goto second
	}
	else if (inspaceflag)   // if inside the '$' state but not number
	{ inspaceflag=0;        // we are no longer in space state
		*state=parsertable[*state].link; // goto next level
	}
	else *state=parsertable[*state].next; // if not found check next key
	}
}

// this takes the pain out of charparser()
void parser(char *string)
{ int state,i;

	state=0;  // initialise state to be zero
	for(i=0;i<strlen(string);i++)
	charparser(string[i],&state);
}

// End of Parser code. The code after this is user code.
// -------------------------------------------------------------------------

int parser_error_flag=0;
int state_request_error_flag=0;
int parser_table_size=0;  // initialise parser table memory used

void errorfunc(char *st1,char *st2)     // function to handle parser error
{
	st1=st1; st2=st2;
	parser_error_flag=1;
}


void parser_command_el(char *st1,char *st2)
{
	float temp;
	st2=st2;
	parser_error_flag=0;        // reset the related error flags
	el_desired_error_flag = 0;
	if (!MODE_TRACK)
	{ sscanf(st1,"%f",&temp);
		if ((temp >= min_el) && (temp <= max_el)) // error checking
		switch (SEQUENCER_STATE)
		{
			case  4: // TRACKING STATE

			case  5: // SLEWING STATE
				requested_elevation_desired = temp;
				if (temp != track_elevation_desired)
					old_track_elevation_desired = track_elevation_desired;
				track_elevation_desired = temp;
				break;

			case  7:
				SEQUENCER_STATE = 4; // STOWED STATE
				requested_elevation_desired = temp;

				if (temp != track_elevation_desired)
					old_track_elevation_desired = track_elevation_desired;

				track_elevation_desired = temp;
				break;

			case 14:
				old_elevation_desired = temp; // to replace the former value
				requested_elevation_desired = temp; // set the displayed value
				SEQUENCER_STATE =15;
				break;   // HOLDING STATE

			case 17:
				SEQUENCER_STATE = 4; // STOWING STATE
				requested_elevation_desired = temp;
				if (temp != track_elevation_desired)
					 old_track_elevation_desired = track_elevation_desired;
				track_elevation_desired = temp;
				break;

			default: break;
		}
		else el_desired_error_flag = 1;
	}
}

void parser_command_ra(char *st1,char *st2)
{
	float temp;
	st2=st2;
	parser_error_flag=0;
	sscanf(st1,"%f",&temp);  // dont forget the '&' again stupid
	if ( (temp<0.0) || (temp>24.0) ) parser_error_flag=1;
	else star_ra = (double) temp/12.0*3.141592653589793238462643;
	//convert hours to radians
}

void parser_command_dec(char *st1,char *st2)
{
	float temp;
	st2=st2;
	parser_error_flag=0;
	sscanf(st1,"%f",&temp);  // dont forget the '&' again stupid
	if ( (temp< -90.0) || (temp>90.0) ) parser_error_flag=1;
	else star_dec = (double) temp/180.0*3.141592653589793238462643;
	// convert degrees to radians
}

void parser_command_az(char *st1,char *st2)
{
	 float temp;
	 st2=st2;
	 parser_error_flag=0;
	 if (!MODE_TRACK)
	 { switch (SEQUENCER_STATE)
		{ case  4: // TRACKING STATE
			case  5: // SLEWING STATE
							 sscanf(st1,"%f",&temp);
							 while (temp >= 360 )
								 temp = temp - 360;
							 while (temp < 0)
								 temp = temp + 360;
							 track_azimuth_desired = temp;
							 break;
			case  7: sscanf(st1,"%f",&temp);
							 while (temp >= 360 )
								 temp = temp - 360;
							 while (temp < 0)
								 temp = temp + 360;
							 track_azimuth_desired = temp;
							 SEQUENCER_STATE = 4; break; // STOWED STATE
			case 14: sscanf(st1,"%f",&temp);
							 while (temp >= 360 )
								 temp = temp - 360;
							 while (temp < 0)
								 temp = temp + 360;
							 old_azimuth_desired = temp;
							 SEQUENCER_STATE =15; break;  // HOLDING STATE
			case 17: sscanf(st1,"%f",&temp);
							 while (temp >= 360 )
								 temp = temp - 360;
							 while (temp < 0)
								 temp = temp + 360;
							 track_azimuth_desired = temp;
							 SEQUENCER_STATE = 4; break;  // STOWING STATE
			default: break;
		}
	 }
}

void parser_command_star(char *st1,char *st2)
{
	st1=st1; st2=st2;
	parser_error_flag=0;
	STAR=1;
}

void parser_command_sat(char *st1,char *st2)
{
	st1=st1; st2=st2;
	parser_error_flag=0;
	STAR=0;
}

void parser_command_manual(char *st1,char *st2)
{
	st1=st1; st2=st2;
	parser_error_flag=0;
	MODE_TRACK=0;
	switch (SEQUENCER_STATE)
	{ case  7: SEQUENCER_STATE = 4; break; // STOWED STATE
		case 14: SEQUENCER_STATE =15; break;  // HOLDING STATE
		default: break;
	}
}

void parser_command_power_on(char *st1,char *st2)
{
	st1=st1; st2=st2;
	parser_error_flag=0;

	if (SEQUENCER_STATE == 0)
	{
		 if (QUITING) QUITING = 0;
		 SEQUENCER_STATE=9;
		 state_request_error_flag=0;
	}
	else
		 state_request_error_flag=1;
}

void parser_command_start(char *st1,char *st2)
{
	st1=st1; st2=st2;
	parser_error_flag=0;
	if (SEQUENCER_STATE == 1)
	{
		 SEQUENCER_STATE=2;
		 state_request_error_flag=0;
	}
	else
		 state_request_error_flag=1;
}

void parser_command_hold(char *st1,char *st2)
{
	st1=st1; st2=st2;
	parser_error_flag=0;

	switch (SEQUENCER_STATE)
	{ case  4: // TRACKING STATE
		case  5: // SLEWING STATE
		case  6: // STOWING1 STATE
		case 17: // STOWING2 STATE
						 SEQUENCER_STATE=3; // goto HOLDING STATE
						 state_request_error_flag=0;
						 break;
		default: state_request_error_flag=1;
						 break;
	}
}

void parser_command_stow(char *st1,char *st2)
{
	st1=st1; st2=st2;
	parser_error_flag=0;

	switch (SEQUENCER_STATE)
	{ case  4: // TRACKING STATE
		case  5: // SLEWING STATE
		case 14: // HOLDING STATE
						 SEQUENCER_STATE=6; // goto HOLDING STATE
						 state_request_error_flag=0;
						 break;
		default: state_request_error_flag=1;
						 break;
	}

}

void parser_command_stop(char *st1,char *st2)
{
	st1=st1; st2=st2;
	parser_error_flag=0;

	if (SEQUENCER_STATE == 7) // STOWED STATE
	{
		 SEQUENCER_STATE=8;
		 state_request_error_flag=0;
	}
	else
		 state_request_error_flag=1;
}

void parser_command_track(char *st1,char *st2)
{
	st1=st1; st2=st2;
	parser_error_flag=0;
	switch (SEQUENCER_STATE)
	{ case  4: MODE_TRACK=1; break; // IF IN TRACKING STATE STAY IN IT
		case  7: MODE_TRACK=1; SEQUENCER_STATE = 4; break; // STOWED STATE
		case 14: MODE_TRACK=1; SEQUENCER_STATE =15; break;  // HOLDING STATE
		default: state_request_error_flag=1; break;
	}
}

void parser_command_quit(char *st1,char *st2)
{ st1=st1; st2=st2;
	QUITING=1; // set QUITING flag on
}

void parser_command_el_az(char *st1,char *st2)
{
	float temp;
	parser_error_flag=0;
	if (!MODE_TRACK)
	{ sscanf(st1,"%f",&temp);  // dont forget the '&' again stupid
		requested_elevation_desired = temp;
		if (temp != track_elevation_desired)
			old_track_elevation_desired = track_elevation_desired;
		track_elevation_desired = temp;
		sscanf(st2,"%f",&temp);
		while (temp >= 360 )
			temp = temp - 360;
		while (temp < 0)
			temp = temp + 360;
		track_azimuth_desired = temp;
	 }
}

void parser_command_dacnone(char *st1,char *st2)
{
	st1=st1; st2=st2;
	parser_error_flag=0;
	DAC_STATE=0;
}

void parser_command_dacvolt(char *st1,char *st2)
{
	st1=st1; st2=st2;
	parser_error_flag=0;
	DAC_STATE=1;
}

void parser_command_dacpolar(char *st1,char *st2)
{
	st1=st1; st2=st2;
	parser_error_flag=0;
	DAC_STATE=2;
}

void parser_command_dacdome(char *st1,char *st2)
{
	st1=st1; st2=st2;
	parser_error_flag=0;
	DAC_STATE=3;
}

void parser_command_shutdown(char *st1,char *st2)
{
	st1=st1; st2=st2;
	CONTINUE=0;

}

void parser_command_mir(char *st1,char *st2)
{
//  extern void SetCurrentSat(int satnum); // from orbit.c

	st1=st1; st2=st2;
	parser_error_flag=0;
	currentsatnum=0;     // Mir's sattelite number is 0
	SatRiseTime= RiseTime ( &SatAzinitial, &SatElptr, &SatAzfinal);
	SatRiseTime+= (10.0/24.0); // add 10 hours to SatRiseTime
	TrueMinute = ( SatRiseTime - (int) SatRiseTime ) * ( 24 * 60 );
	GetDate( (long) SatRiseTime, &RiseYear, &RiseMonth, &RiseDay );
	RiseMinute = TrueMinute % 60;
	RiseHour = ( TrueMinute - RiseMinute ) / 60;
}

void parser_command_hour(char *st1,char *st2)
{
	float temp;
	st2=st2;
	parser_error_flag=0;
	sscanf(st1,"%f",&temp);  // dont forget the '&' again stupid
	if ( (temp<0.0) || (temp>24.0) ) parser_error_flag=1;
	else currtime.hour = (int) temp;
}

void parser_command_minute(char *st1,char *st2)
{
	float temp;
	st2=st2;
	parser_error_flag=0;
	sscanf(st1,"%f",&temp);  // dont forget the '&' again stupid
	if ( (temp<0.0) || (temp>60.0) ) parser_error_flag=1;
	else currtime.minute = (int) temp;
}

void parser_command_second(char *st1,char *st2)
{
	float temp;
	st2=st2;
	parser_error_flag=0;
	sscanf(st1,"%f",&temp);  // dont forget the '&' again stupid
	if ( (temp<0.0) || (temp>60.0) ) parser_error_flag=1;
	else currtime.second = (int) temp;
}

void parser_command_date(char *st1,char *st2)
{
	float temp;
	st2=st2;
	parser_error_flag=0;
	sscanf(st1,"%f",&temp);  // dont forget the '&' again stupid
	if ( (temp<1.0) || (temp>31.0) ) parser_error_flag=1;
	else currtime.day = (int) temp;
}

void parser_command_month(char *st1,char *st2)
{
	float temp;
	st2=st2;
	parser_error_flag=0;
	sscanf(st1,"%f",&temp);  // dont forget the '&' again stupid
	if ( (temp<1.0) || (temp>12.0) ) parser_error_flag=1;
	else currtime.month = (int) temp;
}

void parser_command_year(char *st1,char *st2)
{
	float temp;
	st2=st2;
	parser_error_flag=0;
	sscanf(st1,"%f",&temp);  // dont forget the '&' again stupid
	if ( (temp<0.0) || (temp>99.0) ) parser_error_flag=1;
	else currtime.year = (int) temp;
}

void init_parser_control(void)
{
	current_screen=0;
	init_parser_table(errorfunc);
	addcommand("el$#",parser_command_el);        // first command
	addcommand("quit",parser_command_quit);
	addcommand("star",parser_command_star);
	addcommand("sat",parser_command_sat);
	addcommand("manual",parser_command_manual);
	addcommand("track",parser_command_track);
	addcommand("ra$#",parser_command_ra);
	addcommand("dec$#",parser_command_dec);
	addcommand("power$on",parser_command_power_on);
	addcommand("start",parser_command_start);
	addcommand("hold",parser_command_hold);
	addcommand("stow",parser_command_stow);
	addcommand("stop",parser_command_stop);
	addcommand("el$#$az$#",parser_command_el_az);
	addcommand("dacnone",parser_command_dacnone);
	addcommand("dacvolt",parser_command_dacvolt);
	addcommand("dacpolar",parser_command_dacpolar);
	addcommand("dacdome",parser_command_dacdome);
	addcommand("shutdown",parser_command_shutdown);
	addcommand("mir",parser_command_mir);
	addcommand("hour$#",parser_command_hour);
	addcommand("minute$#",parser_command_minute);
	addcommand("second$#",parser_command_second);
	addcommand("date$#",parser_command_date);
	addcommand("month$#",parser_command_month);
	addcommand("year$#",parser_command_year);
	parser_table_size=addcommand("az$#",parser_command_az); // last command

}

void parser_control_task(void* dummy)
{ unsigned char mess_buf[2];
	unsigned int  mess_lgth;
	int state=0;

	dummy=dummy;

	for(;;)
	{
		rcv_mess ( mess_buf , &mess_lgth, 0);
		switch (current_screen) {
		case 0 : { charparser(mess_buf[0],&state);
							 if (mess_buf[0]==ctrlM) state=0;
							 break;
						 }
		case 1 : { charparser(mess_buf[0],&state);
							 if (mess_buf[0]==ctrlM) state=0;
							 break; }
		case 2 : break;

		default: {}
		} //end switch
	}
}