
#ifndef __ANTSIMEX_H
#define __ANTSIMEX_H
/*
	Routine to init simulator or real hardware interface addresses.
	Should be called by atcumain.c after setting UNOS up.
*/
extern void init_antenna_io ( void );


/* 	Routines used by simulator only */
void put_drive_signals ( char controlV, char azbrake, char elbrake,
		char azflcw, char azflccw, char elflup, char elfldown,
		char emstop, char smoke, char door1, char door2, char cb36, char cb4,
		char cb5, char azrun, char elrun, char trip, char fail );

extern void put_encoder ( unsigned int axis, unsigned long enc );

extern void get_drive_signals ( char * stop, char * start,
						char * reset,
						char * PowerOn );

extern void get_rate_volts ( int * az_v1, int * el_v1);


#endif /* end ifndef __ANTSIMEX_H */