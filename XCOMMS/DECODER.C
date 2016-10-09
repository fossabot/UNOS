/*
********************************************************************************
								decoder.c
Task to decode message from server.

Author: S. Siew 1992/L. J. Sciacca

rjlov			27/7/1995		Added control of ACTU via serial
											line, and sending of data in packed ASCII
											format only when changed.

********************************************************************************
*/

#include <stdio.h>
#include <string.h>
#include <dos.h>
#include "queue.h"
#include "general.h"
#include "unos.h"
#include "seq.h"
#include "seqext.h"
#include "posext.h"
#include "plc.h"
#include <math.h>
#include "serinfo.h"

#include "kbtask.h"
#include "nvramext.h" // Import select_new_beacon()
#include "seqscr.h"

/*
#include <conio.h>
#include <dos.h>

#include "err_nums.h"


#include "seqscr.h"

#include "scrext.h"
*/


#define SER_CMD_POLLALL 0
#define SER_CMD_POLLNEW 1

#define SER_CMD_HOLD 32
#define SER_CMD_GO 33
#define SER_CMD_STOW 34
#define SER_CMD_CLRNVRAM 35

#define SER_CMD_LIGHTS 64
#define SER_CMD_DRIVE 65
#define SER_CMD_SETAZ 66
#define SER_CMD_SETEL 67
#define SER_CMD_POWER 68
#define SER_CMD_WRAP 69
#define SER_CMD_MODE 70
#define SER_CMD_LOGDATA 71
#define SER_CMD_BEACON 72
#define SER_CMD_SCAN 73
#define SER_CMD_MINEL 74
#define SER_CMD_MAXEL 75
#define SER_CMD_MINAZ 76
#define SER_CMD_MAXAZ 77

extern queueclass outqueue;
extern queueclass inqueue;

extern ser_struct serinfo;

int PROTOCOL_SEQUENCER = 0;

extern unsigned char LightsOn;	  /* defines if lights are turned on */

extern void ReadPLC ( PLC_struct * PLC );  /* Import from plc.c */

/*
********************************************************************************
** packnum()
** This converts a string representing a number into a packed string,
** for sending over the serial line.
********************************************************************************
*/
void packnum(unsigned char *s1, unsigned char *s, int *maxlen)
{	unsigned char nib[2];
	int i,j,k;

	k=0;

	for(i=0; i< (*maxlen); i+=2) {
		nib[0]=nib[1] = 0x0f;
		for (j=0;((j<2)&&(i+j < *maxlen)) ;++j) {
			switch (s1[j+i]) {
				case ' ':
					++i;
					--j;
					break;
				case 'e':
				case 'E':
					nib[j] = 0x0d;
					break;
				case '-':
					nib[j] = 0x0b;
					break;
				case '+':
					nib[j] = 0x0a;
					break;
				case '.':
					nib[j] = 0x0c;
					break;
				default :
					if ((s1[j+i] <= '9')
						&& (s1[j+i] >= '0'))
						nib[j] = s1[j+i] - '0';
					else
						nib[j] =0x0f;
			}	/* end of switch (s1[j+i]) */
		}   /* end of for(j=0;j<....) */
		s[k++] = ((nib[0] << 4) | nib[1]);
	}	/* end of for(i=0;i<printed;++i); */

	*maxlen = k;
	return;

} /* end of packnum() */

/*
********************************************************************************
** unpacknum()
** This a packed string into a normal string
** after sending over the serial line.
********************************************************************************
*/
void unpacknum(unsigned char *s1, int maxlen)
{	unsigned char mask, inbyte;
	static char nyblist[] = "0123456789+-.e ";
			/* this string must be exactly fifteen characters long, so that
			** nyblist[0x0f] == '\0'
			*/
	int i,length,shift,j,k;

	while (!(inqueue.isnotempty()))
		delay(50);
	length = (int)inqueue.dequeue();

	if (maxlen < (length * 2) + 1)
		return;

	i=0; shift = 0;
	/* The following line is correct. For each iteration
	** we increase i, the index in the unpacked string, but
	** the limit for the loop depends on j, the number
	** of incoming bytes we have exhausted.
	*/
	for (j=0;j<length;++i) {
		if (shift == 0) {
			while (!(inqueue.isnotempty()))
				delay(50);
			inbyte = inqueue.dequeue();

			mask = 0xf0; shift = 4;
		} else {
			mask = 0x0f; shift = 0; j++;
		}
		s1[i] = nyblist[(int)(inbyte & mask) >> shift];
	}
	s1[i] = '\0';
/*
	strcpy(serinfo.mesg,s1);
	serinfo.mesgchange = 1;
*/

} /* end of unpacknum() */


/*
********************************************************************************
putfloat()

********************************************************************************
*/
void putfloat(float fvar)
{
/*
** This is the new putfloat
*/
	unsigned char buffer[10], s1[30];
	int length,maxlen,i;
	maxlen = 10;

	if (fabs((double)fvar) < 1e-4) fvar = 0;
	length=sprintf(s1,"%.4g",fvar);
	if (length > maxlen * 2)
		return;
	packnum(s1,buffer,&length);
	outqueue.enqueue((unsigned char)length);
	for(i=0;i<length;++i)
		outqueue.enqueue(buffer[i]);

/*
**	This is the old putfloat

	void *ptr;
	unsigned char byte;

	ptr=&fvar;

	byte=*((unsigned char *)ptr+0);
	outqueue.enqueue(byte);

	byte=*((unsigned char *)ptr+1);
	outqueue.enqueue(byte);

	byte=*((unsigned char *)ptr+2);
	outqueue.enqueue(byte);

	byte=*((unsigned char *)ptr+3);
	outqueue.enqueue(byte);
*/

} // end of putfloat()

/*
********************************************************************************
putint()

********************************************************************************
*/
void putint(int ivar) {
/*
** This is the new putint()
*/
	unsigned char s1[30],buffer[10];
	int length,maxlen,i;
	maxlen = 10;

	length=sprintf(s1,"%d",ivar);
	if (length > maxlen * 2)
		return;
	packnum(s1,buffer,&length);
	outqueue.enqueue((unsigned char)length);
	for(i=0;i<length;++i)
		outqueue.enqueue(buffer[i]);

/* This is the old putint
**

	void *ptr;
	unsigned char byte;

	ptr=&ivar;

	byte=*((unsigned char *)ptr+0);
	outqueue.enqueue(byte);

	byte=*((unsigned char *)ptr+1);
	outqueue.enqueue(byte);

*/

}
/*
********************************************************************************
getint()

********************************************************************************
*/
int getint() {
	unsigned char s1[30];
	int maxlen,i;
	maxlen = 30;

	unpacknum(s1,maxlen);
	sscanf(s1,"%d",&i);
	return(i);
}

/*
********************************************************************************
getfloat()

********************************************************************************
*/
float getfloat() {
	unsigned char s1[30];
	int maxlen;
	float f1;
	maxlen = 30;

	unpacknum(s1,maxlen);
	sscanf(s1,"%f",&f1);
	return(f1);
}


/*
********************************************************************************
init_decoder()

********************************************************************************
*/
void init_decoder_task (void) {
;
}

/*
********************************************************************************
decoder_task()

********************************************************************************
*/
void decoder_task (void* dummy) {
	char *rtn_addr_ptr,mess_buf[80],bytein,notftime;
	unsigned int mess_lgth;
	static double az_pos;
	void *ptr;
	float ftemp1,ftemp2, az_rate, el_rate;
	char buff1[80],buff2[80];
	float oldf[30],a1;
	int oldi[30],a2,pollall,i,doneflag;

	static software_limit_struct sw_limits;  /*---- Software Limits */
	static axis az;
	static axis el;
	static PLC_struct PLC;
	static atcu_struct atcu;
	static cp_data_struct cp_data;

/*
**	Current assignments are as follows:
**	128:	Measured azimuth     (double)
**	129:	Measured elevation   (double)
**  130:  Azimuth rate         (float)
**  131:  Elevation rate       (float)
**	132:  Commanded azimuth    (double)
**  133:  Commanded elevation  (double)
**        Error in azimuth     (double)
**        Error in elevation   (double)
**        Clockwise software azimuth limit  (double)
**        Upper software elevation limit  (double)
**        Anit-Clockwise software azimuth limit  (double)
**        Lower software elevation limit  (double)
**        Voltage to azimuth motor    (double)
**        Volatage to elevation motor  (double)
**      	ATCU status		   		 (int)
**				Lights status				 (int)
**				Azimuth brake	 (int)
**				Elevation brake (int)
**				PLC CB1 circuit breaker (int)
**				PLC CB2 circuit breaker (int)
**				PLC CB3 circuit breaker (int)
**				PLC CB4 circuit breaker (int)
**				Azimuth clockwise hard limit reached (int)
**				Azimuth anti-clockwise hard limit reached (int)
**				Elevation up hard limit reached (int)
**				Elevation down hard limit reached (int)
**				Motor 1 running (int)
**				Motor 2 running (int)
**				Door 1 (int)
**				Door 2 (int)
**				Emergency Stop (int)
**				Smoke Detector (int)
**				PLC trip (int)        --/- I don't know exactly
**				PLC trip1 (int)       -/   what these two are.
**				ATCU mode (int)
*/

	static void * curp[]={
		&(az.msd_absolute), &(el.msd),
		&az_rate, &el_rate,
		&(az.cmd_absolute), &(el.cmd),
		&(az.err), &(el.err),
		&(sw_limits.az_cw_limit), &(sw_limits.el_up_limit),
		&(sw_limits.az_ccw_limit), &(sw_limits.el_down_limit),
		&(az.rate_volts), &(el.rate_volts),
		&(atcu.state), &LightsOn,
		&(PLC.AzBrake), &(PLC.ElBrake),
		&(PLC.C1), &(PLC.C2),
		&(PLC.CB3), &(PLC.CB4),
		&(PLC.AzFinalCW), &(PLC.AzFinalCCW),
		&(PLC.ElFinalUp), &(PLC.ElFinalDwn),
		&(PLC.RUN1), &(PLC.RUN2),
		&(PLC.Door1), &(PLC.Door2),
		&(PLC.EmergStop), &(PLC.SmokeDetect),
		&(PLC.Trip), &(PLC.Trip1),
		&(atcu.mode), &(cp_data.signal)
		};
	static void * oldp[]={
		&oldf[0], &oldf[1],
		&oldf[2], &oldf[3],
		&oldf[4], &oldf[5],
		&oldf[6], &oldf[7],
		&oldf[8], &oldf[9],
		&oldf[10], &oldf[11],
		&oldf[12], &oldf[13],
		&oldi[0], &oldi[1],
		&oldi[2], &oldi[3],
		&oldi[4], &oldi[5],
		&oldi[6], &oldi[7],
		&oldi[8], &oldi[9],
		&oldi[10], &oldi[11],
		&oldi[12], &oldi[13],
		&oldi[14], &oldi[15],
		&oldi[16], &oldi[17],
		&oldi[18], &oldi[19],
		&oldi[20], &oldf[14]
		};
	static int    vtype[] = {
		4,4,		/* Measured az,el */
		3,3,		/* Rate of az,el */
		4,4,		/* Commanded az,el */
		4,4,		/* Error in az,el */
		4,4,
		4,4,
		4,4,
		1,2,
		2,2,
		2,2,
		2,2,                      
		2,2,
		2,2,
		2,2,
		2,2,
		2,2,
		2,2,
		1,4
		};

	/* vtype contains the type of the object pointed to by
	** the corresponding element of curp. if the number
	** is negative, then don't send it.
	** 1  - integer
	** 2  - unsigned char
	** 3  - float
	** 4  - double
	*/

	dummy=dummy;
	notftime = 0;

	for(i=0;i<30;++i) {
		oldf[i] = 0;
		oldi[i] = 0;
	}
	while ( 1 )
	{
		/* Just a timer   */
		rtn_addr_ptr = rcv_mess (mess_buf, &mess_lgth, 1);
		rtn_addr_ptr = rtn_addr_ptr;

		if (inqueue.isnotempty() || !notftime)    /* receive packet from port */
		{
			if (notftime)
				bytein=inqueue.dequeue();
			else {
				notftime = 1;
				bytein = SER_CMD_POLLALL;
			}
			pollall = 0;

			switch ( bytein ) {
				case SER_CMD_POLLALL: /* polling all quantities */
					pollall = 1;
				case SER_CMD_POLLNEW: /* poll only new quantites */
					az = return_azimuth ( );
					el = return_elevation ( );

					az_rate = return_az_control ( );
					el_rate = return_el_control ( );

					get_seq_software_limits(&sw_limits);
					/* Read current status of PLC */
					ReadPLC(&PLC);

					/* Return atcu modes and states */
					atcu = return_atcu ( );

					/* Return current CP data */
					cp_data = return_cp_data ( );

					for(i=0; i < sizeof(vtype)/sizeof(int); i++) {
						if (vtype[i] > 2) {	/* if the value is a double or float */
							/* Get the current value of the quantity,
							** and convert it to a float if it is a double
							*/
							if (vtype[i] == 4)
								a1 = (float)(*((double *)curp[i]));
							else if (vtype[i] == 3)
								a1 = *((float *)curp[i]);
							/* if the new value is different from the old
							** value, then send the new value and update the
							** old value.
							*/
							if ((a1 != *((float *)oldp[i])) || pollall) {
								outqueue.enqueue((unsigned char)i + 128);
								putfloat(a1);
								*((float *)oldp[i]) = a1;
							}     /* end of if the new value is different */

						} else if (vtype[i] > 0) {
										/* if the value is an integer or char */
							if (vtype[i] == 2)
								a2 = (int)*((unsigned char *)curp[i]);
							else if (vtype[i] == 1)
								a2 = *((int *)curp[i]);
							if ((a2 != *((int *)oldp[i])) || pollall) {
								outqueue.enqueue((unsigned char)i + 128);
								putint(a2);
								*((int *)oldp[i]) = a2;
							}     /* end of if the new value is different */
						}				/* end of if the value is an integer */
					}				/* end of for i loop */
					break;

				case SER_CMD_SETAZ:           /* Adjust azimuth  */
					doneflag = 1;
					switch ( cp_data.mode ) {
						case RATE_MODE:
							cp_data.az_dem_rate = (double)getfloat();
							break;

						case POSITION_MODE:
							az_pos = fmod ( (double)getfloat(), 360.0 );
							if ( az_pos < 0 )
								cp_data.az_cmd = az_pos + 360;
							else
								cp_data.az_cmd = az_pos;

							break;

						case POSITION_RADEC_MODE:
							az_pos = fmod ( (double)getfloat(), 24.0 );
							if ( az_pos < 0 )
								cp_data.ra_cmd = az_pos + 24;
							else
								cp_data.ra_cmd = az_pos;
							break;

						case STARTRACK_MODE:
							cp_data.az_ramp_rate = (double)getfloat();
							break;

						default:
							doneflag = 0;
							break;

					} /* switch */

					if (doneflag) set_cp_data ( &cp_data );
					break;

				case SER_CMD_BEACON:			/* Change Beacon Number */
					break;

				case CONTROL_G:
					break;

				case SER_CMD_CLRNVRAM:			/* Clear NVRAM Error*/
					ClearNVRAMError ( );
					break;

				case SER_CMD_SETEL:
					/* Adjust Elevation */
					doneflag = 1;
					switch ( cp_data.mode ) {
						case RATE_MODE:
							cp_data.el_dem_rate = (double)getfloat();
							break;

						case POSITION_MODE:
							cp_data.el_cmd = (double)getfloat();
							break;

						case POSITION_RADEC_MODE:
							cp_data.dec_cmd = (double)getfloat();
							break;

						case STARTRACK_MODE:
							cp_data.el_ramp_rate = (double)getfloat();
							break;

						default:
							doneflag = 0;
							break;

						} /* switch */

					if (doneflag) set_cp_data ( &cp_data );
					break;

				/* Change modes */
				case SER_CMD_MODE:
					cp_data.mode = getint();
					if ( ( atcu.state == STANDBY ) ||
							( atcu.state == HOLDING ) ||
							( atcu.state == OFF_STATE )
							) {

						if ( cp_data.mode > TOTAL_MODES )
							cp_data.mode = 0;

						cp_data.cmd = IDLE_CMD;
						if ( cp_data.mode == RATE_MODE ) {
							cp_data.az_dem_rate = 0.0;
							cp_data.el_dem_rate = 0.0;
							}

						set_cp_data ( &cp_data );

						}
					else {
						/* rjlov - printerror */
						i = serinfo.mesgidx = (serinfo.mesgidx + 1) % serinfo.nummesgs;
						sprintf(serinfo.mesg[i],"Can't change modes, in state %d",
							atcu.state);

						process_error ( BAD_KEY_ENTRY );
					}

					break;

								/* DRIVE ON */
				case SER_CMD_DRIVE:
					if (getint())
						cp_data.cmd = DRIVE_START_CMD;
					else
						cp_data.cmd = DRIVE_STOP_CMD;
					set_cp_data( &cp_data);
					break;

								/* save data to FILE */
				case SER_CMD_LOGDATA:
					if (getint())
						cp_data.cmd = DATA_ON_CMD;
					else
						cp_data.cmd = DATA_OFF_CMD;
					set_cp_data ( &cp_data );
					break;

								/* GO */
				case SER_CMD_GO:
					cp_data.cmd = GO_CMD;
					set_cp_data ( &cp_data );
					break;

								/* HOLD */
				case SER_CMD_HOLD:
					cp_data.cmd = HOLD_CMD;
					set_cp_data ( &cp_data );
					break;

								/* Lights On/Off */
				case SER_CMD_LIGHTS:
					if (getint())
						cp_data.cmd = LIGHTS_ON_CMD;
					else
						cp_data.cmd = LIGHTS_OFF_CMD;

					set_cp_data ( &cp_data );
					break;

				case 'z':
				case 'Z':
					break;

				/* Power On/Off i.e. Close/Open C1 and C2 contactors */
				case SER_CMD_POWER:
					if (getint())
						cp_data.cmd = POWER_ON_CMD;
					else
						cp_data.cmd = POWER_OFF_CMD;
					set_cp_data ( &cp_data );
					break;

					/* STOW */
				case SER_CMD_STOW:
					cp_data.cmd = STOW_CMD;
					set_cp_data ( &cp_data );
					break;

				case SER_CMD_WRAP:
					cp_data.wrap = getint();
					if ( cp_data.wrap > 2 )
						cp_data.wrap = 0;

					set_cp_data ( &cp_data );
					break;

							/* SCAN TOGGLING COMMAND */
				case SER_CMD_SCAN:
					if (getint())
						cp_data.cmd = SCAN_ON_CMD;
					else
						cp_data.cmd = SCAN_OFF_CMD;
					set_cp_data ( &cp_data );
					break;

				case SER_CMD_MINEL:
					get_seq_software_limits(&sw_limits);
					sw_limits.el_down_limit = (double)getfloat();
					put_seq_software_limits ( &sw_limits );
					break;
				case SER_CMD_MAXEL:
					get_seq_software_limits(&sw_limits);
					sw_limits.el_up_limit = (double)getfloat();
					put_seq_software_limits ( &sw_limits ) ;
					break;
				case SER_CMD_MINAZ:
					get_seq_software_limits(&sw_limits);
					sw_limits.az_ccw_limit = (double)getfloat();
					put_seq_software_limits ( &sw_limits ) ;
					break;
				case SER_CMD_MAXAZ:
					get_seq_software_limits(&sw_limits);
					sw_limits.az_cw_limit = (double)getfloat();
					put_seq_software_limits ( &sw_limits );
					break;

				default:
					/* ? should send a "bad-command" back to sequencer */
					/* so sequencer can send this info to the serial comms */
					/* rjlov - printerror */
					i = serinfo.mesgidx = (serinfo.mesgidx + 1) % serinfo.nummesgs;
					sprintf(serinfo.mesg[i],"Bad number: %2x ",bytein);

					process_error ( BAD_KEY_ENTRY );
					break;
			} /* end switch bytein;  */

		} /* end of if inqueue is not empty */

	}  /* end of while true */
} /* end of decoder_task */

/*
**	These are the old quantites that used to be transmitted
**
							putfloat((float)el.msd);
							putfloat((float)az.msd_absolute);
							putfloat((float)el.err);
							putfloat((float)az.err);
							putfloat((float)el.cmd);
							putfloat((float)az.cmd_absolute);
							putfloat(0.0);  // el velocity
							putfloat(0.0);  // az velocity
							putfloat((float)el_rate);
							putfloat((float)az_rate);
							putfloat(0.0 );//(float) star_ra);
							putfloat(0.0); //(float) star_dec);
							putint(0 ); //(int) STAR);
							putint((int) atcu.mode);
							putint(0); //parser_error_flag);
							putint(0); //el_range_error_flag);
							putint(0); //el_desired_error_flag);
							putint(0); //state_request_error_flag);
							putint((int)atcu.state_summary);
							break;
*/










