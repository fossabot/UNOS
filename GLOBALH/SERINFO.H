/* serinfo.h   */
/* This holds the type definition for the
** structure which holds data about the serial
** process that we want to display on
** the IO screen.
*/

typedef
	struct {
		unsigned char crcin1;
		unsigned char crcin2;
		unsigned char crccalc1;
		unsigned char crccalc2;
		unsigned char inid;
		unsigned char insize;
		unsigned char lastinid;
		unsigned char nextinid;
		unsigned char lastbyte;
		unsigned char lastoutid;
		unsigned char nummesgs;
		unsigned char mesgidx;
		unsigned char mesg[20][80];
	} ser_struct;
