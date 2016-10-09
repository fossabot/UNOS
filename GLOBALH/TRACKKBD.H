/* To be included in trackkbd.h
   This function is used to decode the kbd inputs meant for the track module
   ksc 12/10/96
*/

#define SUN  0	/* hash definitions of the stars to be tracked */
#define MOON 1
#define VELA 2
#define NUMSTARS 3  /* the total number of stars that we are tracking */

/* FOLLOW mode tracks the star as it moves across the sky */
/* SCROLL mode moves the antenna from left to right from a stars
   calculated position. This is useful for testing readouts */

#define FOLLOW 0
#define SCROLL 1
#define NUMMODES 2

/* ON and OFF define the tracking status */
#define OFF 0
#define ON  1

/* intervals at which the calculations are made */
#define DEFAULT 300 	   /* 5 minutes  */
#define MAX_INTERVAL 1800  /* 30 minutes */
#define MIN_INTERVAL  10   /* 10 seconds  */
#define CHANGE_INTERVAL 10 /* 10 seconds */
#define MAX_CHANGE      100
#define MIN_CHANGE      1


/* Exported variables to be used in trackscr and trackkbd.c */
extern int star;
extern int track_mode;
extern int track_status;
extern int track_interval;
extern int change;
extern void decode_track_key ( unsigned char key_pressed );
