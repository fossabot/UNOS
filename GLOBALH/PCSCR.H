/*****************************************************************************
* POLYTRON VERSION CONTROL SOFTWARE _ PVCS (tm) Header Information
*
* 	$Revision:   2.6  $
*	$Date:   06 Mar 1992 18:04:56  $
*
*****************************************************************************/
/*
****************************************************************************

			PC Screen Write Support Routines External Prototypes
							Header File
								by

							L. Sciacca
					Department of Electrical Engineering
						University of Newcastle
						  	Nov 1990

Latest:	2 Nov 1990

Include this header file should be included by modules wanting access to the 
ansi routines in scransi.c

****************************************************************************
*/

extern void pcscr_put_text ( unsigned int x, unsigned int y, 
					char * text, unsigned int mode );

extern void pcscr_put_double ( unsigned int x, unsigned int y, 
					char * fmt, double dble, unsigned int mode );

extern void pcscr_put_int ( unsigned int x, unsigned int y, 
					char * fmt, int intval, unsigned int mode );

extern void pcscr_draw_line ( unsigned int length, unsigned int x,
					unsigned int y, int dir, int move );

extern void pcscr_corner ( unsigned int x, unsigned int y, unsigned int corner );

extern void pcscr_draw_box ( unsigned int xstart, unsigned int ystart,
					unsigned int xfinish, unsigned int yfinish, 
					char* title, unsigned int mode );

extern void pcscr_draw_bar ( char * label, double value, 
					double pivot, double max_value,
					unsigned int xstart, unsigned int ystart,
					int direction, unsigned int length );

extern void pcscr_draw_hor_bar ( double value, double max_value,
					unsigned int xstart, unsigned int ystart, unsigned int length );
					
extern void pcscr_function_menu ( unsigned int number_of_keys, char * farray[]);
extern void setcursor ( unsigned int shape );
extern void restore_pc_screen ( void );
					

/* Define horizontal or vertical directions */
#define HOR	0
#define VER 1

/* Define text attributes */
#define normal 0
#define bold 1
#define BOLD 1
#define NORMAL 0


/* Corners */
#define LOWERLEFT		0
#define UPPERRIGHT		1
#define UPPERLEFT		2
#define LOWERRIGHT		3
#define TLEFT			4
#define TRIGHT			5
#define TDOWN			6
#define TUP				7
#define INTERSECT		8

#define HASH 0xb1

