/*
*******************************************************************

		Include File : KBTASK

		Keyboard Task definitions and external prototypes

		L. J. Sciacca

		1 - May -1991

		ksc/tzqh
		27/3/96

*******************************************************************
*/
extern unsigned int return_command_dest ( void );

extern unsigned int return_screen_page ( void );
extern void set_screen_page ( char );

extern void set_byte_out (char result);
extern char return_mainbyte ( void );

extern double get_increment ( void );
extern void set_increment ( double increment );

extern void set_os_termination ( int );
extern int return_os_termination ( void );

extern void restore_keyboard ( void );

#define ATCU_DEST		0
#define MODEL_DEST		1

/* Page definitions used by screen routines */
/* Change these if the function keys are to be changed
	 Also, the function key names in screen.c has to be changed as well.

	 It is imperative that the changed keys are defined in the same order!!!
	 ksc/tzqh  XX/XX/96
*/

#define ATCU_PAGE   0
#define PLC_PAGE    1
#define TUNE_PAGE   2
#define IO_PAGE     3
#define GRAPH_PAGE  4
#define TRACK_PAGE  5
#define CREDIT_PAGE 6
#define NUM_PAGES   7  /* to define the total number of pages in kbdtask.c */

#define CONTROL_A	1
#define CONTROL_B	2
#define CONTROL_C	3
#define CONTROL_D	4
#define CONTROL_E	5
#define CONTROL_F	6
#define CONTROL_G	7
#define CONTROL_H	8
#define CONTROL_I	9
#define CONTROL_J	10
#define CONTROL_K	11
#define CONTROL_L	12
#define CONTROL_M	13
#define CONTROL_N	14
#define CONTROL_O	15
#define CONTROL_P	16
#define CONTROL_Q	17
#define CONTROL_R	18
#define CONTROL_S	19
#define CONTROL_T	20
#define CONTROL_U	21
#define CONTROL_V	22
#define CONTROL_W	23
#define CONTROL_X	24
#define CONTROL_Y	25
#define CONTROL_Z	26


