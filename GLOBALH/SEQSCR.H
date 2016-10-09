/*
****************************************************************************

						Sequencer Screen Routines
								Prototypes

									by

								L. J. Sciacca

_latest: 22-Oct-1990
		22-oct-1990
		29-march-1991 ljs - Clean up of code.

	Called from screen task. This was done to ensure only one task ever 
writes to the screen at any time.

****************************************************************************
*/
#define	NO_ERROR			0
#define	NO_CONTROL_ERROR	1
#define	DRIVE_TALK_ERROR	2
#define	DRIVE_FAILURE		3
#define	BAD_KEY_ENTRY		4

extern void process_error ( unsigned int temp );
extern void display_atcu ( void );
extern void display_unos ( void );

extern void init_parser ( void );

extern unsigned int return_mode ( void );
extern unsigned int return_atcu_command ( void );

extern void set_atcu_graph_page ( void );
extern unsigned int return_show_atcu_graph_page ( void );

extern void set_latest_error ( unsigned int value );
