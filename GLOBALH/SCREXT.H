/*
********************************************************************************
								scrext.h
					Header for screen global routines

Author: L. J. Sciacca
		ASR (NSW) P/L

Product: TS3000

Latest: 4 June 1994

********************************************************************************
*/

extern void protect_screen ( void );
extern void unprotect_screen ( void );
extern void update_screen ( unsigned int time );
extern void display_function_keys ( void );

