
#ifndef __SIMINTEL_H
#define __SIMINTEL_H


/*
*******************************************************************************
								TIED SOFTWARE
 PROJECT	:- TS3000

 MODULE 	:- GLOBALH

 FILE 		:- SIMINTEL.H

 PROGRAMMER :- L. Sciacca
				Australian Systems Research (NSW) Pty Limited

Intelsat Simulator routine. Called by Sequencer (at end of sequencer main loop)
		This code can be found in SIMINTEL.c

Latest: 7 June 1994

*******************************************************************************
*/
extern void simintel ( double dish_az, double dish_el );
extern double get_intelsat_sim_bcn ( void );
extern double get_intelsat_sim_az ( void );
extern double get_intelsat_sim_el ( void );
extern void init_simintel ( void );

#endif



