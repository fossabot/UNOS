
/********************************************************************/
/*																	*/
/*																	*/
/*						 HEADER FILE FOR TASK NAMES					*/
/*																	*/
/*									by								*/
/*								Robert Betz							*/
/*			  Department of Electrical and Computer Engineering		*/
/*						  University of Newcastle					*/
/*							 (Copyright 1992)						*/
/*																	*/
/*																	*/
/********************************************************************/




/* each task in the system is given a unique name which is used by
other tasks as the mailing address when using the mailbox mechanism.
The name can be of any length up to the limit set by the 'C' compiler
for string variables. NOTE: it is the string name which is used as the
address by the mail box routines.
*/
#ifndef __TASKNAME_H
#define __TASKNAME_H


/* User adds own task names here, in similar vein. Define each in your
	code - either in a .C or .H file -, e.g.
char chSequencerTaskName[] = "SEQUENCER Task";
*/
extern char kbd_parser_name [];
extern char ScreenTaskName [];
extern char ch_0_rx[];
extern char ch_0_tx[];
extern char ch_1_rx[];
extern char ch_1_tx[];
extern char ch_2_rx[];
extern char ch_2_tx[];
extern char ch_3_rx[];
extern char ch_3_tx[];
extern char chSequencerTaskName[];
extern char chLCMUParserTaskName[];
extern char chRCMUParserTaskName[];
extern char chRX0ProtocolTaskName[];
extern char chRX1ProtocolTaskName[];
extern char chRX2ProtocolTaskName[];
extern char chRX3ProtocolTaskName[];
extern char chOrbitTrackTaskName[];
extern char chBeaconSimulatorTaskName[];
extern char chPLCTaskName[];
extern char chServerProtocolTaskName[];
extern char chSLPTaskName [];
extern char chDecoderTaskName[];



#endif
