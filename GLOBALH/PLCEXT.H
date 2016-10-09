/*
********************************************************************************
								PLCEXT.h
Header file for PLC routines


********************************************************************************
*/
void plc_task ( void * );

void ReadPLC ( PLC_struct * PLC );
void WritePLC ( int word );

