/*
********************************************************************************
							iohw.h

********************************************************************************
*/

void init_pc74 ( void );

void dac_out ( int ch, double voltage, double min, double max );

void OutputAzVolts_temp ( double volts );

void OutputElVolts_temp ( double volts );

float InputSigVolts ( void );

float TachoSig ( void );

void init_pc14 ( void );

double read_az_encoder ( void );

double read_el_encoder ( void );
