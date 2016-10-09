/*
********************************************************************************
							SDC.h
				Synchro-to-Digital Converter Card Protoypes

Author: L. J. Sciacca
		Australian Systems Research (NSW) Pty Limited

Latest: 2 July 1994

********************************************************************************
*/


#define AZ_SDC_CHANNEL0 0
#define AZ_SDC_CHANNEL1 2

#define AZ_COINCIDENCE	17.5640

#define EL_SDC_CHANNEL0	1
#define EL_SDC_CHANNEL1 3

#define EL_COINCIDENCE  5.5543

/*
********************************************************************************
Read_SDC_Axis ()

Routine to return angle in degrees.

!!!!Note that unless each axis is wired up with correct polarity we will have
to make special conditions for each axis (or separate routines to hardwire
the differences).

The fine and coarse coils of the resolver should be wired (for convenicne
of future changes) to coincide with the same direction.

Parameters:
	double * angle
	unsigned char coarse_channel 0-5
	unsigned char fine_channel 0-5
	double coincidence - angle between 16x zero and 1x zero

Returns:
	unsigned char error

********************************************************************************
*/
unsigned char Read_SDC_Axis ( double * angle,
				unsigned char coarse_channel, unsigned char fine_channel,
				double coincidence );
