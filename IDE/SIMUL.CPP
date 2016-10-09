
void fvdRunSimulator(void)
{
	double dfTrueCurrentAz, dfTrueCurrentEl;
	/* initial antenna position */
	static double dfEncoderAzimuth, dfEncoderElevation;
	static BOOLEAN boFirstCall = TRUE;

	if (boFirstCall)
	{
		/* Set up the Simulator's epoch.							*/
		ReadClock(&tstSimulatorEpoch);
		tstSimulatorEpoch.hundredths = 0;
		tstSimulatorEpoch.seconds = 0;
		tstSimulatorEpoch.minutes = 0;
		tstSimulatorEpoch.hours = 0;

		dfEncoderAzimuth = tstSimulatorEncoder.az_coarse;
		dfEncoderElevation = tstSimulatorEncoder.el_coarse;

		df_PolariserAngle = tst_SimPolariserSetup.coarse;

		boFirstCall = FALSE;
	}

 	/* update the antenna model */

	fvdAntenna(&dfEncoderAzimuth, &dfEncoderElevation);

	/* convert the current antenna position into a fine reading */

	uinSimulatorAzFine = fuinCalculateFineReading(dfEncoderAzimuth,
							tstSimulatorEncoder.az_gear_ratio);

	uinSimulatorElFine = fuinCalculateFineReading(dfEncoderElevation,
							tstSimulatorEncoder.el_gear_ratio);

	/* copy variables to global vars to be sent to maintenance page */

	dfTrueCurrentAz = dfEncoderAzimuth + tstSimulatorEncoder.az_offset;
	dfTrueCurrentEl = dfEncoderElevation + tstSimulatorEncoder.el_offset;

	/* Update satellite positions and calculate beacon signal strength   */
	/* Note: use the antenna's pointing angle - not the indicated angle  */

	dfBeaconLevel = fdfCalculateBeaconLevel(dfTrueCurrentAz,dfTrueCurrentEl);

	/* Update simulated polariser position.								*/
	fvd_SimulatedPolariser();



	/* copy to variables to display on sequencer maintenance page */

	dfmpTrueCurrentAz = dfTrueCurrentAz;
	dfmpTrueCurrentEl = dfTrueCurrentEl;
	dfmpBeaconLevel = dfBeaconLevel;

	fvdCalculateTrackingPerformance();
}


static double fdfAntennaPattern(double dfDishAz, double dfDishEl,
						double dfSatAz, double dfSatEl)
{
	double dfAlpha,dfOBA,dfX,dfLinearValue,dfAzOBA;

	dfAlpha = 2.8/dfSimulatorBeamwidth;
		/* The above constant was changed from  root (6)	*/
		/* to 2.8 / bw by RHM 8/6/92. This gives the correct	*/
		/* beamwidth (-3db to -3db) as well as the correct	*/
		/* concavity at the top of the beam.			*/

	/* check this line should it be +/- 180 */

	dfAzOBA = fdfFixAzError(dfSatAz - dfDishAz);

	/* find the actual off-boresight-angle */

	dfOBA = sqrt((dfSatEl - dfDishEl)*(dfSatEl - dfDishEl) +
			cos(dfSatEl*DEG_TO_RAD) * cos(dfDishEl*DEG_TO_RAD) *
			(dfAzOBA)*(dfAzOBA));

	dfX = dfOBA * dfAlpha;

	if (dfOBA < 0.01)   /* for small oba use taylor series approximation */
		dfLinearValue = (1 - (dfX*dfX)/6.0 + (dfX*dfX*dfX*dfX)/120.0);
	else
		dfLinearValue = sin(dfX)/dfX;

	return(dfLinearValue*dfLinearValue);

}


/*************************************************************************

	Function:	fdfCalculateBeaconLevel
	Author:		Adrian.Bastiani
	Status:		requires testing
	Summary:	Calculates the azimuth and elevation of the satellite the
				beacon receiver is tuned to and takes the current dish az
				and el and returns the output of the beacon in dB. Adds in
				thermal noise. Assumes there is a thermal noise floor of
				-10dB.
				Channel 5 picks up satellite A (number 3).
				Channel 2 picks up satellite B (number 7).
				The rest of the channels return random noise.

				The on boresight power level is calculated as follows:

				1dB => 1/10 = 0.1 => 10^0.1 = 1.258925;

	Commenced:		9/8/91
	Last Revised:	30/7/92
	Modifications: 	30/7/92: Set the dfSatAz and dfSatEl to 0.0 intially
				to prevent rubbish from the stack being
				returned from these angles should the
				beacon not be tuned to a satellite.  AB.

***************************************************************************/

static double fdfCalculateBeaconLevel(double dfDishAz, double dfDishEl)
{
	double dfLinearPower = 0.0;
	double dfLinearAntennaGain = 0.0;
	double dfTimeFromEpoch;
	TimeRecord tstCurrentTime, tstEpochTime;
	double dfSatAz = 0.0, dfSatEl = 0.0;

	switch(get_current_beacon_number())
	{
		case 5:     /* tuned to satellite 3 */
					ReadClock(&tstCurrentTime);
					dfTimeFromEpoch = fdfDurationInDaysFrom(&tstSimulatorEpoch,
										&tstCurrentTime);
					fvdGetIntelsatAzEl(dfTimeFromEpoch, &dfSatAz, &dfSatEl,
																TRUE, 3);
					dfLinearAntennaGain = fdfAntennaPattern(dfDishAz,
											dfDishEl, dfSatAz, dfSatEl);
					/* Max power level scaled by Antenna pattern */
					dfLinearPower = dfLinearAntennaGain * dfSat3Offset;
					break;

		case 2: 	/* tuned to satellite 7 */
					ReadClock(&tstCurrentTime);
					dfTimeFromEpoch = fdfDurationInDaysFrom(&tstSimulatorEpoch,
										&tstCurrentTime);
					dfTimeFromEpoch = fdfDurationInDaysFrom(&tstEpochTime,
										&tstCurrentTime);
					fvdGetIntelsatAzEl(dfTimeFromEpoch, &dfSatAz, &dfSatEl,
																TRUE, 7);
					dfLinearAntennaGain = fdfAntennaPattern(dfDishAz,
										dfDishEl, dfSatAz, dfSatEl);
					/* Max power level scaled by Antenna pattern */
					dfLinearPower = dfLinearAntennaGain * dfSat7Offset;
					break;

		case 8:     /* Any of the other satellites transmit on channel 8 */
					ReadClock(&tstCurrentTime);
					dfTimeFromEpoch = fdfDurationInDaysFrom(&tstSimulatorEpoch,
										&tstCurrentTime);
					/* No particular satellite, just not 3 or 7 */
					fvdGetIntelsatAzEl(dfTimeFromEpoch, &dfSatAz, &dfSatEl,
																TRUE, 11);
					dfLinearAntennaGain = fdfAntennaPattern(dfDishAz,
										dfDishEl, dfSatAz, dfSatEl);
					/* Max power level scaled by Antenna pattern */
					dfLinearPower = dfLinearAntennaGain * dfSatOffset;
					break;

		default:    /* just return random noise + noise floor */;
	}

	/* record the satellite azimuth and elevation for the maintenance page */

	dfmpSatAz = dfSatAz;
	dfmpSatEl = dfSatEl;

	/* add -10dB noise floor */

	dfLinearPower = dfLinearPower + 0.1;

	/* convert to dB and add thermal noise */

	return( 10.0 * log10(dfLinearPower) +
			NOISE_LEVEL * 2.0 * (((double)rand()/(double)RAND_MAX)-0.5) );

}




/*************************************************************************

	Function:	fdfGetSimulatorBeaconLevel
	Author:		Adrian.Bastiani

	Status:		requires testing

	Summary:	Returns the output of the beacon in dB. The beacon
				level has been calculated in the simulator.

	Commenced:		8/4/92
	Last Revised:	8/4/92

	Modifications:

***************************************************************************/

double fdfGetSimulatorBeaconLevel(void)
{
	return(dfBeaconLevel);
}

void fvdGetTrackingPerformance(double *dfAzError, double *dfElError,
	double *dfAzRMS, double *dfElRMS, double *dfAzPeak, double *dfElPeak)
{
	*dfAzError = dfmpAzError;
	*dfElError = dfmpElError;
	*dfAzRMS =  sqrt ( dfmpAzMS );
	*dfElRMS =   sqrt ( dfmpElMS );
	*dfAzPeak =  dfmpAzPeak;
	*dfElPeak =  dfmpElPeak;
}


static void fvdCalculateTrackingPerformance(void)
{
	double dfAbsAzError, dfAbsElError;
	static unsigned long int ulginSampleNumber = 1;

	dfmpAzError = fdfFixAzError(dfmpTrueCurrentAz - dfmpSatAz);
	dfmpElError = dfmpTrueCurrentEl - dfmpSatEl;

	dfAbsAzError = fabs(dfmpAzError);
	dfAbsElError = fabs(dfmpElError);

	if (boResetTrackingMonitor)
	{
		dfmpAzPeak = 0.0;
		dfmpElPeak = 0.0;
		dfmpElMS = 0.0;
		dfmpAzMS = 0.0;
		disable();
			boResetTrackingMonitor = FALSE;
		enable();
		ulginSampleNumber = 1;
	}
	else
	{
		if (dfAbsAzError>dfmpAzPeak)
			dfmpAzPeak = dfAbsAzError;
		if (dfAbsElError>dfmpElPeak)
			dfmpElPeak = dfAbsElError;
      	/*
			Calculate the rms of the errors using a recursive formula
			(thanks owing to P.Moylan and R.H.Middleton)
		*/

		dfmpAzMS = dfmpAzMS * ( (double)(ulginSampleNumber - 1)/
				(double)(ulginSampleNumber) ) +
				(dfAbsAzError * dfAbsAzError)/(double)(ulginSampleNumber);

		dfmpElMS = dfmpElMS * ( (double)(ulginSampleNumber - 1)/
				(double)(ulginSampleNumber) ) +
				(dfAbsElError * dfAbsElError)/(double)(ulginSampleNumber);

		ulginSampleNumber++;
	}

}


void fvdInitialiseSimulSTORE(void)
{
	station_data_struct			tstStation;
	software_limits_struct  	tstSWLimits;
	IntelsatDataStructure		tstSat1, tstSat2;
	IntelsatTimeStructure   	tstSat1Time, tstSat2Time;
	tracking_parameters_struct 	tstTrackParams;
	beacon_details_struct 		tstBeacon;
	TimeRecord					tstEpoch;

	ReadClock(&tstEpoch);
	tstEpoch.hundredths = 0;
	tstEpoch.seconds = 0;
	tstEpoch.minutes = 0;
	tstEpoch.hours = 0;

	/* Data for Satellite A - stored as satellite number 3 */

	tstSat1.dfLm0   = 	220.0;
	tstSat1.dfLm1   =   0.0003;
	tstSat1.dfLm2   =   0.001;
	tstSat1.dfLonc  =   0.01;
	tstSat1.dfLonc1 =   -0.002;
	tstSat1.dfLons  =   0.01;
	tstSat1.dfLons1 =   -0.01;
	tstSat1.dfLatc  =   3.54;
	tstSat1.dfLatc1 =   0.06;
	tstSat1.dfLats  =   3.54;
	tstSat1.dfLats1 =   0.02;
	fvdPutIntelsatData (3, &tstSat1);
	tstSat1Time.IntelsatTimeIncrement = 3; /* minutes */
	tstSat1Time.IntelsatDuration = 5;
	tstSat1Time.EpochTime = tstEpoch;
	put_Intelsat_times (3, &tstSat1Time);
	put_satellite_name (3, "101");
	put_beacon_number (3, 5);
	put_beacon_offset (3, 2);

	/* Data for Satellite B - stored as satellite number 7 */

	tstSat2.dfLm0 	= 	200.0;
	tstSat2.dfLm1 	= 	0.0;
	tstSat2.dfLm2 	= 	0.0;
	tstSat2.dfLonc 	= 	0.0;
	tstSat2.dfLonc1 = 	0.0;
	tstSat2.dfLons	=  	0.0;
	tstSat2.dfLons1 = 	0.0;
	tstSat2.dfLatc 	=  	0.0;
	tstSat2.dfLatc1 = 	0.0;
	tstSat2.dfLats 	=  	0.0;
	tstSat2.dfLats1 = 	0.0;
	fvdPutIntelsatData (7, &tstSat2);
	tstSat2Time.IntelsatTimeIncrement = 10; /* minutes */
	tstSat2Time.IntelsatDuration = 6;
	tstSat2Time.EpochTime = tstEpoch;
	put_Intelsat_times (7, &tstSat2Time);
	put_satellite_name (7, "202");
	put_beacon_number (7, 2);
	put_beacon_offset (7, 1);

  	/* General station data */

	tstStation.latitude 	=  	-35.0;
	tstStation.longitude 	=  	220.0;
	tstStation.height		= 	0.02;
	tstStation.mount_tilt 	=  	0.0;
	tstStation.mount_twist 	= 	0.0;
	tstStation.fallback 	= 	0;    /* fallback not enabled */
	put_station_data(&tstStation);
  
	/* Antenna Parameters */
	tstSimulatorEncoder.az_offset  = 2.3;
	tstSimulatorEncoder.el_offset  = -5.2;
	tstSimulatorEncoder.az_coarse  = 350.0;
	tstSimulatorEncoder.el_coarse  = 38.0;
	tstSimulatorEncoder.az_speed   = 0.002;
	tstSimulatorEncoder.el_speed   = 0.002;
	tstSimulatorEncoder.az_dead_band = 0.016;
	tstSimulatorEncoder.el_dead_band = 0.016;
	tstSimulatorEncoder.az_gear_ratio =	12;
    tstSimulatorEncoder.el_gear_ratio = 12;
	put_encoder_setup(&tstSimulatorEncoder);
	/* Polariser Parameters */

	tst_SimPolariserSetup.offset = 0.5;
	tst_SimPolariserSetup.coarse = 45.0;
	tst_SimPolariserSetup.speed  =  5.0;
	tst_SimPolariserSetup.dead_band = 0.3;
	tst_SimPolariserSetup.gear_ratio = 10.0;
	put_polariser_setup(&tst_SimPolariserSetup);
	/* Load Software Limits */

	tstSWLimits.az_min = 300.0;
	tstSWLimits.az_max = 60.0;
	tstSWLimits.el_min = 10.0;
	tstSWLimits.el_max = 65.0;
	put_software_limits(&tstSWLimits);

	/* System Tracking parameters */

	tstTrackParams.beamwidth = 0.5;
	tstTrackParams.move_size = 0.04;
	put_tracking_parameters (&tstTrackParams);

	put_max_time_increment(30.0); /* minutes */

	/* Beacon setup parameters */

	tstBeacon.lo_limit 	= -3.0;
	tstBeacon.hi_limit 	=  5.0;
	tstBeacon.gradient 	=  1.0;
	tstBeacon.slew_limit=  0.3;
	put_beacon_details (&tstBeacon);

	fvdPutCurrentSatelliteNumber(3);

	put_beacon_number(3, 5);


}


