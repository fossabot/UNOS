/*
****************************************************************************

					Antenna Tracking Control Unit

				Position Loop Controller Include File

						Controller Definitions

_author:	L. Sciacca

_date:		23-Oct-1990


Date 30/5/92 by DJLB. Modified to reflect lower sensitivity (10v vs 8v max)
		of drive controllers at Geraldton.

****************************************************************************
*/
/*
*/

#define AZ_AXIS 0
#define EL_AXIS 1


#define SAMPLE_PER				0.0666666

	/* setup Az default parameters */
#define AZ_PROP_GAIN			1.0    /* Was 25000.0 */
#define AZ_INTEGRAL_GAIN		0.001     /* Was 900.0 */

#define AZ_RES_BW				5.0        /* was 1.1 */
#define AZ_DIFF_BW				4.0
#define AZ_INT_POSITIVE_LIMIT	5.0
#define AZ_INT_NEGATIVE_LIMIT	-5.0
#define AZ_MIT_ALPHA			2.2
#define AZ_MIT_BETA				0.01
#define AZ_MIT_FF_HIGH			2.0
#define AZ_MIT_FF_LOW			0.00
#define AZ_MIT_MAXINCR			0.0005
#define AZ_SIGN					1
#define AZ_MIT_MAXREF			0.2
#define AZ_PARABOLIC_GAIN		0.054
#define AZ_MAX_VOLTS			5.0   /* Was 8.0 */
#define AZ_MIN_VOLTS			-5.0  /* Was -8.0 */

#define AZ_GEAR_RATIO			150.0

#define AZ_MIT_FF_INITIAL_GAIN	1.3
#define AZ_RATE_LIMIT_REGION	0
#define AZ_MAX_ACCELERATION		0.2		/* deg/sec/sec */
#define AZ_RATE_GAIN			1.0	/* SWEO Motor gain deg/sec/V */
										   /* Was 1312.5 */
										   /* UNIMEL project doesn't
										   /* require this gain */
// Rate Limiting
#define AZ_CONTROL_RATE_MAX 15000.0	/* Max control rate deg/sec */
//#define AZ_S_STEP_MAX		4000.0			/* deg/sec inside inner region */
//#define AZ_STEP_MAX 		6000.0				/* Max deg/sec per step */
#define AZ_INNER_REGION 	6000.0			/* deg/sec */


	/* setup El default parameters */
#define EL_PROP_GAIN			1.0
#define EL_INTEGRAL_GAIN		0.001
#define EL_RES_BW				3.0
#define EL_DIFF_BW				4.0
#define EL_INT_POSITIVE_LIMIT	5.0
#define EL_INT_NEGATIVE_LIMIT	-5.0
#define EL_MIT_ALPHA			2.2
#define EL_MIT_BETA				0.01
#define EL_MIT_FF_HIGH			2.0
#define EL_MIT_FF_LOW			0.00
#define EL_MIT_MAXINCR			0.0005
#define EL_SIGN					1.0
#define EL_MIT_MAXREF			0.2
#define EL_PARABOLIC_GAIN		0.054
#define EL_MAX_VOLTS			5.0
#define EL_MIN_VOLTS			-5.0

#define EL_MIT_FF_INITIAL_GAIN	1.3

#define EL_MAX_ACCELERATION		0.2		/* deg/sec/sec */
#define EL_RATE_LIMIT_REGION	0

#define EL_GEAR_RATIO			150.0
#define EL_RATE_GAIN			-1.0	/* Motor deg/sec/V */
							/* neg since for el motor -ve voltage = up dirn */

#define EL_CONTROL_RATE_MAX 15000	/* Max control rate deg/sec */
#define EL_S_STEP_MAX 4000			/* deg/sec inside inner region */
#define EL_STEP_MAX 6000				/* Max deg/sec per step */
#define EL_INNER_REGION 3000			/* deg/sec */

#define SLEW_REGION_DEF	5.0		/* region where slewing is done rather than pos loop */

