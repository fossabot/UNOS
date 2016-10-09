/*****************************************************************************
* POLYTRON VERSION CONTROL SOFTWARE _ PVCS (tm) Header Information
*
* 	$Revision:   2.6  $
*	$Date:   06 Mar 1992 17:58:50  $
*
*****************************************************************************/
/****************************************************************************
* FILE:    NVRAMTST.C  (Project file: NVRAMTST.PRJ)
*
* DESCRIPTION: This is a test program for the ATCU NVRAM Controller software.
*
* PROGRAMMER:	R. Cockerell
*
* DATE:   27/3/91
*
****************************************************************************
*   The test program consists of a menu driven selection of various functions
* to be performed on the NVRAM. The program does not do any functional checks
* of entered data so it is far from crash-proof.
*   Run the program first (NVRAMTST.EXE) and see what happens. Then look through
* the code for more information (i.e. no separate documentation).
*   When more routines are added, entries will need to be made in the "PUT" &
* "GET" menus, the "ENTER_PUT_DATA()" & "DISPLAY_GET_DATA()" routines. For
* large data it may be necessary to allow fixed data to be "put()" instead
* of operator entered data ?
* 2-3-92. DJLB added facility to force load of default values.
*
* LJS April 1994. New NVRAM card that uses a window of 2k into the NVRAM chips
* This means that the nvram is nolonger a linear address space but must be accessed
* using a virtual-to-paged address mapping. Thus the virtual addresses can now be
* viewed as before using "display_pointers" but these pointers are now the
* linear virtual addresses. The driver in nvrampc.c converts these linear addresses
* to paged memory.
****************************************************************************/

#include <stdlib.h>
#include <conio.h>
#include <stdio.h>
#include <dos.h>
#include <string.h>

#include "nvram.h"
#include "nvramext.h"
#include "posext.h"
#include "..\posloop\poscon.h"
#include "intelsat.h"


/**************  Pointer Declaration  ******************
* This requires that the 'static' qualifier be removed
* from these pointers in "NVRAM.C" for this test.
*******************************************************/

extern unsigned long sat_details_ptr[ NUM_SATELLITES ],
			star_details_ptr[ NUM_STARS ],
			beacon_details_ptr,
			scan_data_ptr,
			orbit_model_ptr[ NUM_SATELLITES ],
			orbit_covariance_ptr[ NUM_SATELLITES ],
			nv_calibration_coefficients_ptr,
			station_position_ptr,
			current_sat_num_ptr,
			orbit_start_ptr,
			current_star_num_ptr,
			sw_limits_ptr,
			stow_pos_ptr,
			con_ptr [ 2 ],	/* two axes of controller constants*/
			intelsat_param_ptr [ NUM_SATELLITES ],
			intelsat_epoch_ptr [ NUM_SATELLITES ];

// The following routine is called by nvrampc
// to indicate that nvram is to be used.

static int nvram_sim;

int get_nvram_sim( void )
{
int temp;

	temp = nvram_sim;

	return temp;
}

/************************
*	MAIN_MENU
************************/


static char main_menu( void )
{
	clrscr();

	gotoxy( 5 , 1 );
	cprintf("                    NVRAM Test Program");

	gotoxy( 20 , 3 );
	highvideo();
	cprintf("MAIN MENU");

	gotoxy( 10 , 5 );
	highvideo();
	cprintf("1. ");
	lowvideo();
	cprintf("Initialise NVRAM board (must be done upon program reset).");

	gotoxy( 10 , 7 );
	highvideo();
	cprintf("2. ");
	lowvideo();
	cprintf("Diagnostic test of NVRAM board.");

	gotoxy( 10 , 9 );
	highvideo();
	cprintf("3. ");
	lowvideo();
	cprintf("Load DEFAULT VALUES into NVRAM board.");

	gotoxy( 10 , 11 );
	highvideo();
	cprintf("4. ");
	lowvideo();
	cprintf("'PUT' operation ...");

	gotoxy( 10 , 13 );
	highvideo();
	cprintf("5. ");
	lowvideo();
	cprintf("'GET' operation ...");

	gotoxy( 10 , 15 );
	highvideo();
	cprintf("6. ");
	lowvideo();
	cprintf("Write a specified byte to NVRAM board (i.e. fault simulator)...");

	gotoxy( 10 , 17 );
	highvideo();
	cprintf("7. ");
	lowvideo();
	cprintf("Dump an area of NVRAM to screen...");

	gotoxy( 10 , 19 );
	highvideo();
	cprintf("8. ");
	lowvideo();
	cprintf("Display pointer addresses.");

	gotoxy( 10 , 21 );
	highvideo();
	cprintf("9. ");
	lowvideo();
	cprintf("Quit program.");

	gotoxy( 10 , 25 );
	normvideo();
	cprintf("ENTER option: ");

	return ( (char)getche() );
}

/************************
*	PUT_MENU
************************/

static char put_menu( void )
{
	clrscr();

	gotoxy( 5 , 1 );
	cprintf("NVRAM Test Program - R. Cockerell, 27/3/91.");

	gotoxy( 20 , 3 );
	highvideo();
	cprintf("PUT MENU");

	gotoxy( 10 , 5 );
	highvideo();
	cprintf("a. ");
	lowvideo();
	cprintf("put_satellite_details()");

	gotoxy( 10 , 6 );
	highvideo();
	cprintf("b. ");
	lowvideo();
	cprintf("put_current_satellite_num()");

	gotoxy( 10 , 7 );
	highvideo();
	cprintf("c. ");
	lowvideo();
	cprintf("put_star_details()");

	gotoxy( 10 , 8 );
	highvideo();
	cprintf("d. ");
	lowvideo();
	cprintf("put_current_star_num()");

	gotoxy( 10 , 9 );
	highvideo();
	cprintf("e. ");
	lowvideo();
	cprintf("put_beacon_details()");

	gotoxy( 10 , 10 );
	highvideo();
	cprintf("f. ");
	lowvideo();
	cprintf("put_orbit_track_start_type()");

	gotoxy( 10 , 11 );
	highvideo();
	cprintf("g. ");
	lowvideo();
	cprintf("put_scan_data()");

	gotoxy( 10 , 12 );
	highvideo();
	cprintf("h. ");
	lowvideo();
	cprintf("put_calibration_coefficients()");

	gotoxy( 10 , 13 );
	highvideo();
	cprintf("j. ");
	lowvideo();
	cprintf("put_station_position()");

	gotoxy( 10 , 14 );
	highvideo();
	cprintf("k. ");
	lowvideo();
	cprintf("put_software_limits()");

	gotoxy( 10 , 15 );
	highvideo();
	cprintf("m. ");
	lowvideo();
	cprintf("put_stow_position()");

	gotoxy( 10 , 16 );
	highvideo();
	cprintf("n. ");
	lowvideo();
	cprintf("put_controller_parameters()");

	gotoxy( 10 , 17 );
	highvideo();
	cprintf("o. ");
	lowvideo();
	cprintf("put_Intelsat_parameters()");

	gotoxy( 10 , 18 );
	highvideo();
	cprintf("p. ");
	lowvideo();
	cprintf("put_Intelsat_Epoch()");

/**** Add more functions here as they become available. ****/

	gotoxy( 10 , 19 );
	normvideo();
	cprintf("ENTER option: ");

	return ( (char)getche() );
}

/************************
*	GET_MENU
************************/

static char get_menu( void )
{
	clrscr();

	gotoxy( 5 , 1 );
	cprintf("NVRAM Test Program - R. Cockerell, 27/3/91.");

	gotoxy( 20 , 3 );
	highvideo();
	cprintf("GET MENU");

	gotoxy( 10 , 5 );
	highvideo();
	cprintf("a. ");
	lowvideo();
	cprintf("get_satellite_details()");

	gotoxy( 10 , 6 );
	highvideo();
	cprintf("b. ");
	lowvideo();
	cprintf("get_current_satellite_num()");

	gotoxy( 10 , 7 );
	highvideo();
	cprintf("c. ");
	lowvideo();
	cprintf("get_star_details()");

	gotoxy( 10 , 8 );
	highvideo();
	cprintf("d. ");
	lowvideo();
	cprintf("get_current_star_num()");

	gotoxy( 10 , 9 );
	highvideo();
	cprintf("e. ");
	lowvideo();
	cprintf("get_beacon_details()");

	gotoxy( 10 , 10 );
	highvideo();
	cprintf("f. ");
	lowvideo();
	cprintf("get_orbit_track_start_type()");

	gotoxy( 10 , 11 );
	highvideo();
	cprintf("g. ");
	lowvideo();
	cprintf("get_scan_data()");

		gotoxy( 10 , 12 );
	highvideo();
	cprintf("h. ");
	lowvideo();
	cprintf("get_calibration_coefficients()");

	gotoxy( 10 , 13 );
	highvideo();
	cprintf("j. ");
	lowvideo();
	cprintf("get_station_position()");

	gotoxy( 10 , 14 );
	highvideo();
	cprintf("k. ");
	lowvideo();
	cprintf("get_software_limits()");

	gotoxy( 10 , 15 );
	highvideo();
	cprintf("m. ");
	lowvideo();
	cprintf("get_stow_position()");

	gotoxy( 10 , 16 );
	highvideo();
	cprintf("n. ");
	lowvideo();
	cprintf("get_controller_parameters()");

	gotoxy( 10 , 17 );
	highvideo();
	cprintf("o. ");
	lowvideo();
	cprintf("get_Intelsat_parameters()");

	gotoxy( 10 , 18 );
	highvideo();
	cprintf("p. ");
	lowvideo();
	cprintf("get_Intelsat_Epoch()");

/**** Add more functions here as they become available. ****/

	gotoxy( 10 , 19 );
	normvideo();
	cprintf("ENTER option: ");

	return ( (char)getche() );
}

/************************
*	ENTER_PUT_DATA
************************/

static void enter_put_data( char command )
{
satellite_details_struct	sat_details;
star_details_struct		star_details;
beacon_details_struct		beacon_details;
scan_data			scan;
int				temp_int, i;
char				string[ 100 ], c;
calibration_coefficients	cal_co;
station_position		station_position;
software_limit_struct           sw_limit;
stow_pos_struct			stow_position;
controller_parameter_struct	controller_parameter;
IntelsatDataStructure intelsat_param;
IntelsatEpochStructure IntelsatEpoch;

	clrscr();

	switch ( command )
	{
		case 'a' :
		    printf("Enter satellite details in the following order on one line per satellite:\n");
			printf("name(8 chr) bcn_offset(f) bcn_sel(i) polariser(i) move_size(f)\n");
			for( i=0; i < NUM_SATELLITES; i++ )
			{
		    	gets( string );
			sscanf( string , "%s %f %i %i %f" ,
			sat_details.name,
			&sat_details.beacon_offset,
			&sat_details.beacon_select,
			&sat_details.polariser,
			&sat_details.move_size );
			put_satellite_details( &sat_details , i );
		    }
			break;

		case 'b' :
			printf("Enter current satellite number: ");
		    scanf("%i" , &temp_int );
		    put_current_satellite_num( &temp_int );
		    break;

		case 'c' :
			printf("Enter star details in the following order on one line per star:\n");
		    printf("name(10 chr) right_ascension(f) declination(f) epoch_mode(i)\n");
		    for( i=0; i < NUM_STARS; i++ )
			{
				gets( string );
			sscanf( string , "%s %lf %lf %i" ,
			star_details.name,
			&star_details.right_ascension,
			&star_details.declination,
			&star_details.epoch_mode );
			put_star_details( &star_details , i );
			}
			break;

		case 'd' :
			printf("Enter current star number: ");
			scanf("%i" , &temp_int );
			put_current_star_num( &temp_int );
			break;

		case 'e' :
			printf("Enter Beacon details in the following order "
		    					"on one line:\n");
		    printf("half_power_bw[0],1,2,3,4  slew_limit  hi_limit "
		    					" lo_limit\n");
                    gets( string );
			sscanf( string , "%f %f %f %f %f "
				     "%f %f %f" ,
				     &beacon_details.half_power_bw[ 0 ],
								 &beacon_details.half_power_bw[ 1 ],
				     &beacon_details.half_power_bw[ 2 ],
				     &beacon_details.half_power_bw[ 3 ],
				     &beacon_details.half_power_bw[ 4 ],
				     &beacon_details.slew_limit,
				     &beacon_details.hi_limit,
				     &beacon_details.lo_limit );

		    put_beacon_details( &beacon_details );
			break;

		case 'f' :
			printf("Enter orbit track start type: ");
			scanf("%i" , &temp_int );
		    put_orbit_start_type( &temp_int );
			break;

		case 'g' :
		    printf("Enter Scan details in the following order "
		    					"on one line:\n");
			printf("time_limit(i)  separation(f)  speed(f) "
							  " size(f) method(i)\n");
					gets( string );
		    sscanf( string , "%d %lf %lf %lf %d %d" ,
		    				&scan.time_limit,
						&scan.separation,
						&scan.search_speed,
						&scan.box_size,
						&scan.method );

		    put_scan_data( &scan );
		    break;

		case 'h' :
		    printf("Enter Calibration Coefficients in the following order "
								"on one line:\n");
		    printf("P1 P2 P3 P4 P5 P6 P7 P8\n");
                    gets( string );
		    sscanf( string , "%lf %lf %lf %lf %lf %lf %lf %lf" ,
			   &cal_co.P1,
			   &cal_co.P2,
			   &cal_co.P3,
			   &cal_co.P4,
			   &cal_co.P5,
			   &cal_co.P6,
			   &cal_co.P7,
			   &cal_co.P8);

			put_calibration_coefficients( &cal_co );
		    break;

		case 'j' :
		    printf("Enter Station Position in the following order "
		    					"on one line:\n");
		    printf("Latitude Longitude\n");
					gets( string );
			sscanf( string , "%lf %lf" ,
				 &station_position.latitude,
				 &station_position.longitude);
		    put_station_position( &station_position );
		    break;

		case 'k' :
		    printf("Enter Software Limits in the following order "
		    					"on one line:\n");
		    printf(
		    "az_cw_limit  az_ccw_limit  el_up_limit  el_down_limit\n");
					gets( string );
		    sscanf( string , "%lf %lf %lf %lf" ,
                             &sw_limit.az_cw_limit,
							 &sw_limit.az_ccw_limit,
                             &sw_limit.el_up_limit,
                             &sw_limit.el_down_limit );

		    put_seq_software_limits( &sw_limit );
		    break;

		case 'm' :
		    printf("Enter Stow Position datain the following order "
								"on one line:\n");
		    printf("Azimuth   Elevation\n");
                    gets( string );
			sscanf( string , "%lf %lf" ,
				&stow_position.az,
				&stow_position.el);

		    put_stow_positions( &stow_position );
		    break;

		case 'n' :
			printf("Which axis? (a for AZIMUTH, e for ELEVATION):");
					c = getche();
			if (c == 'a')
		    {
		    	i = AZ_AXIS;
		    	printf("\rEnter Controller parameters for the AZIMUTH axis as they are"
		    	" asked for.\n Press Return only, to accept default.\n");
					}
		    else
		    {
		     	i = EL_AXIS;
		    	printf("\rEnter Controller parameters for the ELEVATION axis as they are"
				" asked for.\n Press Return only, to accept default.\n");
		    }
		    get_controller_parameters( i, &controller_parameter );

		    printf("prop_gain (%lf) :",controller_parameter.prop_gain);
		    gets(string);
                    if( strlen(string) )
		    sscanf(string,"%lf ",&controller_parameter.prop_gain);
		    printf("\nintegral_gain (%lf) :",controller_parameter.integral_gain );
		    gets(string);
                    if( strlen(string) )
		    sscanf(string,"%lf ",&controller_parameter.integral_gain);
			printf("\nres_bw (%lf) :",controller_parameter.res_bw );
		    gets(string);
                    if( strlen(string) )
			sscanf(string,"%lf ",&controller_parameter.res_bw);
			printf("\ndiff_bw (%lf) :",controller_parameter.diff_bw );
			gets(string);
					if( strlen(string) )
		    sscanf(string,"%lf ",&controller_parameter.diff_bw);
		    printf("\nint_positive_limit (%lf) :", controller_parameter.int_positive_limit);
		    gets(string);
                    if( strlen(string) )
			sscanf(string,"%lf ",&controller_parameter.int_positive_limit);
			printf("\nint_negative_limit (%lf) :", controller_parameter.int_negative_limit);
			gets(string);
                    if( strlen(string) )
		    sscanf(string,"%lf ",&controller_parameter.int_negative_limit);
		    printf("\nsign (%lf) :",controller_parameter.sign);
		    gets(string);
					if( strlen(string) )
		    sscanf(string,"%lf ",&controller_parameter.sign);
		    printf("\nmit_alpha (%lf) :", controller_parameter.mit_alpha);
		    gets(string);
                    if( strlen(string) )
			sscanf(string,"%lf ",&controller_parameter.mit_alpha);
		    printf("\nmit_beta (%lf) :", controller_parameter.mit_beta);
		    gets(string);
					if( strlen(string) )
		    sscanf(string,"%lf ",&controller_parameter.mit_beta);
		    printf("\nmit_maxref (%lf) :", controller_parameter.mit_maxref);
		    gets(string);
                    if( strlen(string) )
		    sscanf(string,"%lf ",&controller_parameter.mit_maxref);
		    printf("\nmit_maxincr (%lf) :", controller_parameter.mit_maxincr);
		    gets(string);
                    if( strlen(string) )
			sscanf(string,"%lf ",&controller_parameter.mit_maxincr);
		    printf("\nmit_ff_high (%lf) :", controller_parameter.mit_ff_high);
		    gets(string);
					if( strlen(string) )
			sscanf(string,"%lf ",&controller_parameter.mit_ff_high);
			printf("\nmit_ff_low (%lf) :", controller_parameter.mit_ff_low);
			gets(string);
                    if( strlen(string) )
		    sscanf(string,"%lf ",&controller_parameter.mit_ff_low);
		    printf("\ninitial_mit_ff_gain (%lf) :", controller_parameter.initial_mit_ff_gain);
		    gets(string);
					if( strlen(string) )
			sscanf(string,"%lf ",&controller_parameter.initial_mit_ff_gain);
			printf("\nmax_volts (%lf) :", controller_parameter.max_volts);
		    gets(string);
					if( strlen(string) )
		    sscanf(string,"%lf ",&controller_parameter.max_volts);
		    printf("\nmin_volts (%lf) :", controller_parameter.min_volts);
			gets(string);
                    if( strlen(string) )
		    sscanf(string,"%lf ",&controller_parameter.min_volts);
			printf("\nmax_acceleration (%lf) :", controller_parameter.max_acceleration);
		    gets(string);
					if( strlen(string) )
		    sscanf(string,"%lf ",&controller_parameter.max_acceleration);
		    printf("\ngear_ratio (%lf) :", controller_parameter.gear_ratio);
			gets(string);
                    if( strlen(string) )
		    sscanf(string,"%lf ",&controller_parameter.gear_ratio);
		    printf("\nrate_gain (%lf) :", controller_parameter.rate_gain);
		    gets(string);
                    if( strlen(string) )
		    sscanf(string,"%lf ",&controller_parameter.rate_gain);
		    printf("\nrate_limit_region (%lf) :", controller_parameter.rate_limit_region);
		    gets(string);
					if( strlen(string) )
		    sscanf(string,"%lf ",&controller_parameter.rate_limit_region);

			put_controller_parameters( i, &controller_parameter );
			break;

		case 'o' :
			printf ("Enter Intelsat parameters on one line per satellite:\n");
			printf ("lm0 lm1 lm2 lonc lonc1 lons lons1 latc latc1 lats lats1\n");
			printf ("To finish entering data enter : and return\n");
			for( i=0; i < NUM_SATELLITES; i++ )
				{
				gets( string );
				if (string[0] == ':')	// terminate this tedious job
					break;

				sscanf( string , "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf" ,
					(IntelsatDataStructure *)&intelsat_param );

				put_intelsat_parameters ( i, &intelsat_param );
				}

			break;

		case 'p' :
			printf ("Enter Intelsat Epochs on one line per satellite:\n");
			printf ("hund, sec, min, hours, dayweek, daymon, month, year, century\n");
			printf ("To finish entering data enter : and return\n");
			for( i=0; i < NUM_SATELLITES; i++ )
				{
				gets( string );
				if (string[0] == ':')	// terminate this tedious job
					break;

				sscanf( string , "%u %u %u %u %u %u %u %u %u" ,
					IntelsatEpoch.hundredths,
					IntelsatEpoch.seconds,
					IntelsatEpoch.minutes,
					IntelsatEpoch.hours,
					IntelsatEpoch.dayofweek,
					IntelsatEpoch.dayofmonth,
					IntelsatEpoch.month,
					IntelsatEpoch.year,
					IntelsatEpoch.century );

				put_intelsat_epoch ( i, &IntelsatEpoch );
				}

			break;


		default :
			break;

	}
}

/************************
*	DISPLAY_GET_DATA
************************/

static void display_get_data( char command )
{
satellite_details_struct	sat_details;
star_details_struct		star_details;
beacon_details_struct		beacon_details;
scan_data			scan;
int				temp_int, i, nvram_OK = 1;
calibration_coefficients	cal_co;
station_position		station_position;
software_limit_struct           sw_limit;
stow_pos_struct			stow_position;
controller_parameter_struct	controller_parameter;
IntelsatDataStructure intelsat_param;
IntelsatEpochStructure IntelsatEpoch;

char	c;

	clrscr();

	switch ( command )
	{
		case 'a' :
		    printf("Satellite details in the following order are:\n");
			printf("name  bcn_offset(f)  bcn_sel(i)  polariser(i)  move_size(f)\n\n");
		    for( i=0; i < NUM_SATELLITES; i++ )
		    {
				if ( !get_satellite_details( &sat_details , i ) )
							nvram_OK = 0;
			printf( "%s %f %i %i %f\n" ,
			sat_details.name,
			sat_details.beacon_offset,
			sat_details.beacon_select,
			sat_details.polariser,
			sat_details.move_size );
		    }
                    break;

		case 'b' :
		    nvram_OK = get_current_satellite_num( &temp_int );

		    printf("Current satellite number = %i" , temp_int);
		    break;

		case 'c' :
			printf("Star details in the following order are:\n");
			printf("name  right_ascension(f)  declination(f)  epoch_mode(i)\n\n");
			for( i=0; i < NUM_STARS; i++ )
			{
				if ( !get_star_details( &star_details , i ) )
							nvram_OK = 0;
			printf( "%s %lf %lf %i\n" ,
			star_details.name,
			star_details.right_ascension,
			star_details.declination,
			star_details.epoch_mode );
			}
			break;

		case 'd' :
		    nvram_OK = get_current_star_num( &temp_int );

			printf("Current star number = %i" , temp_int);
		    break;

		case 'e' :
			nvram_OK = get_beacon_details( &beacon_details );

		    printf("Beacon details in the following order are:\n");
			printf("half_power_bw[0],1,2,3,4  slew_limit  "
		    				"hi_limit  lo_limit\n");
		    printf( "%5.3f %5.3f %5.3f %5.3f %5.3f "
					"%5.3f %5.3f %5.3f\n",
			    	beacon_details.half_power_bw[ 0 ],
				beacon_details.half_power_bw[ 1 ],
				beacon_details.half_power_bw[ 2 ],
				beacon_details.half_power_bw[ 3 ],
				beacon_details.half_power_bw[ 4 ],
				beacon_details.slew_limit,
				beacon_details.hi_limit,
				beacon_details.lo_limit );
		    break;

		case 'f' :
		    nvram_OK = get_orbit_start_type( &temp_int );

		    printf("Orbit track start type = %i\n" , temp_int);
		    break;

		case 'g' :
		    nvram_OK = get_scan_data( &scan );

		    printf("Scan details in the following order are:\n");
		    printf("time_limit(i)  separation(f)  speed(f) "
							" size(f) method(i)\n\n");
		    printf( "%d %5.3lf %5.3lf %5.3lf %d \n" ,
		    			scan.time_limit,
					scan.separation,
					scan.search_speed,
					scan.box_size,
					scan.method );
			break;

		case 'h' :
			nvram_OK = get_calibration_coefficients( &cal_co );

		    printf("Calibration Coefficients are:\n"
			   "P1	%5.3lf\n"
		    	   "P2	%5.3lf\n"
			   "P3	%5.3lf\n"
			   "P4	%5.3lf\n"
			   "P5	%5.3lf\n"
			   "P6	%5.3lf\n"
			   "P7	%5.3lf\n"
			   "P8	%5.3lf\n\n",
			   cal_co.P1,
			   cal_co.P2,
			   cal_co.P3,
			   cal_co.P4,
			   cal_co.P5,
			   cal_co.P6,
			   cal_co.P7,
			   cal_co.P8);
			break;

		case 'j' :
		    nvram_OK = get_station_position( &station_position );

		    printf("Station position data is:-\n\n"
					"Latitude       %5.3lf\n"
				"Longitude      %5.3lf\n\n",
				 station_position.latitude,
								 station_position.longitude);
		    break;

		case 'k' :
			nvram_OK = get_seq_software_limits( &sw_limit );

		    printf(	"Software Limits are:-\n\n"
				"az_cw_limit    %5.3lf\n"
				"az_ccw_limit   %5.3lf\n"
				"el_up_limit    %5.3lf\n"
				"el_down_limit  %5.3lf\n\n",
				sw_limit.az_cw_limit,
				sw_limit.az_ccw_limit,
				sw_limit.el_up_limit,
				sw_limit.el_down_limit);
		    break;

		case 'm' :
		    nvram_OK = get_stow_positions( &stow_position );
		    printf("Stow Position data is:-\n\n"
				"Azimuth        %5.3lf\n"
				"Elevation      %5.3lf\n",
			    	stow_position.az,
					stow_position.el);
		    break;

		case 'n' :
		    printf("Which axis? (a for az, e for el):");
		    c = getche();
		    if (c == 'a')i = AZ_AXIS;
                    else i = EL_AXIS;

			nvram_OK = get_controller_parameters( i, &controller_parameter );

		    if(i == 0)printf("\rController Parameters for the AZIMUTH axis are:\n");
			else printf("\rController Parameters for the ELEVATION axis are:\n");

		    printf("\nprop_gain:          %lf ",controller_parameter.prop_gain);
		    printf("\nintegral_gain:      %lf ",controller_parameter.integral_gain);
			printf("\nres_bw:             %lf ",controller_parameter.res_bw);
		    printf("\ndiff_bw:            %lf ",controller_parameter.diff_bw);
		    printf("\nint_positive_limit: %lf ",controller_parameter.int_positive_limit);
			printf("\nint_negative_limit: %lf ",controller_parameter.int_negative_limit);
		    printf("\nsign:        %lf ",controller_parameter.sign);
		    printf("\nmit_alpha:   %lf ",controller_parameter.mit_alpha);
			printf("\nmit_beta:    %lf ",controller_parameter.mit_beta);
			printf("\nmit_maxref:  %lf ",controller_parameter.mit_maxref);
		    printf("\nmit_maxincr: %lf ",controller_parameter.mit_maxincr);
		    printf("\nmit_ff_high: %lf ",controller_parameter.mit_ff_high);
		    printf("\nmit_ff_low:  %lf ",controller_parameter.mit_ff_low);
		    printf("\ninitial_mit_ff_gain: %lf ",controller_parameter.initial_mit_ff_gain);
		    printf("\nmax_volts:           %lf ",controller_parameter.max_volts);
		    printf("\nmin_volts:           %lf ",controller_parameter.min_volts);
		    printf("\nmax_acceleration:    %lf ",controller_parameter.max_acceleration);
		    printf("\ngear_ratio:          %lf ",controller_parameter.gear_ratio);
		    printf("\nrate_gain:           %lf ",controller_parameter.rate_gain);
		    printf("\nrate_limit_region:   %lf ",controller_parameter.rate_limit_region);
		    break;

		case 'o' :
			printf("Intelsat Parameters:\n");
			printf("lm0 lm1 lm2 lonc lonc1 lons lons1 latc latc1 lats lats1\n\n");
			for( i=0; i < NUM_SATELLITES; i++ )
				{
				if ( !get_intelsat_parameters ( i, &intelsat_param ) )
							nvram_OK = 0;

				printf( "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n" ,
					(IntelsatDataStructure)(intelsat_param) );
			}
			break;

		case 'p' :
			printf("Intelsat Epoch:\n");
			printf("hund, sec, min, hours, dayweek, daymon, month, year, century\n\n");
			for( i=0; i < NUM_SATELLITES; i++ )
				{
				if ( !get_intelsat_epoch ( i, &IntelsatEpoch ) )
							nvram_OK = 0;

				printf( "%u %u %u %u %u %u %u %u %u\n" ,
					IntelsatEpoch.hundredths,
					IntelsatEpoch.seconds,
					IntelsatEpoch.minutes,
					IntelsatEpoch.hours,
					IntelsatEpoch.dayofweek,
					IntelsatEpoch.dayofmonth,
					IntelsatEpoch.month,
					IntelsatEpoch.year,
					IntelsatEpoch.century
					);
					//(IntelsatEpochStructure)(IntelsatEpoch) );
			}
			break;

		default :
		    break;

	}

	if ( !nvram_OK )
		printf("\n\n\aError in reading NVRAM data");

	printf("\n\nHit any key to continue...");
	while( !kbhit() );
}

/************************
*	DISPLAY_NVRAM
************************/

static void display_NVRAM( void )
{
unsigned char	*ptr;
unsigned long	nvram_base_addr;
unsigned int	dump_size,
				i,
				j;

	clrscr();

	printf("The NVRAM 64K base address is %lX." , NVRAM_BASE_ADDR );
	printf("\nEnter desired start address (D123:4567 entered as D1234567) : ");
	scanf("%X" , &nvram_base_addr );
	printf("\nEnter desired dump size in bytes (max 65535) : ");
	scanf("%u" , &dump_size );

	ptr = (unsigned char*)nvram_base_addr;

	for( i=0; i < dump_size; i+=16 )
	{
		printf("\n%08lX:  ",nvram_base_addr + (unsigned long)i);
		for( j = 0; j < 16; j++ )
			printf("%02X " , (unsigned int)*(ptr+i+j) );

		for( j = 0; j < 16; j++ )
		{
			if ( isalnum( (int)*(ptr+i+j) ) )
				printf("%c" , *(ptr+i+j) );
			else
				printf(".");
		}
	}

	printf("\n\nHit any key to continue...");
	while( !kbhit() );
}

/************************
*	DISPLAY_POINTERS
************************/

static void display_pointers( void )
{
int	i;

	clrscr();

	for( i=0; i < NUM_SATELLITES; i++ )
	{
		gotoxy( 1 , 1 + i );
		cprintf("sat_details_ptr[ %i ] = %0lX" , i , (unsigned long)sat_details_ptr[i] );
	}

	gotoxy( 1 , 1 + NUM_SATELLITES );
	cprintf("current_sat_num_ptr = %0lX" , (unsigned long)current_sat_num_ptr );

	for( i=0; i < NUM_STARS; i++ )
	{
		gotoxy( 1 , 1 + NUM_SATELLITES + i );
		cprintf("star_details_ptr[ %i ] = %0lX" , i , (unsigned long)star_details_ptr[i] );
	}

	gotoxy( 1 , 1 + NUM_SATELLITES + NUM_STARS );
	cprintf("current_star_num_ptr = %0lX" , (unsigned long)current_star_num_ptr );

	gotoxy( 1 , 1 + NUM_SATELLITES + NUM_STARS + 1);
	cprintf("beacon_details_ptr = %0lX" , (unsigned long)beacon_details_ptr );

	gotoxy( 1 , 1 + NUM_SATELLITES + NUM_STARS + 2);
	cprintf("orbit_track_start_ptr = %0lX" , (unsigned long)orbit_start_ptr );

	gotoxy( 1 , 1 + NUM_SATELLITES + NUM_STARS + 3 );
	cprintf("scan_data_ptr = %0lX" , (unsigned long)scan_data_ptr );

	gotoxy( 1 , 1 + NUM_SATELLITES + NUM_STARS + 4 );
	cprintf("calibration_coeff_ptr = %0lX" , (unsigned long)nv_calibration_coefficients_ptr );

	gotoxy( 1 , 1 + NUM_SATELLITES + NUM_STARS + 5);
	cprintf("station_position_ptr = %0lX" , (unsigned long)station_position_ptr );

	gotoxy( 1 , 1 + NUM_SATELLITES + NUM_STARS + 6 );
	cprintf("software_limits_ptr = %0lX" , (unsigned long)sw_limits_ptr );

	gotoxy( 1 , 1 + NUM_SATELLITES + NUM_STARS + 7 );
	cprintf("stow_position_ptr = %0lX" , (unsigned long)stow_pos_ptr );

	for( i=0; i < NUM_SATELLITES; i++ )
	{
		gotoxy( 40 , 1 + i );
		cprintf("orbit_model_ptr[ %i ] = %0lX" , i , (unsigned long)orbit_model_ptr[i] );
	}

	for( i=0; i < NUM_SATELLITES; i++ )
	{
		gotoxy( 40 , 1 + NUM_SATELLITES + i );
		cprintf("orbit_covariance_ptr[ %i ] = %0lX" , i , (unsigned long)orbit_covariance_ptr[i] );
	}

	gotoxy( 40, 1 + 2 * NUM_SATELLITES + 1 );
	cprintf("Az axis cont. param. ptr = %0lX", (unsigned long)con_ptr[AZ_AXIS] );

	gotoxy( 40, 1 + 2 * NUM_SATELLITES + 2 );
	cprintf("El axis cont. param. ptr = %0lX", (unsigned long)con_ptr[EL_AXIS] );



	gotoxy( 1 , 25 );
	cprintf("Hit any key to continue...");
	while ( !kbhit() );
	getch ();

	clrscr ();
	for( i=0; i < NUM_SATELLITES; i++ )
	{
		gotoxy( 1 , 1 + i );
		cprintf("intelsat_param_ptr[ %i ] = %0lX" , i , (unsigned long)intelsat_param_ptr[i] );
	}

	for( i=0; i < NUM_SATELLITES; i++ )
	{
		gotoxy( 1 , NUM_SATELLITES + 1 + i );
		cprintf("intelsat_epoch_ptr[ %i ] = %0lX" , i , (unsigned long)intelsat_epoch_ptr[i] );
	}


	gotoxy( 1 , 25 );
	cprintf("Hit any key to continue...");

	while( !kbhit() );
	getch ();

}

/************************
*	MAIN
************************/

void main( int argc, char *argv[] )
{
char	main_command,
		put_command,
		get_command;

unsigned char	*ptr;

unsigned long	nvram_addr;
int	i;

	/* Do a little advice thing. */
	if( argc > 1 && *argv[1] == '?' )
	{
		printf("\n one command-line parameter is available.\n"
	"It allows this program to run when no NVRAM card is present.\n"
	"	r - uses NVRAM card (default);\n"
	"	s - does not need NVRAM card.\n\n"
	" If no parameter is given, the default condition is used.\n");

		exit(0); 
        }

	/* Set the NVRAM simulation on or off, according to the
    	command-line input:-
	`r' = real NVRAM exists, (default) and
	`s' = no NVRAM exists.
    */
		if(argc > 1 && *argv[1] == 's')
			nvram_sim = 1;
		else
			nvram_sim = 0;

	do
	{
		/* Ask the user what function is required. */

		main_command = main_menu();

		switch ( main_command )
		{
			case '1' :
						/* Initialise the NVRAM hardware/software */

						gotoxy( 1 , 22 );
						clreol();
						cprintf("NVRAM board initialisation ");

						if ( !nvram_init( NVRAM_INIT_MODE ) )
							cprintf("unsuccessful.\a");
						else
							cprintf("successful.");

						cprintf("    Hit any key to continue...");
						while( !kbhit() );

						break;

			case '2' :
						gotoxy( 1 , 22 );
						clreol();
						cprintf("NVRAM Controller Diagnostic test ");

						if ( !nvram_init( NVRAM_TEST_MODE ) )
							cprintf("failed.\a");
						else
							cprintf("passed.");

						cprintf("    Hit any key to continue...");
						while( !kbhit() );

						break;
			case '3' :
						gotoxy( 1 , 22 );
						clreol();
						cprintf("Load of Default values ");

						if ( !nvram_init( LOAD_DEFAULT_VALUES ) )
							cprintf("failed.\a");
						else
							cprintf("succeeded.");

						cprintf("    Hit any key to continue...");
						while( !kbhit() );

						break;
			case '4' :
						put_command = put_menu();

						enter_put_data( put_command );
						break;
			case '5' :
						get_command = get_menu();

						display_get_data( get_command );
						break;
			case '6' :
						clrscr();
						printf("The 64K base address is %lX." , NVRAM_BASE_ADDR );
						printf("\nEnter desired address (D123:4567 entered as D1234567) : ");
						scanf("%X" , &nvram_addr );
						printf("\nEnter desired byte value in hex : ");
						scanf("%x" , &i );

						ptr = (unsigned char*)nvram_addr;
						*ptr = (unsigned char)i;
						break;

			case '7' :
						display_NVRAM();
                        break;
			case '8' :
						display_pointers();
                        break;
			default :
						break;
		}
	} while ( main_command != '9' );

	clrscr();
    printf("Program terminated.\n");
}