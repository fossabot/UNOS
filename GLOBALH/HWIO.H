/***************************************************************************
								TIED SOFTWARE
 PROJECT	:- TS3000

 MODULE 	:- GLOBALH

 FILE 		:- HWIO_EXT.H

 PROGRAMMER :- R Mortenson

 Copyright 1994. The University of Newcastle Research Associates Ltd.

****************************************************************************
* POLYTRON VERSION CONTROL SOFTWARE _ PVCS (tm) Information
*
*      $Revision:   1.1.1.0  $
*      $Date:   29 Apr 1994 19:41:10  $
*
****************************************************************************/
#ifdef INACTIVE
#undef INACTIVE
#endif
#ifdef ACTIVE
#undef ACTIVE
#endif


typedef enum { elevation_axis, azimuth_axis } antenna_axis;

typedef enum { INACTIVE, ACTIVE } io_pin_state;

typedef enum { inactive_when_open, inactive_when_closed } contact_state;

typedef enum { 	energize_azimuth_drive,
				stop_drives,
				start_drives,
				energize_elevation_drive,
				reset_drives,
				notify_ATCU_OK
				} antenna_output_pin;

/*
 * Define a data type which enumerates the antenna drive status pins which
 * the TUNRA software needs to monitor.
 */

typedef enum {
			fuse_f1_blown,
			azimuth_drive_ready,
			elevation_drive_ready,
			azimuth_drive_on,
			elevation_drive_on,
			drive_system_failure,
			azimuth_final_limits,
			elevation_final_limits,
			fuse_f2_blown,
			azimuth_cw_prelimit,
			azimuth_ccw_prelimit,
			elevation_up_prelimit,
			elevation_down_prelimit,
			ATCU_off,
			spare_D2,
			resolver_power_supply } antenna_input_pin;

typedef struct {
		 int Port;                /* address of port in memory map*/
		 int Bit;                 /* bit position within port */
		 contact_state Contact;         /* is pin normally open | closed ? */
		 unsigned char Mask; /* mask to set | test bit,
											complement will reset bit */
		} hardware_information;


unsigned char ReadAzPosition ( double * pos );
unsigned char ReadElPosition ( double * pos );


void OutputAzVolts ( double voltage );

void OutputElVolts ( double voltage );

/*
 * Define a data type which describes a table of information about
 * hardware pins.
 */

typedef hardware_information hardware_assignments[];

/*
 * Name :- set_drive_control
 *
 * Abstract :- This function sets the state of a control to ACTIVE or
 * INACTIVE, as specified by the caller.
 *
 * Functional Description :-
 *
 *
 * Interface :-
 *     input parameter - "new_state_for", the required state ( ACTIVE or
 *                        INACTIVE.
 *     input parameter - "pin", the control pin to be changed.
 *
 */

void set_drive_control ( io_pin_state new_state, antenna_output_pin pin );

void output_drive_keyswitch ( int selection );


/*
 * Declare a function which reads the encoder and converts it to angle
 * ( in units of degrees ).
 */


void huge *shared_port_addresses ( void );

void set_drive_control ( io_pin_state new_state, antenna_output_pin pin );

int computer_control_is_inhibited ( void );

int the_azimuth_drive_is_ready ( void );

int the_azimuth_drive_is_on ( void );

int the_elevation_drive_is_ready ( void );

int the_elevation_drive_is_on ( void );

int the_drive_system_has_failed ( void );

int ATCU_go_to_OFF ( void );

int acs_fuse_f1_blown ( void );

int acs_fuse_f2_blown ( void );

int hardware_mode_is_real ( void );

int the_azimuth_is_in_final_limits ( void );

int the_elevation_is_in_final_limits ( void );

int ATCU_go_to_OFF ( void );

int the_azimuth_is_in_cw_prelimit ( void );

int the_azimuth_is_in_ccw_prelimit ( void );

int the_elevation_is_in_up_prelimit ( void );

int the_elevation_is_in_down_prelimit ( void );

int Resolver_power_supply_is_bad ( void );

void IOPage ( void );
