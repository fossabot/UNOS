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

#define NUMBER_OF_OUTPUT_PINS 		7
#define NUMBER_OF_INPUT_PINS 		11
#define INHIBIT_MANUAL_SELECT	    1   /* Open the ATCU contacts to permit
											manual control					*/
#define NOT_INHIBIT_MANUAL_SELECT   0

/*
 * Define a data type for the mode of operation of the system. That is, the
 * TUNRA software may be doing I/O on real or simulated hardware.
 */

typedef enum { real_mode, simulation_mode } operational_mode;

/*
 * Define a data type which describes the axis being referenced.
 */

typedef enum { elevation_axis, azimuth_axis } antenna_axis;

/*
 * Define a data type which enumerates the antenna drive control pins which the
 * TUNRA software needs to manipulate.
 */

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
			azimuth_final_limit,
			elevation_final_limit,
			fuse_f2_blown,
			azimuth_prelimit_cw,
			azimuth_prelimit_ccw,
			elevation_prelimit_up,
			elevation_prelimit_down,
			ATCU_off,
			resolver_power_supply } antenna_input_pin;


/*
 * Define the possibe states of these pins. This data type abstracts from
 * caller the need for knowledge of whether the pin is normally open or
 * closed. Instead, each pin is regarded as being inactive by default. If
 * the pin changes to its non-default state, this is regarded as an active
 * state. For an output, this means that the output pin is normally de-
 * activated. To turn the control "on", the caller simply commands that the
 * pin be set to an active state. For an input, the inactive ( default )
 * state means that the condition is not true. The condition will be true
 * when the input pin changes status to active.
 */

typedef enum { INACTIVE, ACTIVE } io_pin_state;

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
 * Subordinates :-
 *    i) set_bit_for - sets a control pin high.
 *   ii) reset_bit_for - resets a control pin low.
 *
 * Preconditions :-
 *   i) The control hardware is connected and operational.
 *
 * Postconditions :-
 *   i) The control pin has been set to the required state.
 *
 * Exceptions :-
 *
 */

void set_drive_control ( io_pin_state new_state, antenna_output_pin pin );

void set_simulated_drive_input
    ( io_pin_state new_state_for,
      antenna_input_pin pin );

void output_drive_keyswitch ( int selection );

/*
 * Define a function which reads the parallel port to determine if
 * the software is to be run in real-mode or simulation-mode.
 */

void assign_operational_mode ( void );


/*
 * Declare a function which reads the encoder and converts it to angle
 * ( in units of degrees ).
 */

int get_angle_of
    ( antenna_axis axis,
      double *angle );


/*
 * Declare a function which writes an encoded angle value into the
 * simulated ports.
 */

void put_angle_of ( antenna_axis axis, unsigned long int value );

io_pin_state the_state_of_simulated_output_pin ( antenna_output_pin pin );


/*
 * Declare a function which writes the demanded angular velocity to the
 * D/A conversion hardware.
 */

void put_volts_for
    ( antenna_axis axis,
      double volts );

/*
 * Declare a function which reads the encoded azimuth / elevation rate from
 * the simulated ports.
 */

void get_encoder_value
    ( antenna_axis axis,
      int *positive_digitized_voltage,
      int *negative_digitized_voltage );

/*
 * Decalre a function whcih returns a generic pointer to an area of memory
 * for the simulated hardware ports.
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