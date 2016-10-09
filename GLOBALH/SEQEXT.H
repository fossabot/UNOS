
/*---- Azimuth and Elevation Structure Return Functions. Returns the
	axis structure information. These structures contain info for each axis such
	as calibrated and precalibrated positions, commanded positions, pos error
*/

extern axis return_azimuth ( void );
extern axis return_elevation ( void );

/*---- ATCU structure return function. The atcu structure contains
	information about the sequencer. e.g. state, mode, internal states of the
	sequencer, etc.
*/
extern atcu_struct return_atcu ( void );

/*---- Return functions for the axis positions. These have been supplied so
	others may read antenna commanded and measured (corrected) positions
	and not the whole axis structure.
*/
extern double return_az_cmd ( void );
extern double return_el_cmd ( void );
extern double return_az_msd ( void );
extern double return_el_msd ( void );

/*---- Using Control Point data structures. The Sequencer keeps the real 
    image of the the CP data structure. The user must first read the entire 
    CP data structure and modify that which needs to be modified. e.g. 
    change state and supply the commanded positions. Then the user sets the 
    CP data structure with their changes. This was done in this way to avoid 
    making changes to the CP structure and the Sequencer not getting a 
    complete image of the CP structure while it was running. The Sequencer 
    only reads the CP data structure once at the start of each loop. In this 
    way incomplete changes cannot be made to its image.
*/
extern cp_data_struct return_cp_data ( void );
extern void set_cp_data ( cp_data_struct * cp_data_parser );

/*---- This routine sets default s/w limits defined in the seq.h header file */
extern void set_default_software_limits ( software_limit_struct * sw_l );
extern void set_default_stow_positions ( stow_pos_struct * stow_pos );

extern void init_default_sequencer ( void );

/* Sequencer missed its control tick count. */
extern unsigned int return_sequencer_missed_count ( void );

/* System ID structure return routine */
extern void return_systemid ( systemid_struct * systemid );
extern void set_systemid ( systemid_struct systemid );

extern double return_az_rate_volts ( void );
extern double return_el_rate_volts ( void );

extern double return_az_speed ( void );
extern double return_el_speed ( void );

/* To see whether the datafile logging flag has been set */
unsigned int return_collect_data ( void );

