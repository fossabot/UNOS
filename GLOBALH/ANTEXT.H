/*
****************************************************************************

				Antenna Simulator External Prototypes

****************************************************************************
*/

extern void antenna_simulator ( void );
extern void init_antenna_simulator ( void );

extern model_struct return_az_model_axis ( void );
extern model_struct return_el_model_axis ( void );
extern model_data_struct return_model_data ( void );
extern from_atcu_struct return_from_atcu ( void );

extern unsigned int return_az_drive_state ( void );
extern unsigned int return_el_drive_state ( void );
extern unsigned int return_io_cnt ( void );
extern unsigned int return_model_cnt ( void );

extern void set_model_in ( drive_out_struct * model_in );
extern void set_model_data ( model_data_struct * model );
extern void set_az_drive_fail ( void );
extern void set_model_drive_power ( int power, int axis_type );

