/************************************************************************/
/*																		*/
/*																		*/
/*																		*/
/*						   COMMAND DECODER HEADER FILE					*/
/*																		*/
/*										by								*/
/*																		*/
/*								    Robert Betz							*/
/*			   Department of Electrical and Computer Engineering		*/
/*							   University of Newcastle					*/
/*																		*/
/*								(Copyright 1992)						*/
/*																		*/
/*																		*/
/*																		*/
/************************************************************************/



/* These definitions below define the way that the decode tables will work 
*/

#define ASCII_DECODING 0
#define BINARY_DECODING 1





/************************************************************************/
/*																		*/
/*							COMMAND DECODER STRUCTURES					*/
/*																		*/
/************************************************************************/




/*------ CHARACTER DECODER INPUT STRUCTURE ------*/
/* This structure is used by the character decode routine in the COMDEC 
module. Refer to the UNOS documentation for more details.
*/

typedef
	struct decode_struc {
		char char_dec_mode;
		unsigned char data_byte;
		unsigned char *valid_data_table_ptr;
		void ( **vector_table_ptr ) ( struct decode_struc * );
		unsigned char *bit_table_ptr;
		void *info_struc_ptr;
	} dec_input_struc;
	


/*------ MESSAGE ADDRESS DECODE STRUCTURE ------*/
/* This structure is used by the command decoder in the COMDEC routine. The
address decoder forms a mapping between the number of a sending task and 
structure used as input to the character decoder. Refer to the UNOS 
documentation for more details.
*/

typedef
	struct {
		char* rtn_addr;
		char **valid_addr_tab_ptr;	// was char huge **
		dec_input_struc **addr_vector_ptr;
		unsigned char *mess_ptr;
		unsigned int mess_lgth;
	} addr_struc;

extern dec_input_struc *addr_decode ( addr_struc *addr_struc_ptr );

extern char * decode_mbx_mess ( addr_struc *addr_struc_ptr, unsigned long
								time_limit );

/*<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< END OF FILE >>>>>>>>>>>>>>>>>>>>>>>>>*/
