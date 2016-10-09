/************************************************************************/
/*																		*/
/*                                                                      */
/*																		*/
/*					  SERIAL DEVICE DRIVER HEADER FILE					*/
/*																		*/
/*								  	 by									*/
/*																		*/
/*			   				     Robert Betz							*/
/*			  Department of Electrical and Computer Engineering			*/
/*							University of Newcastle						*/
/*					  (Copyright 1988, 1989, 1990, 1992)				*/
/*																		*/
/*																		*/
/*																		*/
/************************************************************************/



/*------ Uart Definitions ------*/

/*
these definitions are used in the serial channel interface routines to
differentiate between the supported uarts.
*/

#define I8274 0
#define I8251A 1
#define NS16450 2

/* Define the parity types in the system */

#define ODD_PARITY 0
#define EVEN_PARITY 1
#define NO_PARITY 2

/* Values used for programming the UARTS */
#define BAUD_9600 9600
#define BAUD_4800 4800
#define ONE_STOP_BIT 1
#define EIGHT_DATA_BITS 8
#define SEVEN_DATA_BITS 7
#define CTRL_P 0x10
#define CTRL_Q 0x11
#define CTRL_S 0x13

#define _BEL '\x07'
#define _BS 	'\x08'
#define _HT 	'\x09'
#define _LF 	'\x0A'
#define _CR 	'\x0D'
#define _Esc '\x1B'



#define	OVERRUN_ERROR_BYTE			0x80
#define	PARITY_ERROR_BYTE			0x81
#define	FRAMING_ERROR_BYTE			0x82
#define	BREAK_INTERRUPT_ERROR_BYTE	0x83

/* The following structure is used to implement a linked list of structures
of the type below that store all the task names that a particular serial
task is linked to.
*/

typedef
	struct rx_connect_struc {
		char* connected_taskname_ptr;
		struct rx_connect_struc* next_taskname_ptr;
	} connected_tasks;




/*------ Transmit Interrupt Buffer Type Definition ------*/

typedef
	struct {
		unsigned int buf_size;
		unsigned char rx_ctrlS_ctrlQ;
		unsigned char cur_tx_int_status;
		unsigned char user_ctrlS_ctrlQ;
		unsigned char tx_interrupt;
		unsigned int data_reg_add;
		unsigned int com_reg_add;
		unsigned char uart_com_data;
		unsigned char tx_enable_disable_bit;
		unsigned char intr_ctrl_reg_offset;
		unsigned char uart_type;
		char uart_number;
		unsigned int buf_room_index;
		unsigned int buf_used;
		unsigned int get_ptr;
		unsigned int put_ptr;
		char* buf_ptr;
		unsigned char xonxoff;
	} tx_int_buffer;




/*------ Receive Interrupt Buffer Type Definition ------*/

typedef
	struct {
		unsigned int buf_size;
		unsigned int buf_low_lim;
		unsigned int buf_up_lim;
		unsigned int err_buf_index;
		unsigned int data_reg_add;
		unsigned int com_reg_add;
		unsigned int uart_com_data;
		unsigned char tx_enable_disable_bit;
		unsigned char intr_ctrl_reg_offset;
		unsigned char uart_type;
		char uart_number;
		unsigned char ctS_ctQ_flg;
		tx_int_buffer* tx_buf_ptr;
		connected_tasks* connected_task_ptr;
		unsigned int buf_sem_index;
		unsigned int buf_free;
		unsigned int get_ptr;
		unsigned int put_ptr;
		char* buf_ptr;
		unsigned char xonxoff;
	} rx_int_buffer;



/************************************************************************/
/*																		*/
/*							  FUNCTION PROTOTYPES						*/
/*																		*/
/************************************************************************/

extern int create_serial_channel (char uart_type, unsigned int baud_rate,
			float stop_bits, char char_lgth, char parity,
			unsigned int command_reg,
			unsigned int data_reg, unsigned char division_ratio,
			unsigned char rx_intr_num,
			unsigned char tx_intr_num, unsigned char tx_enable_disable_bit,
			unsigned char intr_ctrl_reg_offset,
			unsigned int rx_int_buf_size,
			unsigned int tx_int_buf_size, unsigned int buf_low_lim,
			unsigned int buf_upper_lim, unsigned int mbx_q_size,
			unsigned int mbx_mess_size, char num_8259s,
			unsigned int addr_8259_mast, unsigned int addr_8259_slave,
			char* ch1_rx_name_ptr, char* ch1_tx_name_ptr,
			char* ch2_rx_name_ptr, char* ch2_tx_name,
			unsigned char rx_task_pri, unsigned char tx_task_pri,
			unsigned char xonxoff );

extern void interrupt_handler_16450 (rx_int_buffer* rx_bufptr,
				tx_int_buffer* tx_bufptr, unsigned char intr_source);

extern void service_tx_intr_16450 (tx_int_buffer* tx_buf_ptr);

extern void service_rxerr_intr_16450 (rx_int_buffer* rx_bufptr);

extern void transmit_interrupt_handler_8251A (tx_int_buffer* tx_buf_ptr);

extern void receive_interrupt_handler_8251A (rx_int_buffer* rx_buf_ptr);

extern void interrupt_handler_8274 (rx_int_buffer* chA_rx_ptr,
							 tx_int_buffer* chA_tx_ptr,
							 rx_int_buffer* chB_rx_ptr,
							 tx_int_buffer* chB_tx_ptr );