/************************************************************************/
/*                                                                      */
/*                                                                      */
/*                                                                      */
/*                LOW LEVEL SERIAL INTERRUPT ROUTINE HEADER FILE        */
/*                                                                      */
/*                                      by                              */
/*                                                                      */
/*                                  Robert Betz                         */
/*              Department of Electrical and Computer Engineering       */
/*                            University of Newcastle                   */
/*                               (Copyright 1992)                       */
/*                                                                      */
/*                                                                      */
/*                                                                      */
/************************************************************************/




extern int setup_interrupt_routine (char uart_type, char num_8259s,
			unsigned int addr_8259_mast, unsigned int addr_8259_slave,
			unsigned char rx_intr_num, unsigned char tx_intr_num,
			rx_int_buffer* rx_int_bufptr, tx_int_buffer* tx_int_bufptr,
			rx_int_buffer* srx_int_bufptr, tx_int_buffer* stx_int_bufptr);
