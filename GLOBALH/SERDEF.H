/***************************************************************************

 PROJECT	:- TS3000

 MODULE 	:- GLOBALH

 FILE 		:- SERDEF.H

 PROGRAMMER :- L Sciacca

 Copyright 1994. The University of Newcastle Research Associates Ltd.

****************************************************************************
* POLYTRON VERSION CONTROL SOFTWARE _ PVCS (tm) Information
*
*      $Revision:   1.1.1.0  $
*      $Date:   29 Apr 1994 19:41:14  $
*****************************************************************************/

/*************************************************************************

					Serial Driver Definition File

							 for PC_RTOS

       This file must always be present. It is used by PC_RTOS.


*************************************************************************
*/
/*
============================================================================

					 USER-CONFIGURABLE CONSTANTS

			Set these to match your hardware and requirements

============================================================================
*/
#ifndef __SERDEF_H
#define __SERDEF_H

#include "serial.h"

/* This defines the number of serial communications ports */
#define		NUM_OF_COMMS	4

/* Hardware-dependent constants and channel setup definitions */
#define UART0_COM_REG_ADDR 	0x3f8
#define UART0_DATA_REG_ADDR 0x3f8

#define UART1_COM_REG_ADDR 	0x2f8
#define UART1_DATA_REG_ADDR 0x2f8

#define UART2_COM_REG_ADDR 	0x3e8
#define UART2_DATA_REG_ADDR 0x3e8

#define UART3_COM_REG_ADDR 	0x2e8
#define UART3_DATA_REG_ADDR 0x2e8

#define CHANNEL_0_IRQ_NUM 	4
#define CHANNEL_1_IRQ_NUM 	3
#define CHANNEL_2_IRQ_NUM   5
#define CHANNEL_3_IRQ_NUM 	7

#define CHANNEL_0_BAUD		9600
#define CHANNEL_1_BAUD		9600
#define CHANNEL_2_BAUD		9600
#define CHANNEL_3_BAUD		9600

#define CHANNEL_0_STOP_BITS	1
#define CHANNEL_1_STOP_BITS	1
#define CHANNEL_2_STOP_BITS	1
#define CHANNEL_3_STOP_BITS	1

#define CHANNEL_0_DATA_BITS	8
#define CHANNEL_1_DATA_BITS	8
#define CHANNEL_2_DATA_BITS	8
#define CHANNEL_3_DATA_BITS	8

/* Define parity seettings:- NO_PARITY, EVEN_PARITY or ODD_PARITY */
#define CHANNEL_0_PARITY	NO_PARITY
#define CHANNEL_1_PARITY   	NO_PARITY
#define CHANNEL_2_PARITY	NO_PARITY
#define CHANNEL_3_PARITY	NO_PARITY

/* Define Software handshake:- YES or NO */
#define CHANNEL_0_XONXOFF   NO
#define CHANNEL_1_XONXOFF   NO
#define CHANNEL_2_XONXOFF   NO
#define CHANNEL_3_XONXOFF   NO

/* Define the internal division ratio requiredy for uarts such as the 8251 -
	Not presently used. */
#define CHANNEL_0_DIVISION_RATIO 	0
#define CHANNEL_1_DIVISION_RATIO 	0
#define CHANNEL_2_DIVISION_RATIO 	0
#define CHANNEL_3_DIVISION_RATIO 	0

/* Serial comms. channels buffer sizes and task mailbox characteristics. */

						/* ***** WARNING **** */

/* BE VERY CAREFUL WHEN SETTING MAILBOX SIZES. REMEMBER THAT IF MESSAGES
ARE BEING PASSED THRU SEVERAL TASKS e.g. parser->protocol->tx->interr

THAT THE MBXS AND INTERRUPT FIFOS WILL BE EXPECTED TO HANDLE THE LARGEST
EXPECTED BUFFER TRANSFER.
*/
/* LCMU */
#define CHANNEL_0_RX_BUF_SIZE 350
#define CHANNEL_0_TX_BUF_SIZE 350

#define CHANNEL_0_BUF_LOW_LIM 	 10
#define CHANNEL_0_BUF_UPPER_LIM (CHANNEL_0_RX_BUF_SIZE - 20)

#define CHANNEL_0_MBX_MESS_SIZE 350
#define CHANNEL_0_MBX_Q_SIZE 	 10


/* RCMU or Dataport */
#define CHANNEL_1_RX_BUF_SIZE 350
#define CHANNEL_1_TX_BUF_SIZE 350

#define CHANNEL_1_BUF_LOW_LIM 	 10
#define CHANNEL_1_BUF_UPPER_LIM (CHANNEL_1_RX_BUF_SIZE - 20)

#define CHANNEL_1_MBX_MESS_SIZE 350
#define CHANNEL_1_MBX_Q_SIZE 	10


/* Control Port */
#define CHANNEL_2_RX_BUF_SIZE 120
#define CHANNEL_2_TX_BUF_SIZE 120

#define CHANNEL_2_BUF_LOW_LIM 	10
#define CHANNEL_2_BUF_UPPER_LIM (CHANNEL_2_RX_BUF_SIZE - 20)

#define CHANNEL_2_MBX_MESS_SIZE 120
#define CHANNEL_2_MBX_Q_SIZE	10


/* Beacon Receiver */
#define CHANNEL_3_RX_BUF_SIZE 5
#define CHANNEL_3_TX_BUF_SIZE 11

#define CHANNEL_3_BUF_LOW_LIM 	1
#define CHANNEL_3_BUF_UPPER_LIM (CHANNEL_3_RX_BUF_SIZE - 2)

#define CHANNEL_3_MBX_MESS_SIZE 9
#define CHANNEL_3_MBX_Q_SIZE	2

/* Define whether receiver errors are to be fed through as special bytes -
	NOTE: This definition applies to ALL channels. */
#define FEEDTHROUGH_ERRORS	YES		/* YES or NO */

#endif