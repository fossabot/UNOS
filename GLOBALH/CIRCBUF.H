/***************************************************************************
								TIED SOFTWARE
 PROJECT	:- TS3000

 MODULE 	:- GLOBALH

 FILE 		:- CIRCBUF.H

 PROGRAMMER :- L Sciacca

 Copyright 1994. The University of Newcastle Research Associates Ltd.

****************************************************************************
* POLYTRON VERSION CONTROL SOFTWARE _ PVCS (tm) Information
*
*      $Revision:   1.1.1.0  $
*      $Date:   29 Apr 1994 19:41:10  $
*
****************************************************************************/

/*
	The circular buffer routines are contained in the file, circbuf.c.

*/

/*---- Circular Buffer Structure */
typedef
	struct {
		int put;
		int get;
		int buf_free;
		void * buf;
		int buf_size;
		int bytes_type;
	} circ_buf_struct;

extern circ_buf_struct * init_central_circ_table ( int max_number_circ_buffers );
extern void * create_circ_buffer ( int circ_buf_num, int size,
												char type, int type_size );

extern int put_circ_buffer ( int circ_buf_num, void * data );
extern int get_circ_buffer ( int circ_buf_num, void * data );
extern void reset_circ_buffer ( int circ_buf_num );

/* The user may add their own types here. It should be possible to have 
circular buffers of structures of any size. e.g. USER_TYPE (sizeof(my_struct))
*/

#define	CHAR_TYPE	0
#define INT_TYPE	1
#define LONG_TYPE	2
#define FLOAT_TYPE	3
#define DOUBLE_TYPE	4
#define USER_TYPE	5


