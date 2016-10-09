
/********************************************************************/
/*																	*/
/*						  HEADER FILE FOR KBDDRV.C					*/
/*																	*/
/*									by								*/
/*								Robert Betz							*/
/*			  Department of Electrical and Computer Engineering		*/
/*						  University of Newcastle					*/
/*							 (Copyright 1992)						*/
/*																	*/
/********************************************************************/

#ifndef __KEYBOARD_H_
#define __KEYBOARD_H_

/* Public keyboard functions 								*/

extern void connect_to_keyboard (char *task_name_ptr);

extern void fvdScanCodeTranslatorTask ( void* );

extern void fvdInitScanCodeTranslatorTask (void);

/* Public keyboard variable 								*/
extern char keyboard_task_name[];

/* Otherwise undefined keyboard codes produced by KBDDRV.C 	*/
#define	NUL	0x00
#define SOH	0x01
#define STX	0x02
#define ETX 0x03
#define EOT	0x04
#define ENQ	0x05
#define ACK	0x06
#define BEL 0x07
#define BS  0x08
#define HT  0x09
#define LF  0x0A
#define VT  0x0B
#define FF  0x0C
#define CR  0x0D
#define SO  0x0E
#define SI  0x0F
#define DLE 0x10
#define DC1 0x11
#define DC2 0x12
#define DC3 0x13
#define DC4 0x14
#define NAK 0x15
#define SYN 0x16
#define ETB 0x17
#define CAN 0x18
#define EM  0x19
#define SUB 0x1A
#define ESC 0x1B
#define FS  0x1C
#define GS  0x1D
#define RS  0x1E
#define US  0x1F
#define SPACE 0x20

#define CTRL_A	0x01
#define CTRL_B	0x02
#define CTRL_C	0x03
#define CTRL_D  0x04
#define CTRL_E	0x05
#define CTRL_F	0x06
#define CTRL_G	0x07
#define CTRL_H	0x08
#define CTRL_I	0x09
#define CTRL_J	0x0A
#define CTRL_K	0x0B
#define CTRL_L	0x0C
#define CTRL_M	0x0D
#define CTRL_N	0x0E
#define CTRL_O	0x0F
#define CTRL_P	0x10
#define CTRL_Q	0x11
#define CTRL_R	0x12
#define CTRL_S	0x13
#define CTRL_T	0x14
#define CTRL_U	0x15
#define CTRL_V	0x16
#define CTRL_W	0x17
#define CTRL_X	0x18
#define CTRL_Y	0x19
#define CTRL_Z	0x1A

#define ALT_A	0x1E
#define ALT_B	0x30
#define ALT_C	0x2E
#define ALT_D   0x20
#define ALT_E	0x12
#define ALT_F	0x06
#define ALT_G	0x22
#define ALT_H	0x23
#define ALT_I	0x17
#define ALT_J	0x24
#define ALT_K	0x25
#define ALT_L	0x26
#define ALT_M	0x32
#define ALT_N	0x31
#define ALT_O	0x18
#define ALT_P	0x19
#define ALT_Q	0x10
#define ALT_R  	0x13
#define ALT_S	0x1F
#define ALT_T	0x14
#define ALT_U	0x16
#define ALT_V	0x2F
#define ALT_W	0x11
#define ALT_X	0x2D
#define ALT_Y	0x15
#define ALT_Z	0x2C

#define HOME		   	0x47
#define CURSOR_UP	   	0x48
#define PAGE_UP		   	0x49
#define CURSOR_LEFT	   	0x4B
#define CURSOR_RIGHT   	0x4D
#define END				0x4F
#define CURSOR_DOWN		0x50
#define PAGE_DOWN  		0x51
#define INS 			0x52
#define DEL 			0x53

#endif