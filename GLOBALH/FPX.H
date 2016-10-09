/****************************************************************************
* POLYTRON VERSION CONTROL SOFTWARE _ PVCS (tm) Information
*
*      $Revision:   2.3  $
*      $Date:   07 Jan 1994 16:17:12  $
* 
*****************************************************************************
*****************************************************************************
* Tracksat Project.
*****************************************************************************
* Copyright (c) 1991. The University of Newcastle Research Associates Ltd.
*****************************************************************************
*
* PROGRAMMER :-     
*                   Industrial Electronics Division,
*                   TUNRA Ltd.,
*                   University of Newcastle, NSW 2308.
*
*****************************************************************************
*****************************************************************************
* MODULE :- GLOBALH                  File : FPX.H
****************************************************************************/
/********************************************************************/
/*																	*/
/*																	*/
/*				UNOS FP COPROCESSOR ROUTINES HEADER MODULE			*/
/*																	*/
/*								  by								*/
/*																	*/
/*							  Robert Betz							*/
/*	    Department of Electrical and Computer Engineering           */
/*						University of Newcastle						*/
/*																	*/
/*						  ( Copyright 1992 )						*/
/*                                                                  */
/*																	*/
/*																	*/
/********************************************************************/



/*

HISTORY

LAST MODIFIED : 16/2/92
BY :            Robert Betz


16/2/92     Initial header file written

*/





/*
DESCRIPTION

This file contains the function prototypes for the save and restore
routines for floating point registers in an 8087 series floating
point unit.

*/
#ifndef __FPX_H
#define __FPX_H


/*------ FUNCTION PROTOTYPES ------*/

void save_fp_regs ( unsigned char *save_fp_area );

void restore_fp_regs ( unsigned char *save_fp_area );

#endif