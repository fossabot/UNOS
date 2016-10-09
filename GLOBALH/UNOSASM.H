/****************************************************************************
* POLYTRON VERSION CONTROL SOFTWARE _ PVCS (tm) Information
*
*      $Revision:   2.3  $
*      $Date:   07 Jan 1994 16:16:46  $
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
* MODULE :- GLOBALH                  File :  UNOSASM.H
****************************************************************************/
/********************************************************************/
/*																	*/
/*																	*/
/*						  HEADER FILE FOR UNOSASM					*/
/*																	*/
/*									by								*/
/*								Robert Betz							*/
/*			  Department of Electrical and Computer Engineering		*/
/*						  University of Newcastle					*/
/*							 (Copyright 1992)						*/
/*																	*/
/*																	*/
/********************************************************************/


/*
DESCRIPTION

This file contains the function prototypes that are required by a module
using the UNOS assembly language routines.

*/
#ifndef _UNOSASM_H
#define _UNOSASM_H
extern char return_interrupt_status ( void );

#endif