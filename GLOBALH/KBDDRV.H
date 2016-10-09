/****************************************************************************
* POLYTRON VERSION CONTROL SOFTWARE _ PVCS (tm) Information
*
*      $Revision:   2.3  $
*      $Date:   07 Jan 1994 16:16:42  $
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
* MODULE :- GLOBALH                  File : KBDRV.H
****************************************************************************/
#ifndef __KBDDRV_H_
#define __KBDDRV_H_

/********************************************************************/
/*																	*/
/*																	*/
/*						  HEADER FILE FOR KBDDRV.C					*/
/*																	*/
/*									by								*/
/*								Robert Betz							*/
/*			  Department of Electrical and Computer Engineering		*/
/*						  University of Newcastle					*/
/*							 (Copyright 1992)						*/
/*																	*/
/*																	*/
/********************************************************************/

extern void fvdScanCodeTranslatorTask ( void* );

extern void fvdConnectToKeyboard (char *pchTaskName);

extern void fvdInitScanCodeTranslatorTask (void);

#endif