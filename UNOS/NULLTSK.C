/****************************************************************************
* POLYTRON VERSION CONTROL SOFTWARE _ PVCS (tm) Information
*
*      $Revision:   1.6  $
*      $Date:   16 Sep 1992 15:25:26  $
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
* MODULE :- UNOS                  File :  NULLTSK.C
****************************************************************************/
#include <stdio.h>
#include <conio.h>
#include <bios.h>
#include <dos.h>
#include "general.h"

extern void start_time_slice ( void );

/*
=========================================================================

null_task

This is the system null task

==========================================================================
*/

void null_task ( void* DummyVariable )
{
	DummyVariable = DummyVariable;

    enable ( );
    start_time_slice ( );
    while ( 1 )
    {
    } /* while */

} /* nulltsk */
