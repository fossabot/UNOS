/* Created to acknowledge the authors of this software
							11/10/96   ksc/tzqh
*/
#include <graphics.h>
#include <stdio.h>
#include <conio.h>
#include <dos.h>
#include <string.h>
#include <stdlib.h>

#include "unos.h"
#include "general.h"

#include "seq.h"
#include "seqext.h"
#include "scrext.h"

#include "pcscr.h"
#include "kbtask.h"
#include "tparser.h"
#include "taskname.h"



void display_credit (void)
{
	protect_screen();
	clrscr();
	setcursor ( 0x2000 );		/* Turn cursor off */

	pcscr_put_text ( 30, 1," ATCU CREDIT PAGE ", BOLD );
	pcscr_draw_line ( 80, 1, 2, HOR, 1 );
	pcscr_put_text ( 2, 4,"Steve Weller ", NORMAL );
	pcscr_put_text ( 2, 5,"Lenn  Sciacca ", NORMAL );
	pcscr_put_text ( 2, 6,"Tony  Huang ", NORMAL );
	pcscr_put_text ( 2, 7,"Daryl Chiam ", NORMAL );
	pcscr_draw_line ( 80, 1, 23, HOR, 1 );
	display_function_keys();
	unprotect_screen();
	while(return_screen_page() == CREDIT_PAGE)
	{
		update_screen(1);
	}
	;

}
