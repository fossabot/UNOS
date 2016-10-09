#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "kalman.h"

#define HOLDING 0
#define TRACKING 1
#define KALMAN 1

clock_t timer,start_time;

unsigned int atcumode,atcustate;
double act_az,act_el,azcmd,elcmd,azmsd,elmsd,power;

main()
{
atcumode=KALMAN;
atcustate=HOLDING;
azcmd=165.1;
azmsd=165.1;
elcmd=29.9;
elmsd=29.9;

 while (atcumode==KALMAN) {
 timer = clock()/CLOCKS_PER_SEC;
 while (timer<=4000) {
 timer = clock()/CLOCKS_PER_SEC;
 /*printf("clk:");
 printf("%d\n",timer);*/

 act_az=az_pos_sat(timer);
 act_el=el_pos_sat(timer);
 power=power_calc(act_az,act_el,azmsd,elmsd);
 /*printf("        power:");
 printf("%.3f\n",power); */

 kalman_sm(&atcustate,&azcmd,&elcmd,&azmsd,&elmsd,&power,timer,&start_time);
 /*printf("     atcu state : ");
 printf("%d\n",atcustate);
 printf("\n");
			printf("                     azant:");
			printf("%.3f\n",azmsd);

			printf("                                    azsat:");
			printf("%.3f\n",act_az);


			printf("                                                    elant:");
			printf("%.3f\n",elmsd);

			printf("                                                                  elsat:");
			printf("%.3f\n",act_el); */
			

 }
 scanf("%d\n");
}
return 0 ;
}