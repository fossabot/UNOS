#include <stdio.h>
#include <math.h>
#include <time.h>
#include <steptrak.h>

#define HOLDING 0
#define TRACKING 1
/*#include <seq.h>*/




double   az,el,act_az, act_el,end_az,azmsd,elmsd,azcmd,elcmd;
										 /*az of the dish,el of the dish,*/
										/*actual az of the satellite,
										actual el of the satellite*/
unsigned int atcustate;

double power;
double width = 0.7;
clock_t t;



void main()
	{
	atcustate=HOLDING;
	azmsd=164;
	elmsd=29;
	azcmd=164;
	elcmd=29;

	while(1)
	{
	t=clock()/CLOCKS_PER_SEC;
	printf("timer :");
	printf("%d\n",t);
	printf("\n");
	act_az=az_pos_sat(t);
	act_el=el_pos_sat(t);

	power=power_calc(act_az,act_el,azmsd,elmsd,width);
	printf("                                                         power = ");
	printf("%.4f\n",power);
	printf("\n");
	hillclimbing_sm(&atcustate,&azcmd,&elcmd,&azmsd,&elmsd,&power);
/*	printf("     atcu state : ");
	printf("%d\n",atcustate);
	printf("\n");*/
	printf("%f\n",act_az);
	printf("               ");
	printf("%f\n",azcmd);

	}
}

