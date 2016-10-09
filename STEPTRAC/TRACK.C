#include <stdio.h>
#include <math.h>

double width = 0.7;
double step = 0.2;

/* Function prototypes */
double power_calc(double act_az,double act_el,double az,double el,double width1);

void move_az(double step,int diraz,double az,double el,
			double act_az,double act_el,double width);

void move_el(double step,int direl,double az,double el,
			double act_az,double act_el,double width);

void which_axis(int select_axis,int diraz,int direl,double az,
					double el,double step,double act_az,double act_el,
					double width);

					
int counter, select_axis, diraz,direl,end; /*counter of the main, selected axis for
									   the first motion, directions of the first
									   az and el motions*/


double  az, el, act_az, act_el; 	 /* power received by the antenna,										az of the dish,el of the dish,
										actual az of the satellite,
										actual el of the satellite*/

double power_calc(double act_az,double act_el,double az,double el,double width)
/* gives the power received by the antenna according to its position and
				the actual position of the satellite*/
{
	double rad_el,power;

	rad_el=(3.14159/180.0)*el;   /* converts el (in deg) into rad_el (in rad)
								for the calculation of the cos in the next line
								*/

	power=1.0-(2.0/(pow(width,2)))*(pow(act_el-el,2)+pow(cos(rad_el),2)
	*pow(act_az-az,2));
	return power;
}


void move_az(double step,int diraz,double az,double el,
			double act_az,double act_el,double width)
{
	double delta,smpl2,smpl1,st,az1,el1;
	az1=az;
	el1=el;

	if (diraz==1)               /* -step if diraz =1 (=west) */
			st=-step;
	else
		st=step;

	delta=0;

	while(delta==0){         /*increases az until power decreases*/
		printf("Azimuth=\t");
		printf("%.1f\n", az1);

		smpl1=power_calc(act_az,act_el,az1,el1,width);
		printf("smpl1=\t");
		printf("%.2f\n",smpl1);
		az1+=st;
        printf("Azimuth=\t");
		printf("%.1f\n", az1);
		smpl2=power_calc(act_az,act_el,az1,el1,width);
		printf("smpl2=\t");
		printf("%.2f\n",smpl2);
		printf("\n");
		if (smpl2>smpl1)
			delta=0;
			else
			delta=1;
	}

	if (diraz==1)             /* reverse gear for the next azimuth motion */
		diraz=0;
	else
		diraz=1;

	az=az1;
	}


 void move_el(double step,int direl,double az,double el,
			double act_az,double act_el,double width)
{
	double delta,smpl2,smpl1,st,az1,el1;
	az1=az;
	el1=el;

	if (direl==1)               /* -step if direl=1(=down) */
			st=-step;
	else
		st=step;

	delta=0;
	
	while (delta==0){   		/*increases el until power decreases*/
		printf("Elevation=\t");
		printf("%.1f\n", el1);

		smpl1=power_calc(act_az,act_el,az1,el1,width);
		printf("smpl1=\t");
		printf("%.2f\n",smpl1);
		el1+=st;
        printf("Elevation=\t");
		printf("%.1f\n", el1);
		smpl2=power_calc(act_az,act_el,az1,el1,width);
		printf("smpl2=\t");
		printf("%.2f\n",smpl2);
		printf("\n");
		if(smpl2>smpl1)
			delta=0;
			else delta=1;
	}

	if (direl==1)             /* reverse gear for the next elevation motion */
		direl=0;
	else
		direl=1;

   el=el1;

 }

 void which_axis(int select_axis,int diraz,int direl,double az,
					double el,double step,double act_az,double act_el,
					double width)
					{
	if (select_axis==1)
		move_az(step,diraz,az,el,act_az,act_el,width);

	else move_el(step,direl,az,el,act_az,act_el,width);
 }

 main()
 {

	counter=0;
	act_az=165;
	act_el=30;

	az=160;
	el=25;

	direl=1;
	diraz=1;
	select_axis=1;

	/*printf("What is the actual AZ of the satellite(in degrees):\n");
	scanf("%f",act_az);

	printf("What is the actual EL of the satellite(in degrees):\n");
	scanf("%f",act_el);

	printf("Which axis for the first motion(1 for az, 0 for el):\n");
	scanf("%d",select_axis); /* 1 for az, 0 for el. Selects the axis for the */
							/* first motion */

   /*	printf("Which direction for the first az motion(1 for West, 0 for East):\n");

	scanf("%d",diraz);  /* initial az motion: 1 for west, 0 for east*/
	
   /*	printf("Which direction for the first el motion(1 for down, 0 for up):\n");
	scanf("%d",direl);	/*  initial el motion: 1 for down, 0 for up*/




	for(counter=0;counter<=3;counter++){


		which_axis(select_axis,diraz,direl,az,el,step,act_az,act_el,
					width);

		 if (select_axis==1)
			select_axis=0;
		else
			select_axis=1;

	}
	scanf("%d",end);
	return 0;
 }           /*END*/
