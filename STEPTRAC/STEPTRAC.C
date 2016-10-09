
/**********************The University of Melbourne-Australia-*******************
					Department of Electrical and Electronic Egineering

*****************Institut National Polytechnique de Grenoble-France-************
											(I.N.P.G.)
			Ecole Nationale Superieure d'Electronique et de Radioelectricite
											(E.N.S.E.R.G.)
*******************************************************************************/

/*This software has been written by Frederick Destruhaut and Eric Pangole
(from ENSERG) during their end of curriculum study engineering project in
Melbourne, from March 24 to June 22 1995.

This real-time software should track satellite with the "step-track" and
"parabola fitting" algorithm. The results of the tracking are sent to the main
programm which control the antenna called "seq.c"*/

#include <stdio.h>
#include <stdlib.h>   /* for rand() */
#include <math.h>
#include <time.h>
#include <seq.h>
#include "steptrak.h"


/*states of "step_state" declaration*/
#define move_az1 0
#define move_el1 1
#define move_az2 2
#define move_el2 3
#define curve_fitting 4
#define step_wait 5

/*states of "move_*" declaration*/
#define wait1 0
#define msd1 1
#define move 2
#define wait2 3
#define msd2 4
#define decide 5

/*states of "curve_fitting" declaration*/
#define curvewait1 0
#define curvemsd1 1
#define curvemovaz 2
#define curvewait2 3
#define curvemsd2 4
#define optimaz 5
#define curvewait3 6
#define curvemovel 7
#define curvewait4 8
#define curvemsd3 9
#define optimel 10
#define curvewait5 11

/*states of "step_wait" declaration*/
#define set_clock 0
#define countdown 1


/*Constants declaration*/
#define step 3

/*double power_calc(double act_az,double act_el,double az,double el);

double el_pos_sat(clock_t t);

double az_pos_sat(clock_t t);

/*local sequencer routine*/
/*void hillclimbing_sm(unsigned int *atcustate,double *azcmd,double *elcmd,
										double *azmsd, double *elmsd,double *power);

/*steptrack state handlers*/
static void move_az1_sm(unsigned int *atcustate,double *azcmd,double *azmsd,
								double *power,unsigned int *step_state);

static void move_el1_sm(unsigned int *atcustate,double *elcmd,double *elmsd,
								double *power,unsigned int *step_state);

static void move_az2_sm(unsigned int *atcustate,double *azcmd,double *azmsd,
								double *power,unsigned int *step_state);

static void move_el2_sm(unsigned int *atcustate,double *elcmd,double *elmsd,
								double *power,unsigned int *step_state);

static void curve_fitting_sm(unsigned int *atcustate,double *azcmd,
										double *elcmd,double *azmsd, double *elmsd,
										double *power,unsigned int *step_state);

static void step_wait_sm(unsigned int *atcustate,unsigned int *step_state);




/*Steptrack states declaration*/
unsigned int step_state;
static int az1_state;
static int el1_state;
static int az2_state;
static int el2_state;
static int curve;
static int stepwait;

/*Counters declaration*/
static int counter1,counter2,counter3,counter4,counter5;

/*Axis direction declaration*/
static int diraz=1;
static int direl=1;

/*other variables*/
static double measure1,measure2,delta,az1,az2,el1,el2;
/*static double  width=0.7; /* beamwidth of the satellite signal */

/*Time variables definition*/
clock_t end_time;
clock_t current_time;

/*Time variables definition*/


/*These functions are ONLY for simulation. They return the model of the motion
of a geostationnary satellite*/
	double el_pos_sat(clock_t t)
{  double temp = t; /*type conversion clock_t to double*/
	return 30+0.00009*temp/3600+0.3*sin((w*temp/3600+3.045)*3.141593/180);
}

 double az_pos_sat(clock_t t)
{  double temp = t; /*type conversion clock_t to double*/
	return 165+0.0005*temp/3600+0.12*sin(w*temp/3600*3.141593/180);
}

/*This function is ONLY for simulation. It returns the noisy beacon power
received by the antenna from a satellite at act_az and act_el when the antenna
points to az and el*/
 double power_calc(double act_az,double act_el,double az,double el)
				/* gives the power received by the antenna according to
				 its position and the actual position of the satellite*/
{  int rand(void);
	double noise;
	noise = 0.2*rand()/32767; /*0.2=std, 32767=MAX_INT given by rand()*/
	return 1.0-(2.0/(pow(width,2)))*(pow(act_el-el,2)+pow(act_az-az,2))+noise;
}



/***********************HILL_CLIMBING STATE MACHINE****************************/
/*This is the main state-machine of the program.
When it is called by seq.c, "atcustate" will be "atcu.state" of seq.c
									"azcmd" will be "az.cmd" of seq.c
									"elcmd" will be "el.cmd" of seq.c
									"azmsd" will be "az.msd of seq.c
									"elmsd" will be "el.msd" of seq.c*/
void hillclimbing_sm(unsigned int *atcustate,double *azcmd,double *elcmd,
										double *azmsd,double *elmsd,double *power){


	printf("                     step_state : ");
	printf("%d\n",step_state);
	printf("\n");            /*these prints are just for simulation*/

	switch(step_state){
		case move_az1:
			move_az1_sm(atcustate,azcmd,azmsd,power,&step_state);
			break;

		case move_el1:
			move_el1_sm(atcustate,elcmd,elmsd,power,&step_state);
			break;

		case move_az2:
			move_az2_sm(atcustate,azcmd,azmsd,power,&step_state);
			break;

		case move_el2:
			move_el2_sm(atcustate,elcmd,elmsd,power,&step_state);
			break;

		case curve_fitting:
			curve_fitting_sm(atcustate,azcmd,elcmd,azmsd,elmsd,power,&step_state);
			break;

		case step_wait:
			step_wait_sm(atcustate,&step_state);
			break;

		default:
			step_state=move_az1;
			break;
	}
}


/*******************MOVE_AZ1 STATE MACHINE****************************/
/*This state-machine moves the antenna in azimuth until the received beacon
 power decreases*/
static void move_az1_sm(unsigned int *atcustate,double *azcmd,double *azmsd,
								double *power,unsigned int *step_state){


	printf("                                     az1_state : ");
	printf("%d\n",az1_state);
	printf("\n");             /*these prints are just for simulation*/

	switch(az1_state){

		case wait1:              /*WAIT 1 second. Can be adjusted by "counter1"*/
			if (counter1--<0)
				az1_state=msd1;

		break;

		/* First sample*/
		case msd1:
			if (*atcustate==TRACKING)
				*atcustate=HOLDING;
			if (*atcustate==HOLDING){

			measure1=*power;			/*POWER must be input!!*/
			az1_state=move;
			}
		break;

		/*move by one step in "diraz" (+1=East, -1=West) direction*/
		case move:
			if (*atcustate==HOLDING)
				*atcustate=TRACKING;
			if (*atcustate==TRACKING){
				*azcmd+=step*diraz;
				counter2=25;
				az1_state=wait2;
				}
		break;

		case wait2:
			if (counter2--<0)	{  /*WAIT 1.25 second: time to move by one step and
										get the measure can be adjusted by "counter2" */
				*azmsd+=step*diraz;  /*for simulation only*/
				az1_state=msd2;
				}
		break;

		/*Second sample*/
		case msd2:
			if (*atcustate==TRACKING)
			*atcustate=HOLDING;
			if (*atcustate==HOLDING){
			measure2=*power;
			az1_state=decide;
			}
		break;

		/*If smpl2>smpl1: keep on moving that direction
						else: reverse the direction diraz for the next movement
								in azimuth and move elevation (move_el1)*/
		case decide:
			delta=measure2-measure1;
			if (delta>0){
				measure1=measure2;
				az1_state=move;
			}
			else{
				diraz=-diraz;
			counter1=20;
			az1_state=wait1;
			*step_state=move_el1;
			}
			break;

		default:
			counter1=20;
			az1_state=wait1;
			break;
			}
}

 /*******************MOVE_EL1 STATE MACHINE****************************/
 /*This state machine moves the antenna in elevation until the received beacon
 power decreases*/

static void move_el1_sm(unsigned int *atcustate,double *elcmd,double *elmsd,
								double *power,unsigned int *step_state){



	printf("                                     el1_state : ");
	printf("%d\n",el1_state);
	printf("\n");                     /*These prints are only for simulation*/


	switch(el1_state){
		case wait1:                /*WAIT 1 second: can be adjuste by counter1*/
			if (counter1--<0)
				el1_state=msd1;
		break;

		/*First sample*/
		case msd1:
			if (*atcustate==TRACKING)
				*atcustate=HOLDING;
			if (*atcustate==HOLDING){
			measure1=*power;
			el1_state=move;
			}
			break;

		/*Move in elevation by one step in "direl" direction (+1=up, -1=down)*/
		case move:
			if (*atcustate==HOLDING)
				*atcustate=TRACKING;
			if (*atcustate==TRACKING){
				*elcmd+=step*direl;  /*for simulation only*/
				counter2=25;
				el1_state=wait2;
				}
				break;

		case wait2:
			if (counter2--<0)	{  /*WAIT 1.25second: time to move by one step */
				*elmsd+=step*direl;
				el1_state=msd2;
				}
				break;      					/* and time to get the measure*/

		/*Second sample*/
		case msd2:
			if (*atcustate==TRACKING)
			*atcustate=HOLDING;
			if (*atcustate==HOLDING){
			measure2=*power;
			el1_state=decide;
			}
			break;

			/*If smpl2>smpl1: keep on moving that direction
						else: reverse the direction direl for the next movement
								in elevation and move azimuth (move_az2)*/
		case decide:
			delta=measure2-measure1;
			if (delta>0){
				measure1=measure2;
				el1_state=move;
				}
			else{
				direl=-direl;
			counter1=20;
			el1_state=wait1;
			*step_state=move_az2;
				}
			break;

		default:
			counter1=20;
			el1_state=wait1;
			break;
			}
}

/*******************MOVE_AZ2 STATE MACHINE****************************/
/*This is the second motion in azimuth, in the reversed direction
		The comments are the same as for MOVE_AZ11 STATE MACHINE*/

static void move_az2_sm(unsigned int *atcustate,double *azcmd,double *azmsd,double *power,
								unsigned int *step_state){



	printf("                                     az2_state : ");
	printf("%d\n",az2_state);
	printf("\n");


	switch(az2_state){
		case wait1:                /*WAIT 1 second*/
			if (counter1--<0)
				az2_state=msd1;
		break;

		case msd1:
			if (*atcustate==TRACKING)
				*atcustate=HOLDING;
			if (*atcustate==HOLDING){

			measure1=*power;
			az2_state=move;
			}
			break;

		case move:
			if (*atcustate==HOLDING)
				*atcustate=TRACKING;
			if (*atcustate==TRACKING){
				*azcmd+=step*diraz;
				counter2=25;
				az2_state=wait2;
				}
				break;

		case wait2:
			if (counter2--<0){	  /*WAIT 1.25second: time to move by one step */
				*azmsd+=step*diraz;
				az2_state=msd2;
				}
				break;      					/* and time to get the measure*/

		case msd2:
			if (*atcustate==TRACKING)
			*atcustate=HOLDING;
			if (*atcustate==HOLDING){            /*POWER must be input!!*/
			measure2=*power;
			az2_state=decide;
			}
			break;

		case decide:
			delta=measure2-measure1;
			if (delta>0){
				measure1=measure2;
				az2_state=move;
				}
			else {
				diraz=-diraz;
				counter1=20;
				az2_state=wait1;
				*step_state=move_el2;
				}
			break;

		default:
			counter1=20;
			az2_state=wait1;
			break;
			}
}

 /*******************MOVE_EL2 STATE MACHINE****************************/
 /*This is the second motion in elevation, in the reversed direction
		The comments are the same as for MOVE_EL1 STATE MACHINE*/

static void move_el2_sm(unsigned int *atcustate,double *elcmd,double *elmsd,
								double *power,unsigned int *step_state){



	printf("                                     el2_state : ");
	printf("%d\n",el2_state);
	printf("\n");


	switch(el2_state){
		case wait1:                /*WAIT 1 second*/
			if (counter1--<0)
				el2_state=msd1;
		break;

		case msd1:
			if (*atcustate==TRACKING)
				*atcustate=HOLDING;
			if (*atcustate==HOLDING){

			measure1=*power;			/*POWER must be input!!*/
			el2_state=move;
			}
			break;

		case move:
			if (*atcustate==HOLDING)
				*atcustate=TRACKING;
			if (*atcustate==TRACKING){
				*elcmd+=step*direl;
				counter2=25;
				el2_state=wait2;
				}
				break;

		case wait2:
			if (counter2--<0){	  /*WAIT 1.25second: time to move by one step */
				*elmsd+=step*direl;
				el2_state=msd2;
				}
				break;      					/* and time to get the measure*/

		case msd2:
			if (*atcustate==TRACKING)
			*atcustate=HOLDING;
			if (*atcustate==HOLDING){            /*POWER must be input!!*/
			measure2=*power;
			el2_state=decide;
			}
			break;

		case decide:
			delta=measure2-measure1;
			if (delta>0){
				measure1=measure2;
				el2_state=move;
				}
			else{
          direl=-direl;
			counter1=20;
			el2_state=wait1;
			*step_state=curve_fitting;
				}
			break;

		default:
			counter1=20;
			el2_state=wait1;
			break;
			}
}

/*************************curve_fitting state machine**************************/

static void curve_fitting_sm(unsigned int *atcustate,double *azcmd,
										double *elcmd,double *azmsd,double *elmsd,
										double *power,unsigned int *step_state){



	printf("                                    curve : ");
	printf("%d\n",curve);
	printf("\n");                   /*These prints are only for simulation*/


	switch (curve){
		case curvewait1:            /*WAIT 1 second:can be adjusted by counter1*/
			if (counter1--<0)
				curve=curvemsd1;
		break;

		case curvemsd1:
			if (*atcustate==TRACKING)
				*atcustate=HOLDING;
			if (*atcustate==HOLDING){
			measure1=*power;
			az1=*azmsd;			
			curve=curvemovaz;

			}
		break;


		case curvemovaz:
			if (*atcustate==HOLDING)
				*atcustate=TRACKING;
			if (*atcustate==TRACKING){
				*azcmd+=step*direl;
				counter2=25;

				curve=curvewait2;
				}
		break;

	  case curvewait2:
			if (counter2--<0){     /*WAIT 1.25second: time to move by one step */
				*azmsd+=step; /*for simulation only*/
				curve=curvemsd2;

			}
				break;
											/* and time to get the measure*/
	  case curvemsd2:
			if (*atcustate==TRACKING)
				*atcustate=HOLDING;
			if (*atcustate==HOLDING){
			measure2=*power;			/*POWER must be input!!*/
			az2=*azmsd;
			curve=optimaz;

			}
		break;

	  case optimaz:
			if (*atcustate==HOLDING)
				*atcustate=TRACKING;

			if (*atcustate==TRACKING){
			*azcmd=(az1+az2)/2+(measure1-measure2)*pow(width,2)/(4*(az1-az2));
			counter3=25;
			curve=curvewait3;
			}
	  break;

	 case curvewait3:                /*WAIT 1 second*/
			if (counter3--<0){
				*azmsd=(az1+az2)/2+(measure1-measure2)*pow(width,2)/(4*(az1-az2));/*simulation*/
				el1=*elmsd;
				measure1=*power;

				curve=curvemovel;
				 
			}
		break;

	 case curvemovel:
			if (*atcustate==HOLDING)
				*atcustate=TRACKING;

			if (*atcustate==TRACKING){
			*elcmd+=step;
			counter4=25;
			
			curve=curvewait4;
			}
	 break;

	 case curvewait4:
		 if (counter4--<0) {    /*WAIT 1.25second: time to move by one step */
			 *elmsd+=step;
			 curve=curvemsd3;
			 }
	 break;      					/* and time to get the measure*/

	 case curvemsd3:
			if (*atcustate==TRACKING)
				*atcustate=HOLDING;
			if (*atcustate==HOLDING){
				measure2=*power;
				el2=*elmsd;
				curve=optimel;
			}
	 break;

	 case optimel:
			if (*atcustate==HOLDING)
				*atcustate=TRACKING;

			if (*atcustate==TRACKING){
			*elcmd=(el1+el2)/2+(measure1-measure2)*pow(width,2)/(4*(el1-el2));
			counter5=25;
			curve=curvewait5;
			}
	  break;

		case curvewait5:                /*WAIT 1 second*/
			if (counter5--<0){
				*elmsd=(el1+el2)/2+(measure1-measure2)*pow(width,2)/(4*(el1-el2));
/*				printf("antenna position after steptrack+parabola fitting : ");
				printf("\n");
				printf("%.2f",*azmsd);
				printf("\n");
				printf("%.2f\n",*elmsd);
				 scanf("%d",x);*/
				 curve=curvewait1;
				*step_state=step_wait;

			}
		break;


	  default:
			counter1=20;
			curve=curvewait1;
	  break;
	 }
}

/*************************step_wait state*********************************/
/******waiting for 30 minutes between 2 steptrakcs*******/

static void step_wait_sm(unsigned int *atcustate,unsigned int *step_state){

  	switch(stepwait){
		case set_clock:
			if (*atcustate==TRACKING)
				*atcustate=HOLDING;
			if (*atcustate==HOLDING){
			end_time=clock()/CLOCKS_PER_SEC;

			stepwait=countdown;
			}
		break;

		case countdown:
			current_time=clock()/CLOCKS_PER_SEC;
			printf("                          NEXT SHOW AT 40'': ");
			printf("%d\n",current_time-end_time);
			printf("\n");
			if (current_time-end_time>=40){
			*step_state=move_az1;
			stepwait=set_clock;
			}

		break;

		default:
			stepwait=set_clock;
		break;

	}
}

