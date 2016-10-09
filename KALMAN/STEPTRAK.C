/*******************************************************************************
	MODULE 	: KALMAN

	FILE		: STEPTRAC.C
	
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

/*include the following libraries :*/
#include <stdio.h>
#include <math.h>
#include <time.h>
#include "kalman.h"

/*#include <seq.h> */
#define HOLDING 0
#define TRACKING 1

/*states of "stepstate" declaration*/
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

/*kalman states definition*/
#define logic_state 5


/*constant declarations*/
#define step 2

/*steptrack state handlers*/
static void move_az1_sm(unsigned int *atcustate,double *azcmd,double *azmsd,
								double *power,unsigned int *stepstate);
static void move_el1_sm(unsigned int *atcustate,double *elcmd,double *elmsd,
								double *power,unsigned int *stepstate);
static void move_az2_sm(unsigned int *atcustate,double *azcmd,double *azmsd,
								double *power,unsigned int *stepstate);
static void move_el2_sm(unsigned int *atcustate,double *elcmd,double *elmsd,
								double *power,unsigned int *stepstate);
static void curve_fitting_sm(unsigned int *atcustate,double *azcmd,
								double *elcmd,double *azmsd, double *elmsd,
								double *power,unsigned int *stepstate);





/*Steptrack states declaration*/
unsigned int stepstate;
static int az1_state;
static int el1_state;
static int az2_state;
static int el2_state;
static int curve;

/*Counters declaration*/
static int counter1,counter2,counter3,counter4,counter5;

/*Axis direction declaration*/
static int diraz=1;
static int direl=1;

static double measure1,measure2,delta,az1,az2,el1,el2;
double  width=0.7;  /*beamwidth of the satellite signal */


/***********************HILL_CLIMBING STATE MACHINE****************************/
/*This is the main state-machine of the steptrack algorithm.
When it is called by seq.c, "atcustate" will be "atcu.state" of seq.c
									"azcmd" will be "az.cmd" of seq.c
									"elcmd" will be "el.cmd" of seq.c
									"azmsd" will be "az.msd of seq.c
									"elmsd" will be "el.msd" of seq.c*/

void hillclimbing_sm(unsigned int *atcustate,double *azcmd,double *elcmd,
										double *azmsd,double *elmsd,double *power,
										unsigned int *kalmanstate,double *mesaz,
										double *mesel){


/*	printf("                                     		  steptrack ");
	printf("\n"); */      /*these prints are just for simulation*/
	switch(stepstate){
		case move_az1:
			move_az1_sm(atcustate,azcmd,azmsd,power,&stepstate);
			break;

		case move_el1:
			move_el1_sm(atcustate,elcmd,elmsd,power,&stepstate);
			break;

		case move_az2:
			move_az2_sm(atcustate,azcmd,azmsd,power,&stepstate);
			break;

		case move_el2:
			move_el2_sm(atcustate,elcmd,elmsd,power,&stepstate);
			break;

		case curve_fitting:
			curve_fitting_sm(atcustate,azcmd,elcmd,azmsd,elmsd,power,&stepstate);
			break;

		case step_wait:
			if (*atcustate==TRACKING)
				*atcustate=HOLDING;
			if (*atcustate==HOLDING){
				*mesaz = *azmsd;
				*mesel = *elmsd;
				stepstate = move_az1;
				*kalmanstate = logic_state;
				}
			break;

		default:
			stepstate=move_az1;
			break;
	}
}


/*******************MOVE_AZ1 STATE MACHINE****************************/
/*This state-machine moves the antenna in azimuth until the received beacon
 power decreases*/
static void move_az1_sm(unsigned int *atcustate,double *azcmd,
							double *azmsd,double *power,unsigned int *stepstate){

		switch(az1_state){

		case wait1:              /*WAIT 1 second. Can be adjusted by "counter1"*/
			if (counter1--<0)
				az1_state=msd1;

		break;

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

				break;      					/* and time to get the measure*/

		/*Second sample*/
		case msd2:
			if (*atcustate==TRACKING)
			*atcustate=HOLDING;
			if (*atcustate==HOLDING){            /*POWER must be input!!*/
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
			*stepstate=move_el1;
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
static void move_el1_sm(unsigned int *atcustate,double *elcmd,double *elmsd,double *power,unsigned int *stepstate){





	switch(el1_state){

		case wait1:                /*WAIT 1 second: can be adjuste by counter1*/
			if (counter1--<0)
				el1_state=msd1;
		break;

		case msd1:
			if (*atcustate==TRACKING)
				*atcustate=HOLDING;
			if (*atcustate==HOLDING){
			measure1=*power;			/*POWER must be input!!*/
			el1_state=move;
			}
			break;

		/*Move in elevation by one step in "direl" direction (+1=up, -1=down)*/
		case move:
			if (*atcustate==HOLDING)
				*atcustate=TRACKING;
			if (*atcustate==TRACKING){
				*elcmd+=step*direl;
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

		case msd2:
			if (*atcustate==TRACKING)
			*atcustate=HOLDING;
			if (*atcustate==HOLDING){            /*POWER must be input!!*/
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
			*stepstate=move_az2;
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
								unsigned int *stepstate){





	switch(az2_state){
		case wait1:                /*WAIT 1 second*/
			if (counter1--<0)
				az2_state=msd1;
		break;

		case msd1:
			if (*atcustate==TRACKING)
				*atcustate=HOLDING;
			if (*atcustate==HOLDING){

			measure1=*power;			/*POWER must be input!!*/
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
				*stepstate=move_el2;
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

static void move_el2_sm(unsigned int *atcustate,double *elcmd,double *elmsd,double *power,
								unsigned int *stepstate){





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
			*stepstate=curve_fitting;
				}
			break;

		default:
			counter1=20;
			el2_state=wait1;
			break;
			}
}

/*************************curve_fitting state machine*****************************/

static void curve_fitting_sm(unsigned int *atcustate,double *azcmd,
										double *elcmd,double *azmsd,double *elmsd,
										double *power,unsigned int *stepstate){
   
	switch (curve){

		case curvewait1:                /*WAIT 1 second*/
			if (counter1--<0)
				curve=curvemsd1;
		break;

		case curvemsd1:
			if (*atcustate==TRACKING)
				*atcustate=HOLDING;
			if (*atcustate==HOLDING){
			measure1=*power;
			az1=*azmsd;			/*POWER must be input!!*/
			curve=curvemovaz;

			}
		break;


		case curvemovaz:
			if (*atcustate==HOLDING)
				*atcustate=TRACKING;
			if (*atcustate==TRACKING){
				*azcmd+=step*diraz;
				counter2=25;

				curve=curvewait2;
				}
		break;

	  case curvewait2:
			if (counter2--<0){     /*WAIT 1.25second: time to move by one step */
				*azmsd+=step*diraz; /*for simulation only*/
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
			*elcmd+=step*direl;
			counter4=25;
			
			curve=curvewait4;
			}
	 break;

	 case curvewait4:
		 if (counter4--<0) {    /*WAIT 1.25second: time to move by one step */
			 *elmsd+=step*direl;
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
				curve=curvewait1;
				*stepstate=step_wait;
			}
	 break;


	 default:
				counter1=20;
				curve=curvewait1;
			    break;
	 }
}


