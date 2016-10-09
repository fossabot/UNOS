/*******************************************************************************
	MODULE 	: KALMAN

	FILE		: KALMAN.C

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
*******************************************************************************/

/*
Functional Description:

	This module contains the Kalman state machine software for the ATCU. It is
intented to be run when the Kalman tracking mode is selected within the real
time operating system.
	The Kalman state machine is divided in 6 states:

	. steptrack : calling the hill-climbing procedure so as to get
	a measurement of the satellite position using the step track method.

	. init_param : This state is necessary to feed the Kalman algorithm with
	initial values.

	. update_model : calculating the new Kalman filter to estimate
	the satellite motion, after receiving a new measurement from the steptrack
	state. For timing consideration, the calculations where divided in 6 other
	states :

			- time_update : calculating the current time varying terms of the model
			and adding a noise term (matrix Q) to the azimuth and elevation
			covariance matrices.

			- calc_kalman_gain_az : calculating the new Kalman gain in azimuth.

			- calc_kalman_gain_el : calculating the new Kalman gain in elevation.

			- observ_update : introducing the new measurement in our model so as to
			calculate the new coefficients.

			- update_cov_mat : updating the azimuth and elevation covariances
			matrices with the new model.

	. estim-cmd : calculating the estimation of the satellite position
	using the current model and returning azimuth and elevation commands to the
	ATCU sequencer.

	. wait_end_move : waiting for the antenna to move to the new position.

	. logic_state : checking if only one steptrack has been done to go to the
	kalman filter initialisation state (init-param).

	. waiting_state : calculating the time to go until next steptrack. If
	the time (30 minutes) is over go to the steptrack_state, if not go to the
	estim-cmd state.
*******************************************************************************/

/*include the following libraries :*/
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "kalman.h"
#include "modelsim.h"

#define HOLDING 0
#define TRACKING 1
#define KALMAN 1
/*include "seq.h" when linking with unimel programm*/

/*kalman states definition*/
#define steptrack 0
#define init_param 1
#define update_model 2
#define estim_cmd 3
#define wait_end_move 4
#define logic_state 5
#define waiting_state 6

/*update model states definition*/
#define time_update 0
#define calc_kalman_gain_az 1
#define calc_kalman_gain_el 2
#define observ_update 3
#define update_cov_mat 4


unsigned int kalman_state,update_model_state;

/*static local variables*/
static int firststep_flag;

/*x_az is the array of 6 predicted coefficients for azimuth state estimate .
  x_el is the array of 6 predicted coefficients for elevation state estimate.
  P_cov_az is the 6*6 state error covariance matrix for azimuth.
  P_cov_el is the 6*6 state error covariance matrix for elevation.
  Q_noise is the  6*6 uncertainty matrice for azimuth and elevation model
  parameters.
  R_az,R_el are the mean square uncertainty in measurements of satellite azimuth
  and elevation obtained from beacon power measurements.
  K_az,K_el are arrays of 6 coefficients for Kalman gains in azimuth and
  elevation.
  model is the array of 6 time varying parameters for the orbit model :
	  C = [ 1 t cos(wt) sin(wt) cos(2wt) sin(2wt) ]
  */
static double x_az[6],x_el[6],P_cov_az[36],P_cov_el[36],Q_noise[36],
			R_az[1],R_el[1],K_az[6],K_el[6];

static double model[6];

static double temp_az1dim[6],temp_az2dim[36],temp_el1dim[6],temp_el2dim[36],
		temp_az[1],temp_el[1],minus[1],temp2_az1dim[6],temp2_el1dim[6];

static int counter;

/*other variables*/
/*mesaz, mesel are the measurement of antenna elevation and azimuth after a
tracking sequence.
*/
double mesaz,mesel;

/****************************KALMAN STATE MACHINE*******************************
/* This state machine moves the antenna according to the Kalman filter
prediction and uses the hill_climbing procedure to feed the model with new
measurements.
When it is called by seq.c, "atcustate" will be "atcu.state" of seq.c
									"azcmd" will be "az.cmd" of seq.c
									"elcmd" will be "el.cmd" of seq.c
									"azmsd" will be "az.msd of seq.c
									"elmsd" will be "el.msd" of seq.c*/

 void kalman_sm(unsigned int *atcustate,double *azcmd,double *elcmd,
							double *azmsd,double *elmsd,double *power,
							clock_t timer,clock_t *start_time) {
	switch(kalman_state) {

	case steptrack :

		hillclimbing_sm(atcustate,azcmd,elcmd,azmsd,elmsd,power,
								&kalman_state,&mesaz,&mesel);
		break;

	case init_param :

		/*This state feeds the Kalman algorithm with
		initial values for the orbit model.*/

		/*initial azimuth estimation*/
		x_az[0] = *azmsd;
		x_az[1] = 0;
		x_az[2] = 0;
		x_az[3] = 0;
		x_az[4] = 0;
		x_az[5] = 0;
		/*initial elevation estimation*/
		x_el[0] = *elmsd;
		x_el[1] = 0;
		x_el[2] = 0;
		x_el[3] = 0;
		x_el[4] = 0;
		x_el[5] = 0;
		/*initial azimuth covariance matrix*/
		IdMatrix(&P_cov_az[0]);
		/*initial elevation covariance matrix*/
		IdMatrix(&P_cov_el[0]);
		R_az[0] = 0.2;
		R_el[0] = 0.2;
		/*initialisation of noise covariance matrix Q*/
		Q_noise[0] =  250*1e-5;
		Q_noise[1] =  0;
		Q_noise[2] =  0;
		Q_noise[3] =  0;
		Q_noise[4] =  0;
		Q_noise[5] =  0;
		Q_noise[6] =  0;
		Q_noise[7] =  1e-9;
		Q_noise[8] =  0;
		Q_noise[9] =  0;
		Q_noise[10] = 0;
		Q_noise[11] = 0;
		Q_noise[12] = 0;
		Q_noise[13] = 0;
		Q_noise[14] = 2*1e-8;
		Q_noise[15] = 0;
		Q_noise[16] = 0;
		Q_noise[17] = 0;
		Q_noise[18] = 0;
		Q_noise[19] = 0;
		Q_noise[20] = 0;
		Q_noise[21] = 3*1e-7;
		Q_noise[22] = 0;
		Q_noise[23] = 0;
		Q_noise[24] = 0;
		Q_noise[25] = 0;
		Q_noise[26] = 0;
		Q_noise[27] = 0;
		Q_noise[28] = 1e-7;
		Q_noise[29] = 0;
		Q_noise[30] = 0;
		Q_noise[31] = 0;
		Q_noise[32] = 0;
		Q_noise[33] = 0;
		Q_noise[34] = 0;
		Q_noise[35] = 1e-7;
		firststep_flag = 1;   /* this flag is set to 1 to tell the logic_state
										that initialisation has been done */
		kalman_state = update_model;
		break;

	case update_model :

		/*This state is splitting the Kalman filter calculations in 5 states for
		timing consideration.*/

		if (*atcustate==TRACKING)
			*atcustate=HOLDING;
		if (*atcustate==HOLDING) {

		switch(update_model_state) {

		case time_update :

			/*start_time is set to the current time and will be used in the waiting
			state to calculate the time to go until next tracking sequence.
			Calculations of current time varying terms of the model are made
			by calling the calc_model procedure.
			Elevation and azimuth covariance matrices are modified to reflect the
			increasing uncertainty in the states estimates : P = P + Q .
			*/

			*start_time = clock()/CLOCKS_PER_SEC;
			calc_model(timer,&model[0]);
			Matrixsum(&P_cov_az[0],&Q_noise[0],&P_cov_az[0]);  /* P = P+Q in az. */
			Matrixsum(&P_cov_el[0],&Q_noise[0],&P_cov_el[0]);  /* P = P+Q in el. */
			update_model_state = calc_kalman_gain_az;
			break;

		case calc_kalman_gain_az :

				/*This state calculates the new Kalman gain in azimuth :
				K = PC/(CPC+R)*/

				Matcolprod(&P_cov_az[0],&model[0],&temp_az1dim[0]); /* PC */
				rowcolprod(&model[0],&temp_az1dim[0],&temp_az[0]);  /* CPC */
				temp_az[0] = 1/(temp_az[0] + R_az[0]);              /* 1/(CPC+R) */
				scalcolprod(&temp_az[0],&model[0],&temp_az1dim[0]); /* C/(CPC+R) */
				Matcolprod(&P_cov_az[0],&temp_az1dim[0],&K_az[0]);  /* PC/(CPC+R) */
				update_model_state = calc_kalman_gain_el;
			break;

		case calc_kalman_gain_el :

				/*This state calculates the new Kalman gain in elevation :
				K = PC/(CPC+R)*/

				Matcolprod(&P_cov_el[0],&model[0],&temp_el1dim[0]);
				rowcolprod(&model[0],&temp_el1dim[0],&temp_el[0]);
				temp_el[0] = 1/(temp_el[0] + R_el[0]);
				scalcolprod(&temp_el[0],&model[0],&temp_el1dim[0]);
				Matcolprod(&P_cov_el[0],&temp_el1dim[0],&K_el[0]);

			update_model_state = observ_update;
		break;

		case observ_update :
				/*update the orbit model for each axis using the new measurement and
				a single recursion of the discrete Kalman filter :
				x = x + K(measure-Cx)*/

				/*azimuth*/
				rowcolprod(&model[0],&x_az[0],&temp_az[0]);        /* Cx */
				temp_az[0] = mesaz - temp_az[0];                   /* measure-Cx */
				scalcolprod(&temp_az[0],&K_az[0],&temp_az1dim[0]); /*K(measure-Cx)*/
				columnsum(&x_az[0],&temp_az1dim[0],&x_az[0]);    /*x+K(measure-Cx)*/

				/*elevation*/
				rowcolprod(&model[0],&x_el[0],&temp_el[0]);
				temp_el[0] = mesel - temp_el[0];
				scalcolprod(&temp_el[0],&K_el[0],&temp_el1dim[0]);
				columnsum(&x_el[0],&temp_el1dim[0],&x_el[0]);

			update_model_state = update_cov_mat;
		break;

		case  update_cov_mat :

				/*this state updates the azimuth and elevation covariances
				matrices using the new Kalman gain : P = P - KCP .
				*/

				minus[0] = -1;
				/*azimuth*/
				scalcolprod(&minus[0],&K_az[0],&temp_az1dim[0]);    /* -K */
				rowMatprod(&model[0],&P_cov_az[0],&temp2_az1dim[0]);/* CP */
				colrowprod(&temp_az1dim[0],&temp2_az1dim[0],&temp_az2dim[0]);/*-KCP*/

				Matrixsum(&P_cov_az[0],&temp_az2dim[0],&P_cov_az[0]);/*P-KCP*/

				/*elevation*/
				scalcolprod(&minus[0],&K_el[0],&temp_el1dim[0]);
				rowMatprod(&model[0],&P_cov_el[0],&temp2_el1dim[0]);
				colrowprod(&temp_el1dim[0],&temp2_el1dim[0],&temp_el2dim[0]);

				Matrixsum(&P_cov_el[0],&temp_el2dim[0],&P_cov_el[0]);
				update_model_state = time_update;
				kalman_state = estim_cmd;
			break;

		default : update_model_state = time_update;
					 break;
		}
	}
	break;

	case estim_cmd :

		 /*This state calculates the prediction of the satellite position
	using the current model.
			It returns azimuth and elevation commands to the
	ATCU sequencer
		*/
		if (*atcustate==HOLDING)
				*atcustate=TRACKING;
		if (*atcustate==TRACKING){

			calc_model(timer,&model[0]);                   /* C(t) */
			rowcolprod(&model[0],&x_az[0],&temp_az[0]);    /* C(t)x */
			*azcmd = temp_az[0];                           /* command=C(t)x */
			rowcolprod(&model[0],&x_el[0],&temp_el[0]);
			*elcmd = temp_el[0];
			counter = 25 ;                  /*set the counter for the next state*/
			kalman_state = wait_end_move;
			}
		break;

	case wait_end_move :

		/*This state is waiting for the antenna to reach the predicted position.
		The counter value was set in the previous state and can be ajusted.
		*/

		if (*atcustate==TRACKING)
			*atcustate=HOLDING;
		if (*atcustate==HOLDING) {
			if (counter--<0){

				*azmsd = temp_az[0];         /* for simulation only */
				*elmsd = temp_el[0];         /* for simulation only */
				kalman_state = waiting_state;
			}
		}
		break;

	case logic_state :

		/* This state is called after each tracking sequence and checks if only
		one steptrack has been done (or not) to go to the
		kalman filter initialisation state (or not).
		*/

		if (firststep_flag==0)
			kalman_state = init_param;
		if (firststep_flag==1){
			update_model_state = time_update;
			kalman_state = update_model;
			}
		break;

	case waiting_state :

			/*This state calculates the time until next tracking sequence.  If
			the time (30 minutes) is over, it goes to the steptrack_state. If not,
			it goes to the estim-cmd state.
			Start_time was set in the time update state.
			The time until next steptrack is in seconds and can be adjusted.
			*/

			if (timer-*start_time>=40){

				kalman_state = steptrack;
				}
			else
				kalman_state = estim_cmd;
			break;

	default :   mesaz = 0;
					mesel = 0;
					firststep_flag = 0;
					kalman_state = steptrack;
					break;
	}
}


