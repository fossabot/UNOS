#include <stdio.h>
#include <math.h>
#include <time.h>
#include "modelsim.h"
#define w 261.8 /*sideral day*/
#define width  0.7



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


void calc_model(clock_t t,double *c) {
		double temp = t; /*type conversion clock_t to double*/
		c[0] = 1;
		c[1] = temp/3600;
		c[2] = cos(w*temp/3600*3.141593/180);
		c[3] = sin(w*temp/3600*3.141593/180);
		c[4] = cos(2*w*temp/3600*3.141593/180);
		c[5] = sin(2*w*temp/3600*3.141593/180);
}

