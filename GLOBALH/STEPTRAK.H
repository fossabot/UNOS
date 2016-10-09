
#include <time.h>
#define w 261.8 /*sideral day*/
#define width 0.7


extern void hillclimbing_sm(unsigned int *atcustate,double *azcmd,double *elcmd ,
									double *azmsd, double *elmsd,double *power);
									
extern double power_calc(double act_az,double act_el,double az,double el);

extern double el_pos_sat(clock_t t);

extern double az_pos_sat(clock_t t);


/***************** SEE TRACK2.C and SEQ.C********************************/
