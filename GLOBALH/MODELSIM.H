#include <stdio.h>
#include <time.h>

extern double power_calc(double act_az,double act_el,double az,double el);

extern double el_pos_sat(clock_t t);

extern double az_pos_sat(clock_t t);

extern void calc_model(clock_t t,double *a1dimMatrixC);



