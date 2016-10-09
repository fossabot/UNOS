#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#define PI    3.141592654
#define TRUE  1
#define FALSE 0

/* function used to calculate the julian day given the current time
   the pseudo source is taken from the web
   ksc
*/
double julian (double hour, double min, double day, double month, double year) {

int a, ggg, s;
double modh, j1, jday;

	/* source from a javascript utility at
	http://saxpds2.tesre.bo.cnr.it/Pds/Actions/julianday.html */
	modh = hour + (min/60);
	year = 1900 + year;

	if (year <= 1585)
		ggg = 0;
	else
		ggg = 1;

	if ( (month - 9) < 0 )
		s = -1;
	else
		s = 1;

	jday = -1 * floor(7 * (floor((month + 9) /12) + year) /4);
	a    = abs(month - 9);
	j1   = floor(year + s*floor(a/7));
	j1   = -1 * floor((floor(j1/100) +1) * 3 /4);
	jday = jday + floor(275 * month / 9) + day + (ggg * j1);
	jday = jday + 1721027 + 2 * ggg + 367 * year - 0.5;
	jday = jday + (modh / 24);

	return (jday);

}


void radec_sun (double jday, double *alpha, double *delta,
			double *alphah, double *alpham, double *alphas,
			double *deltah, double *deltam, double *deltas) {

int delta_neg, alpha_neg;
double n, L, g, f, t, lamda, beta, epsilon;
/*double alphah, alpham, alphas, deltah, deltam, deltas, alpha, delta;*/

	/* from the 1996 astronomical almanac pp C24 */
	n = jday - 2451545;
	L = 280.461 + 0.9856474*n;
	g = 357.528 + 0.9856003*n;

	/* convert L and g into multiples of 360 degrees */
	while ( L >= 360 )
		L = L - 360;

	while ( L < 0 )
		L = L +360;

	while ( g >= 360 )
		g = g - 360;

	while ( g < 0 )
		g = g + 360;

	lamda = L + 1.915*sin(g*PI/180) + 0.020*sin(2*g*PI/180);
	beta = 0;


	epsilon = 23.439 - 0.0000004*n;
	f     = 180/PI;
	t     = tan(epsilon*PI/360) * tan(epsilon*PI/360);
	*alpha = lamda - f*t*sin(lamda*PI/90) + f/2*t*t*sin(lamda*PI/45);
	*delta = asin( sin(epsilon*PI/180) * sin(lamda*PI/180) ) * 180 / PI;


	/* convert ascension estimates into h/m/s */
	if ( *alpha < 0 ) {
		alpha_neg = TRUE;	/* convert to +ve values */
		*alpha = -(*alpha);	/* floor works on +ve values only */
	}
	else
		alpha_neg = FALSE;
	*alphah = *alpha/360*24;
	*alpham = (*alphah - floor(*alphah))*60;
	*alphas = (*alpham - floor(*alpham))*60;
	*alphah = floor(*alphah);
	*alpham = floor(*alpham);

	if ( alpha_neg == TRUE ) {
		*alphah = -(*alphah);
		*alpha  = -(*alpha);
	}
/*	printf("Right ascension = %.0lf %.0lf %.2lf\n", alphah, alpham, alphas); */



	/* convert declination estimates into hours/min/seconds */
	if ( *delta < 0 ) {
		delta_neg = TRUE;
		*delta = -(*delta);
	}
	else
		delta_neg = FALSE;
	*deltah = floor(*delta);
	*deltam = floor((*delta - *deltah)*60);
	*deltas = (((*delta - *deltah)*60) - (*deltam))*60;

	if (delta_neg == TRUE) {
		*deltah = -(*deltah);
		*delta  = -(*delta);
	}

/*printf("Declination     = %.0lf %.0lf %.2lf\n", deltah, deltam, deltas);*/

}


