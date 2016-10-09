/***************************************************************************
 FILE 		:- STARTRAK.C

 PROGRAMMER :- Prof. P Hamilton

 Copyright 1994. The University of Tasmania.

*/

#include <math.h>

long julday(int day, int month, int year);

double juldat(int day, int month, int year,
              int hour, int minute, double second);

double sidtime(double jul_epoch, double longitude);

void once(double latitude);

void daily(double jul_epoch);

void convert(double tlong, double tlat, int mode,
             double sidtim,
             double *ha, double *dc,
             double *az, double *el);

void risetime(double ha, double dc, double latitude, double elevlim,
              double *risetim, double *uptim, double *riseaz);

/*  Primary constants. */
#define  PI  3.141592653589793238462643
const double    pi     = PI;
const double    twopi  = 2*PI;
//const double    degrad = PI/180.0;
//const double    raddeg = 180.0/PI;
const double    julcen = 36525.0;
const double    j2000  = 2451545.0;

/* Constant of refraction:  */
const double    ref0 = 0.018 * PI / 180.0;

/* global variables initiallised by routine once */
static double          etoa11, etoa12, etoa13;
static double          etoa21, etoa22, etoa23;
static double          etoa31, etoa32, etoa33;
static double          elnodiff, trelsp, apelsp;

/* global variables initiallised by routine daily */
static double          eqeqnx;
static double          vonc1, vonc2, vonc3;
static double          pmat11, pmat12, pmat13;
static double          pmat21, pmat22, pmat23;
static double          pmat31, pmat32, pmat33;

unsigned int once_flag = 0;
/*
****************************************************************************
star_track

	Routine that calls the appropriate routines to performs star tracking.

	Note that future versions may wish to do something like determine rise 
times of objects.

	Author: LJS 31-7-91

Inputs:
	RA, DEC of star
	day, month, year, hour, minute, second (fract)
	station Longitude, station latitude

Ouputs:
	AZ, EL commands

****************************************************************************
*/
void star_track ( double star_ra, double star_dec, int epoch,
					int day, int month, int year, int hour, int minute,
					double second, 
					double station_long, double station_lat,
					double *azcom,
					double *elcom ) {

	double julian_date, siderial_time;
	double hour_ang, dec;
	int mode;

	/* Call once once */
	if ( !once_flag ) {
		once ( station_lat );
		once_flag = 1;
		}

	julian_date = juldat ( day, month, year, hour, minute, second );

	daily ( julian_date );

	siderial_time = sidtime ( julian_date, station_long);

	/* Convert epoch value to `mode' (`epoch is obtained via PARSER from
		CMU, where the operator is best left with `epochs' like
                1950 etc rather than having hi deal in `modes' 1-5 ).
	epoch 1-3  => mode 1-3  ( See convert() code further on. )
	epoch 2000 => mode 4
	epoch 1950 => mode 5.  */
        
	if( epoch == 2000 ) mode = 4;
	else if( epoch == 1950 ) mode = 5;
        else mode = epoch;

	convert ( star_ra, star_dec, mode,
             siderial_time,
             &hour_ang, &dec,
			 azcom, elcom );

} /* end of star_track */


long julday(int day, int month, int year)
    /*
    Subroutine to return the Julian Day Number which begins at noon
    of the calendar date specified by day, month, year.  Note that
    this code is appropriate only for dates after 15-Oct-1582, when
    the Gregorian calendar was adopted.
    Call:
          jd = julday(day, month, year)
    where
       day          is the day of the month
       month        is the month of the year
       year         is the year in full (i.e. 1991 rather than 91)
    */
{
    long            jul;
    int             ja, jy, jm;
    if (month > 2) {
       jy=year;
       jm=month+1;
    } else {
       jy=year-1;
       jm=month+13;
    }
    jul = (long) (floor(365.25*jy) + floor(30.6001*jm) + day + 1720995L);
    ja = 0.01*jy;
    jul += 2 - ja + (int)(0.25*ja);
    return jul;
}

double juldat(int day, int month, int year,
              int hour, int minute, double second)
    /*
    Subroutine to return the epoch in Julian days corresponding to
    the time (hour, minute, second) on date (day, month, year).
    Call:
          jul_epoch = juldat(day, month, year, hour, minute, second)
    where
       day          is the day of the month
       month        is the month of the year
       year         is the year in full (i.e. 1991 rather than 91)
       hour         is the hour of the day, Universal time
       minute       is the minute of the hour
       second       is the second of the minute
    Note that second could be changed to a double precision quantity
    to allow for fractions of a second. No change in the body of the
    routine would be required.
    */
{
    return  (double) julday(day, month, year)   -  0.5
            + ((second/60.0 + minute)/60.0 + hour)/24.0;
}

double sidtime(double jul_epoch, double longitude)
    /*
    Routine to compute the sidereal time (in days)
    =  mean sidereal time [A.A. suppl. 1984 p.S13] + eqeqnx.
    Call:
          sid_time = sidtime(jul_epoch, longitude, eqeqnx)
    where:
       jul_epoch    is the epoch in Julian days, generally obtained
                    via a preceding call of `juldat'
       longitude    is the observatory longitude (radians, east +)
       eqeqnx       is the equation of time, in radians, generally
                    obtained by a preceding call of `daily'
    */
{
    /*  Coefficients for sidereal time computation. */
    const double    k1      = 101.0 + 24110.54841 / 86400.0;
    const double    k2      = 8640184.812866 / 86400.0;
    const double    k3      = 0.093104 / 86400.0;
    const double    k4      = - 0.0000062 / 86400.0;
    const double    solsid  = 1.002737909350795;

    double          jd0, time, tu;

    jd0  = jul_epoch - (time = fmod((jul_epoch+0.5),1.0));
    tu   = (jd0 - j2000) / julcen;
    return fmod(((k4*tu + k3)*tu + k2)*tu + k1
                + time*solsid + (longitude + eqeqnx)/twopi, 1.0);
}

void once(double latitude)
    /*
    Subroutine to be called once, when the system is initiallised,
    to set up the global quantities
        elnodiff, trelsp, apelsp
    used in refraction calculations and elements of
        matrix [etoa]
    which rotates coordinates from (HA,decl) to (Az,El).
    Call:
          once(latitude)
    where
       latitude     is the latitude of the antenna, in radians
    */
{
    /*  Find the elevation above which no correction is needed. */
    elnodiff = atan(ref0 / 1.0e-8);

    /*  Compute the true elevation for which the apparent elevation
    approximation gives a stationary point.  Find the corresponding
    apparent elevation. */
    trelsp = asin(sqrt(ref0));
    apelsp = trelsp + ref0 / tan(trelsp);

    /*  Compute the rotation matrix  equat -> azel. */
    etoa11 = - (etoa33 = sin(latitude));
    etoa13 =   (etoa31 = cos(latitude));
    etoa22 = - 1.0;
    etoa12 = etoa21 = etoa23 = etoa32 = 0.0;
}

void daily(double jul_epoch)
    /*
    Subroutine to compute the global quantities
         eqeqnx     equation of equinoxes (radians)
         vonc*      aberration vector (dimensionless)
         pmat**     precession matrix  J2000 -> current epoch
    corresponding to the input parameter
         jul_epoch  epoch in Julian days
 
    The sense of the precession matrix is given by
       (true coords at date)  =  [pmat]  *  (mean coords at J2000.0)
 
    This version conforms to the new standard epoch of J2000,
    the new precession constants and the new theory for nutation, as
    described in the Explanatory Supplement to the Astronomical Almanac
    of 1984.

    Call:
          daily(jul_epoch);
    where
       jul_epoch    is the epoch in Julian days, generally obtained
                    via a preceding call of juldat
    */
{
    void series(double jcents,
                double *omega, double *mlanom, double *f,
                double *d,     double *e,      double *rma,
                double *ea,    double *perihl, double *eps0);

    double          jcents, omega, dpsi, deps, eps0, eps;
    double          e, rma, ea, f, x, seps, ceps;
    double          perihl, cph, sph, cea, sea, efac;
    double          mlanom, d, arg1, arg2, arg9, arg10;
    double          arg11, arg12, arg13, arg31, arg32;
    double          arg33, arg34, arg35, arg36;
    double          zeta, z, theta, coszet, sinzet, costhe;
    double          sinthe, cosz, sinz;
    double          gp11, gp12, gp13, gp21, gp22, gp23;
    double          gp31, gp32, gp33, nu11, nu12, nu13;
    double          nu21, nu22, nu23, nu31, nu32, nu33;

    /*  Compute arguments for various series - i.e. the interval from
    J2000.0 to date in Julian centuries. */
    jcents = (jul_epoch - j2000) / julcen;

    /*  Evaluate the series and related quantities. */
    series(jcents,&omega,&mlanom,&f,&d,&e,&rma,&ea,&perihl,&eps0);
    efac = sqrt(1.0 - e*e);
    cea = cos(ea);     sea = sin(ea);
    cph = cos(perihl); sph = sin(perihl);

    /*  Compute the nutation in longitude and obliquity of the sun
    [A.A. Suppl. 1984, p S26] according to the 1980 IAU Theory
    of Nutation including terms with amplitudes greater than
    0.01 arcsecond. The nutation matrix is used to compute true
    place from mean place:
            (true vector) = [nu]  *  (mean place vector)
    where the three components of each vector are the direction
    cosines with respect to the mean equinox and equator.   */
    arg1  = omega;
    arg2  = 2.0 * omega;
    arg9  = 2.0 * (f - d + omega);
    arg10 = rma;
    arg11 = arg9 + arg10;
    arg12 = arg9 - arg10;
    arg13 = arg9 - arg1;
    arg31 = 2.0 * (f + omega);
    arg32 = mlanom;
    arg33 = arg31 - arg1;
    arg34 = arg31 + arg32;
    arg35 = mlanom - 2.0 * d;
    arg36 = arg31 - arg32;

    dpsi = - 0.000083386 * sin(arg1)
           + 0.000001000 * sin(arg2)
           - 0.000006393 * sin(arg9)
           + 0.000000691 * sin(arg10)
           - 0.000000251 * sin(arg11)
           + 0.000000105 * sin(arg12)
           + 0.000000063 * sin(arg13)
           - 0.000001102 * sin(arg31)
           + 0.000000345 * sin(arg32)
           - 0.000000187 * sin(arg33)
           - 0.000000146 * sin(arg34)
           - 0.000000077 * sin(arg35)
           + 0.000000060 * sin(arg36);

    deps = + 0.000044615 * cos(arg1)
           - 0.000000434 * cos(arg2)
           + 0.000002781 * cos(arg9)
           + 0.000000109 * cos(arg11)
           + 0.000000474 * cos(arg31)
           + 0.000000097 * cos(arg33)
           + 0.000000063 * cos(arg34);
    eps = eps0 + deps; ceps = cos(eps); seps = sin(eps);

    /*  Evaluate the matrix, also obtaining the equation of the
           equinoxes = dpsi*cos(eps) = nu(2,1). */
    nu11 = nu22 = nu33 = 1.0;
    nu12 = - (nu21 = eqeqnx = dpsi * ceps);
    nu13 = - (nu31 = dpsi * seps);
    nu23 = - (nu32 = deps);

    /*  Compute the components of the aberration vector. */
    x = 0.00009936508 / (1.0 - e*cea);
    vonc1 = x * (- cph*sea      - efac*sph*cea);
    vonc2 = x * (- sph*ceps*sea + efac*cph*ceps*cea);
    vonc3 = x * (- sph*seps*sea + efac*cph*seps*cea);

    /*  Calculate the general precession matrix [gp], valid for dates AFTER
    1984.0 (JD = 2445700.5). Given the position of an object referred
    to the equator and equinox of the epoch J2000 its position referred to
    the equator and equinox of current epoch is calculated as follows:
     1.  express position as direction cosine 3-vector, (V1)
     2.  obtain the corresponding vector (V2) for epoch Js by
                  (V2) = [gp] x (V1).
    First calculate the Equatorial precession parameters (ref. USNO
    Circular no. 163 1981, Lieske et al., Astron. & Astrophys., 58, 1 1977). */
    zeta  = ((0.0000000872568*jcents + 0.00000146356)*jcents
                                     + 0.0111808609)*jcents;
    z     = ((0.0000000882506*jcents + 0.000005307158)*jcents
                                     + 0.0111808609)*jcents;
    theta = ((-0.000000202812*jcents - 0.00000206846)*jcents
                                     + 0.00971717346)*jcents;
 
    /*  Now calculate the matrix [gp]. */
    coszet = cos(zeta);  sinzet = sin(zeta);
    cosz   = cos(z);     sinz   = sin(z);
    costhe = cos(theta); sinthe = sin(theta);
    gp11 =   coszet * cosz * costhe - sinzet * sinz;
    gp12 = - sinzet * cosz * costhe - coszet * sinz;
    gp13 = - cosz * sinthe;
    gp21 =   coszet * sinz * costhe + sinzet * cosz;
    gp22 = - sinzet * sinz * costhe + coszet * cosz;
    gp23 = - sinz * sinthe;
    gp31 =   coszet * sinthe;
    gp32 = - sinzet * sinthe;
    gp33 =   costhe;

    /*  The matrices [gp] computed above and [nu] computed earlier have
    significance as indicated in the expression:
      (true coords at date)  =  [nu] * [gp]  *  (mean coords at J2000).
    We now compute [pmat] = [nu] * [gp].   */
    pmat11 = nu11*gp11 + nu12*gp21 + nu13*gp31;
    pmat12 = nu11*gp12 + nu12*gp22 + nu13*gp32;
    pmat13 = nu11*gp13 + nu12*gp23 + nu13*gp33;
    pmat21 = nu21*gp11 + nu22*gp21 + nu23*gp31;
    pmat22 = nu21*gp12 + nu22*gp22 + nu23*gp32;
    pmat23 = nu21*gp13 + nu22*gp23 + nu23*gp33;
    pmat31 = nu31*gp11 + nu32*gp21 + nu33*gp31;
    pmat32 = nu31*gp12 + nu32*gp22 + nu33*gp32;
    pmat33 = nu31*gp13 + nu32*gp23 + nu33*gp33;
}

void series(double jcents,
            double *omega, double *mlanom, double *f,
            double *d,     double *e,      double *rma,
            double *ea,    double *perihl, double *eps0)
    /*
    Subroutine to compute ephemeris quantities required by `daily'
    which are expressed as series in `jcents', the interval from J2000.0
    to date in Julian centuries.  Except where otherwise stated, the
    reference is:
    "Explanatory Supplement to the Astronomical Almanac", 1984, p.S26.
    This routine could be incorporated into `daily' as it is only called
    at one point.  It was extracted as a separate routine because otherwise
    `daily' was too large for the optimizer in Microsoft C v6.0 to be used.
    */
{
    double          x;

#define posmod(x,y,r)  if ((r=fmod(x,y)) < 0.0) r += y

    /*  Compute the longitude of the mean ascending node of the lunar
    orbit on the ecliptic, in radians. */
    posmod(((0.000000039*jcents + 0.000036143) * jcents
                                 - 33.75704593) * jcents
                                 + 2.182438624 , twopi, *omega);

    /*  Compute the Mean anomaly of the moon, in radians. */
    posmod(((0.000000310*jcents + 0.000151795) * jcents
                                + 8328.691422884) * jcents
                                + 2.355548394 , twopi, *mlanom);

    /*  Compute the longitude of the moon from ascending node, in radians. */
    posmod(((0.0000000533*jcents - 0.0000642717) * jcents
                                 + 8433.466158318) * jcents
                                 + 1.627901934 , twopi, *f);

    /*  Compute the mean elongation of the moon from the sun, in radians. */
    posmod(((0.0000000921*jcents - 0.000033409) * jcents
                                 + 7771.3771461706) * jcents
                                 + 5.1984695136 , twopi, *d);

    /*  Compute the eccentricity of the earth's orbit (dimensionless).
    Reference: "Explanatory Supplement to the Astronomical Ephemeris",
    1961, p.98 (the origin is shifted to J2000.0). */
    *e = (- 0.000000126*jcents - 0.00004205) * jcents + 0.016709114;

    /*  Compute the mean anomaly, in radians. */
    posmod(((- 0.000000058*jcents - 0.000002797) * jcents
                                  + 628.3019560242) * jcents
                                  + 6.2400359393 , twopi, *rma);

    /*  Compute the eccentric anomaly, in radians, by iteratively
         solving ea  =  e*sin(ea) - rma. */
    *ea = *rma;
    do {
      x  = *ea;
      *ea = x + (*rma - x + *e*sin(x)) / (1.0 - *e*cos(x));
    } while (fabs(*ea - x) > 1.0e-9);

    /*  Compute the mean longitude of perihelion, in radians
         (reference as for `e'). */
    *perihl = ((0.00000005817764*jcents + 0.000008077) * jcents
                                        + 0.030010190) * jcents
                                        + 1.796613066;

    /*  Compute the mean obliquity of the ecliptic, in radians. */
    *eps0 = ((0.0000000087897*jcents - 0.00000000286) * jcents
                                     - 0.0002269655) * jcents
                                     + 0.409092804;
}

void convert(double tlong, double tlat, int mode,
             double sidtim,
             double *ha, double *dc,
             double *az, double *el)

    /*
    Subroutine to convert demanded coordinates (long, lat, mode)
    to (ha, dc) and (az, el).
    Call:
          convert(tlong,tlat,mode,sidtim,&ha,&dc,&az,&el);
    where:
       tlong        input `longitude' in radians
       tlat         input `latitude' in radians
       mode         specifies the input coordinate mode, thus:
                       1  =  az,el
                       2  =  ha,dec
                       3  =  ra,dec (date)
                       4  =  ra,dec (J2000)
                       5  =  ra,dec (B1950)
       sidtim       is the current sidereal time, in days
       (ha, dc)     receive the ha/dec coordinates demanded
                       by (long, lat), in radians
       (az, el)     receive the az/el coordinates demanded
                       by (long, lat), in radians
    Uses values of these global variables:
       pmat**       the J2000 -> current epoch precession matrix
       vonc*        the current aberration vector
       etoa**       the rotation matrix: equat -> azel (own inverse).

    The expression for atmospheric refraction is taken from
    `Astrophysical Quantities', by C.W. Allen (3rd edition, page 124),
    which reference gives
        N = 1 - (7.8e-5 * P + 0.39 * e/T)/T
        ref0 = (N*N - 1)/2*N*N
        R = ref0/tan(alt)
    where  P    atmospheric pressure in mb (hPa)
           e    water vapour pressure in mb
           T    temperature in kelvin
           alt  altitude
    A value of 0.018 degrees is used for ref0, calculated using
        P = 1003 hPa,  e = 9 hPa,  T = 288 K.
    */
{
    /*  Matrix for rotating B1950 positions to J2000 positions. */
    const double    bj11 = + 0.9999256782;
    const double    bj12 = - 0.0111820610;
    const double    bj13 = - 0.0048579477;
    const double    bj21 = + 0.0111820609;
    const double    bj22 = + 0.9999374784;
    const double    bj23 = - 0.0000271765;
    const double    bj31 = + 0.0048579479;
    const double    bj32 = - 0.0000271474;
    const double    bj33 = + 0.9999881997;

    /*  Coefficients of E-terms. */
    const double    et1 = -1.62557e-6;
    const double    et2 = -0.31919e-6;
    const double    et3 = -0.13843e-6;

    double          ra, r1, r2, r3, t1, t2, t3, w, cd;

#define posmod(x,y,r)  if ((r=fmod(x,y)) < 0.0) r += y

    /*  Convert input polar coordinates to rectangular coordinates. */
    cd = cos(tlat);
    r1 = cos(tlong)*cd;
    r2 = sin(tlong)*cd;
    r3 = sin(tlat);

    if (mode >= 5) {
      /* Convert B1950 position to J2000.
      Reference:  Aoki,S., et al, 1983. Astron.Astrophys.,128, p263.
      Allow for e-terms. */
      w  = r1*et1 + r2*et2 + r3*et3;
      t1 = r1 - et1 + w*r1;
      t2 = r2 - et2 + w*r2;
      t3 = r3 - et3 + w*r3;
      /* Precess from B1950 to J2000. */
      r1 = bj11*t1 + bj12*t2 + bj13*t3;
      r2 = bj21*t1 + bj22*t2 + bj23*t3;
      r3 = bj31*t1 + bj32*t2 + bj33*t3;
    }

    if (mode >= 4) {
      /* Precess J2000 position to date. */
      t1 = pmat11*r1 + pmat12*r2 + pmat13*r3;
      t2 = pmat21*r1 + pmat22*r2 + pmat23*r3;
      t3 = pmat31*r1 + pmat32*r2 + pmat33*r3;
      r1 = t1; r2 = t2; r3 = t3;
    }

    if (mode >= 3) {
      /* Convert to geometrical equatorial coords
      by adding the aberration vector (denormalises the r*). */
      r1 += vonc1;
      r2 += vonc2;
      r3 += vonc3;
      /* Convert to polar coordinates. */
      ra = atan2(r2,r1);
      *dc = asin(r3 / sqrt(r1*r1 + r2*r2 + r3*r3));
      /* Convert to ha/dc. */
      posmod(sidtim*twopi - ra , twopi, *ha);
      /* Convert to back rectangular coordinates. */
      cd = cos(*dc);
      r1 = cos(*ha)*cd;
      r2 = sin(*ha)*cd;
      r3 = sin(*dc);
    }

    if (mode >= 2) {
      /* Convert to az,el. */
      t1 = etoa11*r1 + etoa12*r2 + etoa13*r3;
      t2 = etoa21*r1 + etoa22*r2 + etoa23*r3;
      t3 = etoa31*r1 + etoa32*r2 + etoa33*r3;
      r1 = t1; r2 = t2; r3 = t3;
    }

    if (mode >= 1) {
      /* Convert to polar coordinates. */
      posmod(atan2(r2,r1), twopi, *az);
      *el = asin(r3 / sqrt(r1*r1 + r2*r2 + r3*r3));
      /* Correct elevation for refraction.  This calculation gives *el as a
      non-decreasing function of *el with continuous first derivative. */
      t1 = (*el > 0.0) ? 1.0 : -1.0;
      if ((*el=fabs(*el)) > trelsp) {
        if (*el <= elnodiff)
          *el += ref0 / tan(*el);
      }
      else
        *el = apelsp * (1.0 - pow(*el/trelsp-1.0,2));
      *el *= t1;
    }
}

void risetime(double ha, double dc, double latitude, double elevlim,
              double *risetim, double *uptim, double *riseaz)
    /*
    Subroutine to compute the time to rising of a source whose current
    position is (ha,dc).  Also computed are the period that the source
    is up and the azimuth at rising.
    Call:
          risetime(ha,dc,latitude,elevlim,&risetim,&uptim,&riseaz);
    where:
       ha           is the source hour angle in radians
       dc           is the source declination (date) in radians
       latitude     is the observatory latitude in radians
       elevlim      is the low elevation limit in radians
       risetim      receives the time to rising in days, negative
                    if the source is already up
       uptim        receives the source up-time in days
       riseaz       receives the azimuth of rising in radians.
    Error return:
       uptim ==  0.0   if source never rises (risetim == 0.0)
       uptim ==  1.0   if source is circumpolar (risetim == 0.0)
       uptim == -1.0   if elevation limit is too small for refraction
                          calculation to be correct.
    */
{
    const double    solsid  = 1.002737909350795;

    double          cdec, sdec, clat, slat, x, y;

#define posmod(x,y,r)  if ((r=fmod(x,y)) < 0.0) r += y

    cdec = cos(dc);       sdec = sin(dc);
    clat = cos(latitude); slat = sin(latitude);

    /*  Convert elevation limit, which is apparent, to true elevation.
    Here we assume tan(true el) is approximated by tan(apparent el). */
    if (elevlim < apelsp) {
      *uptim = -1.0;
      return;
    }
    if (elevlim <= elnodiff) elevlim -= ref0 / tan(elevlim);

    /* Find the hour angle of the source when rising.  Check whether
    observer is near a pole, source is near a pole, source is
    circumpolar, or source is never up. */
    x = sin(elevlim) - slat*sdec;
    y = clat*cdec;
    if (y < 1.0e-15 || fabs(x /= y) >= 1.0) {
      *risetim = 0.0;
      *uptim = (x >= 0.0) ? 0.0 : 1.0;
      return;
    }

    /* Now we have x = cos(ha at rising).  Find the azimuth at rising. */
    y = (clat*sdec - slat*cdec*x) / cos(elevlim);
    if (y >= 1.0)
      *riseaz = 0.0;
    else if (y <= -1.0)
      *riseaz = pi;
    else
      *riseaz = acos(y);

    /* Find the hour angle at rising and the up time in solar days. */
    x = twopi - (*uptim = acos(x));
    *uptim /= 0.5*twopi*solsid;

    /* Find rising time in solar days, negative if the source currently up. */
    posmod(ha,twopi,ha);
    *risetim = x - ha;
    if (twopi - ha > x) *risetim -= twopi;
    *risetim /= twopi*solsid;
}
