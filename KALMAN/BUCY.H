#include <time.h>
#define width 0.7
#define w 0.2618
extern void calc_model(clock_t t,double *a1dimMatrixC);

/* prototypes */

extern void a2a1dimMatprod(double *a2dimMatrixA,double *a1dimMatrixB,
													double *a1dimMatrixC);

extern void a1a2dimMatprod(double *a1dimMatrixA,double *a2dimMatrixB,
													double *a1dimMatrixC);

extern void colrowMatprod(double *col,double *row,
													double *a2dimMatrixC);

extern void rowcolMatprod(double *row,double *col,
													double *number);

extern void colcolMatsum(double *colA,double *colB,
													double *colC);

extern void scalcolMatprod(double *scal,double *colB,
													double *colC);

extern void a2a2dimMatsum(double *a2dimMatrixA,double *a2dimMatrixB,
													double *a2dimMatrixC);

 void IdMatrix(double *a2dimMatrixC);

 void a2IdMatprod (double *a2dimMatrixA, double *a2dimMatrixC);
/***************** SEE TRACK2.C and SEQU.C********************************/
