#include <time.h>

extern void kalman_sm(unsigned int *atcustate,double *azcmd,double *elcmd,
							double *azmsd,double *elmsd,double *power,
							clock_t timer,clock_t *start_time);

extern void hillclimbing_sm(unsigned int *atcustate,double *azcmd,double *elcmd ,
									double *azmsd, double *elmsd,double *power,
									unsigned int *kalmanstate,double *mesaz,
									double *mesel);


/* prototypes */

extern void Matcolprod(double *MatrixA,double *colB,
													double *colC);

extern void rowMatprod(double *rowA,double *MatrixB,
													double *rowC);

extern void colrowprod(double *col,double *row,
													double *MatrixC);

extern void rowcolprod(double *row,double *col,
													double *number);

extern void columnsum(double *colA,double *colB,
													double *colC);

extern void scalcolprod(double *scal,double *colB,
													double *colC);

extern void Matrixsum(double *MatrixA,double *MatrixB,
													double *MatrixC);

extern void IdMatrix(double *MatrixC);


/***************** SEE KALMAN.C and SEQ.C********************************/
