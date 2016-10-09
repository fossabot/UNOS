/* matrices procedures needed with the kalman filter algorithm*/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "kalman.h"

 void Matcolprod(double *MatrixA,double *colB,
														double *colC)
{
	colC[0] = MatrixA[0]*colB[0] + MatrixA[1]*
			colB[1] + MatrixA[2]*colB[2] + MatrixA[3]*
			colB[3] + MatrixA[4]*colB[4] + MatrixA[5]*
			colB[5];

	colC[1] = MatrixA[6]*colB[0] + MatrixA[7]*
			colB[1] + MatrixA[8]*colB[2] + MatrixA[9]*
			colB[3] + MatrixA[10]*colB[4] + MatrixA[11]*
			colB[5];

	colC[2] = MatrixA[12]*colB[0] + MatrixA[13]*
			colB[1] + MatrixA[14]*colB[2] + MatrixA[15]*
			colB[3] + MatrixA[16]*colB[4] + MatrixA[17]*
			colB[5];

	colC[3] = MatrixA[18]*colB[0] + MatrixA[19]*
			colB[1] + MatrixA[20]*colB[2] + MatrixA[21]*
			colB[3] + MatrixA[22]*colB[4] + MatrixA[23]*
			colB[5];

	colC[4] = MatrixA[24]*colB[0] + MatrixA[25]*
			colB[1] + MatrixA[26]*colB[2] + MatrixA[27]*
			colB[3] + MatrixA[28]*colB[4] + MatrixA[29]*
			colB[5];

	colC[5] = MatrixA[30]*colB[0] + MatrixA[31]*
			colB[1] + MatrixA[32]*colB[2] + MatrixA[33]*
			colB[3] + MatrixA[34]*colB[4] + MatrixA[35]*
			colB[5];
}

 void rowMatprod(double *rowA,double *MatrixB,
														double *rowC)
{
	rowC[0] = rowA[0]*MatrixB[0] + rowA[1]*
			MatrixB[6] + rowA[2]*MatrixB[12] + rowA[3]*
			MatrixB[18] + rowA[4]*MatrixB[24] + rowA[5]*
			MatrixB[30];

	rowC[1] = rowA[0]*MatrixB[1] + rowA[1]*
			MatrixB[7] + rowA[2]*MatrixB[13] + rowA[3]*
			MatrixB[19] + rowA[4]*MatrixB[25] + rowA[5]*
			MatrixB[31];

	rowC[2] = rowA[0]*MatrixB[2] + rowA[1]*
			MatrixB[8] + rowA[2]*MatrixB[14] + rowA[3]*
			MatrixB[20] + rowA[4]*MatrixB[26] + rowA[5]*
			MatrixB[32];

	rowC[3] = rowA[0]*MatrixB[3] + rowA[1]*
			MatrixB[9] + rowA[2]*MatrixB[15] + rowA[3]*
			MatrixB[21] + rowA[4]*MatrixB[27] + rowA[5]*
			MatrixB[33];

	rowC[4] = rowA[0]*MatrixB[4] + rowA[1]*
			MatrixB[10] + rowA[2]*MatrixB[16] + rowA[3]*
			MatrixB[22] + rowA[4]*MatrixB[28] + rowA[5]*
			MatrixB[34];

	rowC[5] = rowA[0]*MatrixB[5] + rowA[1]*
			MatrixB[11] + rowA[2]*MatrixB[17] + rowA[3]*
			MatrixB[23] + rowA[4]*MatrixB[29] + rowA[5]*
			MatrixB[35];
}

 void colrowprod(double *col,double *row,
													double *MatrixC)

{
	MatrixC[0]= col[0]*row[0];
	MatrixC[1]= col[0]*row[1];
	MatrixC[2]= col[0]*row[2];
	MatrixC[3]= col[0]*row[3];
	MatrixC[4]= col[0]*row[4];
	MatrixC[5]= col[0]*row[5];
	MatrixC[6]= col[1]*row[0];
	MatrixC[7]= col[1]*row[1];
	MatrixC[8]= col[1]*row[2];
	MatrixC[9]= col[1]*row[3];
	MatrixC[10]= col[1]*row[4];
	MatrixC[11]= col[1]*row[5];
	MatrixC[12]= col[2]*row[0];
	MatrixC[13]= col[2]*row[1];
	MatrixC[14]= col[2]*row[2];
	MatrixC[15]= col[2]*row[3];
	MatrixC[16]= col[2]*row[4];
	MatrixC[17]= col[2]*row[5];
	MatrixC[18]= col[3]*row[0];
	MatrixC[19]= col[3]*row[1];
	MatrixC[20]= col[3]*row[2];
	MatrixC[21]= col[3]*row[3];
	MatrixC[22]= col[3]*row[4];
	MatrixC[23]= col[3]*row[5];
	MatrixC[24]= col[4]*row[0];
	MatrixC[25]= col[4]*row[1];
	MatrixC[26]= col[4]*row[2];
	MatrixC[27]= col[4]*row[3];
	MatrixC[28]= col[4]*row[4];
	MatrixC[29]= col[4]*row[5];
	MatrixC[30]= col[5]*row[0];
	MatrixC[31]= col[5]*row[1];
	MatrixC[32]= col[5]*row[2];
	MatrixC[33]= col[5]*row[3];
	MatrixC[34]= col[5]*row[4];
	MatrixC[35]= col[5]*row[5];

}

 void rowcolprod(double *row,double *col,
													double *number)

{
	number[0] = row[0] * col[0]+ row[1] * col[1]+row[2] * col[2]+row[3] * col[3]+
				row[4] * col[4]+row[5] * col[5];

}

 void columnsum(double *colA,double *colB,
													double *colC)

{
	colC[0]=colA[0]+colB[0];
	colC[1]=colA[1]+colB[1];
	colC[2]=colA[2]+colB[2];
	colC[3]=colA[3]+colB[3];
	colC[4]=colA[4]+colB[4];
	colC[5]=colA[5]+colB[5];

}

 void scalcolprod(double *scal,double *colB,
													double *colC)

{
	colC[0]= scal[0] * colB[0];
	colC[1]= scal[0] * colB[1];
	colC[2]= scal[0] * colB[2];
	colC[3]= scal[0] * colB[3];
	colC[4]= scal[0] * colB[4];
	colC[5]= scal[0] * colB[5];

}

 void Matrixsum(double *MatrixA,double *MatrixB,
													double *MatrixC)

{

   MatrixC[0]= MatrixA[0]+MatrixB[0];
	MatrixC[1]= MatrixA[1]+MatrixB[1];
	MatrixC[2]= MatrixA[2]+MatrixB[2];
	MatrixC[3]= MatrixA[3]+MatrixB[3];
	MatrixC[4]= MatrixA[4]+MatrixB[4];
	MatrixC[5]= MatrixA[5]+MatrixB[5];
	MatrixC[6]= MatrixA[6]+MatrixB[6];
	MatrixC[7]= MatrixA[7]+MatrixB[7];
	MatrixC[8]= MatrixA[8]+MatrixB[8];
	MatrixC[9]= MatrixA[9]+MatrixB[9];
	MatrixC[10]= MatrixA[10]+MatrixB[10];
	MatrixC[11]= MatrixA[11]+MatrixB[11];
	MatrixC[12]= MatrixA[12]+MatrixB[12];
	MatrixC[13]= MatrixA[13]+MatrixB[13];
	MatrixC[14]= MatrixA[14]+MatrixB[14];
	MatrixC[15]= MatrixA[15]+MatrixB[15];
	MatrixC[16]= MatrixA[16]+MatrixB[16];
	MatrixC[17]= MatrixA[17]+MatrixB[17];
	MatrixC[18]= MatrixA[18]+MatrixB[18];
	MatrixC[19]= MatrixA[19]+MatrixB[19];
	MatrixC[20]= MatrixA[20]+MatrixB[20];
	MatrixC[21]= MatrixA[21]+MatrixB[21];
	MatrixC[22]= MatrixA[22]+MatrixB[22];
	MatrixC[23]= MatrixA[23]+MatrixB[23];
	MatrixC[24]= MatrixA[24]+MatrixB[24];
	MatrixC[25]= MatrixA[25]+MatrixB[25];
	MatrixC[26]= MatrixA[26]+MatrixB[26];
	MatrixC[27]= MatrixA[27]+MatrixB[27];
	MatrixC[28]= MatrixA[28]+MatrixB[28];
	MatrixC[29]= MatrixA[29]+MatrixB[29];
	MatrixC[30]= MatrixA[30]+MatrixB[30];
	MatrixC[31]= MatrixA[31]+MatrixB[32];
	MatrixC[32]= MatrixA[32]+MatrixB[32];
	MatrixC[33]= MatrixA[33]+MatrixB[33];
	MatrixC[34]= MatrixA[34]+MatrixB[34];
	MatrixC[35]= MatrixA[35]+MatrixB[35];

}

 void IdMatrix( double *MatrixC)
{
	MatrixC[0]= 1;
	MatrixC[1]= 0;
	MatrixC[2]= 0;
	MatrixC[3]= 0;
	MatrixC[4]= 0;
	MatrixC[5]= 0;
	MatrixC[6]= 0;
	MatrixC[7]= 1;
	MatrixC[8]= 0;
	MatrixC[9]= 0;
	MatrixC[10]= 0;
	MatrixC[11]= 0;
	MatrixC[12]= 0;
	MatrixC[13]= 0;
	MatrixC[14]= 1;
	MatrixC[15]= 0;
	MatrixC[16]= 0;
	MatrixC[17]= 0;
	MatrixC[18]= 0;
	MatrixC[19]= 0;
	MatrixC[20]= 0;
	MatrixC[21]= 1;
	MatrixC[22]= 0;
	MatrixC[23]= 0;
	MatrixC[24]= 0;
	MatrixC[25]= 0;
	MatrixC[26]= 0;
	MatrixC[27]= 0;
	MatrixC[28]= 1;
	MatrixC[29]= 0;
	MatrixC[30]= 0;
	MatrixC[31]= 0;
	MatrixC[32]= 0;
	MatrixC[33]= 0;
	MatrixC[34]= 0;
	MatrixC[35]= 1;
}

 
