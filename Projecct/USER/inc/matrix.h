#ifndef _MATRIX_H
#define _MATRIX_H


void Matrix_Zero(unsigned short numRows, unsigned short numCols, float *A);
void Matrix_Copy(float *pSrc, unsigned short numRows, unsigned short numCols, float *pDst);
void Maxtrix_Add(float *pSrcA, float *pSrcB, unsigned short numRows, unsigned short numCols, float *pDst);
void Maxtrix_Sub(float *pSrcA, float *pSrcB, unsigned short numRows, unsigned short numCols, float *pDst);
void Matrix_Multiply(float* pSrcA, unsigned short numRowsA, unsigned short numColsA, float* pSrcB, unsigned short numColsB, float* pDst);
void Maxtrix_Transpose(float *pSrc, unsigned short nRows, unsigned short nCols, float *pDst);
int Matrix_Inverse(float * pSrc, unsigned short n, float* pDst);
void Matrix_Multiply_With_Transpose(float *A, unsigned short nrows, unsigned short ncols, float *B, unsigned short mrows, float *C);


#endif


