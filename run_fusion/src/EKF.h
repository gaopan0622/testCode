#ifndef EKF_H
#define EKF_H
#include <stdio.h>
#include <stdlib.h> 
#include "matrix/Matrix_XTJ.h"
#include <stdbool.h>
#ifdef __cplusplus
extern "C" {
#endif
typedef Matrix_X * (*PTREKFPHI) (Matrix_X *,void * );
typedef Matrix_X * (*PTREKFH) (Matrix_X *,void * ) ;
typedef Matrix_X * (*PTREKFSPE) (Matrix_X *,Matrix_X *,void * );
typedef Matrix_X * (*PTREKFSPEZ) (Matrix_X *,Matrix_X *,void * );
bool EKF(Matrix_X * X, Matrix_X * Z,PTREKFPHI GetPhi,PTREKFH GetH, PTREKFSPE SpeFuc,PTREKFSPEZ SpeZ,Matrix_X * P,Matrix_X * Q,Matrix_X * R,void * para);
Matrix_X * CalJacobian(Matrix_X * X,PTREKFSPE func,void * para);

#ifdef __cplusplus
}
#endif
#endif

/*
Matrix_X * GetHAPHiGPS (Matrix_X * X,void * para);
Matrix_X * GetHAHGPS (Matrix_X * X,void * para) ;
Matrix_X * GetXYPHiGPS (Matrix_X * X,void * para);
Matrix_X * GetXYHGPS (Matrix_X * X,void * para) ;
Matrix_X * GetHAPHi (Matrix_X * X,void * para);
Matrix_X * GetHAH (Matrix_X * X,void * para);
Matrix_X * GetXYPHi (Matrix_X * X,void * para);
Matrix_X * GetXYH (Matrix_X * X,void * para) ;
Matrix_X * GetHAX (Matrix_X * X,Matrix_X * Phi,void * para);
Matrix_X * GetX (Matrix_X * X,Matrix_X * Phi,void * para);
Matrix_X * GetHAXGPS (Matrix_X * X,Matrix_X * Phi,void * para);
Matrix_X * GetXGPS (Matrix_X * X,Matrix_X * Phi,void * para);
 */