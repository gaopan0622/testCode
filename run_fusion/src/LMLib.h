#ifndef LMLIB_H
#define LMLIB_H

#include "matrix/Matrix_XTJ.h"
#ifdef __cplusplus
extern "C" {
#endif
typedef Matrix_X * (* OBJ_FUNC)(Matrix_X *,void * );
typedef Matrix_X * (* JAC_FUNC)(Matrix_X *,void * );
typedef struct _LMSolveS
{
	Matrix_X * X;//初始数据
    JAC_FUNC JFunc;//雅克比矩阵
	//Matrix_X *(* HFunc)(struct _LMSolveS * ,Matrix_X *);//伪黑森矩阵
    OBJ_FUNC OFunc;//目标函数

	double lamda;//阻尼 >1
	//double step;
	double epend ;//结束梯度条件，注意此处为梯度的平方 >0
	double dend;//结束增量条件，注意此处为平方 >0
	int    n_iters;//迭代总次数 >1
	int    Code;//优化状态：1、达到梯度条件，2、达到增量条件，\
				-1、参数错误,-2,梯度计算错误，-3目标计算错误,-4迭代发散,-5、超过预订迭代次数，
	int    it;//迭代次数
	Matrix_X *output;//输出数据,注意算法不会自动减去该数据，如果该数据不为0\
						需要在OFunc函数中相应减去。
	void *  Para;//参数指针

}LMSolveS;
LMSolveS * NewLMSolve();
int LMSolve(LMSolveS* LMS,Matrix_X * Ans);
void FreeLMSolve(LMSolveS* LMS);
Matrix_X * Jacobian(Matrix_X * X,OBJ_FUNC func,void * para);
#ifdef __cplusplus
}
#endif
#endif