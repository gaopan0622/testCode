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
	Matrix_X * X;//��ʼ����
    JAC_FUNC JFunc;//�ſ˱Ⱦ���
	//Matrix_X *(* HFunc)(struct _LMSolveS * ,Matrix_X *);//α��ɭ����
    OBJ_FUNC OFunc;//Ŀ�꺯��

	double lamda;//���� >1
	//double step;
	double epend ;//�����ݶ�������ע��˴�Ϊ�ݶȵ�ƽ�� >0
	double dend;//��������������ע��˴�Ϊƽ�� >0
	int    n_iters;//�����ܴ��� >1
	int    Code;//�Ż�״̬��1���ﵽ�ݶ�������2���ﵽ����������\
				-1����������,-2,�ݶȼ������-3Ŀ��������,-4������ɢ,-5������Ԥ������������
	int    it;//��������
	Matrix_X *output;//�������,ע���㷨�����Զ���ȥ�����ݣ���������ݲ�Ϊ0\
						��Ҫ��OFunc��������Ӧ��ȥ��
	void *  Para;//����ָ��

}LMSolveS;
LMSolveS * NewLMSolve();
int LMSolve(LMSolveS* LMS,Matrix_X * Ans);
void FreeLMSolve(LMSolveS* LMS);
Matrix_X * Jacobian(Matrix_X * X,OBJ_FUNC func,void * para);
#ifdef __cplusplus
}
#endif
#endif