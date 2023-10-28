#include "LMLib.h"
int LMSolve(LMSolveS* LMS,Matrix_X * Ans)
{
	//Matrix_X *y_est,*y_est_lm,*d,*H,*dp,*X_lm;
	//MathUnit e,e_lm;
	Matrix_X * Temp1,*Temp2,*Temp3;
	Matrix_X *A,*Hlm,*fp,* J, *Jt,*g;
	unsigned char  found;
	MathUnit v, u, Hlm_Dot, X_Dot, p;
	LMS->Code = -1;
	if(LMS==NULL)
		return -1;
	/*if(LMS->HFunc==NULL)
		return NULL;*/
	if(LMS->OFunc==NULL)
		return -1;
	if(LMS->JFunc==NULL)
		return -1;
	if(LMS->epend<=0)
		return -1;
	if(LMS->dend <= 0)
		return -1;
	if(LMS->lamda<=1)
		return -1;
	if(LMS->n_iters<=0)
		return -1;

	J = Jt = A = g = fp = Hlm=NULL;// = Errp = Errp_new = NULL;
	Temp1=Temp2=Temp3 = NULL;
	LMS->it = 0;
	v = LMS->lamda;
	J = LMS->JFunc(LMS->X,LMS->Para);
    //PrintMatrix(J,"J");
    //PrintMatrix(LMS->X,"X");
    //exit(0);
	if (J == NULL)
	{
		LMS->Code = -2;
		return -2;
	}
	Jt = MatrixT(J);
	A = MatrixMux(Jt, J);
	fp = LMS->OFunc(LMS->X,LMS->Para);
	if (fp == NULL)
	{
		LMS->Code = -3;
		FreeMatrix(J);
		FreeMatrix(A);
		FreeMatrix(Jt);
		return -3;
	}
    //PrintMatrix(fp,"fp");
    //PrintMatrix(Jt,"Jt");
	g = MatrixMux(Jt, fp);
	u = MaxiItem(A)*0.001;
    //PrintMatrix(g,"g");
	if ((DotProduct(g, g) < LMS->epend))
		found = 1;
	else
		found = 0;
	while ((!found) && (LMS->it < LMS->n_iters))
	{
		LMS->it++;
		Temp1 = CreatUnitM(A->Height);
		Temp2 = MatrixMuxF(Temp1, u);
		FreeMatrix(Temp1);
		Temp3 = MatrixAdd(A, Temp2);
		FreeMatrix(Temp2);
		Temp1 = MatrixInv(Temp3);
        if (Temp1 == NULL)
        {
            PrintMatrix(Temp3,"Temp3");
            Temp1 = MatrixInv(Temp3);
        }
		FreeMatrix(Temp3);
		if (Temp1 == NULL)
		{
			LMS->Code = -4;
			break;
		}
		Temp2 = MatrixMuxF(g, -1);

		if (Hlm != NULL)
			FreeMatrix(Hlm);

		Hlm = MatrixMux(Temp1, Temp2);
		FreeMatrix(Temp1);
		FreeMatrix(Temp2);

        //PrintMatrix(Hlm,"Hlm");
		Hlm_Dot = DotProduct(Hlm, Hlm);
        //Hlm_Dot=sqrt(Hlm_Dot);
		X_Dot = DotProduct(LMS->X, LMS->X);
        //X_Dot=sqrt(X_Dot);
		if (Hlm_Dot < LMS->dend*(X_Dot + LMS->dend))
		{
		    //PrintMatrix(Jt,"Jt");
            fp = LMS->OFunc(LMS->X,LMS->Para);
            //PrintMatrix(LMS->X,"X");
            //PrintMatrix(fp,"fp");
			found = 2;
		}
		else
		{
			Matrix_X * X_new,  *Fp_new;//*Errp_new,
			MathUnit Errp_Dot,Errp_new_Dot,Temp;
			//PrintMatrix(Hlm,"Hlm");
			X_new = MatrixAdd(LMS->X, Hlm);
			Errp_Dot = DotProduct(fp, fp);
			//printf("Errp_Dot %f\r\n",Errp_Dot);

			Fp_new= LMS->OFunc(X_new,LMS->Para);
			if (Fp_new == NULL)
			{
				FreeMatrix(LMS->X);
				LMS->X = X_new;
				LMS->Code = -3;
				break;
			}
            //PrintMatrix(X_new,"X_new");
            //PrintMatrix(Fp_new,"Fp_new");
			Errp_new_Dot = DotProduct(Fp_new, Fp_new);
            //printf("Errp_new_Dot %f\r\n",Errp_new_Dot);
			Temp1 = MatrixMuxF(Hlm, u);
			Temp2 = MatrixSub(Temp1, g);
			FreeMatrix(Temp1);
			Temp = DotProduct(Hlm, Temp2)/2;
			FreeMatrix(Temp2);


			p = (Errp_Dot - Errp_new_Dot) / (Temp);
            //printf("p= %f\r\n",p);
			if (p > 0)
			{
				FreeMatrix(LMS->X);
				LMS->X = X_new;
				FreeMatrix(J);
				J = LMS->JFunc(LMS->X,LMS->Para);
				if (J == NULL)
				{
					FreeMatrix(LMS->X);
					LMS->X = X_new;
					LMS->Code = -2;
					break;
				}
				FreeMatrix(Jt);
				Jt = MatrixT(J);
				FreeMatrix(A);
				A = MatrixMux(Jt, J);
				FreeMatrix(fp);
				fp = Fp_new;
				FreeMatrix(g);
				g = MatrixMux(Jt, fp);
				if ((DotProduct(g, g) < LMS->epend))
				{
					found = 1;

				}

				Temp = 2 * p - 1;
				Temp = Temp * Temp*Temp;
				Temp = 1 - Temp;
				if (Temp < (1.0 / 3))
					Temp = 1.0 / 3;

				u = u * Temp;
				v = LMS->lamda;
				//printf("Accept\r\n");
			}
			else
			{
				FreeMatrix(X_new);
				FreeMatrix(Fp_new);
				u = u * v;
				v = v * 2;
				//printf("No Accept\r\n");
			}
		}

	}

	if (found == 0)
	{
		if (LMS->it >= LMS->n_iters)
		{
			LMS->Code = -5;
		}
	}
	else
	{
		LMS->Code = found;
	}
	if(Ans!=NULL)
		memcpy(Ans->data,LMS->X->data,sizeof(MathUnit)*LMS->X->Height);

	if (A != NULL)
		FreeMatrix(A);
	if (Hlm != NULL)
		FreeMatrix(Hlm);
	if (fp != NULL)
		FreeMatrix(fp);
	if (J != NULL)
		FreeMatrix(J);
	if (Jt != NULL)
		FreeMatrix(Jt);
	if (g != NULL)
		FreeMatrix(g);


	return LMS->Code;
}

LMSolveS * NewLMSolve()
{
	LMSolveS * LMS;
	LMS = (LMSolveS * ) calloc(1,sizeof(LMSolveS));
	return LMS;

}
void FreeLMSolve(LMSolveS* LMS)
{
	if(LMS==NULL)
		return;

	FreeMatrix(LMS->X);
	FreeMatrix(LMS->output);

	free(LMS);
	return;
}
MathUnit  Neville (Matrix_X * jacobianGroup[],MathUnit dGroup[],MathUnit in,int x,int y,int i,int j)
{
    if(i==j)
    {
        return GetMatrixItem(jacobianGroup[i],x,y);
        //return jacobianGroup[i]->data[y];
    }else
    {
        double delta1=dGroup[j]-in;
        double delta2=in-dGroup[i];
        double delta3=dGroup[j]-dGroup[i];
        double n1=Neville(jacobianGroup,dGroup,in,x,y,i,j-1);
        double n2=Neville(jacobianGroup,dGroup,in,x,y,i+1,j);
        return (delta1*n1+delta2*n2)/delta3;
    }
}
#define NEV_POWER (4)
Matrix_X * Jacobian(Matrix_X * X,OBJ_FUNC func,void * para)
{
    Matrix_X * X_Up;
    Matrix_X * X_Down;
    Matrix_X * jacobian;
    Matrix_X * jacobianGroup[NEV_POWER];

    double d=0.0001;
    double dGroup[NEV_POWER];
    X_Up=MatrixCopy(X);
    X_Down=MatrixCopy(X);
    jacobian=NULL;
    for(int i=0;i<NEV_POWER;i++)
    {
        dGroup[i]=d;
        for(int n=0;n<X->Height;n++)
        {
            X_Up->data[n]=X->data[n]+d;
            X_Down->data[n]=X->data[n]-d;
            Matrix_X *  F_Up=func(X_Up, para);
            Matrix_X *  F_Down=func(X_Down, para);
            if((F_Up==NULL)&&(F_Down==NULL))
            {
                if(jacobian!=NULL)
                    FreeMatrix(jacobian);
                return NULL;
            }
            if(n==0)
            {
                jacobianGroup[i]=CreateMatrix(X->Height,F_Up->Height);
            }
            for(int j=0;j<F_Up->Height;j++)
            {
               //printf("Up %.8f Down %.8f d %.8f\r\n",F_Up->data[j],F_Down->data[j],(F_Up->data[j]-F_Down->data[j])/(d*2));
                SetMatrixItem(jacobianGroup[i],n,j,(F_Up->data[j]-F_Down->data[j])/(d*2));
            }
            X_Up->data[n]=X->data[n];
            X_Down->data[n]=X->data[n];
            FreeMatrix(F_Up);
            FreeMatrix(F_Down);
        }
        d=d*4;
        //PrintMatrix(jacobianGroup[i],"jacobianGroup");
    }
    jacobian=CreateMatrix(jacobianGroup[0]->Width,jacobianGroup[0]->Height);
    //PrintMatrix(jacobianGroup,"jacobianGroup");
    for(int x=0;x<jacobian->Width;x++)
        for(int y=0;y<jacobian->Height;y++)
        {
            double v=Neville(jacobianGroup,dGroup,0,x,y,0,NEV_POWER-1);
            SetMatrixItem(jacobian,x,y,v);
        }
    FreeMatrix(X_Up);
    FreeMatrix(X_Down);
    for(int i=0;i<NEV_POWER;i++)
    {
        FreeMatrix(jacobianGroup[i]);
    }
    return jacobian;
}