#include "EKF.h"
bool EKF(Matrix_X * X, Matrix_X * Z,PTREKFPHI GetPhi,PTREKFH GetH, PTREKFSPE SpeFuc,PTREKFSPEZ SpeZ,Matrix_X * P,Matrix_X * Q,Matrix_X * R,void * para)
{
	if((X==NULL)||(Z==NULL)||(GetPhi==NULL)||(GetH==NULL)||(SpeFuc==NULL)||(P==NULL)||(Q==NULL)||(R==NULL))
		return false;
	        Matrix_X *EKFPhi,*EKFPhiT,*EKFPspe,*EKFHT,\
                *EKFK,*EKFXspe,*EKFH,*EKFX,*EKFP;
        Matrix_X * temp0,*temp1,*temp2,*temp3;


        //P=Phi P PhiT + Q
        if((EKFPhi=GetPhi(X,para))==NULL)
        {
            printf("ERR ： P=Phi P PhiT + Q 1 \n");
        	return false;
        }
        EKFPhiT=MatrixT(EKFPhi);

        if((temp0=MatrixMux(EKFPhi,P))==NULL)
        {
            printf("ERR ： P=Phi P PhiT + Q 2 \n");
        	FreeMatrix(EKFPhiT);
        	FreeMatrix(EKFPhi);
        	return false;
        }
        if((temp1=MatrixMux(temp0,EKFPhiT))==NULL)
        {
            printf("ERR ： P=Phi P PhiT + Q 3 \n");
        	FreeMatrix(temp0);
        	FreeMatrix(EKFPhiT);
        	FreeMatrix(EKFPhi);
        	return false;        	
        }

        if((EKFPspe=MatrixAdd(temp1,Q))==NULL)
        {
            printf("ERR ： P=Phi P PhiT + Q 4 \n");
        	FreeMatrix(temp0);
        	FreeMatrix(temp1);
        	FreeMatrix(EKFPhiT);
        	FreeMatrix(EKFPhi);
        	return false; 
        }
        FreeMatrix(temp0);
        FreeMatrix(temp1);

        //K=P HT (H P HT +R)-1
        if((EKFH=GetH(X,para))==NULL)
        {
            printf("ERR ： K=P HT (H P HT +R)-1 1 \n");
        	FreeMatrix(EKFPhi);
        	FreeMatrix(EKFPhiT);
        	FreeMatrix(EKFPspe);
            return false;
        }
        if((EKFHT=MatrixT(EKFH))==NULL)
        {
            printf("ERR ： K=P HT (H P HT +R)-1 2 \n");
         	FreeMatrix(EKFPhi);
        	FreeMatrix(EKFPhiT);
        	FreeMatrix(EKFPspe);
        	FreeMatrix(EKFH);  
            return false;     	
        }
        if((temp0=MatrixMux(EKFPspe,EKFHT))==NULL)
        {
            printf("ERR ： K=P HT (H P HT +R)-1 3 \n");
        	FreeMatrix(EKFPhi);
        	FreeMatrix(EKFPhiT);
        	FreeMatrix(EKFPspe);
        	FreeMatrix(EKFH);            	
         	FreeMatrix(EKFHT);   
            return false;         	
        }
        temp1=MatrixMux(EKFH,temp0);

        if((temp2=MatrixAdd(temp1,R))==NULL)
        {
            printf("ERR ： K=P HT (H P HT +R)-1 4 \n");
        	FreeMatrix(EKFPhi);
        	FreeMatrix(EKFPhiT);
        	FreeMatrix(EKFPspe);
        	FreeMatrix(EKFH);
         	FreeMatrix(EKFHT);
         	FreeMatrix(temp0);
         	FreeMatrix(temp1);
         	return false;
        }

        FreeMatrix(temp1);

        if((temp1=MatrixInv(temp2))==NULL)
        {
            //printf("ERR ： K=P HT (H P HT +R)-1 5 \n");
            EKFXspe=SpeFuc(X,EKFPhi,para);
            memcpy(X->data,EKFXspe->data,sizeof(MathUnit)*X->Height);
            FreeMatrix(EKFPhi);
            FreeMatrix(EKFPhiT);
            FreeMatrix(EKFPspe);
            FreeMatrix(EKFXspe);
            FreeMatrix(EKFH);
            FreeMatrix(EKFHT);
            FreeMatrix(temp0);
            FreeMatrix(temp2);
            return false;
        } 

        EKFK=MatrixMux(temp0,temp1);
        FreeMatrix(temp2);
        FreeMatrix(temp1);
        FreeMatrix(temp0);
        //PrintMatrix(EKFK,"EKFK");

        //X= Phi X;
        EKFXspe=SpeFuc(X,EKFPhi,para);
        //X = X + K(Z -z(X))
        temp0=SpeZ(EKFXspe,EKFH,para);
        if(temp0==NULL)
        {
            printf("ERR ： X = X + K(Z -z(X)) \n");
            FreeMatrix(EKFPhi);
            FreeMatrix(EKFPhiT);
            FreeMatrix(EKFPspe);
            FreeMatrix(EKFH);
            FreeMatrix(EKFHT);
            FreeMatrix(EKFXspe);
            return false;            
        }
        if((temp1=MatrixSub(Z,temp0))==NULL)
        {
            printf("ERR ： X = X + K(Z -z(X)) 2 \n");
        	FreeMatrix(EKFPhi);
        	FreeMatrix(EKFPhiT);
        	FreeMatrix(EKFPspe);
        	FreeMatrix(EKFH);
         	FreeMatrix(EKFHT);
         	FreeMatrix(temp0);
         	FreeMatrix(EKFXspe);
         	return false;
        }

        temp2=MatrixMux(EKFK,temp1);
        EKFX=MatrixAdd(EKFXspe,temp2);
        //EKFX=MatrixCopy(EKFXspe);
        //PrintMatrix(EKFX,"EKFX");
        //PrintMatrix(EKFXspe,"EKFXspe");

        memcpy(X->data,EKFX->data,sizeof(MathUnit)*X->Height);
        FreeMatrix(EKFX);

        FreeMatrix(temp0);
        FreeMatrix(temp1);
        FreeMatrix(temp2);

        //P =( I-K H )P (I - K H )T + K R KT;

        temp0=MatrixMux(EKFK,EKFH);
        temp1=CreatUnitM(temp0->Height);
        temp2=MatrixSub(temp1,temp0);
        FreeMatrix(temp0);
        temp0=MatrixT(temp2);
        FreeMatrix(temp1);
        temp1=MatrixMux(temp2,EKFPspe);
        FreeMatrix(temp2);
        temp3=MatrixMux(temp1,temp0);
        FreeMatrix(temp1);
        FreeMatrix(temp0);

        temp0=MatrixMux(EKFK,R);
        temp1=MatrixT(EKFK);
        temp2=MatrixMux(temp0,temp1);
        FreeMatrix(temp1);
        FreeMatrix(temp0);
        EKFP=MatrixAdd(temp3,temp2);
        memcpy(P->data,EKFP->data,sizeof(MathUnit)*P->Height*P->Width);
        FreeMatrix(EKFP);
        FreeMatrix(temp3);
        FreeMatrix(temp2);

        //P =( I-K H )P
        /*temp0=MatrixMux(EKFK,HAEKFH);
        temp1=CreatUnitM(temp0->Height);
        temp2=MatrixSub(temp1,temp0);
        FreeMatrix(HAEKFP);
        HAEKFP=MatrixMux(temp2,EKFPspe);
        FreeMatrix(temp2);
        FreeMatrix(temp1);
        FreeMatrix(temp0);*/

        FreeMatrix(EKFPhi);
        FreeMatrix(EKFPhiT);
        FreeMatrix(EKFPspe);
        FreeMatrix(EKFHT);
        FreeMatrix(EKFH);
        FreeMatrix(EKFK);
        FreeMatrix(EKFXspe);

        return true;
}

Matrix_X * CalJacobian(Matrix_X * X,PTREKFSPE func,void * para)
{
    Matrix_X * X_Up;
    Matrix_X * X_Down;
    Matrix_X * jacobian;
    double d=0.000000001;
    X_Up=MatrixCopy(X);
    X_Down=MatrixCopy(X);
    jacobian=NULL;
    for(int n=0;n<X->Height;n++)
    {
        X_Up->data[n]=X->data[n]+d;
        X_Down->data[n]=X->data[n]-d;
        Matrix_X  * F_Up=func(X_Up,NULL, para);
        Matrix_X  * F_Down=func(X_Down,NULL, para);
        if((F_Up==NULL)&&(F_Down==NULL))
        {
            if(jacobian!=NULL)
                FreeMatrix(jacobian);
            return NULL;
        }

        if(n==0)
        {
            jacobian=CreateMatrix(X->Height,F_Up->Height);
        }
        for(int i=0;i<F_Up->Height;i++)
        {
            SetMatrixItem(jacobian,n,i,(F_Up->data[i]-F_Down->data[i])/(d*2));
        }
        X_Up->data[n]=X->data[n];
        X_Down->data[n]=X->data[n];
        FreeMatrix(F_Up);
        FreeMatrix(F_Down);
    }
    FreeMatrix(X_Up);
    FreeMatrix(X_Down);
    return jacobian;
}