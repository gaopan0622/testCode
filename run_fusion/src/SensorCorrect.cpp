
// Created by guimu on 2021/6/11.
//

#include "SensorCorrect.h"
#include "LMLib.h"
#include "LocationServerError.h"

#define MAX_TIME (99999999999999999)
#define PI M_PI

Matrix_X *correctIMUObj(Matrix_X *x, void *para);

Matrix_X *correctIMUJac(Matrix_X *x, void *para);

Matrix_X *correctVehJac(Matrix_X *x, void *para);

Matrix_X *correctVehObj(Matrix_X *x, void *para);

Matrix_X *correctVehHeadObj(Matrix_X *x, void *para);

Matrix_X *correctVehHeadJac(Matrix_X *x, void *para);

CorrectVector &imuUpdata(const CorrectVector &stateVector, double dTime, const double *wk, const double *wd,
                         const list<ImuDataDef>::iterator &imu, CorrectVector &preStateVector);

CorrectVector &
vehUpdata(const CorrectVector &stateVector, double dTime, double *T, VehDataDef *veh,
          CorrectVector &preStateVector);

SensorCorrect::SensorCorrect() {
    memset((void *) &mCorrectParam, 0, sizeof(mCorrectParam));
    mCorrectParam.imuZK = 1;
    mCorrectParam.vehK[0] = 1;
    mCorrectParam.vehK[4] = 1;
    mCorrectParam.vehK[8] = 1;

    mIMUIni = false;
    if(mParamService.getDouble("sensorCorrect/imuZK", mCorrectParam.imuZK))
    {
        imuZKIni=true;
        LOGINFO("[PARAM] sensorCorrect/imuZK %f",mCorrectParam.imuZK);
    } else{
        LOGWARN("[PARAM] sensorCorrect/imuZK error");
        mCorrectParam.imuZK=1;
        imuZKIni =false;
    }

    for(int n=0;n<3;n++)
    {
        string paramName;
        paramName="sensorCorrect/imuD"+to_string(n);
        if(mParamService.getDouble(paramName, mCorrectParam.imuD[n]))
        {
            imuDIni[n]=true;
            LOGINFO("[PARAM] %s %f",paramName.c_str(),mCorrectParam.imuD[n]);
        } else{
            imuDIni[n]=false;
            LOGWARN("[PARAM] %s error",paramName.c_str());
            mCorrectParam.imuD[n]=0;
        }
    }

    for(int n=0;n<9;n++)
    {
        string paramName;
        paramName="sensorCorrect/vehK"+to_string(n);
        if(mParamService.getDouble(paramName, mCorrectParam.vehK[n]))
        {
            vehKIni[n] =true;
            LOGINFO("[PARAM] %s %f",paramName.c_str(),mCorrectParam.vehK[n]);
        } else{
            if(n%4==0)
                mCorrectParam.vehK[n]=1;
            else
                mCorrectParam.vehK[n]=0;
            vehKIni[n] =false;

            LOGWARN("[PARAM] %s error",paramName.c_str());
        }
    }
    //mCorrectParam.vehK[2]=0.3;
}

void SensorCorrect::correctData(CorrectDataDef &data) {
    unsigned long imuTimeLast, vehTimeLast, imuTimeCur, vehTimeCur;

    //LOGINFO("[COR] correctDataFunc start %p",&data);
    checkData(data);
    CorrectIMU(data);
    CorrectVeh(data);
    CorrectVehHead(data);
    //LOGINFO("[COR] correctDataFunc end");

}

void SensorCorrect::correctPath(CorrectDataDef &data) {
/*
    checkData(data);
    CorrectTunnelTrajectory(data);
    */

}

void SensorCorrect::CorrectVehHead(CorrectDataDef &data) {
    LMSolveS *lmSolveS = NewLMSolve();
    int CorrectIndex = 0;
    lmSolveS->lamda = 4;
    lmSolveS->epend = 1e-10;
    lmSolveS->dend = 1e-20;
    lmSolveS->n_iters = 100;
    if (mVehXCorrect)
        CorrectIndex += 1;
    if (mVehYCorrect)
        CorrectIndex += 1;
    if (mVehWCorrect)
        CorrectIndex += 1;
    if (CorrectIndex == 0)
        return;
    if (CorrectIndex == 1 && mVehWCorrect)
        return;

    lmSolveS->X = CreateMatrix(1, CorrectIndex);

    CorrectIndex = 0;
    if (mVehXCorrect) {
        lmSolveS->X->data[CorrectIndex] = mCorrectParam.vehK[6];
        CorrectIndex++;
    }
    if (mVehYCorrect) {
        lmSolveS->X->data[CorrectIndex] = mCorrectParam.vehK[7];
        CorrectIndex++;
    }
    if (mVehWCorrect) {
        lmSolveS->X->data[CorrectIndex] = mCorrectParam.vehK[8];
        CorrectIndex++;
    }


    lmSolveS->JFunc = correctVehHeadJac;
    lmSolveS->OFunc = correctVehHeadObj;


    lmSolveS->Para = (void *) &data;
    data.show = false;
    data.mVehXCorrect = mVehXCorrect;
    data.mVehYCorrect = mVehYCorrect;
    data.mVehWCorrect = mVehWCorrect;
    data.correctParam = mCorrectParam;
    Matrix_X *fp = lmSolveS->OFunc(lmSolveS->X, lmSolveS->Para);
////cout<<"lmSolveS "<<fp<<endl;
//PrintMatrix(fp,"fp");
    FreeMatrix(fp);
    LMSolve(lmSolveS, NULL);


    //double p[12];
//int n,m;
//n=mCorrectData.gpsDataList.size()-1;
//m=12;
//double *x=(double*)calloc(n,sizeof(double));
//double opts[LM_OPTS_SZ], info[LM_INFO_SZ];
//opts[0]=LM_INIT_MU; opts[1]=1E-15; opts[2]=1E-15; opts[3]=1E-20;
//opts[4]= LM_DIFF_DELTA;
//int ret=dlevmar_dif(CorrectObj4M, p, x, m, n, 1000, opts, info, NULL, NULL, (void*)&mCorrectData); // no Jacobian, caller allocates work memory, covariance estimated
//free(x);
//PrintMatrix(lmSolveS->X,"X");

    LOGINFO("[COR] VehHead it %d  code %d",lmSolveS->it , lmSolveS->Code);

    if ((lmSolveS->it > 1) && (lmSolveS->Code > 0)) {
        //PrintMatrix(lmSolveS->X,"VehHead");


        Matrix_X * obj=lmSolveS->OFunc(lmSolveS->X,lmSolveS->Para);
        LOGINFO("[COR] VehHead obj %f",obj->data[0]);
        //PrintMatrix(obj,"objHead");
        if(obj->data[0]<1)
        {
            CorrectIndex = 0;
            if (mVehXCorrect) {
                setCorrectParam(lmSolveS->X->data[CorrectIndex], mCorrectParam.vehK[6], vehKIni[6], \

                                "sensorCorrect/vehK6", 0, 0.05, LocationServerError::VEH_UNUSUAL);
                CorrectIndex++;
            }
            if (mVehYCorrect) {

                setCorrectParam(lmSolveS->X->data[CorrectIndex], mCorrectParam.vehK[7], vehKIni[7], \

                                "sensorCorrect/vehK7", 0, 0.05, LocationServerError::VEH_UNUSUAL);
                CorrectIndex++;
            }
            if (mVehWCorrect) {
                setCorrectParam(lmSolveS->X->data[CorrectIndex], mCorrectParam.vehK[8], vehKIni[8], \

                                "sensorCorrect/vehK8", 0, 0.05, LocationServerError::VEH_UNUSUAL);
                CorrectIndex++;
            }
        }
        FreeMatrix(obj);
    }


    FreeLMSolve(lmSolveS);
}

void SensorCorrect::setCorrectParam(double correctValue, double &correctParam, bool &Initialized, string paramName,
                                    double normalValue, double threshold, int errorCode) {
    if(fabs(correctValue-normalValue)<threshold)
    {
        if(Initialized)
        {
            correctParam = correctParam * 0.9 + correctValue * 0.1;
        }
        else
        {
            Initialized=true;
            correctParam = correctValue ;
        }
        mParamService.setDouble(paramName, correctParam);
        LOGINFO("[COR] %s %f", paramName.c_str(), correctValue);
    } else{
        NodeErrorHandler::errorOccured(errorCode);
        LOGWARN("[COR] Unusual %s %f", paramName.c_str(), correctValue);
    }
}

void SensorCorrect::CorrectVeh(CorrectDataDef &data) {
    LMSolveS *lmSolveS = NewLMSolve();
    int CorrectIndex = 0;
    lmSolveS->lamda = 4;
    lmSolveS->epend = 1e-10;
    lmSolveS->dend = 1e-20;
    lmSolveS->n_iters = 100;
    if (mVehXCorrect)
        CorrectIndex += 2;
    if (mVehYCorrect)
        CorrectIndex += 2;
    if (mVehWCorrect)
        CorrectIndex += 2;
    if (CorrectIndex == 0)
        return;
    if (CorrectIndex == 2 && mVehWCorrect)
        return;

    lmSolveS->X = CreateMatrix(1, CorrectIndex);

    CorrectIndex = 0;
    if (mVehXCorrect) {
        lmSolveS->X->data[CorrectIndex] = mCorrectParam.vehK[0];
        CorrectIndex++;
        lmSolveS->X->data[CorrectIndex] = mCorrectParam.vehK[3];
        CorrectIndex++;
    }
    if (mVehYCorrect) {
        lmSolveS->X->data[CorrectIndex] = mCorrectParam.vehK[1];
        CorrectIndex++;
        lmSolveS->X->data[CorrectIndex] = mCorrectParam.vehK[4];
        CorrectIndex++;
    }
    if (mVehWCorrect) {
        //printf("vehK[2]%f\n",mCorrectParam.vehK[2]);
        lmSolveS->X->data[CorrectIndex] = mCorrectParam.vehK[2];
        CorrectIndex++;
        lmSolveS->X->data[CorrectIndex] = mCorrectParam.vehK[5];
        CorrectIndex++;
    }


    lmSolveS->JFunc = correctVehJac;
    lmSolveS->OFunc = correctVehObj;


    lmSolveS->Para = (void *) &data;
    data.show = false;
    data.mVehXCorrect = mVehXCorrect;
    data.mVehYCorrect = mVehYCorrect;
    data.mVehWCorrect = mVehWCorrect;
    data.correctParam = mCorrectParam;

    LMSolve(lmSolveS, NULL);


    //LOGINFO("[COR] VehPos  it %d code %d ",lmSolveS->it,lmSolveS->Code);
    //printf("[COR] VehPos  it %d code %d \n",lmSolveS->it,lmSolveS->Code);
    //if ((lmSolveS->it > 1) && (lmSolveS->Code > 0)) {
    //PrintMatrix(lmSolveS->X,"VehPos");
        Matrix_X * obj=lmSolveS->OFunc(lmSolveS->X,lmSolveS->Para);
        //PrintMatrix(obj,"objVeh");
        //LOGINFO("[COR] VehPos obj %f",obj->data[0]);
        //printf("[COR] VehPos obj %f\n",obj->data[0]);
        if(obj->data[0]<1)
        {
            CorrectIndex = 0;
            if (mVehXCorrect) {
                setCorrectParam(lmSolveS->X->data[CorrectIndex], mCorrectParam.vehK[0], vehKIni[0], \

                                "sensorCorrect/vehK0", 1, 0.1, LocationServerError::VEH_UNUSUAL);
                CorrectIndex++;
                setCorrectParam(lmSolveS->X->data[CorrectIndex], mCorrectParam.vehK[3], vehKIni[3], \

                                "sensorCorrect/vehK3", 0, 0.05, LocationServerError::VEH_UNUSUAL);
                CorrectIndex++;

            }
            if (mVehYCorrect) {
                setCorrectParam(lmSolveS->X->data[CorrectIndex], mCorrectParam.vehK[1], vehKIni[1], \

                                "sensorCorrect/vehK1", 0, 0.05, LocationServerError::VEH_UNUSUAL);
                CorrectIndex++;
                setCorrectParam(lmSolveS->X->data[CorrectIndex], mCorrectParam.vehK[4], vehKIni[4], \

                                "sensorCorrect/vehK4", 1, 0.05, LocationServerError::VEH_UNUSUAL);
                CorrectIndex++;
            }
            if (mVehWCorrect) {
                setCorrectParam(lmSolveS->X->data[CorrectIndex], mCorrectParam.vehK[2], vehKIni[2], \

                                "sensorCorrect/vehK2", 0, 0.05, LocationServerError::VEH_UNUSUAL);
                CorrectIndex++;
                setCorrectParam(lmSolveS->X->data[CorrectIndex], mCorrectParam.vehK[5], vehKIni[5], \

                                "sensorCorrect/vehK5", 0, 0.05, LocationServerError::VEH_UNUSUAL);
                CorrectIndex++;
            }
        }
        FreeMatrix(obj);


    //}


    FreeLMSolve(lmSolveS);
}

void SensorCorrect::iniIMU(CorrectDataDef &data) {
    CorrectDataDef *correctData = (CorrectDataDef *)  &data;


    auto veh = correctData->vehDataList.begin();
    auto imu = correctData->imuDataList.begin();

    for (; imu != correctData->imuDataList.end(); imu++) {
        if (imu->time > veh->time) {
            break;
        }
    }
    veh++;
    while(veh!=correctData->vehDataList.end())
    {
        while(imu!=correctData->imuDataList.end())
        {
            if((veh->vxyw[0]*veh->vxyw[0]+veh->vxyw[1]*veh->vxyw[1]+veh->vxyw[2]*veh->vxyw[2])<1e-5)
            {
                IMU_Data correctData;
                correctData.data[0] = imu->Gyro[0];
                correctData.data[1] = imu->Gyro[1];
                correctData.data[2] = imu->Gyro[2];
                imuCorrectPreList.push_back(correctData);
                if (imuCorrectPreList.size() > 1000) {
                    int i = 0;
                    imuCorrectList.clear();
                    while ((i + 50) < imuCorrectPreList.size()) {
                        if (i > 50) {
                            imuCorrectList.push_back(imuCorrectPreList.front());
                        }
                        imuCorrectPreList.pop_front();
                        i++;
                    }
                    imuCorrectPreList.clear();
                    if (imuCorrectList.size() > 300) {
                        IMU_Data correctData;
                        IMU_Data correctDataI;
                        int counter;
                        counter = 0;
                        correctDataI.data[0] = correctDataI.data[1] = correctDataI.data[2] = 0;
                        for (auto i = imuCorrectList.begin(); i != imuCorrectList.end(); i++) {
                            correctData = *i;
                            for (int n = 0; n < 3; n++) {
                                correctDataI.data[n] += correctData.data[n];
                            }
                            counter++;
                        }
                        for (int n = 0; n < 3; n++)
                            correctDataI.data[n] = -correctDataI.data[n] / counter;

                        for(int n=0;n<3;n++)
                        {
                            string paramName;
                            paramName="sensorCorrect/imuD"+to_string(n);
                            setCorrectParam(correctDataI.data[n], mCorrectParam.imuD[n], imuDIni[n], paramName, \

                                            0, 0.05, LocationServerError::IMU_UNUSUAL);
                            //mParamService.setDouble(paramName, mCorrectParam.imuD[n]);
                        }
                        mIMUIni=true;
                        LOGINFO("[COR] IMU Ini Correct %f %f %f", mCorrectParam.imuD[0], mCorrectParam.imuD[1], mCorrectParam.imuD[2]);
                        imuCorrectList.clear();
                    }
                }
            }
            else{
                imuCorrectPreList.clear();
            }
            imu++;
            if(imu->time>veh->time)
                break;
        }
        veh++;
    }
}
bool SensorCorrect::isIMUini()
{
    return mIMUIni;
}
void SensorCorrect::CorrectIMU(CorrectDataDef &data) {

    LMSolveS *lmSolveS = NewLMSolve();

    lmSolveS->lamda = 4;
    lmSolveS->epend = 1e-10;
    lmSolveS->dend = 1e-20;
    lmSolveS->n_iters = 100;
    if (mIMUkCorrect) {
        lmSolveS->X = CreateMatrix(1, 2);
        lmSolveS->X->data[0] = mCorrectParam.imuD[2];
        lmSolveS->X->data[1] = mCorrectParam.imuZK;
    } else {
        lmSolveS->X = CreateMatrix(1, 1);
        lmSolveS->X->data[0] = mCorrectParam.imuD[2];
    }

    lmSolveS->JFunc = correctIMUJac;
    lmSolveS->OFunc = correctIMUObj;


    lmSolveS->Para = (void *) &data;
    data.show = false;
    data.mIMUkCorrect = mIMUkCorrect;
    data.correctParam = mCorrectParam;

    LMSolve(lmSolveS, NULL);

    LOGINFO("[COR] IMU  it %d code %d",lmSolveS->it,lmSolveS->Code);




    if ((lmSolveS->it > 1)) {
        Matrix_X * obj=lmSolveS->OFunc(lmSolveS->X,lmSolveS->Para);
        //PrintMatrix(obj,"obj");
        LOGINFO("[COR] IMU obj %f",obj->data[0]);
        if(obj->data[0]<1e-3)
        {
            if (mIMUkCorrect) {
                setCorrectParam(lmSolveS->X->data[1], mCorrectParam.imuZK, imuZKIni, \

                                "sensorCorrect/imuZK", 1, 0.05, LocationServerError::IMU_UNUSUAL);
            }
            setCorrectParam(lmSolveS->X->data[0], mCorrectParam.imuD[2], imuDIni[2], \

                            "sensorCorrect/imuD2", 0, 0.05, LocationServerError::IMU_UNUSUAL);
        }
        FreeMatrix(obj);
    }


    FreeLMSolve(lmSolveS);
}


void SensorCorrect::checkData(CorrectDataDef &data) {
    ImuDataDef imuDataCheck;
    memset(&(imuDataCheck), 0, sizeof(ImuDataDef));
    mIMUkCorrect = mVehXCorrect = mVehYCorrect = mVehWCorrect = false;
    for (auto imu = data.imuDataList.begin(); imu != data.imuDataList.end(); imu++) {
        imuDataCheck.Gyro[0] += fabs(imu->Gyro[0]);
        imuDataCheck.Gyro[1] += fabs(imu->Gyro[1]);
        imuDataCheck.Gyro[2] += fabs(imu->Gyro[2]);
        //LOGINFO("[COR] IMU time  %lu ",imu->time);
        //LOGINFO("[COR] IMU Gyro[2] %f %f ",imu->Gyro[2],imuDataCheck.Gyro[2]);

    }
    imuDataCheck.Gyro[0] /= data.imuDataList.size();
    imuDataCheck.Gyro[1] /= data.imuDataList.size();
    imuDataCheck.Gyro[2] /= data.imuDataList.size();

    if (imuDataCheck.Gyro[2] > 0.01)
        mIMUkCorrect = true;
    LOGINFO("[COR] imuDataCheck %f mIMUkCorrect %d",imuDataCheck.Gyro[2],mIMUkCorrect);
    data.mIMUkCorrect=mIMUkCorrect;

    VehDataDef vehDataCheck;
    memset(&(vehDataCheck), 0, sizeof(VehDataDef));
    for (auto veh = data.vehDataList.begin(); veh != data.vehDataList.end(); veh++) {
        vehDataCheck.vxyw[0] += fabs(veh->vxyw[0]);
        vehDataCheck.vxyw[1] += fabs(veh->vxyw[1]);
        vehDataCheck.vxyw[2] += fabs(veh->vxyw[2]);
    }
    vehDataCheck.vxyw[0] /= data.vehDataList.size();
    vehDataCheck.vxyw[1] /= data.vehDataList.size();
    vehDataCheck.vxyw[2] /= data.vehDataList.size();
    if (vehDataCheck.vxyw[0] > 0.5){
        mVehXCorrect = true;
//        printf("SURIPRSE X! %f\n",vehDataCheck.vxyw[0]);
//        double start_time = data.vehDataList.begin()->time;
//        double end_time = data.vehDataList.end()->time;
//        printf("start_time: %f\n", start_time);
//        printf("end_time: %f\n", end_time);
    }
    if (vehDataCheck.vxyw[1] > 0.5) {
        mVehYCorrect = true;
//        printf("SURIPRSE Y! %f\n",vehDataCheck.vxyw[1]);
//        double start_time = data.vehDataList.begin()->time;
//        double end_time = data.vehDataList.end()->time;
//        printf("start_time: %f\n", start_time);
//        printf("end_time: %f\n", end_time);
    }
    if (vehDataCheck.vxyw[2] > 0.1)
        mVehWCorrect = true;
    //mVehWCorrect=false;
    data.mVehXCorrect=mVehXCorrect;
    data.mVehYCorrect=mVehYCorrect;
    data.mVehWCorrect=mVehWCorrect;

    LOGINFO("[COR] vehDataCheck %f %f %f mVehCorrect %d %d %d",vehDataCheck.vxyw[0],vehDataCheck.vxyw[1],vehDataCheck.vxyw[2],\
    mVehXCorrect,mVehYCorrect,mVehWCorrect);
}

struct CostFunctorXY{
    CorrectDataDef correctData;
    CostFunctorXY(CorrectDataDef def) : correctData(def){}
    template<typename T>
    bool operator()(const T* const x, T *residual) const{
        auto gps = correctData.gpsDataList.begin();
        auto imu = correctData.imuDataList.begin();
        auto veh = correctData.vehDataList.begin();
        unsigned long vehTimeLast, vehTimeCur, imuTimeCur;
        unsigned long initial_time = correctData.gpsDataList.front().time;
        unsigned long target_time = correctData.gpsDataList.back().time;
        // printf("initial_time: %lu, taget_time: %lu", initial_time,target_time);
        double initial_theta = correctData.gpsDataList.front().yaw_Sync;
        double initial_x = correctData.gpsDataList.front().XYZH[0], initial_y = correctData.gpsDataList.front().XYZH[1];
//         printf("Initial GPS point: %lu, %f, %f, %f\n",initial_time,initial_x,initial_y,initial_theta);
        double target_x = correctData.gpsDataList.back().XYZH[0], target_y = correctData.gpsDataList.back().XYZH[1];
        double target_theta = correctData.gpsDataList.back().XYZH[3];
        // printf("VehSize: %zu, ImuSize: %zu\n",correctData.vehDataList.size(), correctData.imuDataList.size());
        for (; imu != correctData.imuDataList.end(); imu++) {
            if (imu->time >= veh->time ) {
                break;
            }
        }
        double dTime = (veh->time - initial_time)*1e-6 ;
        // Define the object function with six optimized parameters & Initialize obj func
        T theta = initial_theta + (imu->Gyro[2]*dTime*x[2]  + x[5]);
        T X = initial_x + dTime*(cos(theta)*veh->vxyw[1]* x[1]);
        T Y = initial_y + dTime*(sin(theta)*veh->vxyw[1]* x[1]);
        imu++;
        veh++;
        for (; veh != correctData.vehDataList.end(); veh++) {
            //Synchronizing veh and imu time stamp
            for (; imu != correctData.imuDataList.end(); imu++) {
                if (imu->time >= veh->time ) {
                    break;
                }
            }
            double dTime = (veh->time - initial_time)*1e-6 ;
//            std::cout<<"IMUData: "<<imu->Gyro[2]<<endl;
            if(imu->Gyro[2] != 0){
                theta = theta + (imu->Gyro[2]*dTime*x[2] + x[5]);
            }
            else{
                theta = theta + (imu->Gyro[2]*dTime+ x[5]);
            }
//            std::cout<<"Theta: "<<theta<<endl;
            if(veh->vxyw[0]>=0){
                X = X + dTime*(cos(theta)*veh->vxyw[1]*x[1]+sin(theta - 0.5*PI)*veh->vxyw[0]);
                Y = Y + dTime*(sin(theta)*veh->vxyw[1]* x[1]+cos(theta - 0.5*PI)*veh->vxyw[0]);
            }
            else{
                X = X + dTime*(cos(theta)*veh->vxyw[1]*x[1]+sin(theta + 0.5*PI)*veh->vxyw[0]);
                Y = Y + dTime*(sin(theta)*veh->vxyw[1]* x[1]+cos(theta + 0.5*PI)*veh->vxyw[0]);
            }
            initial_time  = veh->time;
        }
        // Correct theta when it is over than 180 or -180
        //        if (theta >=  PI){
        //            theta = theta - 2*PI;
        //        }
        //        if (theta <= - PI){
        //            theta = 2* PI + theta;
        //        }
        residual[0] = target_x - X;
        residual[1] = target_y - Y;
//        residual[2] = target_theta - theta;
        residual[2] = (correctData.target_theta - theta)*100.0;
//        std::cout<<"the last theta = :"<<theta<<endl;
//        printf("GPS initial theta & target theta: , %f, %f, \n",initial_theta, target_theta);
//        std::cout<<"residual X = :"<<residual[0]<<" residual Y = "<<residual[1]<<" residual theta = "<<residual[2]<<endl;
        return true;
    }
private:
    CorrectDataDef def;
};

struct CostFunctorY{
    CorrectDataDef correctData;
    CostFunctorY(CorrectDataDef def) : correctData(def){}
    template<typename T>
    bool operator()(const T* const x, T *residual) const{
        auto gps = correctData.gpsDataList.begin();
        auto imu = correctData.imuDataList.begin();
        auto veh = correctData.vehDataList.begin();
        unsigned long vehTimeLast, vehTimeCur, imuTimeCur;
        unsigned long initial_time = correctData.gpsDataList.front().time;
        unsigned long target_time = correctData.gpsDataList.back().time;
        // printf("initial_time: %lu, taget_time: %lu", initial_time,target_time);
        double initial_theta = correctData.gpsDataList.front().XYZH[3];
        double initial_x = correctData.gpsDataList.front().XYZH[0], initial_y = correctData.gpsDataList.front().XYZH[1];
        // printf("Initial GPS point: %lu, %f, %f, %f\n",initial_time,initial_x,initial_y,initial_theta);
        double target_x = correctData.gpsDataList.back().XYZH[0], target_y = correctData.gpsDataList.back().XYZH[1];
        double target_theta = correctData.gpsDataList.back().XYZH[3];
        // printf("VehSize: %zu, ImuSize: %zu\n",correctData.vehDataList.size(), correctData.imuDataList.size());
        for (; imu != correctData.imuDataList.end(); imu++) {
            if (imu->time >= veh->time ) {
                break;
            }
        }
        double dTime = (veh->time - initial_time)*1e-6 ;
        //std::cout<< "COST dTime: "<< dTime <<" veh->time: " << veh->time <<" initial_time: "<< \
    initial_time <<endl;
        // Define the object function with six optimized parameters & Initialize obj func
        T theta = initial_theta + (imu->Gyro[2]*dTime*x[2]  + x[5]);
        T X = initial_x + dTime*(cos(theta)*veh->vxyw[1]* x[1]);
        T Y = initial_y + dTime*(sin(theta)*veh->vxyw[1]* x[1]);
        imu++;
        veh++;
        for (; veh != correctData.vehDataList.end(); veh++) {
            //Synchronizing veh and imu time stamp
            for (; imu != correctData.imuDataList.end(); imu++) {
                if (imu->time >= veh->time ) {
                    break;
                }
            }
            double dTime = (veh->time - initial_time)*1e-6 ;
            if(imu->Gyro[2] != 0){
                theta = theta + (imu->Gyro[2]*dTime*x[2] + x[5]);
            }
            else{
                theta = theta + (imu->Gyro[2]*dTime+ x[5]);
            }
            X = X + dTime*(cos(theta)*veh->vxyw[1] * x[1]);
            Y = Y + dTime*(sin(theta)*veh->vxyw[1] * x[1]);

            //std::cout<< "COST X: "<< X <<" Y: " << Y <<" theta: "<< theta << "[1] " <<veh->vxyw[1]<< "[2] "<<veh->vxyw[2]\
            <<" imu: "<<imu->Gyro[2]<<" dtime: " <<dTime<<endl;

            initial_time  = veh->time;
        }
        // Correct theta when it is over than 180 or -180
        //        if (theta >=  PI){
        //            theta = theta - 2*PI;
        //        }
        //        if (theta <= - PI){
        //            theta = 2* PI + theta;
        //        }
        residual[0] = target_x - X;
        residual[1] = target_y - Y;
//        residual[2] = target_theta - theta;
        residual[2] = correctData.target_theta - theta;
//        std::cout<<"the last theta = :"<<theta<<endl;
//        printf("GPS initial theta & target theta: , %f, %f, \n",initial_theta, target_theta);
//        std::cout<<"residual X = :"<<residual[0]<<" residual Y = "<<residual[1]<<" residual theta = "<<residual[2]<<endl;
        return true;
    }
private:
    CorrectDataDef def;
};

struct CostFunctorX{
    CorrectDataDef correctData;
    CostFunctorX(CorrectDataDef def) : correctData(def){}
    template<typename T>
    bool operator()(const T* const x, T *residual) const{
        auto gps = correctData.gpsDataList.begin();
        auto imu = correctData.imuDataList.begin();
        auto veh = correctData.vehDataList.begin();
        unsigned long vehTimeLast, vehTimeCur, imuTimeCur;
        unsigned long initial_time = correctData.gpsDataList.front().time;
        unsigned long target_time = correctData.gpsDataList.back().time;
//        printf("initial_time: %lu, taget_time: %lu", initial_time,target_time);
        double initial_theta = correctData.gpsDataList.front().yaw_Sync;
        double initial_x = correctData.gpsDataList.front().XYZH[0], initial_y = correctData.gpsDataList.front().XYZH[1];
        // printf("Initial GPS point: %lu, %f, %f, %f\n",initial_time,initial_x,initial_y,initial_theta);
        double target_x = correctData.gpsDataList.back().XYZH[0], target_y = correctData.gpsDataList.back().XYZH[1];
        double target_theta = correctData.gpsDataList.back().XYZH[3];
//        printf("VehSize: %zu, ImuSize: %zu\n",correctData.vehDataList.size(), correctData.imuDataList.size());
        for (; imu != correctData.imuDataList.end(); imu++) {
            if (imu->time >= veh->time ) {
                break;
            }
        }
        double dTime = (veh->time - initial_time)*1e-6 ;
        // Define the object function with six optimized parameters & Initialize obj func
        T theta = initial_theta + (imu->Gyro[2]*dTime*x[2] + x[5]);
        T X = initial_x +dTime*(sin(theta - 0.5*PI)*veh->vxyw[0]*x[0]);
        T Y = initial_y + dTime*(cos(theta - 0.5*PI)*veh->vxyw[0]*x[0]);
        imu++;
        veh++;
        for (; veh != correctData.vehDataList.end(); veh++) {
            //Synchronizing veh and imu time stamp
            for (; imu != correctData.imuDataList.end(); imu++) {
                if (imu->time >= veh->time ) {
                    break;
                }
            }
//            printf("imu_v: %f,  veh_v :%f\n",imu->Gyro[2],veh->vxyw[1]);
            double dTime = (veh->time - initial_time)*1e-6 ;
//            std::cout<<"dtime"<<dTime<<endl;
            if(imu->Gyro[2] != 0){
                theta = theta + (imu->Gyro[2]*dTime*x[2] + x[5]);
            }
            else{
                theta = theta + (imu->Gyro[2]*dTime+ x[5]);
            }
            // Correct theta when it is over than 180 or -180
            if (theta >=  PI){
                theta = theta - 2*PI;
            }
            if (theta <= - PI){
                theta = 2* PI + theta;
            }
            X = X + dTime*(sin(theta - 0.5*PI)*veh->vxyw[0]*x[0]);
            Y = Y + dTime*(cos(theta - 0.5*PI)*veh->vxyw[0]*x[0]);
            initial_time  = veh->time;
        }
//        std::cout<<"start theta"<<initial_theta<<"target theta: "<<target_theta<<"theta: "<<theta<<endl;
//        std::cout<<"start x"<<initial_x<<"target x: "<<target_x<<"X: "<<X<<endl;
//        std::cout<<"start y"<<initial_y<<"target y: "<<target_y<<"Y: "<<Y<<endl;
        //Residual blocks
        residual[0] = target_x - X;
        residual[1] = target_y - Y;
//        residual[2] = 2*PI*correctData.circle_num+target_theta - theta;
        residual[2] = correctData.target_theta - theta;
//        std::cout<<"circle_num"<<correctData.circle_num<<endl;
//        std::cout<<"residual X = :"<<residual[0]<<" residual Y = "<<residual[1]<<endl;
        return true;
    }
private:
    CorrectDataDef def;
};

void SensorCorrect::CorrectTunnelTrajectory(CorrectDataDef &data) {
/*    auto test_imu = data.imuDataList.begin();
    auto imu = data.imuDataList.begin();
    auto veh = data.vehDataList.begin();
    double initial_start_time = data.vehDataList.front().time;
    double initial_start_theta = data.gpsDataList.front().yaw_Sync;
    double initial_target_theta = data.gpsDataList.back().yaw_Sync;
//    printf("theta_start: %f theta_end: %f\n",initial_start_theta, initial_target_theta);
    for (; veh != data.vehDataList.end(); veh++) {
        //Synchronizing veh and imu time stamp
        for (; imu != data.imuDataList.end(); imu++) {
            if (imu->time >= veh->time) {
                break;
            }
        }
        double initial_dTime = (veh->time - initial_start_time)*1e-6 ;
        initial_start_theta = initial_start_theta + imu->Gyro[2]*initial_dTime;

        initial_start_time  = veh->time;
    }

    double diff_theta = initial_start_theta - data.gpsDataList.front().yaw_Sync;
//    printf("imu theta & diff theta: %f;%f\n",initial_start_theta, diff_theta);
    if (diff_theta < 0){
        if (initial_start_theta <= -PI){
            int circle_num = fabs((initial_start_theta - data.gpsDataList.front().yaw_Sync))/(2*PI);
            data.target_theta = initial_target_theta - 2*PI*(1+circle_num);
//            printf("thtarget_theta & circle_num: %f;%d\n", data.target_theta,circle_num);
        }
        else{
            data.target_theta = initial_target_theta;
        }
    }
    else if (diff_theta > 0){
        if (initial_start_theta >= PI){
            int circle_num = fabs((initial_start_theta - data.gpsDataList.front().yaw_Sync))/(2*PI);
            data.target_theta = initial_target_theta + 2*PI*(1+circle_num);
//            printf("thtarget_theta & circle_num: %f;%f;%d\n", initial_target_theta,data.target_theta,circle_num);
        }
        else{
            data.target_theta = initial_target_theta;
        }
    }
    else{
        data.target_theta = initial_target_theta;
    }


    ceres::Problem problem;
    //The variable to solve for with their initial values
    //auto-differentiation to obtain the derivative(Jocobian) define residual num and param num
    //test
    LOGINFO("[COR] Status %d %d %d", data.mIMUkCorrect, data.mVehXCorrect, data.mVehYCorrect);
    // data.mIMUkCorrect=1;
    // data.mVehXCorrect=1;
    // data.mVehYCorrect=1;
    if((data.mVehYCorrect &&  !data.mVehXCorrect && data.mIMUkCorrect) || (data.mVehYCorrect &&  data.mVehXCorrect && data.mIMUkCorrect) || (data.mVehYCorrect &&  !data.mVehXCorrect && !data.mIMUkCorrect)){
        std::cout<<"1ST mVehYCorrect"<<data.mVehYCorrect<<"mVehXCorrect"<<data.mVehXCorrect<<"mIMUkCorrect"<<data.mIMUkCorrect<<endl;
        double x[6] = {1,1,1,0,0,0};
        ceres::CostFunction *cost_function  = new ceres::AutoDiffCostFunction<CostFunctorXY,3,6>(new CostFunctorXY(data));//put the CorrectDataaDef &data into CostFunctorY
        problem.AddResidualBlock(cost_function, nullptr, x);
        problem.SetParameterLowerBound(x,2,0.9);
        problem.SetParameterUpperBound(x,2,1.1);
        //Run the Solver
        ceres::Solver::Options options;
//    Using LEVENBERG_MARQUARDT trust region
        options.linear_solver_type = ceres::DENSE_QR;
        options.minimizer_progress_to_stdout = false;
        options.max_num_iterations = 200;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        std::cout<<summary.BriefReport()<<"\n"; //Obtain convergence report , it could be commented
        mCorrectParam.path[0] = 1;
        mCorrectParam.path[1] = x[1];
        mCorrectParam.path[2] = x[2];
        mCorrectParam.path[3] = 0;
        mCorrectParam.path[4] = 0;
        mCorrectParam.path[5] = x[5];
    }
//    else if (data.mVehYCorrect &&  !data.mVehXCorrect && !data.mIMUkCorrect){
//        std::cout<<"2ND mVehYCorrect"<<data.mVehYCorrect<<"mVehXCorrect"<<!data.mVehXCorrect<<"mIMUkCorrect"<<!data.mIMUkCorrect<<endl;
//        double x[6] = {1,1,1,0,0,0};
//        ceres::CostFunction *cost_function  = new ceres::AutoDiffCostFunction<CostFunctorY,3,6>(new CostFunctorY(data));//put the CorrectDataaDef &data into CostFunctorY
//        problem.AddResidualBlock(cost_function, nullptr, x);
//        //    problem.SetParameterLowerBound(x,0,1.2);
//        //    problem.SetParameterUpperBound(x,1,1.2);
//        //Run the Solver
//        ceres::Solver::Options options;
////    Using LEVENBERG_MARQUARDT trust region
//        options.linear_solver_type = ceres::DENSE_QR;
//        options.minimizer_progress_to_stdout = false;
//        options.max_num_iterations = 200;
//        ceres::Solver::Summary summary;
//        ceres::Solve(options, &problem, &summary);
//        std::cout<<summary.BriefReport()<<"\n"; //Obtain convergence report , it could be commented
//        mCorrectParam.path[0] = 0;
//        mCorrectParam.path[1] = x[1];
//        mCorrectParam.path[2] = x[2];
//        mCorrectParam.path[3] = 0;
//        mCorrectParam.path[4] = 0;
//        mCorrectParam.path[5] = x[5];
//    }
    else if(data.mVehXCorrect && !data.mVehYCorrect && data.mIMUkCorrect ){
        std::cout<<"2ND mVehYCorrect"<<!data.mVehYCorrect<<"mVehXCorrect"<<data.mVehXCorrect<<"mIMUkCorrect"<<data.mIMUkCorrect<<endl;
        double x[6] = {1,1,1,0,0,0};
        ceres::CostFunction *cost_function  = new ceres::AutoDiffCostFunction<CostFunctorX,3,6>(new CostFunctorX(data));//put the CorrectDataaDef &data into CostFunctorXY
        problem.AddResidualBlock(cost_function, nullptr, x);
        //Run the Solver
        ceres::Solver::Options options;
        //    Using LEVENBERG_MARQUARDT trust region
        options.linear_solver_type = ceres::DENSE_QR;
        options.minimizer_progress_to_stdout = false;
        options.max_num_iterations = 200;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        std::cout<<summary.BriefReport()<<"\n"; //Obtain convergence report , it could be commented
        mCorrectParam.path[0] = x[0];
        mCorrectParam.path[1] = 1;
        mCorrectParam.path[2] =x[2];
        mCorrectParam.path[3] = 0;
        mCorrectParam.path[4] = 0;
        mCorrectParam.path[5] = x[5];
    }
    */
}

Matrix_X *correctIMUObj(Matrix_X *x, void *para) {
    CorrectVector stateVector;
    double gps_head_last, gps_head;
    unsigned long imuTimeLast, vehTimeLast, imuTimeCur, vehTimeCur;
    CorrectDataDef *correctData = (CorrectDataDef *) para;

    MathUnit *wd = x->data;
    MathUnit *wk;
    if (correctData->mIMUkCorrect)
        wk = x->data + 1;
    else
        wk = &(correctData->correctParam.imuZK);

    Matrix_X *obj = CreateMatrix(1, 1);
    obj->data[0] = 0;

    auto gps = correctData->gpsDataList.begin();
    auto imu = correctData->imuDataList.begin();

    stateVector.q0 = gps->roll_Sync;
    stateVector.q1 = gps->pitch_Sync;
    stateVector.q2 = gps->yaw_Sync;

    gps_head = gps_head_last = gps->yaw_Sync;
    imuTimeLast = gps->time;
    for (; imu != correctData->imuDataList.end(); imu++) {
        if (imu->time > gps->time) {
            break;
        }
    }
    imuTimeCur = imu->time;
    gps++;
    CorrectVector preStateVector;
    for (; gps != correctData->gpsDataList.end(); gps++) {
        while (imuTimeCur < gps->time) {
            preStateVector = stateVector;
            double dTime = (imu->time - imuTimeLast) * 1e-6;

            //LOGINFO("[COR] time %lu %lu %f", imuTimeLast  , imu->time,dTime);
            imuTimeLast = imu->time;
            preStateVector = imuUpdata(stateVector, dTime, wk, wd, imu, preStateVector);
            imu++;
            if (imu == correctData->imuDataList.end()) {
                imu--;
                imuTimeCur = MAX_TIME;
            } else {
                imuTimeCur = imu->time;
            }
            stateVector = preStateVector;
        }
        {
            double dTime = (gps->time - imuTimeLast) * 1e-6;
            preStateVector = imuUpdata(stateVector, dTime, wk, wd, imu, preStateVector);
            stateVector = preStateVector;
        }
        {
            /*double yaw=atan2(2 * (stateVector.q0 * stateVector.q3 +
                                      stateVector.q1 * stateVector.q2),
                                 stateVector.q0 * stateVector.q0 -
                                 stateVector.q1 * stateVector.q1 +
                                 stateVector.q2 * stateVector.q2 -
                                 stateVector.q3 * stateVector.q3);

            double pitch = atan2((2 * stateVector.q0 * stateVector.q1 +
                                2 * stateVector.q2 * stateVector.q3),
                               1 - 2 * (stateVector.q1 * stateVector.q1 +
                                        stateVector.q2 * stateVector.q2));
            double roll = asin(2 * (stateVector.q0 * stateVector.q2 -
                                  stateVector.q1 * stateVector.q3));*/

            double roll_Sync = stateVector.q0;
            double pitch_Sync = stateVector.q1;
            double yaw_Sync = stateVector.q2;


            gps_head = gps_head + SortAngle(gps->yaw_Sync - gps_head_last);
            //LOGINFO("[COR] Head %f %f ",gps_head , yaw_Sync);

            double objCell = (gps_head - yaw_Sync);
            obj->data[0] += objCell * objCell;
            gps_head_last = gps->yaw_Sync;
            if (correctData->show) {
                //cout<<objCell<<", ";
                cout << gps->yaw_Sync << "," << yaw_Sync << "," << imu->Gyro[2] << endl;
            }
        }
    }

    return obj;
}

Matrix_X *correctIMUJac(Matrix_X *x, void *para) {
    Matrix_X *J = Jacobian(x, correctIMUObj, para);
    return J;
}


CorrectVector &imuUpdata(const CorrectVector &stateVector, double dTime, const double *wk, const double *wd,
                         const list<ImuDataDef>::iterator &imu, CorrectVector &preStateVector) {

    double wx, wy, wz;
    wx = imu->Gyro[0];
    wy = imu->Gyro[1];
    wz = wk[0] * imu->Gyro[2] + wd[0];
    /*preStateVector.q0 = stateVector.q0 - 0.5e0 *  wx * dTime * stateVector.q1 -
                        0.5e0 *  wy * dTime * stateVector.q2 -
                        0.5e0 *  wz * dTime * stateVector.q3;
    preStateVector.q1 = 0.5e0 *  wx * dTime * stateVector.q0 + stateVector.q1 +
                        0.5e0 *  wz * dTime * stateVector.q2 -
                        0.5e0 *  wy * dTime * stateVector.q3;
    preStateVector.q2 =
            0.5e0 *  wy * dTime * stateVector.q0 - 0.5e0 *  wz * dTime * stateVector.q1 +
            stateVector.q2 + 0.5e0 *  wx * dTime * stateVector.q3;
    preStateVector.q3 =
            0.5e0 *  wz * dTime * stateVector.q0 + 0.5e0 *  wy * dTime * stateVector.q1 -
            0.5e0 *  wx * dTime * stateVector.q2 + stateVector.q3;
    double sq=preStateVector.q0*preStateVector.q0+preStateVector.q1*preStateVector.q1+\
    preStateVector.q2*preStateVector.q2+preStateVector.q3*preStateVector.q3;
    sq=sqrt(sq);
    preStateVector.q0 =preStateVector.q0/ sq;
    preStateVector.q1 =preStateVector.q1/ sq;
    preStateVector.q2 =preStateVector.q2/ sq;
    preStateVector.q3 =preStateVector.q3/ sq;*/

    preStateVector.q0 = stateVector.q0 + wx * dTime;
    preStateVector.q1 = stateVector.q1 + wy * dTime;
    preStateVector.q2 = stateVector.q2 + wz * dTime;


    return preStateVector;
}

Matrix_X *correctVehObj(Matrix_X *x, void *para) {
    CorrectVector correctVector;
    unsigned long vehTimeLast, vehTimeCur;
    CorrectDataDef *correctData = (CorrectDataDef *) para;

    MathUnit T[6];


    Matrix_X *obj = CreateMatrix(1, 1);
    Matrix_X *objCell = CreateMatrix(1, 3);

    auto gps = correctData->gpsDataList.begin();
    auto veh = correctData->vehDataList.begin();
    vehTimeLast = gps->time;
    for (; veh != correctData->vehDataList.end(); veh++) {
        if (veh->time > gps->time) {
            break;
        }
    }

    int xIndex = 0;
    if (correctData->mVehXCorrect) {
        T[0] = x->data[xIndex];
        xIndex++;
        T[3] = x->data[xIndex];
        xIndex++;
    } else {
        T[0] = correctData->correctParam.vehK[0];
        T[3] = correctData->correctParam.vehK[3];
    }

    if (correctData->mVehYCorrect) {
        T[1] = x->data[xIndex];
        xIndex++;
        T[4] = x->data[xIndex];
        xIndex++;
    } else {
        T[1] = correctData->correctParam.vehK[1];
        T[4] = correctData->correctParam.vehK[4];
    }

    if (correctData->mVehWCorrect) {
        T[2] = x->data[xIndex];
        xIndex++;
        T[5] = x->data[xIndex];
        xIndex++;
    } else {
        T[2] = correctData->correctParam.vehK[2];
        T[5] = correctData->correctParam.vehK[5];
    }


    vehTimeCur = veh->time;

    CorrectVector preCorrectVector;
    correctVector.posX = gps->XYZH[0];
    correctVector.posY = gps->XYZH[1];
    correctVector.posZ = gps->XYZH[2];
    gps++;
    for (; gps != correctData->gpsDataList.end(); gps++) {

        while ((vehTimeCur < gps->time)) {
            preCorrectVector = correctVector;
            double dTime = (veh->time - vehTimeLast) * 1e-6;
            vehTimeLast = veh->time;
            VehDataDef vehCal=*veh;
            vehCal.vxyw[2]=correctData->correctParam.vehK[6] * veh->vxyw[0] + \
            correctData->correctParam.vehK[7] * veh->vxyw[1] +correctData->correctParam.vehK[8] * veh->vxyw[2];

            preCorrectVector = vehUpdata(correctVector, dTime, T, &vehCal, preCorrectVector);
            veh++;
            if (veh == correctData->vehDataList.end()) {
                veh--;
                vehTimeCur = MAX_TIME;
            } else {

                vehTimeCur = veh->time;
            }
            correctVector = preCorrectVector;
        }
        {

            double dTime = (gps->time - vehTimeLast) * 1e-6;
            VehDataDef vehCal=*veh;
            vehCal.vxyw[2]=correctData->correctParam.vehK[6] * veh->vxyw[0] + \
            correctData->correctParam.vehK[7] * veh->vxyw[1] +correctData->correctParam.vehK[8] * veh->vxyw[2];

            preCorrectVector = vehUpdata(correctVector, dTime, T,  &vehCal, preCorrectVector);
            correctVector = preCorrectVector;
        }
        {

            objCell->data[0] = gps->XYZH[0] - correctVector.posX;
            objCell->data[1] = gps->XYZH[1] - correctVector.posY;
            objCell->data[2] = (gps->XYZH[2] - correctVector.posZ) * 0.01;
            //cout<<gps->XYZH[1] << "  "<<correctVector.posY<<endl;
            for (int i = 0; i < 3; i++) {
                objCell->data[i] = objCell->data[i] * objCell->data[i];
                obj->data[0] += objCell->data[i];
            }
        }

    }
    //cout<<"T "<<T[0]<<" "<<T[1]<<" "<<T[2]<<" "<<T[3]<<" "<<T[4]<<" "<<T[5]<<" "<<endl;
    //cout<<"obj "<<obj->data[0]<<endl;
    FreeMatrix(objCell);
    //free(T);
    //PrintMatrix(obj,"VehO");
    return obj;
}

CorrectVector &
vehUpdata(const CorrectVector &stateVector, double dTime, double *T, VehDataDef *veh,
          CorrectVector &preStateVector) {


    Matrix_X *vehC;
    Matrix_X *vehO = CreateMatrix(1, 3);

    //(Matrix_X *)malloc(sizeof(Matrix_X));



    vehO->data[0] = veh->vxyw[0];
    vehO->data[1] = veh->vxyw[1];
    vehO->data[2] = veh->vxyw[2];
    //vehO->data[2] = vehOmega;

    Matrix_X *vehT = (Matrix_X *) malloc(sizeof(Matrix_X));
    vehT->Height = 2;
    vehT->Width = 3;
    vehT->data = T;

    vehC = MatrixMux(vehT, vehO);
    FreeMatrix(vehO);
    //free(vehO);
    free(vehT);
    preStateVector.posX = dTime * ((veh->q0 * veh->q0 + veh->q1 * veh->q1 -
                                    veh->q2 * veh->q2 - veh->q3 * veh->q3) *
                                   vehC->data[0] + (-2 * veh->q0 * veh->q3 +
                                                    2 * veh->q1 * veh->q2) * vehC->data[1]) +
                          stateVector.posX;
    preStateVector.posY = dTime *
                          ((2 * veh->q0 * veh->q3 + 2 * veh->q1 * veh->q2) *
                           vehC->data[0] + (veh->q0 * veh->q0 - veh->q1 * veh->q1 +
                                            veh->q2 * veh->q2 - veh->q3 * veh->q3) *
                                           vehC->data[1]) + stateVector.posY;
    preStateVector.posZ = dTime *
                          ((-2 * veh->q0 * veh->q2 + 2 * veh->q1 * veh->q3) *
                           vehC->data[0] +
                           (2 * veh->q0 * veh->q1 + 2 * veh->q2 * veh->q3) *
                           vehC->data[1]) + stateVector.posZ;
    FreeMatrix(vehC);
    return preStateVector;
}

Matrix_X *correctVehJac(Matrix_X *x, void *para) {
    Matrix_X *J = Jacobian(x, correctVehObj, para);
    //PrintMatrix(J,"VehJ");
    return J;
}


Matrix_X *correctVehHeadObj(Matrix_X *x, void *para) {
    int xIndex = 0;
    CorrectVector correctVector;
    unsigned long vehTimeLast, vehTimeCur;
    CorrectDataDef *correctData = (CorrectDataDef *) para;

    MathUnit T[3];
    int coutner;
    double IMUheading, vehHeading, vehOmega;

    if (correctData->mVehXCorrect) {
        T[0] = x->data[xIndex];
        xIndex++;
    } else {
        T[0] = correctData->correctParam.vehK[6];
    }

    if (correctData->mVehYCorrect) {
        T[1] = x->data[xIndex];
        xIndex++;
    } else {
        T[1] = correctData->correctParam.vehK[7];
    }

    if (correctData->mVehWCorrect) {
        T[2] = x->data[xIndex];
        xIndex++;
    } else {
        T[2] = correctData->correctParam.vehK[8];
    }

    Matrix_X *obj = CreateMatrix(1, 1);

    IMUheading = vehHeading = 0;

    auto veh = correctData->vehDataList.begin();
    vehTimeLast = veh->time;
    coutner = 0;
    for (; veh != correctData->vehDataList.end(); veh++) {
        double dTime = (veh->time - vehTimeLast) * 1e-6;
        vehTimeLast = veh->time;
        vehOmega = T[0] * veh->vxyw[0] + T[1] * veh->vxyw[1] + T[2] * veh->vxyw[2];
        IMUheading += veh->w * dTime;
        vehHeading += vehOmega * dTime;
        coutner++;
        if (coutner == 100) {
            obj->data[0] += fabs(IMUheading - vehHeading);
            coutner = 0;
        }
    }
    return obj;
}

Matrix_X *correctVehHeadJac(Matrix_X *x, void *para) {
    Matrix_X *J = Jacobian(x, correctVehHeadObj, para);
    return J;
}