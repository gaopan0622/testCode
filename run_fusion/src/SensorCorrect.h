//
// Created by guimu on 2021/6/11.
//

#ifndef SRC_SENSORCORRECT_H
#define SRC_SENSORCORRECT_H

#include "matrix/Matrix_XTJ.h"
#include <vector>
#include "glog_helper/GLogHelper.h"
#include "json/json.h"
#include "LocationDefine.h"
#include  "param_service/ParamService.h"
#include "LMLib.h"
#include <error_handler/NodeErrorHandler.h>
//#include <ceres/ceres.h>
typedef struct {
    list<ImuDataDef> imuDataList;
    list<VehDataDef> vehDataList;
    list<GpsCorrectDef> gpsDataList;
    double Q[4];
    bool show;
    bool mIMUkCorrect,mVehXCorrect,mVehYCorrect,mVehWCorrect;
    double target_theta;
    CorrectParam correctParam;
}CorrectDataDef;



class SensorCorrect:public NodeErrorHandler {
public:
    SensorCorrect();
    ~SensorCorrect(){};
    void correctData(CorrectDataDef & data);
    void correctPath(CorrectDataDef & data);
    void iniIMU(CorrectDataDef &data);
    CorrectParam mCorrectParam;
    bool isIMUini();
private:
    bool mIMUIni;
    bool imuZKIni;
    bool imuDIni[3];
    bool vehKIni[9];
    list<IMU_Data> imuCorrectPreList;
    list<IMU_Data> imuCorrectList;
    ParamService   mParamService;

    void checkData( CorrectDataDef &data) ;
    bool mIMUkCorrect,mVehXCorrect,mVehYCorrect,mVehWCorrect;

    void CorrectIMU(CorrectDataDef &data) ;
    void CorrectVeh(CorrectDataDef &data) ;
    void CorrectVehHead(CorrectDataDef &data);
    void CorrectTunnelTrajectory(CorrectDataDef &data);
    void setCorrectParam(double correctValue, double &correctParam, bool &Initialized, string paramName,
                         double normalValue, double threshold, int errorCode);
};


#endif //SRC_SENSORCORRECT_H
