//
// Created by gaopan on 2023/6/4.
//

#ifndef SRC_UDP_GPS_CLIENT_H
#define SRC_UDP_GPS_CLIENT_H
#include<iostream>
#include<glog_helper/GLogHelper.h>
#include "easylogger/elog.h"
#include <ros/ros.h>
#include "socketbase/socketbase.h"
#include <error_handler/NodeErrorHandler.h>
#include "common/common_all.h"
#include "param_service/ParamService.h"
#include "matrix/Matrix_XTJ.h"
#include "GPSError.h"
#include "common_api/commonApi.h"

#define EARTH_LONG_AXIS 6378137.0
#define EARTH_MINOR_AXIS 6356752.3142
using namespace std;

#pragma pack(1)
typedef struct
{
    uint16_t head;
    char spareField [26];
    uint32_t command;
} LocationCommand;

typedef struct RobotCoreParams {    //机器人中心的经纬高，航向，地固坐标系xyz
    double GpsHARobot;
    double GpsBRobot;
    double GpsLRobot;
    double GpsHRobot;
    double GpsXRobot;
    double GpsYRobot;
    double GpsZRobot;

    RobotCoreParams() : GpsHARobot(0), GpsBRobot(0), GpsLRobot(0), GpsHRobot(0), GpsXRobot(0), GpsYRobot(0), GpsZRobot(0){}
} RobotCoreParams;

typedef struct
{
    uint16_t dataHead;
    uint16_t version;
    unsigned int length;
    double id;
    double packageNumber;
    char undefined [4];
    int functionCode;
    double systemTime;
    double latitude;
    double longitude;
    double altitude;
    double gpsTime;
    double heading;
    uint8_t locationUpdate;
    uint8_t heightUpdate;
    uint8_t headingUpdate;
    uint8_t gpsState;
    uint8_t numberVS;
    uint8_t calculateVS;
    uint8_t battery;
    int sum;
} GpsResponse;

typedef struct GpsAntennaParams{    //GPS两个天线的原始数据（经纬高）
    double BM;
    double BS;
    double LM;
    double LS;
    double HM;
    double HS;
    double HAM;

    GpsAntennaParams() : BM(0), BS(0), LM(0), LS(0), HM(0), HS(0), HAM(0) {}
} GpsAntennaParams;

typedef struct RobotAntennaPos {
    double GpsAntXd;
    double GpsAntYd;
    double GpsAntLd;
    double GpsAntHAd;

    RobotAntennaPos() : GpsAntXd(0), GpsAntYd(0), GpsAntLd(0), GpsAntHAd(0) {}
} RobotAntennaPos;

#pragma pack()

class UDPCLIENT : public SocketUDP_X
{
public:
    UDPCLIENT();
    ~UDPCLIENT();
    void OnOpen();
    void RequestDataCommand(const ros::TimerEvent &e);
    void OnReceive(int Port, char IP[],char * Data,int DataLen);
    double getTimeUs();
    unsigned int sumCheck(char *data,int dataLen);
    void sendGpsData();
   // void calcRobotPos();
    void BLH2XYZ(MathUnit B,MathUnit L,MathUnit H,MathUnit *X,MathUnit *Y,MathUnit *Z);
    void bindUDP();
private:
    Matrix_X *headDir;
    double B, L, H, HA;
    ParamService gpsNodeParams;
    LocationCommand locationCommand;
    ros::NodeHandle n;
    ros::Publisher gpsDataPub;
    common::GpsData gpsData;
    RobotCoreParams robotCoreParams;
    GpsAntennaParams gpsAntennaParams;
    RobotAntennaPos robotAntennaPos;
    NodeErrorHandler errorHandler;
    ros::Timer  tickTimer;
};

#endif //SRC_UDP_GPS_CLIENT_H

