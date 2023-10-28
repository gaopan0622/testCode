//
// Created by gaopan on 2023/7/25.
//

#ifndef SRC_GPS_FUSION_H
#define SRC_GPS_FUSION_H
#include <math.h>
#include <list>
#include <iostream>
#include "matrix/Matrix_XTJ.h"
#include "glog_helper/GLogHelper.h"
#include "common/common_all.h"
#include "common_api/commonApi.h"
#include <ros/ros.h>
#include "EKF.h"
#include <numeric>
#include <vector>
#include <error_handler/NodeErrorHandler.h>
#include "SensorCorrect.h"
#include  "param_service/ParamService.h"
#include "LocationDefine.h"
#include "LocationServerError.h"
#include <nav_msgs/Odometry.h>
#define CORRECT_DATA_DEEP (3)
using namespace std;
#define MAX_TIME (99999999999999999)
typedef struct {
    MathUnit fai;
    MathUnit posX, posY;
    MathUnit vX, vY;
    MathUnit aX, aY;
    MathUnit vW;
} StateVector;

typedef struct {
    MathUnit aImuX, aImuY;
    MathUnit posGpsX, posGpsY;
    MathUnit vVehX, vVehY, vVehW;
} ObservVector;
typedef list<GpsDataDef> GpsDataList;
typedef list<ImuDataDef> ImuDataList;
typedef list<VehDataDef> VehDataList;
class RunFusion {
public:
    RunFusion();
    ~RunFusion();

    void imuDataTopicCallback(const common::ImuData::ConstPtr& msg);
    void gpsDataTopicCallback(const common::GpsData::ConstPtr& msg);
    void motionVehicleStateCallback(const nav_msgs::Odometry::ConstPtr& msg);
    //bool setCoordinateServerCallback(common::SetCoordinate::Request &req ,common::SetCoordinate::Response &res);
    bool setCoordinate(Matrix_X *coordinate);
    void setCoordinateParam();

    void setImudata(unsigned long time, double *data);
    void setGpsData(unsigned long time, double *BLHH, double *pos, double *dir, bool posUpdated, bool headUpdated);
    void setMotionVehicleState(unsigned long time, double vxyz[3]);

    unsigned long getTimeUs();
    void InitEKFState();
    void calObservationData();
    void setEKF();
    void advertiseSyncLocation(Matrix_X * sync_pos);
    void calCurGPSData();
    void calHeadingInit();
    double LeastSquares(vector<double>& X, vector<double>& Y, double pushData);
    void getMeanStd(const vector<double>& vec, double& mean, double& stdd);

    void iniIMUThread();
    void tickEvent(const ros::TimerEvent &e);
    void systemTick();
    void locationServerTick();
    void calCurrentData();
    void upDataCurrLoc(Matrix_Xp &exfX, double dt, GpsDataDef *gpsIt, VehDataDef *vehIt, ImuDataDef *imuIt);
    void advertiseCurrentLocation();
    void readParam();
    void setLogPath(std::string logPath);
    void logSyncData(ElogInfo *logFile, Source_Sync &source, Matrix_X *EKF_x);
protected:
    double mTickTime;
    //void   robotStopping();
    virtual void logSysTime(ElogInfo *info);
private:
    NodeErrorHandler  mErrorHandler;
    double mGPSPosMaxError;
    VehDataDef vehDataCurr;
    ImuDataDef imuDataCurr;
    GpsDataDef gpsDataCurr;
    ElogInfo *mRunFusionLocationLogSync;
    ElogInfo *mRunFusionLocationLogCurr;
    ElogInfo * mRunFusionGPSDataLog;
    ElogInfo * mRunFusionIMUDataLog;
    ElogInfo * mRunFusionVEHDataLog;
    ros::Timer  locationTickTimer;
    ros::NodeHandle mRunFusionNode;
    ros::Subscriber mImuDataTopicSubscriber;
    ros::Subscriber mGpsDataTopicSubscriber;
    ros::Subscriber mMotionPlatformVelTopicSubscriber;
    ros::ServiceServer setCoordinateServerHandle;
    ros::Publisher mSyncLocationAndVelocityTopic;
    ros::Publisher mCurrentLocationAndVelocityTopic;
    pthread_mutex_t mIni_Mutex;//用于初始化
    pthread_mutex_t mSyncData_Mutex;//用于同步数据队列
    pthread_mutex_t mCoordinate_Mutex;//用于锁住坐标矩阵
    pthread_mutex_t mEKFX_Sync_Mutex;//用于锁住同步状态向量
    pthread_mutex_t mEKFX_Curr_Mutex;//用于锁住实时状态向量

    unsigned long mLastSyncTime;
    unsigned long gpsLastTime, currLastTime, currLastGpsTime;
    Source_Sync mEKFSourceLogCurr, mEKFSourceLogSync;
    SyncPara mCurrentSyncPara;

    double headingWorkInit;
    Matrix_X *mEKFX_Sync, *mEKFP_Sync, *mEKFQ_Sync, *mEKFR_Sync;//有GPS时的EKF参数,同步
    Matrix_X *mEKFX_Curr, *mEKFP_Cur, *mEKFQ_Cur, *mEKFR_Cur;
    double yaw_Sync, yaw_Curr;
    Matrix_X *mCoordinate;
    GpsDataList mGpsDataList;
    ImuDataList mImuDataList;
    VehDataList mVehDataList;
    SyncPara mHistroySyncPara;
    double mXWork_Curr, mYWork_Curr, mHAWork_Curr;//工作航向角逆时针为正，x向为0,弧度制,同步

    double mGPSHAWork_Sync, mGPSXWork_Sync, mGPSYWork_Sync, mGPSZWork_Sync;
    double mPreGPSXWork_Sync, mPreGPSYWork_Sync;
    vector<double> gpsX;
    vector<double> gpsY;
   // double x_pre, y_pre;
    SensorCorrect sensorCorrect;
    int mCorrectDataIndex, mRecvCorrDataIndex;
    bool mGpsWorkHeadingInit;
    CorrectDataDef mCorrectData[CORRECT_DATA_DEEP];
    std::string mLogPath;
    bool ekfIsRun;
    double meanX, stddX;
    double meanY, stddY;
    //double preCurGpsWorkX,preCurGpsWorkY;
};

#endif //SRC_GPS_FUSION_H
