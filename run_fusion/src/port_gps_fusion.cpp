 //
// Created by gaopan on 2023/7/25.
//

#include "port_gps_fusion.h"
Matrix_X *getSyncX(Matrix_X *X, Matrix_X *Phi, void *para); //获取EKF最后融合状态向量

Matrix_X *getSyncZ(Matrix_X *X, Matrix_X *H, void *para);   //获取EKF的观测值

Matrix_X *getSyncPhi(Matrix_X *X, void *para);  //计算预测方程由非线性化到线性化的雅克比矩阵

Matrix_X *getSyncH(Matrix_X *X, void *para);    //计算观测方程由非线性化到线性化的雅克比矩阵

void BLH2XYZ(double B, double L, double H, double *X, double *Y, double *Z);

void XYZ2BLH(double X, double Y, double Z, double *B, double *L, double *H);
#define EARTHLONGAXIS (6378137.0)
#define EARTHSHORTAXIS (6356752.3142)

int main(int argc, char **argv)
{
    ros::init(argc, argv, "run_fusion");
    LOGINFO("location_server start");
//GLogHelper::Init("/home/log", ros::this_node::getName());
    RunFusion *runFusion = new RunFusion();
    ros::spin();
    return 0;
}

RunFusion::RunFusion()
{

    robotAntennaPos.GpsAntLd = 0.35; //0.35; //如果gps安装位置与机器人中心位置重叠 则都设为0，gps安装位置与机器人的角度，与航向角一致
    robotAntennaPos.GpsAntHAd = M_PI;  //M_PI;
    mCoordinate_Mutex = mIni_Mutex = mSyncData_Mutex = mEKFX_Sync_Mutex=\
    mEKFX_Curr_Mutex=PTHREAD_MUTEX_INITIALIZER;
    ekfIsRun = IsCharge = false;
    meanX = stddX = meanY = stddY = mClearLocationError = 0;
    mPositionSrvStatus = PositionSrvStatus::INVALID;
    mCorrectDataIndex = mRecvCorrDataIndex = 0;
    mGPSHAWork_Sync = mGPSXWork_Sync = mGPSYWork_Sync = mGPSZWork_Sync = 0;
    mCoordinate = mEKFX_Curr = mEKFX_Sync = mEKFP_Sync = mEKFQ_Sync = mEKFR_Sync = NULL;
    mLastSyncTime = gpsLastTime = gpsNoUpdateNum = imuNum = VX = 0;
    mGpsWorkHeadingInit = mCalGpsWorkHeading = false;
    mGPSPOverTime = 100;     //1s后gps传感器数据未更新进来，则gps传感器出现故障，因为没有gps可以用imu预测一段时间
    mIMUOverTime =  10;       //0.1s后未收到IMU数据，则imu故障
    mVehicleOverTime = 10;     //0.1s后未收到odom数据，则odom故障
    mGPSPOverTimeCouter = mIMUOverTimeCouter = mVehicleOverTimeCouter = 0;

    memset(&mEKFSourceLogSync, 0, sizeof(mEKFSourceLogSync));
    memset(&mEKFSourceLogCurr, 0, sizeof(mEKFSourceLogCurr));
    mTickTime = 0.01;
    mImuDataTopicSubscriber= mRunFusionNode.subscribe("imuDataTopic", 1, &RunFusion::imuDataTopicCallback,this);
    mGpsDataTopicSubscriber= mRunFusionNode.subscribe("gpsDataTopic", 3, &RunFusion::gpsDataTopicCallback,this);
    //mMotionPlatformVelTopicSubscriber= mRunFusionNode.subscribe("motionVehicleStateTopic", 1, &RunFusion::motionVehicleStateCallback,this);
    mMotionPlatformVelTopicSubscriber= mRunFusionNode.subscribe("ranger_base_node/odom", 1, &RunFusion::motionVehicleStateCallback,this);
    mClearErrorSubscriber = mRunFusionNode.subscribe("clearErrorTopic", 1, &RunFusion::clearLocationError,this);
    mRobotSysStateTopicSubscriber= mRunFusionNode.subscribe("robotSysStateTopic", 5, &RunFusion::robotSysStateTopicCallback,this);
    mChargeModeSubscriber = mRunFusionNode.subscribe("chargeModeTopic", 1, &RunFusion::chargeModeTopicCallback,this);
    //setCoordinateServerHandle = mRunFusionNode.advertiseService( "setCoordinateServer",&RunFusion::setCoordinateServerCallback,this);
   // mSyncLocationAndVelocityTopic = mRunFusionNode.advertise<common::SyncLocationAndVelocity>("RunFusionsyncLocationAndVelocityTopic", 100);
    mSyncLocationAndVelocityTopic = mRunFusionNode.advertise<common::SyncLocationAndVelocity>("syncLocationAndVelocityTopic", 100);
    //mCurrentLocationAndVelocityTopic = mRunFusionNode.advertise<common::CurrentLocationAndVelocity>("RunFusioncurrentLocationAndVelocityTopic", 1);
    mCurrentLocationAndVelocityTopic = mRunFusionNode.advertise<common::CurrentLocationAndVelocity>("currentLocationAndVelocityTopic", 1);
    mPositionStatusTopic = mRunFusionNode.advertise<common::PositionStatus>("positionStatusTopic", 1);
    mErrorHandler.initialize(BetterEnum2Vector<LocationServerError, int>(), 10);
    readParam();
    setCoordinateParam();
    setEKF();
    locationTickTimer = mRunFusionNode.createTimer(ros::Duration(mTickTime), &RunFusion::tickEvent,this);

}

void RunFusion::chargeModeTopicCallback(const common::ChargeMode::ConstPtr& msg)
{
    common::ChargeMode info;
    info = *(msg.get());
    if(info.mode == 1) IsCharge =true;
    else IsCharge = false;
    LOGINFO("IsCharge = %d", IsCharge);
}

void RunFusion::clearLocationError(const common::ClearError::ConstPtr &msg)
{
    common::ClearError info;
    info = *(msg.get());
    mClearLocationError = info.readyToClearError;
    LOGINFO("mClearLocationError %d",mClearLocationError);
    if(mClearLocationError == 1)
    {
        if(pthread_mutex_trylock(&mEKFX_Sync_Mutex)==0)
        {
            if(pthread_mutex_trylock(&mEKFX_Curr_Mutex)==0)
            {
                if (mEKFX_Sync != NULL)
                    FreeMatrix(mEKFX_Sync);
                if (mEKFX_Curr != NULL)
                    FreeMatrix(mEKFX_Curr);
                mEKFX_Sync = NULL;
                mEKFX_Curr = NULL;
                gpsPotionJITTERCounter = gpsNoUpdateNum = mGPSPOverTimeCouter = mIMUOverTimeCouter = mVehicleOverTimeCouter = 0;
                mGPSHAWork_Sync = mHAWork_Curr; //清除错误后，将数据全部清空重新计算位置，但是航向不清空
                LOGINFO("Clear Location Error, mGPSHAWork_sync = %.8lf",mGPSHAWork_Sync);
                pthread_mutex_unlock(&mEKFX_Curr_Mutex);
            }
            pthread_mutex_unlock(&mEKFX_Sync_Mutex);
        }
    }
}

void RunFusion::calR2GParam(double X, double Y)
{
    robotAntennaPos.GpsAntLd = atan2(Y, X);
    robotAntennaPos.GpsAntHAd = sqrt(X * X + Y * Y);
    LOGINFO("robotAntennaPos.GpsAntLd = %lf",robotAntennaPos.GpsAntLd);
    LOGINFO("robotAntennaPos.GpsAntHAd = %lf",robotAntennaPos.GpsAntHAd);
}

void RunFusion::readParam()
{
    std::string mLogPath;
    mLogPath = "/home/log/";
    GLogHelper::Init(mLogPath, "runFusion", LogLevel::ERROR);

    setLogPath(mLogPath);

    if(mRunFusionGPSDataLog!=NULL)
        deleteLogFile(mRunFusionGPSDataLog);
    mRunFusionGPSDataLog=getNewLogFile((char *)mLogPath.c_str(),"RunFusionGPSData");
    if(mRunFusionIMUDataLog!=NULL)
        deleteLogFile(mRunFusionIMUDataLog);
    mRunFusionIMUDataLog=getNewLogFile((char *)mLogPath.c_str(),"RunFusionIMUData");
    if(mRunFusionVEHDataLog!=NULL)
        deleteLogFile(mRunFusionVEHDataLog);
    mRunFusionVEHDataLog=getNewLogFile((char *)mLogPath.c_str(),"RunFusionVEHData");

}

void RunFusion::setLogPath(std::string logPath)    //设置csv输出格式日志的路径
{
    mLogPath = logPath;
    //LOGINFO("setLogPath\r\n");
    GLogHelper::Init(mLogPath, "runFusion", LogLevel::ERROR);

    //LOGINFO("start set log\n");
    if (mRunFusionLocationLogSync != NULL)
        deleteLogFile(mRunFusionLocationLogSync);
    mRunFusionLocationLogSync = getNewLogFile((char *) mLogPath.c_str(), "runFusionLocationSyncData");
    //    LOGINFO("end mRunFusionLocationLogSync\n");

    if (mRunFusionLocationLogCurr != NULL)
        deleteLogFile(mRunFusionLocationLogCurr);
    mRunFusionLocationLogCurr = getNewLogFile((char *) mLogPath.c_str(), "runFusionLocationCurrData");
    //LOGINFO("end set log \n");
}

void  RunFusion::logSyncData(ElogInfo *logFile, Source_Sync &source, Matrix_X *EKF_x) {
    //TIME_FUNC(1000);
    LOGINFO("[logSyncData] [logSyncData]");
    char logBug[2048];
    if (logFile == NULL)
        return;
    //1
    logSysTime(logFile);
    //log_time(logFile);
    //2
    sprintf(logBug, "%ld,", source.time);
    log_r(logFile, logBug);
    //3
    sprintf(logBug, "%0.8f,%.8f,%.8f,", source.GPS_X, source.GPS_Y, source.GPS_Heading);
    log_r(logFile, logBug);
    //6
    sprintf(logBug, "%.8f,%.8f,%.8f,", source.XGyro, source.YGyro, source.ZGyro);
    log_r(logFile, logBug); //imu数据
    //9
    sprintf(logBug, "%.8f,%.8f,%.8f,", source.XAcc, source.YAcc, source.ZAcc);
    log_r(logFile, logBug); //imu数据
    //12
    sprintf(logBug, "%.8f,%.8f,%.8f,", source.Robot_vx, source.Robot_vy,
            source.Robot_w);    //odom数据
    log_r(logFile, logBug);
    //15
    if (EKF_x) {
        StateVector *stateVector = (StateVector *) EKF_x->data;
        //15
        sprintf(logBug, "%.8f,", stateVector->fai);
        log_r(logFile, logBug);
        //16
        sprintf(logBug, "%.8f,%.8f,", stateVector->posX, stateVector->posY);
        log_r(logFile, logBug);
        //18
        sprintf(logBug, "%.8f,%.8f,", stateVector->vX, stateVector->vY);
        log_r(logFile, logBug);
        //20
        sprintf(logBug, "%.8f,%.8f,", stateVector->aX, stateVector->aY);
        log_r(logFile, logBug);
        //22
        sprintf(logBug, "%.8f,", stateVector->vW);  //使用的是imu数据
        log_r(logFile, logBug);
    } else {
        //16
        log_r(logFile, "0,");
        //17
        log_r(logFile, "0,0,");
        //19
        log_r(logFile, "0,0,");
        //21
        log_r(logFile, "0,0,");
        //22
        log_r(logFile, "0,");

    }
    //23
    sprintf(logBug, "%d,%d,%d,%d,", source.GPSPosUpdata, source.GPSAngleUpdata,source.GyroUpdata, source.MovePlatUpdata);
    log_r(logFile, logBug);
    //27
    sprintf(logBug, "%.8f,",source.headWork);
    log_r(logFile, logBug);

    log_r(logFile, "\r\n");

}

void RunFusion::logSysTime(ElogInfo *info) {
    log_time(info);
}

//设置Q R
void RunFusion::tickEvent(const ros::TimerEvent &e)
{
    //TRACE_FUNC
    systemTick();
}

void RunFusion::systemTick() {
    TICK_FUNC(20000); //计算两个时间戳之间的间隔是否超过给定的阈值，如果超出阈值则输出日志提醒
    checkRecvData();
    //checkSrvStatus();
    locationServerTick();
}

void RunFusion::locationServerTick() {
    LOGINFO("locationServerTick() START");
    if (mCoordinate)
    {
        if(mGpsWorkHeadingInit == true)
        {
            mErrorHandler.errorDisappeared(LocationServerError::ROBOT_HEADING_UNINIT);
            LOGINFO("[locationServerTick()] EKF RUN");
            pthread_mutex_lock(&mIni_Mutex);
            calCurrentData();
            pthread_mutex_unlock(&mIni_Mutex);
            checkSrvStatus();
            advertiseCurrentLocation();
            //logLocationData();
        }
        else
        {
            mErrorHandler.errorOccured(LocationServerError::ROBOT_HEADING_UNINIT);
            LOGINFO("[locationServerTick()] Only GPS, Need Init Robot Heading");
            advertiseCurrentGPS();
        }
    }
}

void RunFusion::advertiseCurrentGPS()
{
    common::CurrentLocationAndVelocity inf;
    string subString;

    inf.x = 0;
    inf.y = 0;
    inf.z = 0;
    inf.workingHeading = 0;


    inf.vx = 0;
    inf.vy = 0;
    inf.omega = 0;

    inf.latitude = gpsDataCurr.BLHH[0];
    inf.longitude = gpsDataCurr.BLHH[1];
    inf.altitude = gpsDataCurr.BLHH[2];
    inf.earthHeading = 0;
    inf.positionStatus = static_cast<int>(mPositionSrvStatus);
    mCurrentLocationAndVelocityTopic.publish(inf);
    LOGINFO("[Only_GPS_PUB] %.3f,%.3f,%.3f | %.3f,%.3f,%.5f | %.5f,| %.5f,%.5f,%.5f|%d", inf.x, inf.y, inf.z, inf.vx, inf.vy,
            inf.omega, inf.workingHeading, inf.latitude, inf.longitude, inf.altitude,
            inf.positionStatus);
}

void RunFusion::robotSysStateTopicCallback(const common::RobotSysState::ConstPtr& msg)
{
    common::RobotSysState inf;
    inf=*(msg.get());
    mCurRobotState = inf.state;
    LOGINFO("[ROBOT]mCurRobotState = %d",mCurRobotState);
    
}

void RunFusion::checkSrvStatus()    //如果在充电屋内报错则是未充电的原因
{
    if ((mIMUOverTimeCouter >= mIMUOverTime) || (mVehicleOverTimeCouter >= mVehicleOverTime) || (mGPSPOverTimeCouter >= mGPSPOverTime) )
        mPositionSrvStatus = PositionSrvStatus::INVALID;
    else
    {
        if(mCurRobotState == 2)
        {
            if(gpsDataCurr.posUpdated)      mPositionSrvStatus = PositionSrvStatus::NORMAL;
            else
            {
                if(gpsNoUpdateNum >= gpsPotionJITTERCounter)    //哪个错误报的多 优先执行哪个
                {
                    //两个不同的判断是由于万一两个变量，都有的时候。
                    if(gpsNoUpdateNum <= 15 && gpsNoUpdateNum > 0) mPositionSrvStatus = PositionSrvStatus::VOLATILE;
                    else if(gpsNoUpdateNum > 15) 
                    {
                        mPositionSrvStatus = PositionSrvStatus::INVALID;
                        LOGWARNING("[ERROR] 2 GPS_POSITIONING_ERROR");
                        mErrorHandler.errorOccured(LocationServerError::POSITIONING_ERROR);
                    }
                }
                else
                {
                    if(gpsPotionJITTERCounter <= 15 && gpsPotionJITTERCounter > 0) mPositionSrvStatus = PositionSrvStatus::VOLATILE;
                    else if(gpsPotionJITTERCounter > 15) 
                    {
                        mPositionSrvStatus = PositionSrvStatus::INVALID;
                        LOGWARNING("[ERROR] 2 GPS_POSITIONING_JITTER");
                        mErrorHandler.errorOccured(LocationServerError::GPS_POSITION_JITTER);
                    }
                }
            }
        }
        else if(mCurRobotState == 7)   //其余状态下如果gps一直有问题也报错，时间要长点，如出充电柱时一直定位未更新则报错。
        {
            if(gpsDataCurr.posUpdated)      mPositionSrvStatus = PositionSrvStatus::NORMAL;
            else
            {
                if(gpsNoUpdateNum >= gpsPotionJITTERCounter)    //哪个错误报的多 优先执行哪个
                {
                    //两个不同的判断是由于万一两个变量，都有的时候。
                    if(gpsNoUpdateNum <=600  && gpsNoUpdateNum > 0) mPositionSrvStatus = PositionSrvStatus::VOLATILE;
                    else if(gpsNoUpdateNum > 600) 
                    {
                        mPositionSrvStatus = PositionSrvStatus::INVALID;
                        LOGWARNING("[ERROR] 7 GPS_POSITIONING_ERROR");
                        mErrorHandler.errorOccured(LocationServerError::POSITIONING_ERROR);
                    }
                }
                else
                {
                    if(gpsPotionJITTERCounter <= 600 && gpsPotionJITTERCounter > 0) mPositionSrvStatus = PositionSrvStatus::VOLATILE;
                    else if(gpsPotionJITTERCounter > 600) 
                    {
                        mPositionSrvStatus = PositionSrvStatus::INVALID;
                        LOGWARNING("[ERROR] 7 GPS_POSITIONING_JITTER");
                        mErrorHandler.errorOccured(LocationServerError::GPS_POSITION_JITTER);
                    }
                }
            }
        }
        else if(mCurRobotState == 8)   //进充电桩时gps未更新 1分钟以内为定位即将失效状态。超过则定位无效 充电桩内不报错
        {
            if(gpsDataCurr.posUpdated)      mPositionSrvStatus = PositionSrvStatus::NORMAL;
            else
            {
                if(gpsNoUpdateNum >= gpsPotionJITTERCounter)    //哪个错误报的多 优先执行哪个
                {
                    //两个不同的判断是由于万一两个变量，都有的时候。
                    if(gpsNoUpdateNum <=600  && gpsNoUpdateNum > 0) mPositionSrvStatus = PositionSrvStatus::VOLATILE;
                    else if(gpsNoUpdateNum > 600) 
                    {
                        mPositionSrvStatus = PositionSrvStatus::INVALID;
                        LOGWARNING("[ERROR] 8 GPS_POSITIONING_ERROR");
                        //mErrorHandler.errorOccured(LocationServerError::POSITIONING_ERROR);
                    }
                }
                else
                {
                    if(gpsPotionJITTERCounter <= 600 && gpsPotionJITTERCounter > 0) mPositionSrvStatus = PositionSrvStatus::VOLATILE;
                    else if(gpsPotionJITTERCounter > 600) 
                    {
                        mPositionSrvStatus = PositionSrvStatus::INVALID;
                        LOGWARNING("[ERROR] 8 GPS_POSITIONING_JITTER");
                        //mErrorHandler.errorOccured(LocationServerError::GPS_POSITION_JITTER);
                    }
                }
            }
        }
        else
        {
            if(gpsDataCurr.posUpdated)      mPositionSrvStatus = PositionSrvStatus::NORMAL;
            else    mPositionSrvStatus = PositionSrvStatus::INVALID;  
        }
    }
    common::PositionStatus inf;
    inf.result = static_cast<int>(mPositionSrvStatus);
    LOGINFO("mPositionSrvStatus = %d",inf.result);
    mPositionStatusTopic.publish(inf);
}

void RunFusion::checkRecvData() //验证接收传感器数据的有效性
{
    if (mCoordinate == NULL)
        return;

    if (mGPSPOverTimeCouter >= mGPSPOverTime) {
        mErrorHandler.errorOccured(LocationServerError::GPS_POSITION_TIMEOUT);    //gps数据出问题时，可以让imu预测，由于此机器人较慢，如果快的话，调整次数。
        LOGWARNING("[SENSOR] GPS_POSITION_TIMEOUT");
    } else {
        mGPSPOverTimeCouter++;
    }

    if (mIMUOverTimeCouter >= mIMUOverTime) {
        mErrorHandler.errorOccured(LocationServerError::IMU_DATA_TIMEOUT);
        LOGWARNING("[SENSOR] IMU_DATA_TIMEOUT");
    } else {
        mIMUOverTimeCouter++;
    }

    if (mVehicleOverTimeCouter >= mVehicleOverTime) {
        mErrorHandler.errorOccured(LocationServerError::VEHICLE_DATA_TIMEOUT);
        LOGWARNING("[SENSOR] VEHICLE_DATA_TIMEOUT");
    } else {
        mVehicleOverTimeCouter++;
    }
}


void RunFusion::calCurrentData()
{
    LOGINFO("calCurrentData()");
    pthread_mutex_lock(&mEKFX_Sync_Mutex);
    pthread_mutex_lock(&mEKFX_Curr_Mutex);
    if (mEKFX_Curr != NULL)
    {
        if(ekfIsRun)
        {
            LOGINFO("[ekf run] write log");
            mEKFSourceLogCurr = mEKFSourceLogSync;
            currLastTime = mEKFSourceLogCurr.time;
            LOGINFO("[mEKFX_Sync] %lf %lf %lf %lf %lf %lf %lf %lf ",mEKFX_Sync->data[0], mEKFX_Sync->data[1],mEKFX_Sync->data[2],mEKFX_Sync->data[3],mEKFX_Sync->data[4],mEKFX_Sync->data[5],mEKFX_Sync->data[6],mEKFX_Sync->data[7]);
            LOGINFO("[mEKFX_Curr] %lf %lf %lf %lf %lf %lf %lf %lf ",mEKFX_Curr->data[0], mEKFX_Curr->data[1],mEKFX_Curr->data[2],mEKFX_Curr->data[3],mEKFX_Curr->data[4],mEKFX_Curr->data[5],mEKFX_Curr->data[6],mEKFX_Curr->data[7]);
            //LOGINFO("[Cur = EKF RUN] %lf %lf %lf | %lf %lf %lf | %lf %lf | %d %d %d %d",mEKFSourceLogSync.GPS_X,mEKFSourceLogSync.GPS_Y,mEKFSourceLogSync.GPS_Z,mEKFSourceLogSync.Robot_vx,mEKFSourceLogSync.Robot_vy,\
            mEKFSourceLogSync.Robot_w,mEKFSourceLogSync.ZGyro,mEKFSourceLogSync.GPSPosUpdata,mEKFSourceLogSync.GPSAngleUpdata,mEKFSourceLogSync.GyroUpdata,mEKFSourceLogSync.MovePlatUpdata);
            advertiseSyncLocation(mEKFX_Curr);
            logSyncData(mRunFusionLocationLogCurr, mEKFSourceLogCurr, mEKFX_Curr);
            ekfIsRun = false;
        }
        else {
            long long dti = imuDataCurr.time - currLastTime;
            currLastTime = imuDataCurr.time;
            if (dti == 0) {
                //dti = 10000;
                LOGWARN("[CURR] imu time stop");
            }
            if (dti > 20000) {
                dti = 20000;
                LOGWARN("[CURR] imu time jump");
            }
            if (dti > 0) {
                LOGINFO("[normal data] run write log");
                double dt = (dti) * 0.000001;
                upDataCurrLoc(mEKFX_Curr, dt, &gpsDataCurr, &vehDataCurr, &imuDataCurr);
                logSyncData(mRunFusionLocationLogCurr, mEKFSourceLogCurr, mEKFX_Curr);
                LOGINFO("[mEKFX_Curr predictive] %lf %lf %lf %lf %lf %lf %lf %lf ",mEKFX_Curr->data[0], mEKFX_Curr->data[1],mEKFX_Curr->data[2],mEKFX_Curr->data[3],mEKFX_Curr->data[4],mEKFX_Curr->data[5],mEKFX_Curr->data[6],mEKFX_Curr->data[7]);
                //LOGINFO("[Cur RUN] %lf %lf %lf | %lf %lf %lf | %lf %lf %lf | %lf %lf %d %d %d",mEKFSourceLogCurr.GPS_X,mEKFSourceLogCurr.GPS_Y,mEKFSourceLogCurr.GPS_Z,mEKFSourceLogCurr.xWork,mEKFSourceLogCurr.yWork,mEKFSourceLogCurr.zWork,mEKFSourceLogCurr.Robot_vx,mEKFSourceLogCurr.Robot_vy,\
                        mEKFSourceLogCurr.Robot_w,mEKFSourceLogCurr.yaw_imu,mEKFSourceLogCurr.ZGyro,mEKFSourceLogCurr.GPSPosUpdata,mEKFSourceLogCurr.GyroUpdata,mEKFSourceLogCurr.MovePlatUpdata);
            } else {
                LOGWARN("[CURR] dti error %d", dti);
            }
        }
    }
    pthread_mutex_unlock(&mEKFX_Sync_Mutex);
    pthread_mutex_unlock(&mEKFX_Curr_Mutex);
}

void RunFusion::upDataCurrLoc(Matrix_Xp &exfX, double dt, GpsDataDef *gpsIt, VehDataDef *vehIt, ImuDataDef *imuIt)
{
    if (exfX != NULL) {
        mCurrentSyncPara.posUpdated = false;
        //}
        mCurrentSyncPara.imuUpdated = true;
        mCurrentSyncPara.vehUpdated = true;
        mCurrentSyncPara.headUpdated = false;
        mEKFSourceLogCurr.GPSAngleUpdata    = mCurrentSyncPara.headUpdated;
        mEKFSourceLogCurr.GPSPosUpdata      = mCurrentSyncPara.posUpdated;
        mEKFSourceLogCurr.MovePlatUpdata    = mCurrentSyncPara.vehUpdated;
        mEKFSourceLogCurr.GyroUpdata        = mCurrentSyncPara.imuUpdated;

        if (mCurrentSyncPara.imuUpdated) {
            
            mEKFSourceLogCurr.XAcc = imuIt->Acc[0];
            mEKFSourceLogCurr.YAcc = -imuIt->Acc[1];
            mEKFSourceLogCurr.ZAcc = imuIt->Acc[2];

            mEKFSourceLogCurr.XGyro = imuIt->Gyro[0];
            mEKFSourceLogCurr.YGyro = imuIt->Gyro[1];
            mEKFSourceLogCurr.ZGyro = imuIt->Gyro[2];
           // mEKFSourceLogCurr.Robot_w = imuIt->Gyro[2];
            mEKFSourceLogCurr.time = imuIt->time;
        }

        if (mCurrentSyncPara.vehUpdated) {
            mEKFSourceLogCurr.Robot_vx = -vehIt->vxyw[0];
            mEKFSourceLogCurr.Robot_vy = -vehIt->vxyw[1];
            mEKFSourceLogCurr.Robot_w = vehIt->vxyw[2];
        }
        mEKFSourceLogCurr.GPS_X = mGPSXWork_Sync;
        mEKFSourceLogCurr.GPS_Y = mGPSYWork_Sync;
        mEKFSourceLogCurr.GPS_Z = mGPSZWork_Sync;
        mEKFSourceLogCurr.headWork = mGPSHAWork_Sync;
        mCurrentSyncPara.dt = dt;
        LOGINFO("[CUR run]dt = %lf",mCurrentSyncPara.dt);
        Matrix_X * x = getSyncX(exfX, NULL, &mCurrentSyncPara); //imu预测
        if(x!=NULL)
        {
            FreeMatrix(exfX);
            exfX=x;
        }
        mEKFSourceLogCurr.GPS_Heading = HA;
        StateVector *stateVector = (StateVector *) exfX->data;  
        //LOGINFO("[StateVectorCURR3] %lf %lf %lf %lf %lf %lf %lf %lf",stateVector->fai,stateVector->posX,stateVector->posY,stateVector->vX,stateVector->vY,stateVector->aX,stateVector->aY,stateVector->vW);

        mXWork_Curr = stateVector->posX;
        mYWork_Curr = stateVector->posY;
        mHAWork_Curr = stateVector->fai;
        yaw_Curr = stateVector->fai;

        mEKFSourceLogCurr.yaw = yaw_Curr;
        mEKFSourceLogCurr.xWork = mXWork_Curr;  //更新为预测位置
        mEKFSourceLogCurr.yWork = mYWork_Curr;  //更新为预测位置
        mEKFSourceLogCurr.zWork = mGPSZWork_Sync;
        mEKFSourceLogCurr.headWork = mGPSHAWork_Sync;
        mEKFSourceLogCurr.yaw_imu = yaw_Curr;
    }
}

void RunFusion::advertiseCurrentLocation()
{
    common::CurrentLocationAndVelocity inf;
    string subString;
    pthread_mutex_lock(&mEKFX_Curr_Mutex);
    if (mEKFX_Curr == NULL)
    {
        LOGINFO("ONLY GPS mEKFX_Curr = NULL");
        advertiseCurrentGPS();
        pthread_mutex_unlock(&mEKFX_Curr_Mutex);
        return;
    }

    StateVector *stateVector = (StateVector *) mEKFX_Curr->data;
    inf.x = stateVector->posX;
    inf.y = stateVector->posY;
    //inf.z = mEKFSourceLogCurr.zWork;
    //inf.workingHeading = SortAngle(stateVector->fai) / M_PI * 180;
    inf.workingHeading = stateVector->fai;
    //inf.roll = mEKFSourceLogCurr.roll;
    //inf.pitch = mEKFSourceLogCurr.pitch;
    inf.yaw = stateVector->fai;
    inf.vx = stateVector->vX;
    inf.vy = stateVector->vY;
    inf.omega = stateVector->vW;
    pthread_mutex_unlock(&mEKFX_Curr_Mutex);
    LOGINFO("[stateVectorPub] %lf  %lf  %lf  %lf  %lf  %lf",stateVector->fai,stateVector->posX,stateVector->posY,stateVector->vX,stateVector->vY,stateVector->vW);
    if (mCoordinate != NULL) {
        Matrix_X *invCoordinate;
        double B, L, H;
        pthread_mutex_lock(&mCoordinate_Mutex);
        invCoordinate = MatrixInv(mCoordinate);
        pthread_mutex_unlock(&mCoordinate_Mutex);
        if (invCoordinate != NULL) {
            Matrix_X *RobotPos;
            Matrix_X *RobotPosWork = CreateMatrix(1, 4);
            RobotPosWork->data[0] = mEKFSourceLogCurr.xWork;
            RobotPosWork->data[1] = mEKFSourceLogCurr.yWork;
            RobotPosWork->data[2] = mEKFSourceLogCurr.zWork; //Z不变，用之前的gps到工作坐标系的Z坐标计算
            LOGINFO("[advertiseCurrentLocation] %lf %lf %lf",RobotPosWork->data[0],RobotPosWork->data[1],RobotPosWork->data[2]);
            RobotPosWork->data[3] = 1;
            RobotPos = MatrixMux(invCoordinate, RobotPosWork);
            XYZ2BLH(RobotPos->data[0], RobotPos->data[1], RobotPos->data[2], &B, &L, &H);
            inf.latitude = B;
            inf.longitude = L;
            inf.altitude = H;
            if(RobotPosWork != NULL)
            FreeMatrix(RobotPosWork);
            if(RobotPos != NULL)
            FreeMatrix(RobotPos);
            if(invCoordinate != NULL)
            FreeMatrix(invCoordinate);
        } else {
            inf.latitude = 0;
            inf.longitude = 0;
            inf.altitude = 0;
        }
        inf.positionStatus = static_cast<int>(mPositionSrvStatus);
    }
    else 
    {
        inf.positionStatus = static_cast<int>(PositionSrvStatus::INVALID);
    }
    mCurrentLocationAndVelocityTopic.publish(inf);

    LOGINFO("[CURR_PUB] %.8lf,%.8lf | %.3lf,%.3lf,%.5lf | %.8lf | %.8lf,%.8lf,%.8lf | %d", inf.x, inf.y, inf.vx,
            inf.vy, inf.omega, inf.workingHeading, inf.latitude, inf.longitude, inf.altitude, inf.positionStatus);
}


void RunFusion::setEKF() {
    
    LOGINFO("setEKF() START");

    if(mEKFQ_Sync != NULL)
        FreeMatrix(mEKFQ_Sync);
    mEKFQ_Sync = CreateMatrix(8,8);
    SetMatrixItem(mEKFQ_Sync, 0, 0, 0.0001);
    SetMatrixItem(mEKFQ_Sync, 1, 1, 0.2); //x y
    SetMatrixItem(mEKFQ_Sync, 2, 2, 0.2);
    SetMatrixItem(mEKFQ_Sync, 3, 3, 1); //vx
    SetMatrixItem(mEKFQ_Sync, 4, 4, 1);
    SetMatrixItem(mEKFQ_Sync, 5, 5, 1);
    SetMatrixItem(mEKFQ_Sync, 6, 6, 1);
    SetMatrixItem(mEKFQ_Sync, 7, 7, 0.01);

    if (mEKFR_Sync != NULL)
        FreeMatrix(mEKFR_Sync);
    mEKFR_Sync = CreateMatrix(8,8);
    SetMatrixItem(mEKFR_Sync, 0, 0, 0.001);    //IMU测量噪声0.001
    SetMatrixItem(mEKFR_Sync, 1, 1, 0.001);
    //SetMatrixItem(mEKFR_Sync, 2, 2, 0.5); //x y 由于将gps数据筛选的严格，所以应让gps的置信度高
    //SetMatrixItem(mEKFR_Sync, 3, 3, 0.5);
    SetMatrixItem(mEKFR_Sync, 2, 2, 0.4); //x y
    SetMatrixItem(mEKFR_Sync, 3, 3, 0.4);
    SetMatrixItem(mEKFR_Sync, 4, 4, 0.005);  //里程计测量噪声0.01
    SetMatrixItem(mEKFR_Sync, 5, 5, 0.005);
    SetMatrixItem(mEKFR_Sync, 6, 6, 0.001);
    SetMatrixItem(mEKFR_Sync, 7, 7, 0.00001);

    if (mEKFP_Sync == NULL)
        mEKFP_Sync = CreateMatrix(8,8);
    SetMatrixItem(mEKFP_Sync, 0, 0, 0.0000001);
    SetMatrixItem(mEKFP_Sync, 1, 1, 0.01);
    SetMatrixItem(mEKFP_Sync, 2, 2, 0.01);
    SetMatrixItem(mEKFP_Sync, 3, 3, 0.01);
    SetMatrixItem(mEKFP_Sync, 4, 4, 0.01);
    SetMatrixItem(mEKFP_Sync, 5, 5, 0.01);
    SetMatrixItem(mEKFP_Sync, 6, 6, 0.01);
    SetMatrixItem(mEKFP_Sync, 7, 7, 0.01);
}

double RunFusion::LeastSquares(vector<double> &X, vector<double> &Y, double pushData) {
    if (X.empty() || Y.empty())
        return false;
    int vec_size = X.size();
    double sum1 = 0, sum2 = 0;
    double x_avg = accumulate(X.begin(), X.end(), 0.0) / vec_size;
    double y_avg = accumulate(Y.begin(), Y.end(), 0.0) / vec_size;

    for (int i = 0; i < vec_size; ++i) {
        sum1 += (X.at(i) * Y.at(i) - x_avg * y_avg);
        sum2 += (X.at(i) * X.at(i) - x_avg * x_avg);
    }
    double k,b;
    k = sum1 / sum2;
    b = y_avg - k * x_avg;
    return (k * pushData + b);
}

void RunFusion::calHeadingInit(){
    //初始化将大地坐标系的xy转换到工作坐标系，进行工作坐标系航向的求解
    pthread_mutex_lock(&mSyncData_Mutex);
    LOGINFO("[EKF Sync] calWorkHeadingInit");
    double gpsXStart, gpsXEnd;
    double gpsYStartCal, gpsYEndCal;
    gpsXStart = gpsX.front();
    gpsXEnd = gpsX.back();
    //利用最小二乘法找到两点
    gpsYStartCal = LeastSquares(gpsX, gpsY, gpsXStart);
    gpsYEndCal = LeastSquares(gpsX, gpsY, gpsXEnd);
    double x_diff, y_diff;
    x_diff = gpsXEnd - gpsXStart;
    y_diff = gpsYEndCal - gpsYStartCal;
    LOGINFO("[calHeadingInit] gpsXStart = %.8lf gpsXEnd = %.8lf gpsYStartCal = %.8lf gpsYEndCal = %.8lf",gpsXStart,\
    gpsXEnd, gpsYStartCal, gpsYEndCal);
    //printf("[calHeadingInit x_diff,y_diff] %.8lf %.8lf", x_diff, y_diff);
    LOGINFO("[calHeadingInit x_diff,y_diff] %.8lf %.8lf", x_diff, y_diff);
    headingWorkInit = atan2(y_diff , x_diff);
    LOGINFO("[calHeading] Odom Vx: %lf", vehDataCurr.vxyw[0]);
    LOGINFO("calHeading OUT = %.8lf",headingWorkInit);
    if(vehDataCurr.vxyw[0] < 0) mGPSHAWork_Sync = headingWorkInit;
    else{
        if(headingWorkInit <= 0) mGPSHAWork_Sync = SortAngle(M_PI + headingWorkInit);
        else mGPSHAWork_Sync = SortAngle((-M_PI) + headingWorkInit);
    }
    //   工作坐标系航向 以vy方向为航向角基础
    mGpsWorkHeadingInit = true;
    LOGINFO("[headingWorkInit] = %.8lf", mGPSHAWork_Sync);
    gpsX.clear();
    gpsY.clear();
    VX = 0;
    pthread_mutex_unlock(&mSyncData_Mutex);
    //printf("[headingWorkInit] = %lf", mGPSHAWork_Sync);
    //mGpsDataList.clear();
}
void RunFusion::calCurGPSData() {
    LOGINFO("calCurGPSData");
    Matrix_X *RobotPos, *RobotPosWork;
    RobotPos = CreateMatrix(1, 4);
    RobotPos->data[3] = 1;
//将gps当前的地心坐标系下的XYZ 为工作坐标系    pthread_mutex_unlock(&mEKFX_Sync_Mutex);。
    RobotPos->data[0] = gpsDataCurr.GPSLoc[0];
    RobotPos->data[1] = gpsDataCurr.GPSLoc[1];
    RobotPos->data[2] = gpsDataCurr.GPSLoc[2];

    pthread_mutex_lock(&mCoordinate_Mutex);
    RobotPosWork = MatrixMux(mCoordinate, RobotPos);
    pthread_mutex_unlock(&mCoordinate_Mutex);

    mGPSXWork_Sync = RobotPosWork->data[0];
    mGPSYWork_Sync = RobotPosWork->data[1];
    mGPSZWork_Sync = RobotPosWork->data[2];

    LOGINFO("[LOC] GPSPOSWork_Sync %.8lf, %.8lf, %.8lf", mGPSXWork_Sync, mGPSYWork_Sync, mGPSZWork_Sync);
    if (RobotPos != NULL)
    FreeMatrix(RobotPos);
    if (RobotPosWork != NULL)
    FreeMatrix(RobotPosWork);

    Matrix_X *DirMatrixWork;
    Matrix_X *RobotDir;
    double Xd, Yd;
    pthread_mutex_lock(&mCoordinate_Mutex);
    RobotDir = CreateMatrix(1, 4);
    RobotDir->data[0] = gpsDataCurr.GPSDir[0];
    RobotDir->data[1] = gpsDataCurr.GPSDir[1];
    RobotDir->data[2] = gpsDataCurr.GPSDir[2];
    RobotDir->data[3] = 0;

    DirMatrixWork = MatrixMux(mCoordinate, RobotDir);
    pthread_mutex_unlock(&mCoordinate_Mutex);
    Xd = DirMatrixWork->data[0];
    Yd = DirMatrixWork->data[1];
    HA = atan2(Yd, Xd);
    //printf("calgps\n");
    //LOGINFO("[LOC] GPSHAWork_Sync %.6f %.8f %.8f %.8f ", mGPSHAWork_Sync, DirMatrixWork->data[0],DirMatrixWork->data[1], DirMatrixWork->data[2]);
    if (gpsDataCurr.headUpdated)
        mWorkHeadingToGPSHeading = SortAngle(gpsDataCurr.BLHH[3] / 180 * M_PI - (-HA));
    FreeMatrix(DirMatrixWork);
    FreeMatrix(RobotDir);
    mEKFSourceLogCurr.GPS_Heading = HA;
    LOGINFO("mEKFSourceLogCurr.GPS_Heading = %.8lf\n",mEKFSourceLogCurr.GPS_Heading );
}

void RunFusion::InitEKFState() {
    LOGINFO("[LOC] Ini mEKFXGPS");
    pthread_mutex_lock(&mEKFX_Sync_Mutex);
    LOGDEBUG ("[CHECK] mEKFX_Sync_Mutex pass");
    pthread_mutex_lock(&mEKFX_Curr_Mutex);
    LOGDEBUG ("[CHECK] mEKFX_Curr_Mutex pass");
    if (mEKFX_Sync != NULL)
        FreeMatrix(mEKFX_Sync);
    mEKFX_Sync = CreateMatrix(1, 8);
    ImuDataDef &currImuData = mImuDataList.back();
    GpsDataDef &currGpsData = mGpsDataList.back();
    VehDataDef &currVehData = mVehDataList.back();
    yaw_Sync = mGPSHAWork_Sync;
    LOGINFO("mGPSHAWork_Sync INIT EKF = %lf",mGPSHAWork_Sync);
       //判断如果静止不动，则让角速度为0
    //if(sqrt(currVehData.vxyw[0] * currVehData.vxyw[0] + currVehData.vxyw[1] * currVehData.vxyw[1]) < 1e-4 && currImuData.Gyro[2] * currImuData.Gyro[2] < 1e-6) currImuData.Gyro[2] = 0;
    mEKFX_Sync->data[0] = mGPSHAWork_Sync; //fai  huduzhi WORK
    mEKFX_Sync->data[1] = mGPSXWork_Sync;  //px work
    mEKFX_Sync->data[2] = mGPSYWork_Sync;  //py work
    mEKFX_Sync->data[3] = -currVehData.vxyw[0]; //vx robot
    mEKFX_Sync->data[4] = -currVehData.vxyw[1]; //vy robot
    mEKFX_Sync->data[5] = currImuData.Acc[0];  //ax robot
    mEKFX_Sync->data[6] = -currImuData.Acc[1]; //ay robot
    mEKFX_Sync->data[7] = currImuData.Gyro[2]; //vw robot

    mEKFSourceLogSync.Robot_vx = mEKFX_Sync->data[3];
    mEKFSourceLogSync.Robot_vy = mEKFX_Sync->data[4];
    mEKFSourceLogSync.Robot_w = currVehData.vxyw[0];
    mEKFSourceLogSync.yaw_imu = yaw_Sync;
    mEKFSourceLogSync.XGyro = currImuData.Gyro[0];//机器人坐标系与odom imu坐标系 关系为： vx = -vx（odom)  vy = -vy（odom)
    mEKFSourceLogSync.YGyro = currImuData.Gyro[1];  //vx = -vx（odom/imu)  vy = -vx（odom/imu)
    mEKFSourceLogSync.ZGyro = currImuData.Gyro[2];
    mEKFSourceLogSync.XAcc = currImuData.Acc[0];
    mEKFSourceLogSync.YAcc = -currImuData.Acc[1];
    mEKFSourceLogSync.ZAcc = currImuData.Acc[2];
    mEKFSourceLogSync.GPS_X = mGPSXWork_Sync;
    mEKFSourceLogSync.GPS_Y = mGPSYWork_Sync;
    mEKFSourceLogSync.GPS_Z = mGPSZWork_Sync;
    mEKFSourceLogSync.xWork = mGPSXWork_Sync;
    mEKFSourceLogSync.yWork = mGPSYWork_Sync;
    mEKFSourceLogSync.zWork = mGPSZWork_Sync;
    mEKFSourceLogSync.headWork = mGPSHAWork_Sync; //工作坐标系 航向 角度 一二象限 【0-pi】 三四象限 【-pi 0】
    mEKFSourceLogSync.time= currGpsData.time;
    mEKFSourceLogSync.yaw = yaw_Sync;   //工作坐标系航向 弧度
    mEKFSourceLogSync.GPSPosUpdata = true;
    mEKFSourceLogSync.GyroUpdata = true;
    mEKFSourceLogSync.MovePlatUpdata = true;
    mEKFSourceLogSync.GPSAngleUpdata = mCalGpsWorkHeading;
    mEKFSourceLogSync.GPS_Heading = HA;
    gpsLastTime = currGpsData.time;
    mLastSyncTime = currGpsData.time;

    if (mEKFX_Curr != NULL) {
        FreeMatrix(mEKFX_Curr);
    }
    mEKFX_Curr = MatrixCopy(mEKFX_Sync);
    currLastTime = currGpsData.time;
    currLastGpsTime = currGpsData.time;
    ekfIsRun = true;
    pthread_mutex_unlock(&mEKFX_Sync_Mutex);
    pthread_mutex_unlock(&mEKFX_Curr_Mutex);
    LOGINFO("[mEKFX_Curr init ekf] %lf %lf %lf %lf %lf %lf %lf %lf ",mEKFX_Sync->data[0], mEKFX_Sync->data[1],mEKFX_Sync->data[2],mEKFX_Sync->data[3],mEKFX_Sync->data[4],mEKFX_Sync->data[5],mEKFX_Sync->data[6],mEKFX_Sync->data[7]);
    logSyncData(mRunFusionLocationLogSync, mEKFSourceLogSync, mEKFX_Sync);
    LOGINFO("[EKF INIT END]");
    //start ekf
}

void RunFusion::calObservationData() {
    //jiasuo
    LOGINFO("[EKF calObservationData]");
    pthread_mutex_lock(&mEKFX_Sync_Mutex);
    pthread_mutex_lock(&mEKFX_Curr_Mutex);
    ImuDataDef &ObImuData = mImuDataList.back();
    GpsDataDef &ObGpsData = mGpsDataList.back();
    VehDataDef &ObVehData = mVehDataList.back();
    if(ObGpsData.time < MAX_TIME)
        mHistroySyncPara.posUpdated = ObGpsData.posUpdated;
    else
        mHistroySyncPara.posUpdated = false;

    if(ObVehData.time < MAX_TIME)
        mHistroySyncPara.vehUpdated = true;
    else
        mHistroySyncPara.vehUpdated = false;
    if(ObImuData.time < MAX_TIME)
        mHistroySyncPara.imuUpdated = true;
    else
        mHistroySyncPara.imuUpdated = false;
    mHistroySyncPara.headUpdated = mCalGpsWorkHeading;
    LOGINFO("mHistroySyncPara.vehUpdated %d %d %d %d",mHistroySyncPara.vehUpdated,mHistroySyncPara.imuUpdated,mHistroySyncPara.posUpdated, mHistroySyncPara.headUpdated);
    Matrix_X *Z;
    Z = CreateMatrix(1, 8); //观测数据
    mEKFSourceLogSync.GPSAngleUpdata = mHistroySyncPara.headUpdated;
    mEKFSourceLogSync.GPSPosUpdata = mHistroySyncPara.posUpdated;
    mEKFSourceLogSync.MovePlatUpdata = mHistroySyncPara.vehUpdated;
    mEKFSourceLogSync.GyroUpdata = mHistroySyncPara.imuUpdated;
    if(mHistroySyncPara.imuUpdated){
        mEKFSourceLogSync.XAcc = ObImuData.Acc[0];
        mEKFSourceLogSync.YAcc = -ObImuData.Acc[1];
        mEKFSourceLogSync.ZAcc = ObImuData.Acc[2];
        mEKFSourceLogSync.XGyro = ObImuData.Gyro[0];
        mEKFSourceLogSync.YGyro = ObImuData.Gyro[1];
        mEKFSourceLogSync.ZGyro = ObImuData.Gyro[2];
       // mHistroySyncPara.ax = ObImuData.Acc[0];
       // mHistroySyncPara.ay = -ObImuData.Acc[1];
       // mHistroySyncPara.vw = ObImuData.Gyro[2];
        Z->data[0] = ObImuData.Acc[0];
        Z->data[1] = -ObImuData.Acc[1];
        Z->data[6] = ObImuData.Gyro[2];
        mHistroySyncPara.dt = (ObImuData.time - mLastSyncTime) / 1000000.0f;
        LOGINFO("mHistroySyncPara.dt = %lf",mHistroySyncPara.dt);
        //mLastSyncTime = ObImuData.time;
        //mEKFSourceLogSync.yaw_imu = yaw_Sync;
    }

    if(mHistroySyncPara.imuUpdated && mHistroySyncPara.vehUpdated){
        mEKFSourceLogSync.Robot_vx = -ObVehData.vxyw[0];
        mEKFSourceLogSync.Robot_vy = -ObVehData.vxyw[1];
        mEKFSourceLogSync.Robot_w = ObVehData.vxyw[2];
        Z->data[4] = -ObVehData.vxyw[0];
        Z->data[5] = -ObVehData.vxyw[1];
    }
    if(mHistroySyncPara.headUpdated)
    {
        Z->data[7] = mGPSHAWork_Sync;
    }   
    if(mHistroySyncPara.posUpdated){
        mEKFSourceLogSync.GPS_X = mGPSXWork_Sync;
        mEKFSourceLogSync.GPS_Y = mGPSYWork_Sync;
        mEKFSourceLogSync.GPS_Z = mGPSZWork_Sync;

        Z->data[2] = mGPSXWork_Sync; 
        Z->data[3] = mGPSYWork_Sync;
        mHistroySyncPara.dt = (ObGpsData.time - mLastSyncTime) / 1000000.0f;
        //mLastSyncTime = ObGpsData.time;
        LOGINFO("mHistroySyncPara.dt = %lf",mHistroySyncPara.dt);
    }

    if(mHistroySyncPara.imuUpdated) mLastSyncTime = ObImuData.time;
    if(mHistroySyncPara.posUpdated) mLastSyncTime = ObGpsData.time;

    LOGINFO("[mEKFX_Sync] %lf %lf %lf %lf %lf %lf %lf %lf ",mEKFX_Sync->data[0], mEKFX_Sync->data[1],mEKFX_Sync->data[2],mEKFX_Sync->data[3],mEKFX_Sync->data[4],mEKFX_Sync->data[5],mEKFX_Sync->data[6],mEKFX_Sync->data[7]);
    LOGINFO("[Z]  %lf %lf %lf %lf %lf  %lf  %lf %lf",Z->data[0],Z->data[1],Z->data[2],Z->data[3],Z->data[4],Z->data[5],Z->data[6],Z->data[7]);

    LOGINFO("[SYNC_CAL] EKF");
    EKF(mEKFX_Sync, Z, getSyncPhi, getSyncH, getSyncX, getSyncZ, mEKFP_Sync, mEKFQ_Sync,
        mEKFR_Sync, (void *) &mHistroySyncPara);
    mEKFSourceLogSync.time = mLastSyncTime;
    StateVector *stateVector = (StateVector *) mEKFX_Sync->data;
    mEKFSourceLogSync.headWork = mGPSHAWork_Sync;
    mEKFSourceLogSync.xWork = stateVector->posX;
    mEKFSourceLogSync.yWork = stateVector->posY;
    mEKFSourceLogSync.zWork = mGPSZWork_Sync;
    mEKFSourceLogSync.GPS_Heading = HA;
    LOGINFO("[SYNC_CAL] EKF sync write data");
    LOGINFO("[SYNC_CAL] EKF : %lf %lf %lf %lf %lf %lf %lf %lf",mEKFX_Sync->data[0],mEKFX_Sync->data[1],mEKFX_Sync->data[2],mEKFX_Sync->data[3],mEKFX_Sync->data[4],mEKFX_Sync->data[5],mEKFX_Sync->data[6],mEKFX_Sync->data[7]);
    logSyncData(mRunFusionLocationLogSync, mEKFSourceLogSync, mEKFX_Sync);

    ekfIsRun = true;
    if(mCalGpsWorkHeading) mCalGpsWorkHeading = false;
    if (mEKFX_Curr != NULL) {
        FreeMatrix(mEKFX_Curr);
    }
    mEKFX_Curr = MatrixCopy(mEKFX_Sync);
    pthread_mutex_unlock(&mEKFX_Sync_Mutex);
    pthread_mutex_unlock(&mEKFX_Curr_Mutex);
}

void RunFusion::advertiseSyncLocation(Matrix_X *sync_pos) {
    LOGINFO("[Pub SyncLocation Data]");
    common::SyncLocationAndVelocity inf;
    if (sync_pos == NULL)
        return;
    inf.time = mEKFSourceLogSync.time;
    StateVector *stateVector = (StateVector *)sync_pos->data;
    inf.x = stateVector->posX;
    inf.y = stateVector->posY;
    inf.z = mEKFSourceLogSync.zWork;
    inf.workingHeading = stateVector->fai;
    inf.yaw = stateVector->fai;
    inf.vx = stateVector->vX;
    inf.vy = stateVector->vY;
    inf.omega = stateVector->vW;

    if(mCoordinate != NULL){
        Matrix_X *invCoordinate;
        double B, L, H;
        pthread_mutex_lock(&mCoordinate_Mutex);
        invCoordinate = MatrixInv(mCoordinate); //求矩阵的逆
        pthread_mutex_unlock(&mCoordinate_Mutex);
        if (invCoordinate != NULL) {
            Matrix_X *RobotPos;
            Matrix_X *RobotPosWork = CreateMatrix(1, 4);
            RobotPosWork->data[0] = inf.x;
            RobotPosWork->data[1] = inf.y;
            RobotPosWork->data[2] = inf.z;
            RobotPosWork->data[3] = 1;
            RobotPos = MatrixMux(invCoordinate, RobotPosWork);  //工作坐标系到大地坐标系的转换
            XYZ2BLH(RobotPos->data[0], RobotPos->data[1], RobotPos->data[2], &B, &L, &H);
            inf.latitude = B;
            inf.longitude = L;
            inf.altitude = H;
            if (mEKFX_Curr != NULL) 
            FreeMatrix(RobotPosWork);
            if (RobotPos != NULL) 
            FreeMatrix(RobotPos);
            if (invCoordinate != NULL) 
            FreeMatrix(invCoordinate);
        } else {
            inf.latitude = 0;
            inf.longitude = 0;
            inf.altitude = 0;
        }
        inf.positionStatus = static_cast<int>(mPositionSrvStatus);      
    }
    else{
        inf.positionStatus = static_cast<int>(PositionSrvStatus::INVALID);
        LOGINFO("mCoordinate INVALID");
    }
    mSyncLocationAndVelocityTopic.publish(inf);
    LOGINFO("[SYNC_PUB] %.8lf,%.8lf,%.8lf | %.8lf,%.8lf,%.8lf | %.8lf | %.8lf,%.8lf,%.8lf | %d", inf.x, inf.y, inf.z, inf.vx, inf.vy,
            inf.omega, inf.workingHeading, inf.latitude, inf.longitude, inf.altitude, inf.positionStatus);
}

void RunFusion::iniIMUThread()
{
    LOGINFO("iniIMUThread()");
    sensorCorrect.iniIMU(mCorrectData[mRecvCorrDataIndex]);
}

void RunFusion::imuDataTopicCallback(const common::ImuData::ConstPtr &msg) {
    common::ImuData info;
    double data[6];
    log_time(mRunFusionIMUDataLog);
    info=*(msg.get());
    data[0]=info.angularVelocity.x;
    data[1]=info.angularVelocity.y;
    data[2]=info.angularVelocity.z;
    data[3]=info.linearAcceleration.x;
    data[4]=info.linearAcceleration.y;
    data[5]=info.linearAcceleration.z;
    setImudata(info.time, data);
    char logBug[2048];

    sprintf(logBug,"%ld,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f\n",info.time,data[0],data[1],data[2],\
    data[3],data[4],data[5]);
    log_r(mRunFusionIMUDataLog,logBug);
    //LOGINFO("[INDATA] IMU %ld,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f", time, data[0], \
    data[1], data[2], data[3], data[4], data[5]);
}

void RunFusion::setImudata(unsigned long time, double *data) {
    LOGINFO("setImudata() START");
    //TIME_FUNC(5000)
    mIMUOverTimeCouter = 0;
    mErrorHandler.errorDisappeared(LocationServerError::IMU_DATA_TIMEOUT);
    if (!sensorCorrect.isIMUini()) {
        ImuDataDef imuData;
        imuData.time = time;
        imuData.Gyro[0] = data[0];
        imuData.Gyro[1] = data[1];
        imuData.Gyro[2] = data[2];
        imuData.Acc[0] = data[3];
        imuData.Acc[1] = data[4];
        imuData.Acc[2] = data[5];
        mCorrectData[mRecvCorrDataIndex].imuDataList.push_back(imuData);
        LOGINFO("mCorrectData[mRecvCorrDataIndex].imuDataList.size() = %d",mCorrectData[mRecvCorrDataIndex].imuDataList.size());
        if (mCorrectData[mRecvCorrDataIndex].imuDataList.size() > 2000) 
        {
            iniIMUThread();
        }
    }
    ImuDataDef ImuData;
    ImuData.Acc[0] = data[3];
    ImuData.Acc[1] = data[4];
    ImuData.Acc[2] = data[5];

    ImuData.Gyro[0] = data[0] + sensorCorrect.mCorrectParam.imuD[0];
    ImuData.Gyro[1] = data[1] + sensorCorrect.mCorrectParam.imuD[1];
    ImuData.Gyro[2] = data[2] + sensorCorrect.mCorrectParam.imuD[2];
    if(ImuData.Gyro[2] * ImuData.Gyro[2] < 1e-5)    //  根据imu静态z角速度最大零偏值进行估算
    {
        imuNum++;
        if(imuNum > 500) 
        {
            ImuData.Gyro[2] = 0;
            LOGINFO("[Curr Status]STATIC");
        }
    }
    else{
        imuNum = 0;
    }
    ImuData.time = time;
    imuDataCurr = ImuData;
    pthread_mutex_lock(&mSyncData_Mutex);
    //LOGINFO("[IMU_LCK]imu lock");
    //Store ImuData to imuDatalist
    mImuDataList.push_back(imuDataCurr);
    //if((mCoordinate == NULL)&&(mRecvImuDataList.size()>100))
    //   mRecvImuDataList.pop_front();
    pthread_mutex_unlock(&mSyncData_Mutex);
    LOGINFO("[INDATA] IMU %ld,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f", time, ImuData.Gyro[0], \
    ImuData.Gyro[1], ImuData.Gyro[2], ImuData.Acc[0], ImuData.Acc[1], ImuData.Acc[2]);
}
void RunFusion::gpsDataTopicCallback(const common::GpsData::ConstPtr &msg) {
    common::GpsData info;
    info = *(msg.get());
    double BLHH[4], pos[3], dir[3];
    log_time(mRunFusionGPSDataLog);
    BLHH[0] = info.latitude;
    BLHH[1] = info.longitude;
    BLHH[2] = info.altitude;
    BLHH[3] = info.heading;
    pos[0] = info.x;
    pos[1] = info.y;
    pos[2] = info.z;
    dir[0] = info.dir0;
    dir[1] = info.dir1;
    dir[2] = info.dir2;
    setGpsData(info.time, BLHH, pos, dir, info.posUpdated, info.headUpdated);
    char logBug[2048];

    sprintf(logBug, "%ld,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%d\n", info.time, info.latitude,
            info.longitude, info.altitude, info.x, info.y, info.z, info.posUpdated);
    log_r(mRunFusionGPSDataLog, logBug);
    LOGINFO("[INDATA] GPS %ld,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%d\n", info.time, info.latitude,
        info.longitude, info.altitude, info.x, info.y, info.z, info.posUpdated);
}

void RunFusion::setGpsData(unsigned long time, double *BLHH, double *pos, double *dir, bool posUpdated,
                           bool headUpdated) 
{
    LOGINFO("setGpsData() START");
    if (gpsLastTime != 0) {
        if (gpsLastTime > time) {
            LOGWARN("[TIME] gpsTime Error %llu", time);
            return;
        }
    }
    gpsDataCurr.time = time;
    gpsLastTime = time;
    gpsDataCurr.posUpdated = posUpdated;
    //gpsDataCurr.headUpdated = headUpdated;
    gpsDataCurr.GPSLoc[0] = pos[0];
    gpsDataCurr.GPSLoc[1] = pos[1];
    gpsDataCurr.GPSLoc[2] = pos[2];
    gpsDataCurr.GPSDir[0] = dir[0];
    gpsDataCurr.GPSDir[1] = dir[1];
    gpsDataCurr.GPSDir[2] = dir[2];
    gpsDataCurr.BLHH[0] = BLHH[0];
    gpsDataCurr.BLHH[1] = BLHH[1];
    gpsDataCurr.BLHH[2] = BLHH[2];
    gpsDataCurr.BLHH[3] = BLHH[3];
    //先判定更新 然后在检验跳变
    if(gpsDataCurr.posUpdated == true)
    {
        gpsNoUpdateNum = 0;
        mErrorHandler.errorDisappeared(LocationServerError::POSITIONING_ERROR);
    } 
    else
    {
        gpsNoUpdateNum++;
        LOGWARNING("[LOC] GPS_POSITIONING_ERROR gpsNoUpdateNum = %d",gpsNoUpdateNum);
    } 

    if (mCoordinate != NULL) 
    {
        if(mGpsWorkHeadingInit == false)
        {
            LOGINFO("mGpsWorkHeadingInit == false");
            calCurGPSData();
            if(gpsDataCurr.posUpdated == true)
            {
                if(VX == 0)
                {
                    VX = vehDataCurr.vxyw[0];
                    LOGINFO("[HeadingInit]cal Robot Heading, VX = %lf",VX);
                } 
                else
                {
                    if(( VX > 0 && vehDataCurr.vxyw[0] > 0 ) || ( VX < 0 && vehDataCurr.vxyw[0] < 0 ))
                    {
                        LOGINFO("[HeadingInit] VX: %lf vehDataCurr.vxyw[0]: %lf",VX, vehDataCurr.vxyw[0]);
                        if((vehDataCurr.vxyw[0] * vehDataCurr.vxyw[0]) >= 1e-2 &&  (vehDataCurr.vxyw[1] * vehDataCurr.vxyw[1]) <= 1e-6\
                            && (vehDataCurr.vxyw[2] * vehDataCurr.vxyw[2]) < 1e-6)
                        {
                            LOGINFO("[HeadingInit] [PUSH GPS XY]   %lf   %lf", vehDataCurr.vxyw[1], vehDataCurr.vxyw[2]);
                            gpsX.push_back(mGPSXWork_Sync);
                            gpsY.push_back(mGPSYWork_Sync);
                            if (gpsX.size() == 25 && gpsY.size() == 25)
                            {
                                getMeanStd(gpsX, meanX, stddX);
                                getMeanStd(gpsY, meanY, stddY);
                                LOGINFO("[HeadingInit] meanX = %.8lf; stddX = %.8lf",meanX, stddX);
                                LOGINFO("[HeadingInit] meanY = %.8lf; stddY = %.8lf",meanY, stddY);
                                //只要有一个方向的均方差大于0.5 就代表这组数据分散情况满足,这个值根据现场数据决定
                                if((stddX > 0.5 ) || (stddY > 0.5 ))
                                {
                                    calHeadingInit();
                                }
                                else
                                {
                                    LOGINFO("[HeadingInit][warn]gps data error");
                                    VX = 0;
                                    gpsX.clear();
                                    gpsY.clear();
                                }
                            }
                        }
                        else{
                            LOGINFO("[HeadingInit]Speed does not match");
                            LOGINFO("[HeadingInit]vehDataCurr.vxyw[1] = %lf, vehDataCurr.vxyw[2] = %lf", vehDataCurr.vxyw[1], vehDataCurr.vxyw[2]);
                            VX = 0;
                            gpsX.clear();
                            gpsY.clear();
                        }
                    }
                    else{
                        LOGINFO("[HeadingInit]Speed is different");
                        LOGINFO("[HeadingInit]VX: %lf vehDataCurr.vxyw[0]: %lf",VX, vehDataCurr.vxyw[0]);
                        VX = 0;
                        gpsX.clear();
                        gpsY.clear();   
                    }
                }
            }
            else
            {   //可能造成反复清除容器
                if((gpsX.size() && gpsY.size()) != 0)
                {
                    gpsX.clear();
                    gpsY.clear();
                }
            }
        }    
        else
        {
            LOGINFO("mGpsWorkHeadingInit == true");
            //初始化航向角以后进行机器人中心gps计算
            pthread_mutex_lock(&mEKFX_Curr_Mutex);
            if(mEKFX_Curr != NULL)
            {
                if(gpsDataCurr.posUpdated != false)
                {
                    SyncPara para;
                    double mXWorkPre, mYWorkPre, mYawPre;
                    LOGINFO("[setGpsData]time = %ld mEKFIME = %ld",time,mEKFSourceLogCurr.time);
                    unsigned long t = time - mEKFSourceLogCurr.time;
                    LOGINFO("[setGpsData] t = %ld", t);
                    para.dt = t /1000000.0f;
                    LOGINFO("[setGpsData] dt = %lf",para.dt);
                    LOGINFO("[setGpsData] mEKFX_Curr  %lf %lf %lf %lf %lf %lf %lf %lf",mEKFX_Curr->data[0],mEKFX_Curr->data[1],mEKFX_Curr->data[2],mEKFX_Curr->data[3],mEKFX_Curr->data[4],mEKFX_Curr->data[5],mEKFX_Curr->data[6],mEKFX_Curr->data[7]);
                    Matrix_X *pre_x = getSyncX(mEKFX_Curr, NULL, &para);
                    LOGINFO("[setGpsData] pre_x %lf %lf %lf %lf %lf %lf %lf %lf",pre_x->data[0],pre_x->data[1],pre_x->data[2],pre_x->data[3],pre_x->data[4],pre_x->data[5],pre_x->data[6],pre_x->data[7]);
                    if (pre_x != NULL) 
                    {
                        LOGINFO("[setGpsData] [Curr] calcular curr gps data 3");
                        StateVector *stateVector = (StateVector *) pre_x->data;
                        mXWorkPre = stateVector->posX;
                        mYWorkPre = stateVector->posY;
                        mYawPre = stateVector->fai;
                        //mGPSHAWork_Sync = mYawPre;
                        //计算机器人中心gps
                        calRobotPos(mYawPre);
                        LOGINFO("[setGpsData] mXWorkPre %lf %lf %lf",mXWorkPre, mYWorkPre, mYawPre);
                        LOGINFO("[setGpsData] mGPSXWork_Sync %lf %lf %lf",mGPSXWork_Sync, mGPSYWork_Sync, mGPSZWork_Sync);
                        //if(gpsNoUpdateNum > 25)
                        //{
                        //    gpsDataCurr.posUpdated = true;
                        //    gpsNoUpdateNum = 0;
                        //}
                        //else     //rtk10次之内更新的话，则将imu预测位置与gps位置进行校正，判断gps是否抖动
                        //{
                            double test_pos;
                            test_pos = sqrt((mXWorkPre - mGPSXWork_Sync) * (mXWorkPre - mGPSXWork_Sync) + \
                            (mYWorkPre - mGPSYWork_Sync) * (mYWorkPre - mGPSYWork_Sync));
                            LOGINFO("[setGpsData] test pos = %.8lf",test_pos);     //这里是由于gps数据与预测数据相差太大
                            mGPSPosMaxError = 1;  //根据最大速度大小 乘上 前一时刻imu与gps时间差的值 这里最大速度初设2m/s
                        
                            if (test_pos <= mGPSPosMaxError)
                            {
                                gpsDataCurr.posUpdated = true;
                                gpsPotionJITTERCounter = 0;
                                mErrorHandler.errorDisappeared(LocationServerError::GPS_POSITION_JITTER);
                                if(VX == 0)
                                {
                                    VX = vehDataCurr.vxyw[0];
                                    LOGINFO("Correct heading init");
                                } 
                                else
                                {
                                    if(( VX > 0 && vehDataCurr.vxyw[0] > 0 ) || ( VX < 0 && vehDataCurr.vxyw[0] < 0 ))
                                    {
                                        LOGINFO("VX: %lf vehDataCurr.vxyw[0]: %lf",VX, vehDataCurr.vxyw[0]);
                                        if((vehDataCurr.vxyw[0] * vehDataCurr.vxyw[0]) >= 1e-2 &&  (vehDataCurr.vxyw[1] * vehDataCurr.vxyw[1]) <= 1e-6\
                                            && (vehDataCurr.vxyw[2] * vehDataCurr.vxyw[2]) < 1e-6)
                                        {
                                            LOGINFO("[PUSH GPS XY]   %lf   %lf", vehDataCurr.vxyw[1], vehDataCurr.vxyw[2]);
                                            gpsX.push_back(mGPSXWork_Sync);
                                            gpsY.push_back(mGPSYWork_Sync);
                                            if (gpsX.size() == 25 && gpsY.size() == 25)
                                            {
                                                getMeanStd(gpsX, meanX, stddX);
                                                getMeanStd(gpsY, meanY, stddY);
                                                LOGINFO("meanX = %.8lf; stddX = %.8lf",meanX, stddX);
                                                LOGINFO("meanY = %.8lf; stddY = %.8lf",meanY, stddY);
                                                //if((stddX > 4 && stddX < 15) || (stddY > 4 && stddY < 15)) //只要有一个方向的均方差大于0.5 就代表这组数据分散情况满足
                                                if((stddX > 4 ) || (stddY > 4 ))
                                                {
                                                    calGpsHeading();
                                                }
                                                else
                                                {
                                                    LOGINFO("[warn]gps data error");
                                                    VX = 0;
                                                    gpsX.clear();
                                                    gpsY.clear();
                                                }
                                            }
                                        }
                                        else{
                                            LOGINFO("Speed does not match");
                                            LOGINFO("vehDataCurr.vxyw[1] = %lf, vehDataCurr.vxyw[2] = %lf", vehDataCurr.vxyw[1], vehDataCurr.vxyw[2]);
                                            VX = 0;
                                            gpsX.clear();
                                            gpsY.clear();
                                        }
                                    }
                                    else{
                                        LOGINFO("Speed is different");
                                        LOGINFO("VX: %lf vehDataCurr.vxyw[0]: %lf",VX, vehDataCurr.vxyw[0]);
                                        VX = 0;
                                        gpsX.clear();
                                        gpsY.clear();   
                                    }
                                }
                            }
                            else 
                            {
                                gpsDataCurr.posUpdated = false;
                                gpsPotionJITTERCounter++;
                                //mErrorHandler.errorOccured(LocationServerError::GPS_POSITION_JITTER);
                                LOGWARNING("[LOC] GPS_POSITION_JITTER EKF %0.6f  %0.6f GPS %0.6f  %0.6f | JITTER num %d", mXWorkPre,\
                                mGPSXWork_Sync, mYWorkPre, mGPSYWork_Sync, gpsPotionJITTERCounter);
                            }

                        //}
                        if (pre_x != NULL)
                        FreeMatrix(pre_x);
                    }
                }
            }
            pthread_mutex_unlock(&mEKFX_Curr_Mutex);
        }
    }

    pthread_mutex_lock(&mSyncData_Mutex);
    mGpsDataList.push_back(gpsDataCurr);
    pthread_mutex_unlock(&mSyncData_Mutex);
//gps不刷新，也需要校正位置，如果gps跳了，否则会导致tes——pos一直变大，然后机器人不动，会导致一直预测，
    if(mGpsWorkHeadingInit == true)
    {
        LOGINFO("[EKF Sync] CALCULAR DATA 8");
        if (mEKFX_Sync == NULL)
        {
            if(gpsDataCurr.posUpdated)
            {
                //ekf初始化，需要知道机器人中心位置
                calRobotPos(mGPSHAWork_Sync);
                InitEKFState();
            }
        }
        else
        {
            calObservationData();
        }
    }
    mGPSPOverTimeCouter = 0;
    mErrorHandler.errorDisappeared(LocationServerError::GPS_POSITION_TIMEOUT);
}

void RunFusion::calGpsHeading()
{
    pthread_mutex_lock(&mSyncData_Mutex);
    LOGINFO("[calGps_Heading");
    double gpsXStart, gpsXEnd;
    double gpsYStartCal, gpsYEndCal;
    gpsXStart = gpsX.front();
    gpsXEnd = gpsX.back();
    //利用最小二乘法找到两点
    gpsYStartCal = LeastSquares(gpsX, gpsY, gpsXStart);
    gpsYEndCal = LeastSquares(gpsX, gpsY, gpsXEnd);
    double x_diff, y_diff;
    x_diff = gpsXEnd - gpsXStart;
    y_diff = gpsYEndCal - gpsYStartCal;
    LOGINFO("[calGps_Heading] gpsXStart = %.8lf gpsXEnd = %.8lf gpsYStartCal = %.8lf gpsYEndCal = %.8lf",gpsXStart,\
    gpsXEnd, gpsYStartCal, gpsYEndCal);
    LOGINFO("[calGps_Heading x_diff,y_diff] %.8lf %.8lf", x_diff, y_diff);
    double calHeading;
    calHeading = atan2(y_diff , x_diff);
    LOGINFO("[calHeading] Odom Vx: %lf", vehDataCurr.vxyw[0]);
    LOGINFO("calHeading OUT = %.8lf",calHeading);
    if(vehDataCurr.vxyw[0] < 0) mGPSHAWork_Sync = calHeading;
    else{
        if(calHeading <= 0) mGPSHAWork_Sync = SortAngle(M_PI + calHeading);
        else mGPSHAWork_Sync = SortAngle((-M_PI) + calHeading);
    }
    //   工作坐标系航向 以vy方向为航向角基础
    mCalGpsWorkHeading = true;
    LOGINFO("[calGpsWorkHeading] = %.8lf", mGPSHAWork_Sync);
    VX = 0;
    gpsX.clear();
    gpsY.clear();
    pthread_mutex_unlock(&mSyncData_Mutex);
}
void RunFusion::calRobotPos(double heading)
{
    LOGINFO("[calRobotPos Run]");
    Matrix_X *RobotPos, *RobotPosWork;
    RobotPos = CreateMatrix(1, 4);
    RobotPos->data[3] = 1;
    //将gps当前的地心坐标系下的XYZ 为工作坐标系 
    RobotPos->data[0] = gpsDataCurr.GPSLoc[0];
    RobotPos->data[1] = gpsDataCurr.GPSLoc[1];
    RobotPos->data[2] = gpsDataCurr.GPSLoc[2];

    pthread_mutex_lock(&mCoordinate_Mutex);
    RobotPosWork = MatrixMux(mCoordinate, RobotPos);
    pthread_mutex_unlock(&mCoordinate_Mutex);
    double mGPSXWrok, mGPSYWork, mGPSZWork;
    mGPSXWrok = RobotPosWork->data[0];
    mGPSYWork = RobotPosWork->data[1];
    mGPSZWork = RobotPosWork->data[2];
    
    mGPSXWork_Sync = mGPSXWrok - robotAntennaPos.GpsAntLd * cos(heading + robotAntennaPos.GpsAntHAd);
    mGPSYWork_Sync = mGPSYWork - robotAntennaPos.GpsAntLd * sin(heading + robotAntennaPos.GpsAntHAd);
    mGPSZWork_Sync = mGPSZWork;
    LOGINFO("[LOC] [calRobotPos Run] GPSPOSWork_Sync %.8lf, %.8lf, %.8lf", mGPSXWork_Sync, mGPSYWork_Sync, mGPSZWork_Sync);
    if (RobotPos != NULL)
    FreeMatrix(RobotPos);
    if (RobotPosWork != NULL)
    FreeMatrix(RobotPosWork);

    //加入gps航向进行比对预测航向
    Matrix_X *DirMatrixWork;
    Matrix_X *RobotDir;
    double Xd, Yd;
    pthread_mutex_lock(&mCoordinate_Mutex);
    RobotDir = CreateMatrix(1, 4);
    RobotDir->data[0] = gpsDataCurr.GPSDir[0];
    RobotDir->data[1] = gpsDataCurr.GPSDir[1];
    RobotDir->data[2] = gpsDataCurr.GPSDir[2];
    RobotDir->data[3] = 0;

    DirMatrixWork = MatrixMux(mCoordinate, RobotDir);
    pthread_mutex_unlock(&mCoordinate_Mutex);
    Xd = DirMatrixWork->data[0];
    Yd = DirMatrixWork->data[1];
    HA = atan2(Yd, Xd);
    //printf("calgps\n");
    //LOGINFO("[LOC] GPSHAWork_Sync %.6f %.8f %.8f %.8f ", mGPSHAWork_Sync, DirMatrixWork->data[0],DirMatrixWork->data[1], DirMatrixWork->data[2]);
    if (gpsDataCurr.headUpdated)
        mWorkHeadingToGPSHeading = SortAngle(gpsDataCurr.BLHH[3] / 180 * M_PI - (-HA));
    FreeMatrix(DirMatrixWork);
    FreeMatrix(RobotDir);
    mEKFSourceLogCurr.GPS_Heading = HA;
    LOGINFO("mEKFSourceLogCurr.GPS_Heading = %.8lf",mEKFSourceLogCurr.GPS_Heading );
}

void RunFusion::getMeanStd(const vector<double>& vec, double& mean, double& stdd)
{
    if(!vec.empty())
    {
        for (int i = 0; i < vec.size(); ++i)
        {
            mean += vec[i];
        }
        mean /= vec.size();

        for (int i = 0; i < vec.size(); ++i)
        {
            stdd += (vec[i] - mean)*(vec[i] - mean);
        }
        stdd /= (vec.size() - 1);   //样本方差除以n-1
	    stdd = sqrt(stdd);
    }
}

/*
void RunFusion::motionVehicleStateCallback(const common::MotionVehicleState::ConstPtr& msg) {
    double vxyw[3];
    common::MotionVehicleState info;
    log_time(mRunFusionVEHDataLog);
    info = *(msg.get());
    unsigned long time;
    //time = getTimeUs();
    vxyw[0] = info.vx;
    vxyw[1] = info.vy;
    vxyw[2] = info.omega;
    setMotionVehicleState(info.time, vxyw);
    char logBug[2048];

    sprintf(logBug,"%ld,%.8f,%.8f,%.8f\n",info.time,info.vx,info.vy,info.omega);
    log_r(mRunFusionVEHDataLog, logBug);
    LOGINFO("[INDATA] VEH %ld,%.8f,%.8f,%.8f", info.time, info.vx,info.vy,info.omega);
}
*/

void RunFusion::motionVehicleStateCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    double vxyw[3];
    nav_msgs::Odometry info;
    log_time(mRunFusionVEHDataLog);
    info = *(msg.get());
    unsigned long time;
    time = getTimeUs();
    vxyw[0] = info.twist.twist.linear.x;
    vxyw[1] = info.twist.twist.linear.y;
    vxyw[2] = info.twist.twist.angular.z;
    setMotionVehicleState(time, vxyw);
    char logBug[2048];

    sprintf(logBug, "%ld,%.8f,%.8f,%.8f\n", time, info.twist.twist.linear.x, info.twist.twist.linear.y, info.twist.twist.angular.z);
    log_r(mRunFusionVEHDataLog, logBug);
    LOGINFO("[INDATA] VEH %ld,%.8f,%.8f,%.8f", time, vxyw[0], vxyw[1], vxyw[2]);
}

unsigned long RunFusion::getTimeUs()
{
    static struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec * 1000000.0 + tv.tv_usec ;
}

void RunFusion::setMotionVehicleState(unsigned long time, double *vxyz) {
    //TIME_FUNC(5000)
    LOGINFO("[Odom]setOdom()");
    mVehicleOverTimeCouter = 0;
    mErrorHandler.errorDisappeared(LocationServerError::VEHICLE_DATA_TIMEOUT);
    VehDataDef vehData;
    vehData.vxyw[0] = vxyz[0];
    vehData.vxyw[1] = vxyz[1];
    vehData.vxyw[2] = vxyz[2];
    vehData.time = time;
    vehDataCurr = vehData;
    if (!sensorCorrect.isIMUini()) {
        mCorrectData[mRecvCorrDataIndex].vehDataList.push_back(vehData);
    }
    pthread_mutex_lock(&mSyncData_Mutex);
    //LOGINFO("[MOTION_LCK]motion lock");

    // Store veh data to vehDataList

    mVehDataList.push_back(vehData);
    /*if ((mCoordinate == NULL) &&(mRecvVehDataList.size()>100)){
        mRecvVehDataList.pop_front();
    }*/
    pthread_mutex_unlock(&mSyncData_Mutex);
    //LOGINFO("[MOTION_LCK]motion unlock");

}

/*bool RunFusion::setCoordinateServerCallback(common::SetCoordinate::Request &req, common::SetCoordinate::Response &res) {
    //cout << "Set Coordinate "<<endl;
    LOGINFO("[INI] Set Coordinate %d", req.coordinate.size());
    if (req.coordinate.size() == 16) {
        Matrix_X *coordinate = CreateMatrix(4, 4);
        for (int j = 0; j < req.coordinate.size(); ++j) {
            coordinate->data[j] = req.coordinate[j];
            LOGINFO("[INI] Coordinate[%d] = %0.10f ", j, req.coordinate[j]);
        }
        if (setCoordinate(coordinate))
            res.result = 0;
        else
            res.result = 2;
        FreeMatrix(coordinate);
    } else {
        res.result = 1;
    }
    return true;
}*/

bool RunFusion::setCoordinate(Matrix_X *coordinate)
//先进行工作坐标系的设置，然后在进行ekf
//获取定位矩阵，在LocationServerComm中setCoordinateServerCallback订阅了定位矩阵话题，当发布即可获取矩阵并设置
{
    if (coordinate == NULL)
        return false;
    LOGDEBUG("[CHECK] setCoordinate");
    pthread_mutex_lock(&mIni_Mutex);
    LOGDEBUG("[CHECK] mIni_Mutex pass");
    pthread_mutex_lock(&mCoordinate_Mutex);
    LOGDEBUG("[CHECK] clear state");
    if (pthread_mutex_trylock(&mEKFX_Sync_Mutex) == 0) {
        LOGDEBUG("[CHECK] mEKFX_Sync_Mutex pass");
        if (pthread_mutex_trylock(&mEKFX_Curr_Mutex) == 0) {
            LOGDEBUG("[CHECK] mEKFX_Curr_Mutex pass");
            if (mCoordinate != NULL)
                FreeMatrix(mCoordinate);

            if (mEKFX_Sync != NULL)
            {
                FreeMatrix(mEKFX_Sync);
                mEKFX_Sync = NULL;
            }
            if (mEKFX_Curr != NULL)
            {
                FreeMatrix(mEKFX_Curr);
                mEKFX_Curr = NULL;
            }

            mCoordinate = MatrixCopy(coordinate);
            LOGINFO("mCoordinate MatrixCopy OK");
            pthread_mutex_unlock(&mEKFX_Curr_Mutex);
        }
        pthread_mutex_unlock(&mEKFX_Sync_Mutex);
    }
    pthread_mutex_unlock(&mCoordinate_Mutex);
    pthread_mutex_unlock(&mIni_Mutex);
    return true;
}

void RunFusion::setCoordinateParam()
{
    LOGINFO("[INI] Set Coordinate START");
    Matrix_X *coordinate = CreateMatrix(4, 4);
    
    for (int i = 0; i < 16; i++) 
    {
        string matrix_id = "communication/matrix_" + to_string(i);
        double matrixData;
        ParamService::getInstance().getDouble(matrix_id, matrixData);
        coordinate->data[i] = matrixData;
        //log_info("[INI] matrixData[%d] = %.10f", i, matrixData);
        LOGINFO("[INI] CoordinateData[%d] = %0.10f ", i, coordinate->data[i]);
    }
     
    LOGINFO("[INI] Set Coordinate Success");
    setCoordinate(coordinate);
    if(coordinate != NULL)
    FreeMatrix(coordinate);
    //return true;
}
/*
void RunFusion::setCoordinateParam()
{
    LOGINFO("[INI] Set Coordinate START");
    Matrix_X *coordinate = CreateMatrix(4, 4);
    mParamService.getDoubleRepeated("locationServer/locationTickCycle", coordinate->data[0],10);
    mParamService.getDoubleRepeated("locationServer/GPSDuration", coordinate->data[1],10);
    mParamService.getDoubleRepeated("motion_vehicle/maxLineAcc", coordinate->data[2],10);
    mParamService.getDoubleRepeated("mission_mgr/lineStoppingAccRatio", coordinate->data[2],10);
    mParamService.getDoubleRepeated("motion_vehicle/maxRotationAcc", coordinate->data[2],10);
    mParamService.getDoubleRepeated("mission_mgr/rotationStoppingAccRatio", coordinate->data[2],10);
    mParamService.getDoubleRepeated("locationServer/GPSPosMaxError", coordinate->data[2],10);
    mParamService.getDoubleRepeated("locationServer/GPSAngleMaxError", coordinate->data[2],10);

    for (int j = 0; j < 16; ++j) {
        LOGINFO("[INI] Coordinate[%d] = %0.10f ", j, coordinate->data[j]);
    }
    LOGINFO("[INI] Set Coordinate Success");
    setCoordinate(coordinate);
    FreeMatrix(coordinate);
    //return true;
}
*/

RunFusion::~RunFusion()
{
    mGpsDataList.clear();
    mImuDataList.clear();
    mVehDataList.clear();
    if(mEKFQ_Sync != NULL)
    FreeMatrix(mEKFQ_Sync);
    if(mEKFR_Sync != NULL)
    FreeMatrix(mEKFR_Sync);
    if(mEKFP_Sync != NULL)
    FreeMatrix(mEKFP_Sync);
    locationTickTimer.stop();
    if (mCoordinate != NULL)
        FreeMatrix(mCoordinate);
    if (mRunFusionLocationLogSync != NULL)
        deleteLogFile(mRunFusionLocationLogSync);
    if (mRunFusionLocationLogCurr != NULL)
        deleteLogFile(mRunFusionLocationLogCurr);
    if (mRunFusionGPSDataLog != NULL)
        deleteLogFile(mRunFusionGPSDataLog);
    if (mRunFusionIMUDataLog != NULL)
        deleteLogFile(mRunFusionIMUDataLog);
    if (mRunFusionVEHDataLog != NULL)
        deleteLogFile(mRunFusionVEHDataLog);
}

Matrix_X *getSyncX(Matrix_X *X, Matrix_X *Phi, void *para) {
    if (para == NULL) {
        return NULL;
    }
    SyncPara *fPata = (SyncPara *) para;
    StateVector *stateVector = (StateVector *) X->data;
    Matrix_X *X_t = CreateMatrix(1, sizeof(StateVector) / sizeof(MathUnit));
    StateVector *preStateVector = (StateVector *) X_t->data;
    preStateVector->fai = SortAngle(stateVector->fai + stateVector->vW * fPata->dt);
    preStateVector->posX = stateVector->posX +
                           (stateVector->vX * cos(stateVector->fai) - stateVector->vY * sin(stateVector->fai)) *
                           fPata->dt;
    preStateVector->posY = stateVector->posY +
                           (stateVector->vX * sin(stateVector->fai) + stateVector->vY * cos(stateVector->fai)) *
                           fPata->dt;
    //preStateVector->vX = stateVector->vX + stateVector->aX * fPata->dt;
    //preStateVector->vY = stateVector->vY + stateVector->aY * fPata->dt;
    preStateVector->vX = stateVector->vX;
    preStateVector->vY = stateVector->vY;
    preStateVector->aX = stateVector->aX;
    preStateVector->aY = stateVector->aY;
    preStateVector->vW = stateVector->vW;

    LOGINFO("[StateVector] %lf %lf %lf %lf %lf %lf %lf %lf ",stateVector->fai,stateVector->posX,stateVector->posY,stateVector->vX,stateVector->vY,stateVector->aX,stateVector->aY,stateVector->vW);
    LOGINFO("[preStateVector] %lf %lf %lf %lf %lf %lf %lf %lf ",preStateVector->fai,preStateVector->posX,preStateVector->posY,preStateVector->vX,preStateVector->vY,preStateVector->aX,preStateVector->aY,preStateVector->vW);
    return X_t;
}

Matrix_X *getSyncPhi(Matrix_X *X, void *para) {
    if (para == NULL)
        return NULL;
    return CalJacobian(X, getSyncX, para);
}

Matrix_X *getSyncZ(Matrix_X *X, Matrix_X *H, void *para) {  //根据预测状态向量计算观测
    if (para == NULL)
        return NULL;
    SyncPara *hPata = (SyncPara *) para;

    Matrix_X *Z_t = CreateMatrix(1, sizeof(ObservVector) / sizeof(MathUnit));
    ObservVector *observVector = (ObservVector *) Z_t->data;
    StateVector *stateVector = (StateVector *) X->data;

    if (hPata->imuUpdated) {
//将机器人坐标系下的加速度分量转换到IMU坐标系下 四元素表示机器人的位姿
        observVector->aImuX = stateVector->aX;
        observVector->aImuY = stateVector->aY;
        observVector->vVehW = stateVector->vW;
    }

    if (hPata->posUpdated) {
        observVector->posGpsX = stateVector->posX;
        observVector->posGpsY = stateVector->posY;
    }

    if(hPata->headUpdated)
    {
        observVector->gpsHeading = stateVector->fai;
    }

    if (hPata->vehUpdated) {
        observVector->vVehX = stateVector->vX;
        observVector->vVehY = stateVector->vY;
        observVector->vVehW = stateVector->vW;
    }
    return Z_t;
}

Matrix_X *getSyncH(Matrix_X *X, void *para) {  //获取观测方程的雅克比矩阵
    if (para == NULL)
        return NULL;
    return CalJacobian(X, getSyncZ, para);
}
void BLH2XYZ(double B, double L, double H, double *X, double *Y, double *Z) {
    const double a = EARTHLONGAXIS;
    const double b = EARTHSHORTAXIS;

    double N, e2;
    double Bc, Lc, Hc;
    e2 = 1 - (b * b) / (a * a);

    Bc = B / 180 * M_PI;
    Lc = L / 180 * M_PI;
    /*Bc=B;
    Lc=L;*/
    Hc = H;
    N = a / sqrt(1 - e2 * sin(Bc) * sin(Bc));

    if (X != NULL)
        *X = (N + Hc) * cos(Bc) * cos(Lc);
    if (Y != NULL)
        *Y = (N + Hc) * cos(Bc) * sin(Lc);
    if (Z != NULL)
        *Z = (N * (1 - e2) + Hc) * sin(Bc);
}

void XYZ2BLH(double X, double Y, double Z, double *B, double *L, double *H) {
    const double a = EARTHLONGAXIS;
    const double b = EARTHSHORTAXIS;
    const double e = sqrt((a * a - b * b) / (a * a));
    const double e2 = sqrt((a * a - b * b) / (b * b));
    double p, theta;
    double lon, lat, height, N;
    double sintheta, costheta;

    p = sqrt(X * X + Y * Y);
    theta = atan((Z * a) / (p * b));
    lon = atan2(Y, X);

    sintheta = sin(theta);
    costheta = cos(theta);
    lat = atan2((Z + e2 * e2 * b * sintheta * sintheta * sintheta),
                (p - e * e * a * costheta * costheta * costheta));

    N = a / sqrt(1 - e * e * sin(lat) * sin(lat));
    height = (p / cos(lat)) - N;


    if (B != NULL)
        *B = lat / M_PI * 180;
    if (L != NULL)
        *L = lon / M_PI * 180;
    if (H != NULL)
        *H = height;
    return;
}