//
// Created by guimu on 2021/6/12.
//

#ifndef SRC_LOCATIONDEFINE_H
#define SRC_LOCATIONDEFINE_H

#define EKFXSize_Sync (12)
#define EKFZSize_Sync (10)
typedef struct {
    double BGPS, LGPS, HGPS, HAGPS;
    Matrix_X *GPSLocation;
   // Matrix_X *GPSDir;
    double XGyro, YGyro, ZGyro;
    double XAcc, YAcc, ZAcc;
    double Robot_vx, Robot_vy, Robot_w;
    bool GPSPosUpdata, GPSAngleUpdata;
    bool GyroUpdata;
    bool MovePlatUpdata;
} Source;


typedef struct {
    uint64_t time;
    double GPS_X, GPS_Y, GPS_Z, GPS_Heading;
    double XGyro, YGyro, ZGyro;
    double XAcc, YAcc, ZAcc;
    double Robot_vx, Robot_vy, Robot_w;
    bool GPSPosUpdata, GPSAngleUpdata;
    bool GyroUpdata;
    bool MovePlatUpdata;
    double A, B;
    double A_GPS, B_GPS;
    double XAcc_T, YAcc_T, ZAcc_T;
    double pitch,roll,yaw;
    double pitch_imu,roll_imu,yaw_imu;
    double xWork, yWork, zWork, headWork;//工作航向角逆时针为正，x向为0,弧度制,同步
    double vXQ,vYQ;
    double corIMUk,corIMUd;
    double corVehk[9];
} Source_Sync;

typedef struct {
    double sLocationTickCycle;
    double sGPSDuration;
    double sMaxLineAcc;
    double sLineStoppingAccRatio;
    double sMaxRotationAcc;
    double sRotationStoppingAccRatio;
    double sGPSPosMaxError;
    double sGPSAngleMaxError;
    double sGPSPOverTime;
    double sGPSHOverTime;
    double sIMUOverTime;
    double sVehicleOverTime;
    double sOmegaRatePk;
} ParaStruct;

typedef struct {
    unsigned long time;
    double XYZH[4];
} GpsSyncDataDef;

//typedef struct {
//    unsigned long time;
//    double X,Y,Z;
//    double yaw,pitch,roll;
//} CorrectDataDef;

typedef struct {
    double imuZK;
    double imuD[3];
    double vehK[9];
    double path[6];
}CorrectParam;

typedef struct {
    Source EKFSource;
    double Para[5];
} EKFPara;



/*typedef struct {
    MathUnit q0, q1, q2, q3;
    MathUnit aX, aY, aZ;
    MathUnit posX, posY, posZ;
    MathUnit vX, vY;
} StateVector;

typedef struct {
    MathUnit aImuX, aImuY, aImuZ;
    MathUnit A, B;
    MathUnit posGpsX, posGpsY, posGpsZ;
    MathUnit vVehX, vVehY;
} ObservVector;
*/
typedef struct {
    MathUnit dt;
    MathUnit wx, wy, wz;
} FPata;

typedef struct {
    MathUnit dt;
    MathUnit g;
} HPata;

typedef struct {
    MathUnit dt;
    //MathUnit wx, wy, wz;
    //MathUnit vx, vy, vw;
    //MathUnit ax, ay;
    //MathUnit g;
    bool posUpdated, headUpdated, imuUpdated, vehUpdated;
} SyncPara;

typedef enum {
    LocationBusy, LocationReady
} LocationStatusFlag;
typedef enum {
    WitchGPSTask, NoGPSTask
} GPSTaskFlag;
typedef struct {
    double data[3];
} IMU_Data;
struct ImuDataDef{
    unsigned long time;
    double Gyro[3];
    double Acc[3];
} ;

typedef struct {
    unsigned long time;
    double BLHH[4];
    double GPSLoc[3];
    double GPSDir[3];
    bool posUpdated;
    bool headUpdated;
} GpsDataDef;

typedef struct {
    unsigned long time;
    double vxyw[3];
    MathUnit q0, q1, q2, q3;
    double w;
} VehDataDef;

typedef struct {
    unsigned long time;
    MathUnit XYZH[4];
    MathUnit yaw_Sync,pitch_Sync,roll_Sync;

} GpsCorrectDef;
typedef struct {
    unsigned long time;
    MathUnit XYZH[4];
    bool posUpdated;
    bool headUpdated;
} GpsCalDef;
typedef struct {
    MathUnit q0, q1, q2, q3;
    MathUnit posX, posY, posZ;
    MathUnit heading;
} CorrectVector;

#endif //SRC_LOCATIONDEFINE_H
