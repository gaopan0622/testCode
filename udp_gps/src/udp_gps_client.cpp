//
// Created by gaopan on 2023/6/4.
//
#include "udp_gps_client.h"
int main(int argc, char **argv)
{
    ros::init(argc, argv, "gpsData");
    GLogHelper::Init("/home/log",ros::this_node::getName());
    UDPCLIENT *udpClient = new UDPCLIENT();
    ros::spin();
    return 0;
}
UDPCLIENT::UDPCLIENT()
{
    //gpsNodeParams.getDoubleOrDefault("gps/GPSHeadingAngle", robotAntennaPos.GpsAntHAd, 90);
    //gpsNodeParams.getDoubleOrDefault("gps/GPSYd", robotAntennaPos.GpsAntYd, -0.5);
    //gpsNodeParams.getDoubleOrDefault("gps/GPSXd", robotAntennaPos.GpsAntXd, 0.3);
    //this->headDir = CreateMatrix(1,4);
    this->bindUDP();
    errorHandler.initialize(BetterEnum2Vector<GpsError, int>(), 0);
    //LOGINFO("[GPSOFFSET]GPSXd:%.2lf\tGPSYd:%.2lf",  robotAntennaPos.GpsAntXd,robotAntennaPos.GpsAntYd);
    gpsDataPub = n.advertise<common::GpsData>("gpsDataTopic",2);
    tickTimer = n.createTimer(ros::Duration(0.1), &UDPCLIENT::RequestDataCommand,this);
    //this->RequestDataCommand();
}

void UDPCLIENT::bindUDP()
{
    Port = 9090;
    strcpy(Host, "192.168.2.188");
    
    if(Open(Port, "192.168.2.10")) LOGINFO("bind UDP success");
    else LOGINFO("bind udp failed");
}
void UDPCLIENT::RequestDataCommand(const ros::TimerEvent &e)
{
    locationCommand.head = 0x00ff;
    for(int i = 0; i < 26 ; i++)
    {
        locationCommand.spareField [i] = {0};
    }
    locationCommand.command = 0x2020;
    /*
    std::stringstream ss;
    for (int i = 0; i < sizeof(locationCommand); i++) {
        ss << std::hex << std::setfill('0') << std::setw(2) << (int)(((unsigned char*)&locationCommand)[i])<< " ";
    }
    cout<<ss.str()<<endl;*/
    int dataLen;
    dataLen = SendData(8088, Host, (char *)&locationCommand, sizeof(locationCommand));
    /*if(dataLen == 32)
    {
        //printf("Request gps data command sent successfully\n");
        LOGINFO("Request gps data command sent successfully");
    }*/
}
void UDPCLIENT::OnReceive(int Port, char IP[], char *Data, int DataLen)
{
 /*     cout << DataLen << endl;
        char databuf[DataLen];
        memset(databuf, 0, sizeof(databuf));
        memcpy(databuf, Data, DataLen);

        for (int i = 0; i < DataLen; i++) {
            //printf("%#x  ",((unsigned char*)dataBuf)[i]);
            printf("%#x  ",databuf[i]&0xff);
        }
        cout<<endl;
        cout<<sizeof(GpsResponse)<<endl;
*/

    if(Data == nullptr) {
        return;
    }
    if(DataLen == sizeof(GpsResponse))
    {
        char dataBuf[DataLen];
        memset(dataBuf, 0, sizeof(dataBuf));
        memcpy(dataBuf, Data, DataLen);
        if((dataBuf[0]&0xff) == 0xfe)
        {
            GpsResponse *gpsResponse = (GpsResponse *)(dataBuf);
            B = gpsResponse->latitude;
            L = gpsResponse->longitude;
            H = gpsResponse->altitude;
            //HA =gpsResponse->heading;

            LOGINFO("latitude %.8lf longitude %.8lf altitude %.8lf PosUpDate %d AltUpDate %d RTK %d", \
            B, L, H, gpsResponse->locationUpdate,gpsResponse->heightUpdate,gpsResponse->gpsState);

            this->gpsData.time = getTimeUs();
            this->gpsData.posUpdated = gpsResponse->locationUpdate&&gpsResponse->heightUpdate;
           // this->gpsData.headUpdated = gpsResponse->headingUpdate;
            //calcRobotPos();
            sendGpsData();
            if(this->gpsData.posUpdated==1)
            {
                errorHandler.errorDisappeared(GpsError::POOR_POS_DATA);
            }
            else
            {
                errorHandler.errorOccured(GpsError::POOR_POS_DATA);
            }

        }
        else{
            return ;
        }
    }
    else {
        return ;
    }
}

void UDPCLIENT::sendGpsData()
{
    this->gpsData.longitude = L;
    this->gpsData.latitude = B;
    this->gpsData.altitude = H;
     Matrix_X * testPointGPS;
    testPointGPS = CreateMatrix(1,3);
    BLH2XYZ(B,L,H,testPointGPS->data, testPointGPS->data+1, testPointGPS->data+2);
    this->gpsData.x = testPointGPS->data[0];
    this->gpsData.y = testPointGPS->data[1];
    this->gpsData.z = testPointGPS->data[2];
    this->gpsDataPub.publish(gpsData);
    LOGINFO("[GPSSEND]longitude:%.8lf, latitude:%.8lf, altitude:%.3lf, x:%.3lf, y:%.3lf, z:%.3lf,posUpdated:%d", \
    gpsData.longitude, gpsData.latitude, gpsData.altitude, gpsData.x, gpsData.y, gpsData.z, gpsData.posUpdated);
}

double UDPCLIENT::getTimeUs()
{
    static struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec * 1000000.0 + tv.tv_usec ;
}

unsigned int UDPCLIENT::sumCheck(char *data,int dataLen)
{
    unsigned int sum = 0;
    for(int i = 0; i < dataLen - 4; ++i)
    {
        sum += *(data + i);
    }
    sum = (0 - sum)&0x000000ff;
    return sum;
}

UDPCLIENT::~UDPCLIENT()
{
    this->Close();
}

void UDPCLIENT::OnOpen() {}

void UDPCLIENT::BLH2XYZ(MathUnit B,MathUnit L,MathUnit H,MathUnit *X,MathUnit *Y,MathUnit *Z)
{
    const double a = EARTH_LONG_AXIS;
    const double b = EARTH_MINOR_AXIS;
    const double e2 = 1-(b * b)/(a * a);
    double N;
    double Bc, Lc, Hc;


    Bc = B / 180 * M_PI;
    Lc = L / 180 * M_PI;
    Hc = H;
    N= a / sqrt(1- e2 * sin(Bc) * sin(Bc));

    if (X != nullptr)
        *X = (N + Hc) * cos(Bc) * cos(Lc);
    if (Y != nullptr)
        *Y = (N + Hc) * cos(Bc) * sin(Lc);
    if (Z != nullptr)
        *Z = (N * (1 - e2) + Hc) * sin(Bc);
}
