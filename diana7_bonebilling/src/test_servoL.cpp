#include "DianaAPIDef.h"
#include "DianaAPI.h"
#include <cstring>
#include <iostream>
#include <sri/ftsensor.hpp>
#include <sri/commethernet.hpp>
#include <kdl/frames.hpp>

using namespace SRI;
#define JOINT_NUM 7
bool isRunning = true;
double wrench[6] = {0.0};
double zero_offset[6] = {-3.0987, -1.25601, 11.7307, -0.000, 0.000, 0.000};
bool isFtSensor = false;
void rtDataHandler(std::vector<RTData<float>> &rtData)
{
    isFtSensor = true;
    static int i = 0;
    // std::cout << "[" << i << "] RT Data is ->  ";
    for (int i = 0; i < rtData.size(); i++)
    {
        for (int j = 0; j < 6; j++)
        {
            // std::cout << "Ch " << j << ": " << rtData[i][j] << "\t";
            wrench[j] = rtData[i][j] - zero_offset[j];
        }
        // std::cout << std::endl;
    }
    i++;
}
void logRobotState(StrRobotStateInfo *pinfo, const char *strIpAddress)
{
    // strIpAddress = "192.168.100.75";
    // static int staCnt = 1;
    // if ((staCnt++ % 1000 == 0) && pinfo)
    // {
    //     for (int i = 0; i < 7; ++i)
    //     {
    //         printf("jointPos[%d] = %f \n", i, pinfo->jointPos[i]);
    //         printf("jointCurrent [%d] = %f \n", i, pinfo->jointCurrent[i]);
    //         printf("jointTorque [%d] = %f \n", i, pinfo->jointTorque[i]);
    //         if (i < 6)
    //         {
    //             printf("tcpPos [%d] = %f \n", i, pinfo->tcpPos[i]);
    //         }
    //     }
    // }
}
void errorControl(int e, const char *strIpAddress)
{
    strIpAddress = "192.168.100.75";
    const char *strError = formatError(e); // 该函数后面会介绍
    printf("error code (%d):%s\n", e, strError);
}
KDL::Frame calplat(KDL::Vector normal, KDL::Vector p)
{
    KDL::Vector gx{1, 0, 0};
    KDL::Vector z = normal;
    z.Normalize();
    KDL::Vector y = z * gx;
    y.Normalize();
    KDL::Vector x = y * z;
    x.Normalize();

    KDL::Rotation rot(x, y, z);
    KDL::Frame frame(rot, p);
    return frame; // 相对于基坐标系的位姿
}
void wait_move(const char *strIpAddress)
{
    usleep(20000);
    while (true)
    {
        const char state = getRobotState(strIpAddress);
        if (state != 0)
        {
            break;
        }
        else
        {
            usleep(1000);
        }
    }
    stop(strIpAddress);
}
int main(int argc, char const *argv[])
{
    // 传感器初始化
    SRI::CommEthernet *ce = new SRI::CommEthernet("192.168.2.109", 4008);
    SRI::FTSensor sensor(ce);
    double x, y, z, mx, my, mz;
    auto rtDataValid = sensor.getRealTimeDataValid();
    auto rtMode = sensor.getRealTimeDataMode();
    sensor.startRealTimeDataRepeatedly<float>(&rtDataHandler, rtMode, rtDataValid);
    const char *strIpAddress = "192.168.100.75";
    srv_net_st *pinfo = new srv_net_st();
    memset(pinfo->SrvIp, 0x00, sizeof(pinfo->SrvIp));
    memcpy(pinfo->SrvIp, "192.168.100.75", strlen("192.168.100.75"));
    pinfo->LocHeartbeatPort = 0;
    pinfo->LocRobotStatePort = 0;
    pinfo->LocSrvPort = 0;
    int ret = initSrv(errorControl, logRobotState, pinfo);
    if (ret < 0)
    {
        printf("192.168.100.75 initSrv failed! Return value = %d\n", ret);
    }
    if (pinfo)
    {
        delete pinfo;
        pinfo = nullptr;
    }
    //
    releaseBrake(strIpAddress);

    const double PI = 3.141592653;
    double joints[7] = {0, 0, 0, PI / 2, 0, -PI / 2, 0}; // 以 7 轴机器人为例
    moveJToTarget(joints, 0.5, 0.5, 0, 0, 0, strIpAddress);
    wait_move(strIpAddress);
    double pose[6] = {0};
    getTcpPos(pose, strIpAddress);
    for (int i = 0; i < 1000; ++i)
    {
        pose[2] = pose[2] + 0.0001;
        ret = servoL(pose, 0.01, 0.1, 300, 1.0, nullptr, strIpAddress);
        if (ret < 0)
            break;
        sleep(0.001);
    }
    // stop(strIpAddress);
    destroySrv(strIpAddress);
    return 0;
}
