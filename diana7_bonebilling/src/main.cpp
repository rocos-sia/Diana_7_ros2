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
    // sensor.startRealTimeDataRepeatedly<float>(&rtDataHandler, rtMode, rtDataValid);
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
    double joints[JOINT_NUM] = {0.0};

    ret = getJointPos(joints, strIpAddress);
    if (ret < 0)
    {
        printf("getJointPos failed! Return value = %d\n", ret);
    }
    else
    {
        std::cout << "joints: " << joints[0] << " " << joints[1] << " " << joints[2] << " " << joints[3] << " " << joints[4] << " " << joints[5] << " " << joints[6] << std::endl;
    }
    double jointInit[JOINT_NUM] = {3.03605, 0.0022888, -1.85946, 1.89547, 0.00231277, -1.24677, 1.17586};
    // TODO movej到初始点位
    double poses[6] = {0.0};
    // 初始点位

    ret = getTcpPos(poses, strIpAddress);
    if (ret < 0)
    {
        printf("getJointPos failed! Return value = %d\n", ret);
    }
    else
    {
        std::cout << "joints: " << joints[0] << " " << joints[1] << " " << joints[2] << " " << joints[3] << " " << joints[4] << " " << joints[5] << " " << joints[6] << std::endl;
    }
    KDL::Vector p(poses[0], poses[1], poses[2]);
    KDL::Vector normal(1, 1, 1);
    KDL::Frame frame = calplat(normal, p);
    KDL::Vector r = frame.M.GetRot();
    KDL::Wrench wrench;
    double pose_target[6] = {frame.p.x(), frame.p.y(), frame.p.z(), r.x(), r.y(), r.z()};
    double vel = 0.1;
    double acc = 0.1;
    double radius = 0.0;
    int zv_shaper_order = 0;
    double zv_shaper_frequency = 0;
    double zv_shaper_damping_ratio = 0;
    ret = moveLToPose(pose_target, vel, acc, nullptr, zv_shaper_order, zv_shaper_frequency, zv_shaper_damping_ratio, strIpAddress);
    wait_move(strIpAddress);
    destroySrv(strIpAddress);
    return 0;
}
