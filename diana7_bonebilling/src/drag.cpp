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
// double zero_offset[6] = {-3.0987, -1.25601, 11.7307, -0.000, 0.000, 0.000};
double zero_offset[6] = {-3.0987, -1.25601, 14.1741, -0.000, 0.000, 0.000};
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

    KDL::Vector x, y;
    KDL::Vector z = normal;
    z.Normalize();
    y = z * gx;
    double norm = y.Normalize();
    if (norm < 0.1)
    {
        KDL::Vector gy{0, 1, 0};
        x = gy * z;
        x.Normalize();
        y = z * x;
        y.Normalize();

    }
    else
    {
        x = y * z;
        x.Normalize();
    }

    KDL::Rotation rot(x, y, z);
    KDL::Frame frame(rot, p);
    return frame; // 相对于基坐标系的位姿
}

KDL::Wrench getTheoryWrench(double *pose_tcp, double mass, KDL::Vector center_of_mass_position)
{
    //! The output of the function is the theory wrench
    KDL::Vector translation(0.0, 0.0, mass * -9.81);
    KDL::Vector axis = KDL::Vector(pose_tcp[3], pose_tcp[4], pose_tcp[5]);
    double norm = axis.Normalize();
    KDL::Frame TCP_base = KDL::Frame(KDL::Rotation::Rot(axis, norm), KDL::Vector(pose_tcp[0], pose_tcp[1], pose_tcp[2]));
    KDL::Rotation f_TCP = KDL::Rotation::RPY(M_PI, 0, 0);
    KDL::Frame f_base;
    f_base.p = TCP_base.p;
    f_base.M = TCP_base.M * f_TCP;
    f_base.M.SetInverse();
    KDL::Vector rotated_translation = f_base.M * translation;
    KDL::Rotation cross_mass = KDL::Rotation(0, -center_of_mass_position[2], center_of_mass_position[1],
                                             center_of_mass_position[2], 0, -center_of_mass_position[0],
                                             -center_of_mass_position[1], center_of_mass_position[0], 0);
    KDL::Wrench wrench_;
    wrench_.force = rotated_translation;
    wrench_.torque = cross_mass * wrench_.force;
    std::cout << "Theory_wrench: " << wrench_.force.data[0] << "," << wrench_.force.data[1] << "," << wrench_.force.data[2] << "," << wrench_.torque.data[0] << "," << wrench_.torque.data[1] << "," << wrench_.torque.data[2] << std::endl;
    return wrench_;
}
void getZeroOffset(double *pose_tcp, double *wrench, double mass, KDL::Vector center_of_mass_position, double *ZeroOffset)
{

    KDL::Wrench Theory_wrench = getTheoryWrench(pose_tcp, mass, center_of_mass_position);
    ZeroOffset[0] = wrench[0] - Theory_wrench.force.data[0];
    ZeroOffset[1] = wrench[1] - Theory_wrench.force.data[1];
    ZeroOffset[2] = wrench[2] - Theory_wrench.force.data[2];
    ZeroOffset[3] = wrench[3] - Theory_wrench.torque.data[0];
    ZeroOffset[4] = wrench[4] - Theory_wrench.torque.data[1];
    ZeroOffset[5] = wrench[5] - Theory_wrench.torque.data[2];
}
KDL::Wrench gravityCompensation(double *pose_tcp, double *wrench, double *Zero_offset, double mass, KDL::Vector center_of_mass_position)
{

    //! The feedback of function is the exit wrench
    // Define the gravity vector
    // KDL::Vector gravity(0.0, 0.0, -9.81);
    // // Define the mass of the end-effector
    // double mass = 0.249072;
    KDL::Vector translation(0.0, 0.0, mass * -9.81);
    // Define the mass center of the end-effector
    // KDL::Vector center_of_mass_position(0.0, 0.0, 0.0366358);
    KDL::Wrench Theory_wrench = getTheoryWrench(pose_tcp, mass, center_of_mass_position);
    KDL::Wrench wrench_;
    wrench_.force.data[0] = wrench[0] - Theory_wrench.force.data[0] - Zero_offset[0];
    wrench_.force.data[1] = wrench[1] - Theory_wrench.force.data[1] - Zero_offset[1];
    wrench_.force.data[2] = wrench[2] - Theory_wrench.force.data[2] - Zero_offset[2];
    wrench_.torque.data[0] = wrench[3] - Theory_wrench.torque.data[0] - Zero_offset[3];
    wrench_.torque.data[1] = wrench[4] - Theory_wrench.torque.data[1] - Zero_offset[4];
    wrench_.torque.data[2] = wrench[5] - Theory_wrench.torque.data[2] - Zero_offset[5];
    return wrench_;
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
    // double joints[JOINT_NUM] = {0.0};

    ret = getJointPos(joints, strIpAddress);
    if (ret < 0)
    {
        printf("getJointPos failed! Return value = %d\n", ret);
    }
    else
    {
        std::cout << "joints: " << joints[0] << " " << joints[1] << " " << joints[2] << " " << joints[3] << " " << joints[4] << " " << joints[5] << " " << joints[6] << std::endl;
    }
    // double jointInit[JOINT_NUM] = {3.03605, 0.0022888, -1.85946, 1.89547, 0.00231277, -1.24677, 1.17586};
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
    KDL::Vector normal(0, 0, 1);
    KDL::Frame frame = calplat(normal, p);
    KDL::Vector r = frame.M.GetRot();
    // KDL::Wrench wrench;
    double pose_target[6] = {frame.p.x(), frame.p.y(), frame.p.z(), r.x(), r.y(), r.z()};
    double vel = 0.1;
    double acc = 0.1;
    double radius = 0.0;
    int zv_shaper_order = 0;
    double zv_shaper_frequency = 0;
    double zv_shaper_damping_ratio = 0;
    ret = moveLToPose(pose_target, vel, acc, nullptr, zv_shaper_order, zv_shaper_frequency, zv_shaper_damping_ratio, strIpAddress);
    wait_move(strIpAddress);

    double B = 10000.0;


    ret = getTcpPos(poses, strIpAddress);

    while (isRunning)
    {
        if (true)
        {
            double delta_x = -wrench[2] / B;
            poses[2] += delta_x;

            servoL(poses, 0.01, 0.1, 300, 1.0, nullptr, strIpAddress);
            isFtSensor = false;

            // std::cout << "wrench: " << delta_x << std::endl;
            usleep(1000);
        }

        // std::cout << "x: " << poses[2] << std::endl;
    }

    destroySrv(strIpAddress);
    return 0;
}
