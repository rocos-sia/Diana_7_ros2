#pragma once
#include "DianaAPIDef.h"
#include "DianaAPI.h"
#include <cstring>
#include <iostream>
#include <sri/ftsensor.hpp>
#include <sri/commethernet.hpp>
#include <kdl/frames.hpp>
class platmotion
{
private:
    /* data */
public:
    platmotion(/* args */);
    ~platmotion();
    KDL::Wrench Sensor_wrench;
    double gravity = 9.81;
    // 力传感器
    SRI::CommEthernet *ce=nullptr;
    SRI::FTSensor *sensor=nullptr;
    bool isFtSensor = false;
    double ZeroOffset[6] = {0.0};
    double mass;
    KDL::Vector center_of_mass_position;
    const char *SensorIpAddress = "192.168.2.109";
    uint16_t SensorPort = 4008;
    // 机械臂
    KDL::Vector normal;
    const char *strIpAddress = "192.168.100.75";
    srv_net_st *pinfo;
    // 力控
    std::vector<double> K;
    std::vector<double> M;
    std::vector<double> B;
    std::vector<double> pose_stop;
    // 工件点
    KDL::Frame object_frame;

public:
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
    KDL::Wrench getTheoryWrench(double *pose_tcp, double mass, KDL::Vector center_of_mass_position)
    {
        //! The output of the function is the theory wrench
        KDL::Vector translation(0.0, 0.0, mass * -9.81);
        KDL::Vector axis = KDL::Vector(pose_tcp[3], pose_tcp[4], pose_tcp[5]);
        double norm = axis.Normalize();
        KDL::Frame TCP_base = KDL::Frame(KDL::Rotation::Rot(axis, norm), KDL::Vector(pose_tcp[0], pose_tcp[1], pose_tcp[2]));
        KDL::Rotation f_TCP = KDL::Rotation::RPY(M_1_PI, 0, 0);
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
    void rtDataHandler(std::vector<RTData<float>> &rtData)
    {
        isFtSensor = true;
        // static int i = 0;
        // std::cout << "[" << i << "] RT Data is ->  ";
        for (int i = 0; i < rtData.size(); i++)
        {
            Sensor_wrench.force.data[0] = rtData[i][0];
            Sensor_wrench.force.data[1] = rtData[i][1];
            Sensor_wrench.force.data[2] = rtData[i][2];
            Sensor_wrench.torque.data[0] = rtData[i][3];
            Sensor_wrench.torque.data[1] = rtData[i][4];
            Sensor_wrench.torque.data[2] = rtData[i][5];
        }
    }
    void logRobotState(StrRobotStateInfo *pinfo, const char *strIpAddress)
    {
    }
    void errorControl(int e, const char *strIpAddress)
    {
        const char *strError = formatError(e); // 该函数后面会介绍
        printf("error code (%d):%s\n", e, strError);
    }
};

platmotion::platmotion(/* args */)
{
    // 传感器初始化
   
    ce = new SRI::CommEthernet(SensorIpAddress, SensorPort);
    sensor = new SRI::FTSensor(ce);
    auto rtDataValid = sensor->getRealTimeDataValid();
    auto rtMode = sensor->getRealTimeDataMode();
    sensor->startRealTimeDataRepeatedly<float>(&platmotion::rtDataHandler, rtMode, rtDataValid);
    // 机械臂初始化
    pinfo = new srv_net_st;
    memset(pinfo->SrvIp, 0x00, sizeof(pinfo->SrvIp));
    memcpy(pinfo->SrvIp, strIpAddress, strlen(strIpAddress));
    pinfo->LocHeartbeatPort = 0;
    pinfo->LocRobotStatePort = 0;
    pinfo->LocSrvPort = 0;
    int ret = initSrv(this->errorControl, this->logRobotState, pinfo);
    if (ret < 0)
    {
        printf("%s initSrv failed! Return value = %d\n", strIpAddress, ret);
    }
    //零漂
    double pose_tcp[6] = {0.0};
    
    

}

platmotion::~platmotion()
{
}
