#include "DianaAPIDef.h"
#include "DianaAPI.h"
#include <cstring>
#include <iostream>
#include <sri/ftsensor.hpp>
#include <sri/commethernet.hpp>
#include <kdl/frames.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "yaml-cpp/yaml.h"
#include "Iir.h"
<<<<<<< Updated upstream
=======
#include <cmath>  // 包含这个头文件以使用 std::pow
>>>>>>> Stashed changes

using namespace SRI;
#define JOINT_NUM 7
bool isRunning = true;
double wrench[6] = {0.0};
const char *strIpAddress = "192.168.100.75";
double enable_wrench[6] = {0};
// double zero_offset[6] = {-3.0987, -1.25601, 11.7307, -0.000, 0.000, 0.000};
// double zero_offset[6] = {-3.0987, -1.25601, 14.1741, -0.000, 0.000, 0.000};
bool isFtSensor = false;
bool noError = true;
std::string yaml_path = "normal.yaml";
YAML::Node yaml_node;
// auto node;
rclcpp::Node::SharedPtr node;

void rtDataHandler(std::vector<RTData<float>> &rtData)
{
    isFtSensor = true;

    // rclcpp::Time current_time =node->get_clock()->now();
    // rclcpp::Duration duration = rclcpp::Duration::from_seconds(global_publish_frequency);

    // std::cout << "[" << i << "] RT Data is ->  ";
    for (int i = 0; i < rtData.size(); i++)
    {
        for (int j = 0; j < 6; j++)
        {
            // std::cout << "Ch " << j << ": " << rtData[i][j] << "\t";
            enable_wrench[j] = rtData[i][j];
        }
    }
}

void logRobotState(StrRobotStateInfo *pinfo, const char *strIpAddress)
{
}
void errorControl(int e, const char *strIpAddress)
{
    strIpAddress = "192.168.100.75";
    const char *strError = formatError(e); // 该函数后面会介绍
    printf("error code (%d):%s\n", e, strError);

    noError = false;
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
    // std::cout << "Theory_wrench: " << wrench_.force.data[0] << "," << wrench_.force.data[1] << "," << wrench_.force.data[2] << "," << wrench_.torque.data[0] << "," << wrench_.torque.data[1] << "," << wrench_.torque.data[2] << std::endl;
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
void signalHandler(int signo)
{
    if (signo == SIGINT)
    {
        std::cout << "\033[1;31m"
                  << "[!!SIGNAL!!]"
                  << "INTERRUPT by CTRL-C"
                  << "\033[0m" << std::endl;

        isRunning = false;
        exit(0);
    }
}
template <typename T>
int sign(T val) {
    return (val > T(0)) - (val < T(0));
}
int main(int argc, char const *argv[])
{
    // yaml初始化
    yaml_node = YAML::LoadFile(yaml_path);
    std::cout << "normal.yaml文件读取成功" << std::endl;
    // 信号处理
    if (signal(SIGINT, signalHandler) == SIG_ERR)
    {
        std::cout << "\033[1;31m"
                  << "Can not catch SIGINT"
                  << "\033[0m" << std::endl;
    }
    // ROS2初始化
    rclcpp::init(argc, argv);
    node = rclcpp::Node::make_shared("joint_state_publisher");
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher;
    publisher = node->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    auto wrench_z = node->create_publisher<geometry_msgs::msg::WrenchStamped>("wrench_z", 10);
    auto wrench_xy = node->create_publisher<geometry_msgs::msg::WrenchStamped>("wrench_xy", 10);
    auto plane_xy = node->create_publisher<geometry_msgs::msg::PolygonStamped>("plane_xy", 10);
    rclcpp::WallRate loop_rate(500); // 1 Hz publishing rate
    geometry_msgs::msg::WrenchStamped wrench_msg_z;
    geometry_msgs::msg::WrenchStamped wrench_msg_xy;
    geometry_msgs::msg::PolygonStamped plane_msg_xy;
    plane_msg_xy.header.stamp = node->get_clock()->now();
    wrench_msg_xy.header.stamp = node->get_clock()->now();
    wrench_msg_xy.header.frame_id = "link_7";
    wrench_msg_z.header.stamp = node->get_clock()->now();
    wrench_msg_z.header.frame_id = "link_7";
    // 低通滤波器
    std::array<Iir::Butterworth::LowPass<2>, 3> filter_array{};
    const float samplingrate = 200;    // Hz
    const float cutoff_frequency = 5; // Hz
    filter_array[0].setup(2, samplingrate, cutoff_frequency);
    filter_array[1].setup(2, samplingrate, cutoff_frequency);
    filter_array[2].setup(2, samplingrate, cutoff_frequency);
    // 传感器初始化
    SRI::CommEthernet *ce = new SRI::CommEthernet("192.168.2.109", 4008);
    SRI::FTSensor sensor(ce);
    double x, y, z, mx, my, mz;
    auto rtDataValid = sensor.getRealTimeDataValid();
    auto rtMode = sensor.getRealTimeDataMode();
    // sensor.startRealTimeDataRepeatedly<float>(&rtDataHandler, rtMode, rtDataValid);
    srv_net_st *pinfo = new srv_net_st();
    memset(pinfo->SrvIp, 0x00, sizeof(pinfo->SrvIp));
    memcpy(pinfo->SrvIp, "192.168.100.75", strlen("192.168.100.75"));
    pinfo->LocHeartbeatPort = 0;
    pinfo->LocRobotStatePort = 0;
    pinfo->LocSrvPort = 0;
    std::cout << "initSrv" << std::endl;
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
    std::cout<<"releaseBrake"<<std::endl;
    const double PI = 3.141592653;
    double joints[7] = {0, 0, 0, PI / 2, 0, -PI / 2, 0}; // 以 7 轴机器人为例

    // double joints[JOINT_NUM] = {0.0};
    double poses[6] = {0.0};
    double last_poses[6] = {0.0};
    double last_real_poses[6] = {0.0};
    double real_poses[6] = {0.0};
    // 初始点位
    moveJToTarget(joints, 0.5, 0.5, 0, 0, 0, strIpAddress);
    wait_move(strIpAddress);
    ret = getTcpPos(poses, strIpAddress);
    if (ret < 0)
    {
        printf("getJointPos failed! Return value = %d\n", ret);
    }
    else
    {
        std::cout << "joints: " << joints[0] << " " << joints[1] << " " << joints[2] << " " << joints[3] << " " << joints[4] << " " << joints[5] << " " << joints[6] << std::endl;
    }

    // 上电采集力信息数据

    int sum = 1000;
    for (int i = 0; i < sum; i++)
    {
        auto rtData = sensor.getRealTimeDataOnce<float>(rtMode, rtDataValid);

        for (int j = 0; j < rtData.size(); j++)
        {
            enable_wrench[0] += rtData[j][0];
            enable_wrench[1] += rtData[j][1];
            enable_wrench[2] += rtData[j][2];
            enable_wrench[3] += rtData[j][3];
            enable_wrench[4] += rtData[j][4];
            enable_wrench[5] += rtData[j][5];
        }
    }
    enable_wrench[0] /= sum;
    enable_wrench[1] /= sum;
    enable_wrench[2] /= sum;
    enable_wrench[3] /= sum;
    enable_wrench[4] /= sum;
    enable_wrench[5] /= sum;
    std::cout << "enable_wrench: " << enable_wrench[0] << "," << enable_wrench[1] << "," << enable_wrench[2] << "," << enable_wrench[3] << "," << enable_wrench[4] << "," << enable_wrench[5] << std::endl;
    double mass = 0.249072 - 0.09;
    KDL::Vector center_of_mass_position(0.0, 0.0, 0.0366358);
    double Zero_offset[6] = {0.0};
    getZeroOffset(poses, enable_wrench, mass, center_of_mass_position, Zero_offset);
    std::cout << "Zero_offset: " << Zero_offset[0] << "," << Zero_offset[1] << "," << Zero_offset[2] << "," << Zero_offset[3] << "," << Zero_offset[4] << "," << Zero_offset[5] << std::endl;

    // 向量平面确定
    KDL::Vector p(poses[0], poses[1], poses[2]);
    // normal.yaml数据读取
    KDL::Vector normal(0, 1, 1);
    if (yaml_node["normal"])
    {
        std::vector<double> normal_data = yaml_node["normal"].as<std::vector<double>>();
        normal = KDL::Vector(normal_data[0], normal_data[1], normal_data[2]);
    }
    else
    {
        std::cout << "normal.yaml文件中没有normal数据,默认为0,1,1" << std::endl;
    }

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
    memset(enable_wrench, 0, sizeof(enable_wrench));
    for (int i = 0; i < sum; i++)
    {
        auto rtData = sensor.getRealTimeDataOnce<float>(rtMode, rtDataValid);

        for (int j = 0; j < rtData.size(); j++)
        {
            enable_wrench[0] += rtData[j][0];
            enable_wrench[1] += rtData[j][1];
            enable_wrench[2] += rtData[j][2];
            enable_wrench[3] += rtData[j][3];
            enable_wrench[4] += rtData[j][4];
            enable_wrench[5] += rtData[j][5];
        }
    }
    enable_wrench[0] /= sum;
    enable_wrench[1] /= sum;
    enable_wrench[2] /= sum;
    enable_wrench[3] /= sum;
    enable_wrench[4] /= sum;
    enable_wrench[5] /= sum;
    getTcpPos(poses, strIpAddress);
    // Todo
    last_real_poses[0]=poses[0];
    real_poses[0]=poses[0];
    //
    KDL::Wrench wrench_compensation = gravityCompensation(poses, enable_wrench, Zero_offset, mass, center_of_mass_position);
    std::cout << "wrench_compensation: " << wrench_compensation.force.data[0] << "," << wrench_compensation.force.data[1] << "," << wrench_compensation.force.data[2] << "," << wrench_compensation.torque.data[0] << "," << wrench_compensation.torque.data[1] << "," << wrench_compensation.torque.data[2] << std::endl;
    double B = 20000.0;
    KDL::Rotation f_tcp = KDL::Rotation::RPY(PI, 0, 0);
    // getTcpPos(poses, strIpAddress);
    KDL::Vector axis = KDL::Vector(poses[3], poses[4], poses[5]);
    double norm = axis.Normalize();
    double rot_poses[6] = {0.0};
    KDL::Frame TCP_base = KDL::Frame(KDL::Rotation::Rot(axis, norm), KDL::Vector(poses[0], poses[1], poses[2]));
    sensor.startRealTimeDataRepeatedly<float>(&rtDataHandler, rtMode, rtDataValid);
    // 任意四个点确定一个平面
<<<<<<< Updated upstream
    double plane_length = 0.2;
=======
    double plane_length = 0.5;
>>>>>>> Stashed changes
    KDL::Frame p1 = TCP_base * KDL::Frame(KDL::Vector(plane_length, plane_length, 0));
    KDL::Frame p2 = TCP_base * KDL::Frame(KDL::Vector(-plane_length, plane_length, 0));
    KDL::Frame p3 = TCP_base * KDL::Frame(KDL::Vector(-plane_length, -plane_length, 0));
    KDL::Frame p4 = TCP_base * KDL::Frame(KDL::Vector(plane_length, -plane_length, 0));
    geometry_msgs::msg::Point32 point1;
    geometry_msgs::msg::Point32 point2;
    geometry_msgs::msg::Point32 point3;
    geometry_msgs::msg::Point32 point4;
<<<<<<< Updated upstream
=======

    point1.x = p1.p.x();
    point1.y = p1.p.y();
    point1.z = p1.p.z();
    point2.x = p2.p.x();
    point2.y = p2.p.y();
    point2.z = p2.p.z();
    point3.x = p3.p.x();
    point3.y = p3.p.y();
    point3.z = p3.p.z();
    point4.x = p4.p.x();
    point4.y = p4.p.y();
    point4.z = p4.p.z();
    plane_msg_xy.header.frame_id = "base"; // 设置坐标系为 "map"

    // 添加多边形顶点
    // plane_msg_xy.polygon.points.resize(4);
    plane_msg_xy.polygon.points.push_back(point1);
    plane_msg_xy.polygon.points.push_back(point2);
    plane_msg_xy.polygon.points.push_back(point3);
    plane_msg_xy.polygon.points.push_back(point4);
    //上一时刻的力和力矩
    KDL::Wrench last_wrench;
    last_wrench.force.x(0);
    last_wrench.force.y(0);
    last_wrench.force.z(0);
    KDL::Wrench wrench_d;
    // !后期删除
    last_poses[0] = poses[0];
    last_poses[1] = poses[1];
    last_poses[2] = poses[2];
>>>>>>> Stashed changes

    point1.x = p1.p.x();
    point1.y = p1.p.y();
    point1.z = p1.p.z();
    point2.x = p2.p.x();
    point2.y = p2.p.y();
    point2.z = p2.p.z();
    point3.x = p3.p.x();
    point3.y = p3.p.y();
    point3.z = p3.p.z();
    point4.x = p4.p.x();
    point4.y = p4.p.y();
    point4.z = p4.p.z();
    plane_msg_xy.header.frame_id = "base"; // 设置坐标系为 "map"

    // 添加多边形顶点
    // plane_msg_xy.polygon.points.resize(4);
    plane_msg_xy.polygon.points.push_back(point1);
    plane_msg_xy.polygon.points.push_back(point2);
    plane_msg_xy.polygon.points.push_back(point3);
    plane_msg_xy.polygon.points.push_back(point4);
    //上一时刻的力和力矩
    KDL::Wrench last_wrench;
    last_wrench.force.x(0);
    last_wrench.force.y(0);
    last_wrench.force.z(0);
    KDL::Wrench wrench_d;
    while (isRunning)
    {
        if (getRobotState(strIpAddress) == 6)
        {
            noError = false;
        }
        // ! ros2 publish
        auto joint_state_ = sensor_msgs::msg::JointState();

        // Set joint names
        joint_state_.name = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7"};

        // Set joint positions
        // joint_state_.position = {0.0, 0.0, 0.0,0,0,0,0};
        double joint_pos[7] = {0.0};
        getJointPos(joint_pos, strIpAddress);
        joint_state_.position = {joint_pos[0], joint_pos[1], joint_pos[2], joint_pos[3], joint_pos[4], joint_pos[5], joint_pos[6]};

        // Set header timestamp
        joint_state_.header.stamp = node->get_clock()->now();

        publisher->publish(joint_state_);
        plane_msg_xy.header.stamp = node->get_clock()->now();
        // plane_msg_xy.polygon.points.push_back(point1);
        // plane_msg_xy.polygon.points.push_back(point2);
        // plane_msg_xy.polygon.points.push_back(point3);
        // plane_msg_xy.polygon.points.push_back(point4);
        plane_xy->publish(plane_msg_xy);

        if (noError)
        {
            // wrench_compensation.force.y(1);
            // TODO 环境刚度判断碰撞
            wrench_compensation = gravityCompensation(poses, enable_wrench, Zero_offset, mass, center_of_mass_position);
            wrench_compensation.force.x(filter_array[0].filter(wrench_compensation.force.x()));
            wrench_compensation.force.y(filter_array[1].filter(wrench_compensation.force.y()));
            wrench_compensation.force.z(filter_array[2].filter(wrench_compensation.force.z()));
<<<<<<< Updated upstream
            wrench_d.force.x((wrench_compensation.force.x() - last_wrench.force.x())*200);
            wrench_d.force.y((wrench_compensation.force.y() - last_wrench.force.y())*200);
            wrench_d.force.z((wrench_compensation.force.z() - last_wrench.force.z())*200);
            last_wrench = wrench_compensation;
            if(abs(wrench_d.force.x()) > 100)
            {
                wrench_compensation.force.x(0*sign(wrench_compensation.force.x()));
                std::cerr<<"x方向发生碰撞: "<<1*sign(wrench_compensation.force.x())<<"last_wrench:"<<last_wrench.force.x()<<std::endl;
                getTcpPos(poses, strIpAddress);
            }
            if(abs(wrench_d.force.y()) > 100)
            {
                wrench_compensation.force.y(0*sign(wrench_compensation.force.y()));
                std::cerr<<"y方向发生碰撞"  << 0*sign(wrench_compensation.force.y()) << std::endl;
                getTcpPos(poses, strIpAddress);
            }
            if(abs(wrench_d.force.z() )> 300)
            {
                wrench_compensation.force.z(0*sign(wrench_compensation.force.z()));
                std::cerr<<"z方向发生碰撞"  << std::endl;
                // getTcpPos(poses, strIpAddress);
            }
=======

            // wrench_d.force.x((wrench_compensation.force.x() - last_wrench.force.x())*200);
            // wrench_d.force.y((wrench_compensation.force.y() - last_wrench.force.y())*200);
            // wrench_d.force.z((wrench_compensation.force.z() - last_wrench.force.z())*200);
            // last_wrench = wrench_compensation;
            // if(abs(wrench_d.force.x()) > 100)
            // {
            //     wrench_compensation.force.x(0*sign(wrench_compensation.force.x()));
            //     std::cerr<<"x方向发生碰撞: "<<1*sign(wrench_compensation.force.x())<<"last_wrench:"<<last_wrench.force.x()<<std::endl;
            //     getTcpPos(poses, strIpAddress);
            // }
            // if(abs(wrench_d.force.y()) > 100)
            // {
            //     wrench_compensation.force.y(0*sign(wrench_compensation.force.y()));
            //     std::cerr<<"y方向发生碰撞"  << 0*sign(wrench_compensation.force.y()) << std::endl;
            //    
            // }
            // if(abs(wrench_d.force.z() )> 300)
            // {
            //     wrench_compensation.force.z(0*sign(wrench_compensation.force.z()));
            //     std::cerr<<"z方向发生碰撞"  << std::endl;
            //     // getTcpPos(poses, strIpAddress);
            // }
>>>>>>> Stashed changes
            
            

            if (abs(wrench_compensation.force.x()) < 0.5)
            {
                wrench_compensation.force.x(0);
            }
            if (abs(wrench_compensation.force.y()) < 0.5)
            {
                wrench_compensation.force.y(0);
            }

            KDL::Wrench wrench_temp = KDL::Rotation::RPY(PI, 0, 0) * wrench_compensation;
            // wrench_compensation.force.x(0);
            // wrench_compensation.force.y(0);
            wrench_msg_z.header.stamp = node->get_clock()->now();

            wrench_msg_z.wrench.force.z = wrench_temp.force.z();
            wrench_msg_xy.header.stamp = node->get_clock()->now();
            wrench_msg_xy.wrench.force.x = wrench_temp.force.x();
            wrench_msg_xy.wrench.force.y = wrench_temp.force.y();

            wrench_z->publish(wrench_msg_z);
            wrench_xy->publish(wrench_msg_xy);

            wrench_compensation.force.z(0);
            wrench_compensation.torque.x(0);
            wrench_compensation.torque.y(0);
            wrench_compensation.torque.z(0);
            KDL::Wrench wrench_base = TCP_base.M * f_tcp * wrench_compensation;
            
            // !TEST 
            getTcpPos(real_poses, strIpAddress);
            // 环境刚度判断，力的前后之差除以实际位置的偏差
            // 当实际的位置和上次位置之差变换很小时，不进行碰撞检测判断。
            poses[0] += wrench_base.force.x() / B;
            poses[1] += wrench_base.force.y() / B;
            poses[2] += wrench_base.force.z() / B;
            double factor=1.0;
            if(abs(real_poses[0]-last_real_poses[0])>0.00001)
            {
                double K_exit=abs(wrench_base.force.x()-last_wrench.force.x())/abs(last_real_poses[0]-real_poses[0]);
                if(K_exit>40000)
                {
                    std::cout<<"collision is detection, K_exit is "<<K_exit<<std::endl;
                    int n=10;
                    double K_lim=5*1e10;
                    factor = std::pow(( K_lim-K_exit) / K_lim, n);
                    std::cout<<"factorx is "<<1-factor<<std::endl;
                     getTcpPos(poses, strIpAddress);
                    
                    // poses[0] += wrench_base.force.x() / B*(1-factor);
                    
                }
                
            }
            if(abs(real_poses[1]-last_real_poses[1])>0.00001)
            {
                double K_exit=abs(wrench_base.force.y()-last_wrench.force.y())/abs(last_real_poses[1]-real_poses[1]);
                if(K_exit>40000)
                {
                    std::cout<<"collision is detection, K_exit is "<<K_exit<<std::endl;
                    int n=5;
                    double K_lim=5*1e8;
                    factor = std::pow(( K_lim-K_exit) / K_lim, n);
                    std::cout<<"factory is "<<1-factor<<std::endl;
                    getTcpPos(poses, strIpAddress);
                    // poses[1] += wrench_base.force.y() / B*(1-factor);
                }
                
            }
            last_real_poses[0]=real_poses[0];
            last_wrench=wrench_base;

            
           

            last_poses[0] = poses[0];
            last_poses[1] = poses[1];
            last_poses[2] = poses[2];

            
           

            // getTcpPos(rot_poses, strIpAddress);
            axis = KDL::Vector(poses[3], poses[4], poses[5]);
            auto n = normal;
            n.Normalize();
            double norm = axis.Normalize();
            double angle = -wrench_compensation.torque.z() / 1000.0;
            auto delta_R = KDL::Rotation::Rot(n, angle);
            auto m = delta_R * KDL::Rotation::Rot(axis, norm);
            TCP_base = KDL::Frame(m, KDL::Vector(poses[0], poses[1], poses[2]));
            KDL::Vector r = TCP_base.M.GetRot();
            poses[3] = r.x();
            poses[4] = r.y();
            poses[5] = r.z();
            // std::cout << "poses: " << poses[0] << "," << poses[1] << "," << poses[2] << std::endl;
            // std::cout<<"Wrench_base: "<<wrench_base.force.x()<<","<<wrench_base.force.y()<<","<<wrench_base.force.z()<<","<<wrench_base.torque.x()<<","<<wrench_base.torque.y()<<","<<wrench_base.torque.z()<<std::endl;

            servoL(poses, 0.01, 0.1, 300, 1.0, nullptr, strIpAddress);

            // std::cout<<"poses: "<<poses[0]<<","<<poses[1]<<","<<poses[2]<<","<<poses[3]<<","<<poses[4]<<","<<poses[5]<<std::endl;
            // std::cout << "wrench z: " << std::fixed << wrench_compensation.torque.z() << " ; Angle: " << angle << std::endl;
            usleep(1000);
        }
        else
        {
            // while(getRobotState(strIpAddress) == 6)
            //     cleanErrorInfo(strIpAddress);
            // std::cout << "ClearErrorInfo!!" << std::endl;
            noError = true;

            getTcpPos(poses, strIpAddress);

            // usleep(10000);

            cleanErrorInfo(strIpAddress);
            setLastError(0, strIpAddress);
            // std::cout << "Clear Error!!" << std::endl;

            usleep(1000);
        }
    }

    destroySrv(strIpAddress);
    return 0;
}
