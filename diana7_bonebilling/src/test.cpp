//
// Created by think on 2021/4/7.
//

#include <sri/ftsensor.hpp>
#include <sri/commethernet.hpp>

#include <iostream>

using namespace SRI;

void rtDataHandler(std::vector<RTData<float>> &rtData)
{
    static int i = 0;
    std::cout << "[" << i << "] RT Data is ->  ";
    for (int i = 0; i < rtData.size(); i++)
    {
        for (int j = 0; j < 6; j++)
        {
            std::cout << "Ch " << j << ": " << rtData[i][j] << "\t";
        }
        std::cout << std::endl;
    }
    i++;
}

int main()
{

    SRI::CommEthernet *ce = new SRI::CommEthernet("192.168.2.109", 4008);

    SRI::FTSensor sensor(ce);
    //    sensor.generateCommandBuffer(SRI::EIP,"?");

    //    std::string s = ACK + EIP + "=" + "127.0.0.1\r\n";
    //    std::string s = ACK + EIP + "=" + "127.0.0.1$ERROR\r\n";
    //    std::vector<int8_t> buf(s.begin(), s.end());
    //    std::string res = sensor.extractResponseBuffer(buf, EIP, "127.0.0.1");
    //    std::cout << res << std::endl;

    std::cout << "IP Address: " << sensor.getIpAddress() << std::endl;

    //    std::cout << sensor.setIpAddress("192.168.3.3") << std::endl;

    Gains gains = sensor.getChannelGains();
    std::cout << "Channel Gains: ";
    for (auto &gain : gains)
    {
        std::cout << gain << " ; ";
    }
    std::cout << std::endl;

    //    Sensitivities sens = {123.321, 234.432, 345.543, 654.234, 234.53};
    //    std::cout << sensor.setSensorSensitivities(sens) << std::endl;

    auto rtMode = sensor.getRealTimeDataMode();
    for (auto &c : rtMode.channelOrder)
    {
        std::cout << c << " , ";
    }
    std::cout << std::endl;

    std::cout << "DataUnit: " << rtMode.DataUnit << std::endl;
    std::cout << "PNpCH: " << rtMode.PNpCH << std::endl;
    std::cout << "FM: " << rtMode.FM << std::endl;

    for (auto &w : rtMode.filterWeights)
    {
        std::cout << w << " , ";
    }
    std::cout << std::endl;

    //    RTDataMode rtDataMode;
    //    std::cout << sensor.setRealTimeDataMode(rtDataMode) << std::endl;

    auto rtDataValid = sensor.getRealTimeDataValid();
    std::cout << "Real-time data valid method is: " << rtDataValid << std::endl;
    double x, y, z,my;
    for (int i = 0; i < 1000; i++)
    {
        auto rtData = sensor.getRealTimeDataOnce<float>(rtMode, rtDataValid);

        // std::cout << "RT Data is: " << std::endl;
        // std::cout << rtData.size() << std::endl;
        for (int i = 0; i < rtData.size(); i++)
        {
            // for (int j = 0; j < rtMode.channelOrder.size(); j++)
            // {
            //     std::cout << "Ch " << j << ": " << rtData[i][j] << "\t";

            // }
            x+=rtData[i][0];
            y+=rtData[i][1];
            z+=rtData[i][2];
            my+=rtData[i][4];
            // std::cout << std::endl;
        }
    }
    std::cout<<"x: "<<x/1000<<" y: "<<y/1000<<" z: "<<z/1000<<"my: "<<my/1000<<std::endl;
    //竖直状态
    // x: -3.0987 y: -1.25601 z: 14.1741
    //水平状态
    // x: -4.92595 y: -1.15198 z: 11.7307
    double gravity= {9.81};
    double zero_offset[3] = {-3.0987, -1.25601, 11.7307};
    double force[3] = {-4.92595,-1.15198,14.1741};
    double m=(force[2]-zero_offset[2])/gravity;
    double mz=(-0.371323+0.281807)/m/gravity;
    std::cout<<"mz: "<<mz<<std::endl;
    std::cout<<"m: "<<m<<std::endl;
//  mz: -0.0366358
//  m: 0.249072
    





    //   sensor.startRealTimeDataRepeatedly<float>(&rtDataHandler, rtMode, rtDataValid);

    // //   std::this_thread::sleep_for(std::chrono::seconds(10));

    // // while(1);

    // sensor.stopRealTimeDataRepeatedly();

    //    std::string s = "Hello World!";
    //    std::vector<char> buf;
    //    buf.assign(s.begin(), s.end());
    //    ce.write(buf);
    //    while(1) {
    //        if(ce.available() > 0) {
    //            std::cout << "Avaliable data bits: " << ce.available() << std::endl;
    //            std::vector<char> buf(ce.available());
    //
    //            uint32_t i = ce.read(buf);
    //            std::string s(buf.begin(), buf.end());
    //            std::cout << "Received from: " << ce.getRemoteAddress() <<"  Length is: " << i << "   Data is: " << s << std::endl;
    //        }
    //    }
}