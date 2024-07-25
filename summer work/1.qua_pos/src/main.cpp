#include <iostream>
#include <vector>
#include <string>
#include <boost/algorithm/string.hpp>
#include "../include/head.h"

#define Pi 3.141592

quaternion quaternion_new(0,0,0);
position position_new(0,0,0);

void test_01(const std::string &str)
{
    double yaw, pitch, roll, x, y, z;
    std::cout << "请输入测试数据:" << std::endl;
    std::cin >> yaw >> pitch >> roll >> x >> y >> z;

    quaternion quaternion_original(yaw, pitch, roll);
    position position_original(x, y, z);
    if (str ==  "Gimbal" )
    {
        
        transform_camera_to_Gimbal(quaternion_original, position_original);
    }
    else if (str=="Odom")
    {
        transform_camera_to_Gimbal(quaternion_original, position_original);
        quaternion quaternion_new_creat(quaternion_new.yaw,quaternion_new.pitch,quaternion_new.roll);
        transform_Gimbal_to_Odom(quaternion_new_creat, position_new);
    }
    else
    {
        transform_camera_to_Gimbal(quaternion_original, position_original);
        quaternion quaternion_new_creat(quaternion_new.yaw,quaternion_new.pitch,quaternion_new.roll);
        transform_Gimbal_to_Shooter(quaternion_new_creat, position_new);
    }
}

int main()
{
    std::string transform_way;
    std::getline(std::cin, transform_way);
    std::vector<std::string> tokens;
    boost::split(tokens, transform_way, boost::is_any_of(std::string(1, '/')));
    test_01(tokens[1]);
    return 0;
}