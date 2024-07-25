#include<iostream>
#include<cmath>
#include "../include/head.h"
void transform_xyz(quaternion &orginal_quaternion, position &orginal_position, double x, double y, double z)
{
    quaternion temp(0, orginal_position.x , orginal_position.y , orginal_position.z );
    quaternion result(0, 0, 0,0);
    quaternion orginal_quaternion_1 = quaternion_inverse(orginal_quaternion);
    result = orginal_quaternion * temp;
    result = result * orginal_quaternion_1;
    position result_position(result.x_quaternion+x, result.y_quaternion+y, result.z_quaternion+z);

    position_new.x=result_position.x;
    position_new.y=result_position.y;
    position_new.z=result_position.z;
    std::cout << result_position;
}
void transform_camera_to_Gimbal(quaternion &orginal_quaternion, position &orginal_position)
{
    quaternion quaternion_rotation(1.03, 1.07, -1.27);
    quaternion result(0, 0, 0);

    result = orginal_quaternion*quaternion_rotation ;
    quaternion quaternion_result(result.w, result.x_quaternion, result.y_quaternion, result.z_quaternion);

    quaternion_new.yaw=quaternion_result.yaw;
    quaternion_new.pitch=quaternion_result.pitch;
    quaternion_new.roll=quaternion_result.roll;

    std::cout<<"this camera to gimbal:"<<std::endl;
    transform_xyz(quaternion_rotation, orginal_position, 0.37, -0.04, 2.24);
    std::cout << "yaw:" << quaternion_result.yaw << " pitch:" << quaternion_result.pitch << "  roll:" << quaternion_result.roll << std::endl;
    std::cout << "w:" << result.w << "  x_quaternion:" << result.x_quaternion << "  y_quaternion" << result.y_quaternion << "  z_quaternion:" << result.z_quaternion << std::endl<<std::endl;
}
void transform_Gimbal_to_Odom(quaternion &orginal_quaternion, position &orginal_position)
{
    quaternion quaternion_rotation(0.7,1.3, 0);
    quaternion result(0, 0, 0);

    result = orginal_quaternion*quaternion_rotation ;

    quaternion quaternion_result(result.w, result.x_quaternion, result.y_quaternion, result.z_quaternion);

    std::cout<<"this gimbal to odom:"<<std::endl;
    transform_xyz(quaternion_rotation, orginal_position, 0,0,0);
    std::cout << "yaw:" << quaternion_result.yaw << " pitch:" << quaternion_result.pitch << "  roll:" << quaternion_result.roll << std::endl;
    std::cout << "w:" << result.w << "  x_quaternion:" << result.x_quaternion << "  y_quaternion" << result.y_quaternion << "  z_quaternion:" << result.z_quaternion << std::endl<<std::endl;
}

void transform_Gimbal_to_Shooter(quaternion &orginal_quaternion, position &orginal_position)
{
    quaternion quaternion_rotation(0, 0, 0);
    quaternion result(0, 0, 0);

    result = orginal_quaternion*quaternion_rotation ;

    quaternion quaternion_result(result.w, result.x_quaternion, result.y_quaternion, result.z_quaternion);

    std::cout<<"this camera to shooter:"<<std::endl;
    transform_xyz(quaternion_rotation, orginal_position, -0.18, 0, 0.05);
    std::cout << "yaw:" << quaternion_result.yaw << " pitch:" << quaternion_result.pitch << "  roll:" << quaternion_result.roll << std::endl;
    std::cout << "w:" << result.w << "  x_quaternion:" << result.x_quaternion << "  y_quaternion" << result.y_quaternion << "  z_quaternion:" << result.z_quaternion << std::endl<<std::endl;
}