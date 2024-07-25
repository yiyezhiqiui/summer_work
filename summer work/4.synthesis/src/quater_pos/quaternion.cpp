#include<iostream>
#include<cmath>
#include "../../include/quater_pos.hpp"

quaternion operator+(quaternion &a, quaternion &b)
{
    quaternion temp(a.yaw,a.pitch,a.roll);
    temp.w=a.w+b.w;
    temp.x_quaternion=a.x_quaternion+b.x_quaternion;
    temp.y_quaternion=a.y_quaternion+b.y_quaternion;
    temp.z_quaternion=a.z_quaternion+b.z_quaternion;
    return temp;
}
quaternion operator*(quaternion &a,quaternion &b)
{
    quaternion temp(a.yaw,a.pitch,a.roll);
    temp.w=a.w*b.w-a.x_quaternion*b.x_quaternion-a.y_quaternion*b.y_quaternion-a.z_quaternion*b.z_quaternion;
    temp.x_quaternion=a.w*b.x_quaternion+a.x_quaternion*b.w+a.y_quaternion*b.z_quaternion-a.z_quaternion*b.y_quaternion;
    temp.y_quaternion=a.w*b.y_quaternion-a.x_quaternion*b.z_quaternion+a.y_quaternion*b.w+a.z_quaternion*b.x_quaternion;
    temp.z_quaternion=a.w*b.z_quaternion+a.x_quaternion*b.y_quaternion-a.y_quaternion*b.x_quaternion+a.z_quaternion*b.w;
    return temp;
}
quaternion quaternion_conjugate(quaternion &a)//四元数共轭定义
{
    quaternion temp(a.yaw,a.pitch,a.roll);
    temp.w=a.w;
    temp.x_quaternion=a.x_quaternion*(-1);
    temp.y_quaternion=a.y_quaternion*(-1);
    temp.z_quaternion=a.z_quaternion*(-1);
    return temp;
}
double quaternion_long(quaternion &a)//四元数的模定义
{
    double qua_long;
    qua_long=sqrt(a.w*a.w+a.x_quaternion*a.x_quaternion+a.y_quaternion*a.y_quaternion+a.z_quaternion*a.z_quaternion);
    return qua_long;
}
quaternion quaternion_inverse(quaternion &a)//四元数的逆定义
{
    quaternion temp(a.yaw,a.pitch,a.roll);
    quaternion temp_conjugate(a.yaw,a.pitch,a.roll);
    double long_quaternion=quaternion_long(temp);
    long_quaternion=long_quaternion*long_quaternion;
    temp_conjugate=quaternion_conjugate(temp);
    temp.w=temp_conjugate.w/long_quaternion;
    temp.x_quaternion=temp_conjugate.x_quaternion/long_quaternion;
    temp.y_quaternion=temp_conjugate.y_quaternion/long_quaternion;
    temp.z_quaternion=temp_conjugate.z_quaternion/long_quaternion;
    return temp;
}
std::istream &operator>>(std::istream &in, quaternion &test)
{
    std::cin>>test.w;
    std::cin>>test.x_quaternion;
    std::cin>>test.y_quaternion;
    std::cin>>test.z_quaternion;
    return in;
}
std::ostream &operator<<(std::ostream &os, const quaternion &test)
{
    std::cout<<test.w
             <<" "<<test.x_quaternion
             <<"i "<<test.y_quaternion
             <<"j "<<test.z_quaternion
             <<"k";
    return os;
}
