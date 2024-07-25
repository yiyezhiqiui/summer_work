#include<iostream>
#include<cmath>
#include "../../include/quater_pos.hpp"
position operator+(position &a, position &b)
{
    position temp(a.x,a.y,a.z);
    temp.x=a.x+b.x;
    temp.y=a.y+b.y;
    temp.z=a.z+b.z;
    
    return temp;
}
position operator-(position &a, position &b)
{
    position temp(a.x,a.y,a.z);
    temp.x=a.x-b.x;
    temp.y=a.y-b.y;
    temp.z=a.z-b.z;
    
    return temp;
}

std::istream &operator>>(std::istream &in, position &test)
{
    std::cin>>test.x;
    std::cin>>test.y;
    std::cin>>test.z;

    return in;
}
std::ostream &operator<<(std::ostream &os, const position &test)
{
    std::cout<<"坐标:("<<test.x<<","<<test.y<<","<<test.z<<")"<<std::endl;
    return os;
}