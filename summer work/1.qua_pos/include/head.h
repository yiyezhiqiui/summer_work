#ifndef _HEAD_H_
#define _HEAD_H_

#include<cmath>
class position;
class quaternion;

class quaternion
{
    public:
    quaternion(double a,double b,double c)
    {
        yaw=a;
        pitch=b;
        roll=c;
        w=cos(roll/2)*cos(pitch/2)*cos(yaw/2)+sin(roll/2)*sin(pitch/2)*sin(yaw/2);
        x_quaternion=sin(roll/2)*cos(pitch/2)*cos(yaw/2)-cos(roll/2)*sin(pitch/2)*sin(yaw/2);
        y_quaternion=cos(roll/2)*sin(pitch/2)*cos(yaw/2)+sin(roll/2)*cos(pitch/2)*sin(yaw/2);
        z_quaternion=cos(roll/2)*cos(pitch/2)*sin(yaw/2)-sin(roll/2)*sin(pitch/2)*cos(yaw/2);
        /*std::cout<<"这是构造函数的输出："<<"yaw:"<<yaw<<"pitch"<<pitch<<"roll"<<roll<<"w:"<<w<<"x_quaternion:"<<x_quaternion<<"y_quaternion:"<<y_quaternion<<"z_quaternion:"<<z_quaternion<<std::endl;*/
    }
    quaternion(double a,double b,double c,double d)
    {
        w=a;
        x_quaternion=b;
        y_quaternion=c;
        z_quaternion=d;
        roll=atan2(2*(w*x_quaternion+y_quaternion*z_quaternion),1-2*(x_quaternion*x_quaternion+y_quaternion*y_quaternion));
        pitch=asin(2*(w*y_quaternion-x_quaternion*z_quaternion));
        yaw=atan2(2*(w*z_quaternion+x_quaternion*y_quaternion),1-2*(y_quaternion*y_quaternion+z_quaternion*z_quaternion));
    }

    ~quaternion(){};
    friend quaternion operator+(quaternion &a, quaternion &b);
    friend quaternion operator*(quaternion &a, quaternion &b);
    friend quaternion quaternion_conjugate(quaternion &a);//计算四元数的共轭形式
    friend quaternion quaternion_inverse(quaternion &a);//四元数的逆定义
    friend double quaternion_long(quaternion &a);//四元数的模长
    
    friend void transform_xyz(quaternion &orginal_quaternion,position &orginal_position,double x,double y,double z);
    friend void transform_camera_to_Gimbal(quaternion &orginal_quaternion,position &orginal_position);
    friend void transform_Gimbal_to_Odom(quaternion &orginal_quaternion,position &orginal_position);
    friend void transform_Gimbal_to_Shooter(quaternion &orginal_quaternion,position &orginal_position);
    friend void test_01(const std::string &str);

    friend std::istream &operator>>(std::istream &in, quaternion &test);
    friend std::ostream &operator<<(std::ostream &os, const quaternion &test);

    private:
    double w,x_quaternion,y_quaternion,z_quaternion;
    double yaw,pitch,roll;
};
class position
{
    public:
    position(double a,double b,double c)
    {
        x=a;
        y=b;
        z=c;
    }
    ~position(){};
    friend position operator+(position &a, position &b);
    friend position operator-(position &a, position &b);
   
   
    friend void transform_xyz(quaternion &orginal_quaternion,position &orginal_position,double x,double y,double z);
    static void transform_camera_to_Gimbal(quaternion &orginal_quaternion,position &orginal_position);
    static void transform_Gimbal_to_Odom(quaternion &orginal_quaternion,position &orginal_position);
    static void transform_Gimbal_to_Shooter(quaternion &orginal_quaternion,position &orginal_position);
    static void test_01(char str[]);

    friend std::istream &operator>>(std::istream &in, position &test);
    friend std::ostream &operator<<(std::ostream &os, const position &test);
    
    private:
    double x,y,z;
};

extern quaternion quaternion_new;
extern position position_new;


#endif