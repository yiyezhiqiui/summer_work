#ifndef QUATER_POSI_HPP
#define QUATER_POSI_HPP

#include <cmath>
#include <iostream>
#include <Eigen/Dense>
class Process;
class quaternion;
class position;

void transform_xyz(quaternion &orginal_quaternion, position &orginal_position, double x, double y, double z);
class quaternion
{
public:
    quaternion(double a, double b, double c)
    {
        yaw = a;
        pitch = b;
        roll = c;
        w = cos(roll / 2) * cos(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * sin(pitch / 2) * sin(yaw / 2);
        x_quaternion = sin(roll / 2) * cos(pitch / 2) * cos(yaw / 2) - cos(roll / 2) * sin(pitch / 2) * sin(yaw / 2);
        y_quaternion = cos(roll / 2) * sin(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * cos(pitch / 2) * sin(yaw / 2);
        z_quaternion = cos(roll / 2) * cos(pitch / 2) * sin(yaw / 2) - sin(roll / 2) * sin(pitch / 2) * cos(yaw / 2);
    }
    quaternion(double a, double b, double c, double d)
    {
        w = a;
        x_quaternion = b;
        y_quaternion = c;
        z_quaternion = d;
        roll = atan2(2 * (w * x_quaternion + y_quaternion * z_quaternion), 1 - 2 * (x_quaternion * x_quaternion + y_quaternion * y_quaternion));
        pitch = asin(2 * (w * y_quaternion - x_quaternion * z_quaternion));
        yaw = atan2(2 * (w * z_quaternion + x_quaternion * y_quaternion), 1 - 2 * (y_quaternion * y_quaternion + z_quaternion * z_quaternion));
    }

    ~quaternion() {}
    friend quaternion operator+(quaternion &a, quaternion &b);
    friend quaternion operator*(quaternion &a, quaternion &b);
    friend quaternion quaternion_conjugate(quaternion &a); //计算四元数的共轭形式
    friend quaternion quaternion_inverse(quaternion &a);    //四元数的逆定义
    friend double quaternion_long(quaternion &a);           //四元数的模长

    friend void transform_xyz(quaternion &revol_quaternion, position &orginal_position, Process &process);
    friend void transform_camera_to_Gimbal(quaternion &orginal_quaternion, position &orginal_position,Process &process);
    friend void transform_Gimbal_to_Odom(quaternion &orginal_quaternion, position &orginal_position,Process &process);
    friend void transform_camera_to_Odom(quaternion &orginal_quaternion, position &orginal_position,Process &process);
    friend void invoke_quater_pos();

    friend std::istream &operator>>(std::istream &in, quaternion &test);
    friend std::ostream &operator<<(std::ostream &os, const quaternion &test);

    double get_roll() const { return roll; }
    double get_pitch() const { return pitch; }
    double get_yaw() const { return yaw; }

    friend class Process;

private:
    double w, x_quaternion, y_quaternion, z_quaternion;
    double yaw, pitch, roll;
};

class position
{
public:
    position(double a, double b, double c)
    {
        x = a;
        y = b;
        z = c;
    }
    ~position() {}
    friend position operator+(position &a, position &b);
    friend position operator-(position &a, position &b);

    friend void transform_xyz(quaternion &revol_quaternion, position &orginal_position, Process &process);
    friend void transform_camera_to_Gimbal(quaternion &orginal_quaternion, position &orginal_position,Process &process);
    friend void transform_Gimbal_to_Odom(quaternion &orginal_quaternion, position &orginal_position,Process &process);
    friend void transform_camera_to_Odom(quaternion &orginal_quaternion, position &orginal_position,Process &process);
    friend void invoke_quater_pos();
    friend void kalman(position &orginal_position,double w,double r,double yaw);

    friend std::istream &operator>>(std::istream &in, position &test);
    friend std::ostream &operator<<(std::ostream &os, const position &test);

    double getX() const { return x; }
    double getY() const { return y; }
    double getZ() const { return z; }

    friend class Process;

private:
    double x, y, z;
};

extern quaternion quaternion_new;
extern position position_new;
#endif
