#include <iostream>
#include <cmath>
#include "../../include/quater_pos.hpp"
#include "../../include/client.hpp"

void transform_xyz(quaternion &revol_quaternion, position &orginal_position, Process &process)
{
    quaternion temp(0, orginal_position.x, orginal_position.y, orginal_position.z);
    quaternion result(0, 0, 0, 0);
    quaternion revol_quaternion_1 = quaternion_inverse(revol_quaternion);
    result = revol_quaternion * temp;
    result = result * revol_quaternion_1;
    process.res_position = position(result.x_quaternion + process.cam_translation.getX(), result.y_quaternion + process.cam_translation.getY(), result.z_quaternion + process.cam_translation.getZ());
}

void transform_camera_to_Gimbal(quaternion &orginal_quaternion, position &orginal_position, Process &process)
{
    quaternion result(0, 0, 0);
    result = orginal_quaternion * process.cam_rotation;
    process.res_quaternion = quaternion(result.w, result.x_quaternion, result.y_quaternion, result.z_quaternion);

    transform_xyz(process.cam_rotation, orginal_position, process);
}

void transform_Gimbal_to_Odom(quaternion &orginal_quaternion, position &orginal_position, Process &process)
{
    quaternion result(0, 0, 0);
    result = orginal_quaternion * process.gim_rotation;
    process.res_quaternion = quaternion(result.w, result.x_quaternion, result.y_quaternion, result.z_quaternion);

    transform_xyz(process.gim_rotation, orginal_position, process);
}

void transform_camera_to_Odom(quaternion &orginal_quaternion, position &orginal_position, Process &process)
{
    quaternion result(0, 0, 0);
    result = orginal_quaternion * process.gim_rotation;
    process.res_quaternion = quaternion(result.w, result.x_quaternion, result.y_quaternion, result.z_quaternion);

    transform_xyz(process.gim_rotation, orginal_position, process);
}
