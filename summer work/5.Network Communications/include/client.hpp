#ifndef CLIENT_HPP
#define CLIENT_HPP

#include <map>
#include <Eigen/Dense>
#include "./quater_pos.hpp"
class Process;

typedef struct
{
    unsigned short Start = 0x0D00;
    unsigned short MessageType;
    unsigned int DataID;
    unsigned int Datatallength;
    unsigned int Offset;
    unsigned int Datalenth;
    unsigned char Data[10218];
    unsigned short END = 0x0721;
} MessageBuffer;

enum MessageType
{
    STRING_MSG = 0x0000,
    IMAGE_MSG = 0x1145,
    CAMERA_INFO = 0x1419,
    TRANSFORM = 0x1981,
    TRANSFORM_REQUEST = 0x1982
};

struct CameraInfoData
{
    double CameraMatrix[9];
    double DistortionCoefficients[5];
};

struct TransformData
{
    double Translation[3];
    double Rotation[4];
};

struct TransformRequestData
{
    char From[10218 / 2];
    char To[10218 / 2];
};

class Process
{
public:
    // Odom到Gimabl
    position gim_translation;
    quaternion gim_rotation;
    // Gimbal到Camera
    position cam_translation;
    quaternion cam_rotation;
    // 结果
    position res_position;
    quaternion res_quaternion;

    Process(double gimtranslation[3], double gimrotation[4], double camtranslation[3], double camrotation[4])
        : gim_translation(gimtranslation[0], gimtranslation[1], gimtranslation[2]),
          gim_rotation(gimrotation[3], gimrotation[0], gimrotation[2], gimrotation[3]),

          cam_translation(camtranslation[0], camtranslation[1], camtranslation[2]),
          cam_rotation(camrotation[3], camrotation[0], camrotation[2], camrotation[3]),

          res_position(0, 0, 0),
          res_quaternion(0, 0, 0, 0)
    {
    }

    ~Process() {};
};

// 声明函数

void encode_and_send(int sock, MessageBuffer &message);
// 1.
void handle_camera_intrinsics(int sock_intrinsics);

// 2.
void transform(int sock_transform, double translation[3], double rotation[4], std::string from, std::string to);

// 3.
void handle_image_string(int sock, Process &process);
void receive_img(std::map<unsigned int, unsigned char *> &data_temp, MessageBuffer *buffer, int socket, Process &process);

// 4.
void send(int sock_send, Process &process);

#endif