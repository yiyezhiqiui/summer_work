#include <iostream>
#include <cstring>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <vector>
#include <map>
#include <opencv2/opencv.hpp>
#include <fstream>

#include "../include/client.hpp"
#include "../include/picture_process.hpp"


void encode_and_send(int sock, MessageBuffer &message)
{
    unsigned char buffer[10240] = {0};
    // 使用 memcpy 将 message 数据拷贝到 buffer 中
    memcpy(buffer, &message, sizeof(MessageBuffer));

    ssize_t sendsat = send(sock, buffer, sizeof(MessageBuffer), 0);
    if (sendsat == -1)
    {
        std::cerr << "Failed to send message to server." << std::endl;
        close(sock);
    }
}


//1.用于接收相机内参的函数
void handle_camera_intrinsics(int sock)
{
    double cameraMatrix[9];
    double distCoeffs[5];
    std::vector<unsigned char> judgement;
    while (true)
    {
        char recbuffer[10240] = {0};
        ssize_t bytes_received = recv(sock, recbuffer, sizeof(recbuffer) - judgement.size(), 0);
        if (bytes_received <= 0)
        {
            std::cerr << "接收消息失败或完成" << std::endl;
            close(sock);
        }
        judgement.insert(judgement.end(), recbuffer, recbuffer + bytes_received);
        while (judgement.size() >= sizeof(MessageBuffer))
        {
            MessageBuffer buffer;
            memcpy(&buffer, judgement.data(), sizeof(MessageBuffer));
            judgement.clear();
            if (buffer.MessageType == CAMERA_INFO)
            {
                CameraInfoData camerainfo;
                cv::Mat cameramatrix = cv::Mat(3, 3, CV_64F, cameraMatrix);
                cv::Mat distcoeffs = cv::Mat(1, 5, CV_64F, distCoeffs);
                std::memcpy(&camerainfo, buffer.Data, buffer.Datalenth);
                std::memcpy(cameraMatrix, camerainfo.CameraMatrix, sizeof(camerainfo.CameraMatrix));
                std::memcpy(distCoeffs, camerainfo.DistortionCoefficients, sizeof(camerainfo.DistortionCoefficients));

                // 将相机内参写入文件，先清空原有数据
                std::ofstream cameraMatrixFile("../data/camera/matrix.txt", std::ios::out | std::ios::trunc);
                if (cameraMatrixFile.is_open())
                {
                    for (int i = 0; i < 9; ++i)
                    {
                        cameraMatrixFile << cameraMatrix[i] << (i % 3 == 2 ? "\n" : " ");
                    }
                    cameraMatrixFile.close();
                }
                else
                {
                    std::cerr << "无法打开 camera_matrix.txt 文件" << std::endl;
                }

                // 将畸变系数写入文件，先清空原有数据
                std::ofstream distCoeffsFile("../data/camera/distcoeffs.txt", std::ios::out | std::ios::trunc);
                if (distCoeffsFile.is_open())
                {
                    for (int i = 0; i < 5; ++i)
                    {
                        distCoeffsFile << distCoeffs[i] << (i < 4 ? " " : "\n");
                    }
                    distCoeffsFile.close();
                }
                else
                {
                    std::cerr << "无法打开 dist_coeffs.txt 文件" << std::endl;
                }
                return;
            }
        }
    }
}
// 2.用于交流
void transform(int sock_transform,double translation[3],double rotation[4],std::string from,std::string to)
{
    MessageBuffer sendbuffer;
    TransformRequestData data;
    sendbuffer.Data[10218] = {0};
    sendbuffer.Start = 0x0D00;
    sendbuffer.MessageType = TRANSFORM_REQUEST;
    memcpy(data.From, from.c_str(), from.size());
    memcpy(data.To, to.c_str(), to.size());
    sendbuffer.Datalenth = sizeof(data);
    memcpy(sendbuffer.Data, &data, sendbuffer.Datalenth);
    sendbuffer.Datatallength = sendbuffer.Datalenth;
    sendbuffer.Offset = 0;
    sendbuffer.END = 0x0721;
    encode_and_send(sock_transform, sendbuffer);
    std::vector<unsigned char> judgement;
    while (true)
    {
        char recbuffer[10240] = {0};
        ssize_t bytes_received = recv(sock_transform, recbuffer, sizeof(recbuffer) - judgement.size(), 0);
        if (bytes_received <= 0)
        {
            std::cerr << "交流消息失败或完成" << std::endl;
            close(sock_transform);
        }
        judgement.insert(judgement.end(), recbuffer, recbuffer + bytes_received);
        while (judgement.size() >= sizeof(MessageBuffer))
        {
            MessageBuffer buffer_rec;
            memcpy(&buffer_rec, judgement.data(), sizeof(MessageBuffer));  
            judgement.clear();
            if (buffer_rec.MessageType == TRANSFORM)
            {
                TransformData transform;
                memcpy(&transform,buffer_rec.Data,buffer_rec.Datalenth);
                memcpy(translation,transform.Translation,sizeof(transform.Translation));
                memcpy(rotation,transform.Rotation,sizeof(transform.Rotation));
                //std::cout<<"test2"<<"x"<<translation[0]<<"  y:"<<translation[1]<<"  z:"<<translation[2]<<std::endl;
            return ;
            }            
        }
    }
}
//3.处理图像和字符串
void handle_image_string(int sock,Process &process)
{
// 连接成功后可以进行通信
    std::map<unsigned int, unsigned char *> data_temp; // 存储接收到的图片分片数据
    std::vector<unsigned char> judgement;
    while (true)
    {
        char recbuffer[10240] = {0};
        ssize_t bytes_received = recv(sock, recbuffer, sizeof(recbuffer) - judgement.size(), 0);
        if (bytes_received <= 0)
        {
            std::cerr << "接收图像消息失败或完成" << std::endl;
            close(sock);
            return ;
        }
        judgement.insert(judgement.end(), recbuffer, recbuffer + bytes_received);
        while (judgement.size() >= sizeof(MessageBuffer))
        {
            MessageBuffer buffer;
            memcpy(&buffer, judgement.data(), sizeof(MessageBuffer));
            judgement.clear();
            if (buffer.MessageType == STRING_MSG)
            {
                std::cout << buffer.Data << std::endl;
            }
            else if (buffer.MessageType == IMAGE_MSG)
            {
                receive_img(data_temp, &buffer,sock,process);
                send(sock,process);
            }
        }
    }
}
void receive_img(std::map<unsigned int, unsigned char *> &data_temp, MessageBuffer *buffer, int sock,Process &process)
{
    unsigned int offset = buffer->Offset;
    unsigned int length = buffer->Datalenth;
    unsigned int total_length = buffer->Datatallength;
    unsigned int dataID = buffer->DataID;

    if (data_temp.find(dataID) == data_temp.end())
    {
        data_temp[dataID] = new unsigned char[total_length];
    }

    memcpy(data_temp[dataID] + offset, buffer->Data, length);

    if (offset + length == total_length)
    {
        unsigned char *data = data_temp[dataID];
        cv::Mat img = cv::imdecode(cv::Mat(1, total_length, CV_8UC1, data), cv::IMREAD_COLOR);
        if (!img.empty())
        {
            /*cv::namedWindow("Received Image", cv::WINDOW_NORMAL);
            cv::resizeWindow("Received Image", 800, 600);*/
            picture_process(img, sock,process);
            // cv::imshow("Received Image", img);
        }
        data_temp.erase(dataID);
        delete[] data;
    }
}

// 4.向服务器发消息
void send(int sock_send,Process &process)
{
    std::string result;
    result = result + "装甲板坐标为(x,y,z):(" + std::to_string(process.res_position.getX()) + "," + std::to_string(process.res_position.getY()) + "," + std::to_string(process.res_position.getZ())  + ")\0";

    // 消息装进结构体
    MessageBuffer sendbuffer;
    std::memset(sendbuffer.Data, 0, sizeof(sendbuffer.Data)); // 清空缓冲区
    sendbuffer.Data[10218] = {0};
    sendbuffer.Start = 0x0D00;
    sendbuffer.MessageType = MessageType::STRING_MSG;
    sendbuffer.DataID = 0;
    sendbuffer.Offset = 0;
    sendbuffer.Datalenth = result.size();
    sendbuffer.Datatallength = sendbuffer.Datalenth;
    memcpy(sendbuffer.Data, result.data(), sendbuffer.Datalenth);
    sendbuffer.END = 0x0721;
    encode_and_send(sock_send, sendbuffer);
}