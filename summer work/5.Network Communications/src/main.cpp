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

#include "../include/client.hpp"

int main()
{
    struct sockaddr_in serv_addr_image, serv_addr_intrinsics,serv_addr_transform,serv_addr_send;
    int sock_image = 0, sock_intrinsics = 0,sock_transform= 0,sock_send=0;
    
    // 创建用于接收图像的socket
    if ((sock_image = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        std::cerr << "Socket for image connection creation failed" << std::endl;
        return 1;
    }

    // 创建用于接收相机内参的socket
    if ((sock_intrinsics = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        std::cerr << "Socket for intrinsics connection creation failed" << std::endl;
        return 1;
    }

    // 创建用于交流的socket
    if ((sock_transform = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        std::cerr << "Socket for transform connection creation failed" << std::endl;
        return 1;
    }

    /*// 创建用于发送结果的socket
    if ((sock_send = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        std::cerr << "Socket for send connection creation failed" << std::endl;
        return 1;
    }*/

    // 设置图像服务器地址
    serv_addr_image.sin_family = AF_INET;
    serv_addr_image.sin_port = htons(8000);
    const char *server_ip = "10.2.20.66";
    if (inet_pton(AF_INET, server_ip, &serv_addr_image.sin_addr) <= 0) {
        std::cerr << "Invalid address/ Address not supported: " << server_ip << std::endl;
        close(sock_image);
        return 1;
    }

    // 设置相机内参服务器地址
    serv_addr_intrinsics.sin_family = AF_INET;
    serv_addr_intrinsics.sin_port = htons(5140); 
    if (inet_pton(AF_INET, server_ip, &serv_addr_intrinsics.sin_addr) <= 0) {
        std::cerr << "Invalid address/ Address not supported: " << server_ip << std::endl;
        close(sock_intrinsics);
        return 1;
    }

    // 设置交流服务器地址
    serv_addr_transform.sin_family = AF_INET;
    serv_addr_transform.sin_port = htons(4399); 
    if (inet_pton(AF_INET, server_ip, &serv_addr_transform.sin_addr) <= 0) {
        std::cerr << "Invalid address/ Address not supported: " << server_ip << std::endl;
        close(sock_transform);
        return 1;
    }

    /*// 设置发送消息服务器地址
    serv_addr_send.sin_family = AF_INET;
    serv_addr_send.sin_port = htons(8000); 
    if (inet_pton(AF_INET, server_ip, &serv_addr_send.sin_addr) <= 0) {
        std::cerr << "Invalid address/ Address not supported: " << server_ip << std::endl;
        close(sock_send);
        return 1;
    }*/

    // 连接图像服务器
    if (connect(sock_image, (struct sockaddr *)&serv_addr_image, sizeof(serv_addr_image)) < 0) {
        std::cerr << "Connection to image server failed" << std::endl;
        close(sock_image);
        return 1;
    }

    // 连接相机内参服务器
    if (connect(sock_intrinsics, (struct sockaddr *)&serv_addr_intrinsics, sizeof(serv_addr_intrinsics)) < 0) {
        std::cerr << "Connection to camera intrinsics server failed" << std::endl;
        close(sock_intrinsics);
        return 1;
    }

    // 连接交流服务器
    if (connect(sock_transform, (struct sockaddr *)&serv_addr_transform, sizeof(serv_addr_transform)) < 0) {
        std::cerr << "Connection to transform server failed" << std::endl;
        close(sock_transform);
        return 1;
    }

    /*// 连接发送消息服务器
    if (connect(sock_send, (struct sockaddr *)&serv_addr_send, sizeof(serv_addr_send)) < 0) {
        std::cerr << "Connection to send server failed" << std::endl;
        close(sock_send);
        return 1;
    }*/

    //1.先接收相机内参
    handle_camera_intrinsics(sock_intrinsics);

    //2.再交流
    double gimtranslation[3];
    double gimrotation[4];
    double camtranslation[3];
    double camrotation[4];
    transform(sock_transform,gimtranslation,gimrotation,"Odom","Gimbal");
    transform(sock_transform,camtranslation,camrotation,"Gimbal","Camera");

    /*std::cout<<"x"<<gimtranslation[0]<<"  y:"<<gimtranslation[1]<<"  z:"<<gimtranslation[2]<<std::endl;
    std::cout<<"x:"<<gimrotation[0]<<" y:"<<gimrotation[1]<<"  z:"<<gimrotation[2]<<"  w:"<<gimrotation[3]<<std::endl;
    std::cout<<"x"<<camtranslation[0]<<"  y:"<<camtranslation[1]<<"  z:"<<camtranslation [2]<<std::endl;
    std::cout<<"x:"<<camrotation[0]<<" y:"<<camrotation[1]<<"  z:"<<camrotation[2]<<"  w:"<<camrotation[3]<<std::endl;*/
    
    //3.处理视频或者字符串
    Process process(gimtranslation,gimrotation,camtranslation,camrotation);
    handle_image_string(sock_image,process);

    //4.发送消息
    //send(sock_send,process);
    return 0;
}