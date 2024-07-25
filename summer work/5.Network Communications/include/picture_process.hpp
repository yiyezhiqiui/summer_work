#ifndef VIDEO_HPP
#define VIDEO_HPP

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


void picture_process(cv::Mat &frame,int sock,Process &process);
void kalman(position &original_position, double w, double r, double yaw);
cv::Mat invoke_armor_visual(cv::Mat frame,cv::Point2f (*points)[4],int& group_cnt,int &type);
#endif
