#ifndef INVOKE_HPP
#define INVOKE_HPP
#include<opencv2/opencv.hpp>
void invoke_quater_pos();
cv::Mat invoke_armor_visual(cv::Mat frame,cv::Point2f (*points)[4],int& group_cnt,int &type);
#endif
