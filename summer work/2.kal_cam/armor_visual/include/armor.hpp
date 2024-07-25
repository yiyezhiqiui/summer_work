#ifndef ARMOR_HPP
#define ARMOR_HPP

#include <opencv2/opencv.hpp>

extern cv::Mat src;          // 在pnp函数中的原图像
extern cv::Point2f point_extern[5][4]; // pnp函数中的4个特征点
extern int group_cnt;

void pnp();

class light
{
public:
    cv::Point2f bottom;
    cv::Point2f top;
};

enum class ArmorType
{
    SMALL,
    LARGE
};

class Armor
{
public:
    light left_light;
    light right_light;
    ArmorType type;
    std::string number;
    cv::Mat number_image;
    double classification_confidence;
    std::string classification_result;
};
#endif