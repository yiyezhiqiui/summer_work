#ifndef ARMOR_HPP
#define ARMOR_HPP

#include <opencv2/opencv.hpp>

void pnp(cv::Mat &src, cv::Point2f (*points)[4], int& group_cnt,std::vector<cv::Vec3d> &output_position,std::vector<cv::Vec3d> &output_quater,int &type);

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
