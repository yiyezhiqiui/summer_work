#include <opencv2/opencv.hpp>
#include <cmath>
#include "../include/armor.hpp"  // 包含 armor.hpp 文件

using namespace std;
using namespace cv;

double crossProduct(const Point2d &v1, const Point2d &v2)
{
    return v1.x * v2.y - v1.y * v2.x;
}

double distance(const cv::Point2f &p1, const cv::Point2f &p2)
{
    return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2));
}

cv::Point2f midpoint(const cv::Point2f &p1, const cv::Point2f &p2)
{
    return cv::Point2f((p1.x + p2.x) / 2, (p1.y + p2.y) / 2);
}

// 比较函数，用于按顺时针方向排序点
bool comparePoints(const cv::Point2f &p1, const cv::Point2f &p2, const cv::Point2f &center)
{
    if (p1.x - center.x >= 0 && p2.x - center.x < 0)
        return true;
    if (p1.x - center.x < 0 && p2.x - center.x >= 0)
        return false;
    if (p1.x - center.x == 0 && p2.x - center.x == 0)
    {
        if (p1.y - center.y >= 0 || p2.y - center.y >= 0)
            return p1.y > p2.y;
        return p2.y > p1.y;
    }

    // 计算叉积
    double det = (p1.x - center.x) * (p2.y - center.y) - (p2.x - center.x) * (p1.y - center.y);
    if (det < 0)
        return true;
    if (det > 0)
        return false;

    // 按距离排序
    double d1 = (p1.x - center.x) * (p1.x - center.x) + (p1.y - center.y) * (p1.y - center.y);
    double d2 = (p2.x - center.x) * (p2.x - center.x) + (p2.y - center.y) * (p2.y - center.y);
    return d1 > d2;
}

void pnp()
{
    cv::Mat dst, gray;
    // 转换为灰度图像
    cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
    // 开运算，边缘检测
    cv::threshold(gray, dst, 210, 255, cv::THRESH_BINARY);

    // 轮廓提取
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(dst, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point());

    std::vector<cv::RotatedRect> minRect(contours.size());
    cv::Point2f rect_points[contours.size() * 2][4], points_tmp[contours.size() * 2][2], point[contours.size() * 2][2];

    int contours_cnt = 0;
    std::cout << contours.size() << std::endl;
    for (size_t i = 0; i < contours.size(); i++)
    {
        double area = cv::contourArea(contours[i]);
        if (area > 20)
        {

            // 拟合最小外接矩形
            minRect[i] = cv::minAreaRect(contours[i]);
            minRect[i].points(rect_points[i]);

            // 计算每条边的长度
            double edgeLengths[4];
            for (int j = 0; j < 4; j++)
            {
                edgeLengths[j] = distance(rect_points[i][j], rect_points[i][(j + 1) % 4]);
            }

            // 找出最短的两条边的索引
            int minIndex1 = 0, minIndex2 = 1;
            for (int j = 1; j < 4; j++)
            {
                if (edgeLengths[j] < edgeLengths[minIndex1])
                {
                    minIndex2 = minIndex1;
                    minIndex1 = j;
                }
                else if (edgeLengths[j] < edgeLengths[minIndex2])
                {
                    minIndex2 = j;
                }
            }

            // 计算较短两条边的中点
            cv::Point2f midpoint1 = midpoint(rect_points[i][minIndex1], rect_points[i][(minIndex1 + 1) % 4]);
            cv::Point2f midpoint2 = midpoint(rect_points[i][minIndex2], rect_points[i][(minIndex2 + 1) % 4]);

            // 存储中点
            points_tmp[contours_cnt][0] = midpoint1;
            points_tmp[contours_cnt][1] = midpoint2;
            cv::line(src, points_tmp[i][0], points_tmp[i][1], cv::Scalar(0, 0, 255), 2, cv::LINE_AA);

            contours_cnt += 1;
        }
    }

    group_cnt = 0;            // 配对成功的组数
    int flag[contours_cnt] = {0}; // 标记每个轮廓是否已经配对
    for (int i = 0; i < contours_cnt; i++)
    {
        std::cout << points_tmp[i][0] << points_tmp[i][1] << std::endl;
    }

    cv::Point2f result_point[5][4];
    for (int i = 0; i < contours_cnt; i++)
    {
        int flag_group = 0;
        if (flag[i] == 0)
        {
            flag_group = 0;                            // 重置标志位
            for (int j = i + 1; j < contours_cnt; j++) // j 从 i+1 开始避免重复检查和自身匹配
            {
                cv::Point2f vec1 = points_tmp[i][1] - points_tmp[i][0];
                cv::Point2f vec2 = points_tmp[j][1] - points_tmp[j][0];

                double cross = crossProduct(vec1, vec2);

                if (abs(cross) < 400)
                {
                    flag_group = 1; // 找到平行线段，标记为已配对

                    std::vector<cv::Point2f> allPoints = {points_tmp[i][0], points_tmp[i][1], points_tmp[j][0], points_tmp[j][1]};
                    cv::Point2f center(0, 0);
                    for (const auto &pt : allPoints)
                    {
                        center += pt;
                    }
                    center *= (1.0 / allPoints.size());

                    // 按照顺时针方向排序
                    std::sort(allPoints.begin(), allPoints.end(), [center](const cv::Point2f &p1, const cv::Point2f &p2)
                              { return comparePoints(p1, p2, center); });

                    for (int k = 0; k < 4; k++)
                    {
                        result_point[group_cnt][k] = allPoints[k];
                    }

                    flag[i] = 1; // 标记这两个轮廓已经配对
                    flag[j] = 1;

                    // 绘制对角线
                    cv::line(src, result_point[group_cnt][0], result_point[group_cnt][2], cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
                    cv::line(src, result_point[group_cnt][1], result_point[group_cnt][3], cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
                    group_cnt += 1;
                    break;
                }
            }
        }
    }
    cv::imshow("src", src);
    cv::waitKey(0);
    for (int i = 0; i < group_cnt; i++)
    {
        point_extern[i][0]=result_point[i][0];
        point_extern[i][1]=result_point[i][1];
        point_extern[i][2]=result_point[i][2];
        point_extern[i][3]=result_point[i][3];
        const float SMALL_ARMOR_WIDTH = 0.135;
        const float SMALL_ARMOR_HEIGHT = 0.055;
        const float LARGE_ARMOR_WIDTH = 0.225;
        const float LARGE_ARMOR_HEIGHT = 0.055;
        // x 垂直装甲板向内，从左下角开始顺时针
        const std::vector<cv::Point3f> SMALL_ARMOR_POINTS = {
            {0, +SMALL_ARMOR_WIDTH / 2, -SMALL_ARMOR_HEIGHT / 2},
            {0, +SMALL_ARMOR_WIDTH / 2, +SMALL_ARMOR_HEIGHT / 2},
            {0, -SMALL_ARMOR_WIDTH / 2, +SMALL_ARMOR_HEIGHT / 2},
            {0, -SMALL_ARMOR_WIDTH / 2, -SMALL_ARMOR_HEIGHT / 2}};
        // x 垂直装甲板向内，从左下角开始顺时针
        const std::vector<cv::Point3f> LARGE_ARMOR_POINTS = {
            {0, +LARGE_ARMOR_WIDTH / 2, -LARGE_ARMOR_HEIGHT / 2},
            {0, +LARGE_ARMOR_WIDTH / 2, +LARGE_ARMOR_HEIGHT / 2},
            {0, -LARGE_ARMOR_WIDTH / 2, +LARGE_ARMOR_HEIGHT / 2},
            {0, -LARGE_ARMOR_WIDTH / 2, -LARGE_ARMOR_HEIGHT / 2}};

        std::vector<cv::Point3f> objectPoints; // 3D空间中的点
        std::vector<cv::Point2f> imagePoints;  // 对应的图像中的点

        // 添加3D空间中的点
        objectPoints = SMALL_ARMOR_POINTS;

        // 添加Pnp
        imagePoints = {result_point[i][0], result_point[i][1], result_point[i][2], result_point[i][3]};

        // 相机内参矩阵
        cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 2102.080562187802, 0, 689.2057889332623, 0, 2094.0179120166754, 496.6622802275393, 0, 0, 1);

        // 畸变系数
        cv::Mat distCoeffs = (cv::Mat_<double>(5, 1) << -0.06478109387525666, 0.39036067923005396, -0.0042514793151166306, 0.008306749648029776, -1.6613800909405605);

        // 输出的旋转向量和平移向量
        cv::Mat rvec, tvec;

        cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);

        std::cout << "Rotation Vector (rvec):" << std::endl
                  << rvec << std::endl;
        std::cout << "Translation Vector (tvec):" << std::endl
                  << tvec << std::endl;
    }
}