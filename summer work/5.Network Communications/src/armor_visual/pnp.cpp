#include <opencv2/opencv.hpp>
#include <cmath>
#include <fstream>

#include "../../include/armor.hpp" // 包含 armor.hpp 文件

double crossProduct(const cv::Point2f &v1, const cv::Point2f &v2)
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

void pnp(cv::Mat &src, cv::Point2f (*points)[4], int &group_cnt, std::vector<cv::Vec3d> &output_position, std::vector<cv::Vec3d> &output_quater, int &type)
{
    src.convertTo(src, -1, 1.5, 30);
    cv::Mat dst, gray;
    // 转换为灰度图像
    cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);

    // 二值化
    cv::threshold(gray, dst, 210, 255, cv::THRESH_BINARY);

    // 膨胀
    cv::Mat struct1, struct2;
    struct1 = cv::getStructuringElement(0, cv::Size(3, 3)); // 矩形结构元素
    cv::dilate(dst, dst, struct1);

    // 调试
    /*cv::namedWindow("dst", cv::WINDOW_NORMAL);
    cv::resizeWindow("dst", 400, 300);
    cv::imshow("dst", dst);*/

    // 轮廓提取
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(dst, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point());

    std::vector<cv::RotatedRect> minRect(contours.size());
    cv::Point2f rect_points[contours.size() * 2][4], points_tmp[contours.size() * 2][2], point[contours.size() * 2][2];

    int contours_cnt = 0;
    // std::cout << contours.size() << std::endl;
    for (ssize_t i = 0; i < contours.size(); i++)
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

            contours_cnt += 1;
        }
    }

    int flag[contours_cnt] = {0}; // 标记每个轮廓是否已经配对

    for (int i = 0; i < contours_cnt; i++)
    {
        if (points_tmp[i][0].y > points_tmp[i][1].y)
        {
            cv::Point2f point_tmp = points_tmp[i][0];
            points_tmp[i][0] = points_tmp[i][1];
            points_tmp[i][1] = point_tmp;
        }
    }
    cv::Point2f result_point[6][4];
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
                double x_y = (points_tmp[i][0].x - points_tmp[j][0].x) / (points_tmp[i][0].y - points_tmp[i][1].y);
                double y_y = (points_tmp[i][1].y - points_tmp[i][0].y) / (points_tmp[j][1].y - points_tmp[j][0].y);
                double x_x = (points_tmp[i][0].x - points_tmp[j][0].x) / (points_tmp[i][1].x - points_tmp[j][1].x);
                double high_y = (points_tmp[i][0].y + points_tmp[i][1].y) / 2 - (points_tmp[j][0].y + points_tmp[j][1].y) / 2;

                bool fy_y = abs(y_y) > 0.75 && abs(y_y) < 1.3;
                bool fx_x = abs(x_x) > 0.75 && abs(x_x) < 1.3;
                bool fx_y = (abs(x_y) > 1.35 && abs(x_y) < 2.7) || (abs(x_y) > 2.9 && abs(x_y) < 3.5);

                // 检查两个灯条间的轮廓数
                cv::Rect roi_rect = cv::boundingRect(std::vector<cv::Point>{points_tmp[i][0], points_tmp[i][1], points_tmp[j][1], points_tmp[j][0]});
                roi_rect &= cv::Rect(0, 0, dst.cols, dst.rows);
                cv::Mat roi = dst(roi_rect);
                std::vector<std::vector<cv::Point>> contours_judgment;
                std::vector<cv::Vec4i> hierarchy_judgment;
                cv::findContours(roi, contours_judgment, hierarchy_judgment, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point());

                /*namedWindow("roi",cv::WINDOW_NORMAL);
                cv::resizeWindow("roi", 400, 300);
                cv::imshow("roi",roi);
                std::cout<<"test:  "<<contours_judgment.size()<<std::endl;*/

                if (abs(cross) < 400 && fx_y && fy_y && fx_x && contours_judgment.size() <= 2 && abs(high_y) < 17)
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

                    // cv::circle(src,result_point[group_cnt][0],5,cv::Scalar(0,0,255),-1);
                    // cv::circle(src,result_point[group_cnt][1],5,cv::Scalar(0,0,255),-1);
                    // cv::circle(src,result_point[group_cnt][2],5,cv::Scalar(0,0,255),-1);
                    // cv::circle(src,result_point[group_cnt][3],5,cv::Scalar(0,0,255),-1);

                    group_cnt += 1;

                    // 判断装甲板类型
                    if (abs(x_y) > 1.35 && abs(x_y) < 2.7)
                    {
                        type = 0;
                    }
                    else
                    {
                        type = 1;
                    }
                    break;
                }
                else
                {
                    /*std::cout << std::endl
                              << "未满足条件" <<i<<"  "<<j<< std::endl;
                    std::cout << "长宽比:" << abs(x_y) << "  " << fx_y << std::endl;
                    std::cout << "宽之比" << abs(y_y) << "  " << fy_y << std::endl;
                    std::cout << "长之比" << abs(x_x) << "  " << fx_x << std::endl;
                    std::cout << "中心点高度差:" << abs(high_y) << "  " << (abs(high_y) < 12) << std::endl;
                    std::cout << "向量相乘" << abs(cross) << std::endl;

                    std::string texti = std::to_string(i);
                    std::string textj = std::to_string(j);
                    cv::putText(src, texti, points_tmp[i][0], cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 165, 255), 2);
                    cv::putText(src, texti, points_tmp[i][1], cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 165, 255), 2);
                    cv::putText(src, textj, points_tmp[j][0], cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 165, 255), 2);
                    cv::putText(src, textj, points_tmp[j][1], cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 165, 255), 2);

                    cv::circle(src, points_tmp[i][0], 5, cv::Scalar(0, 165, 255), -1);
                    cv::circle(src, points_tmp[i][1], 5, cv::Scalar(0, 165, 255), -1);
                    cv::circle(src, points_tmp[j][0], 5, cv::Scalar(0, 165, 255), -1);
                    cv::circle(src, points_tmp[j][1], 5, cv::Scalar(0, 165, 255), -1);*/
                }
            }
        }
    }

    // std::cout<<result_point[0][0]<<std::endl<<result_point[0][1]<<std::endl<<result_point[0][2]<<std::endl<<result_point[0][3];
    for (int i = 0; i < group_cnt; i++)
    {

        /*std::cout << "特征点:" << result_point[0][0] << std::endl
                  << result_point[0][1] << std::endl
                  << result_point[0][2] << std::endl
                  << result_point[0][3];*/
        // 绘制
        cv::line(src, result_point[i][0], result_point[i][3], cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
        cv::line(src, result_point[i][1], result_point[i][2], cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
        points[i][0] = result_point[i][0];
        points[i][1] = result_point[i][1];
        points[i][2] = result_point[i][2];
        points[i][3] = result_point[i][3];
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

        std::ifstream cameraFile("../data/camera/matrix.txt");
        std::ifstream distFile("../data/camera/distcoeffs.txt");

        //从文件中读取相机内参和畸变系数
        cv::Mat cameraMatrix = cv::Mat::zeros(3, 3, CV_64F);
        cv::Mat distCoeffs = cv::Mat::zeros(5, 1, CV_64F);
        if (!cameraFile.is_open() || !distFile.is_open())
        {
            std::cerr << "无法打开文件" << std::endl;
            return;
        }

        // 读取相机内参
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                cameraFile >> cameraMatrix.at<double>(i, j);
            }
        }

        // 读取畸变系数
        for (int i = 0; i < 5; ++i)
        {
            distFile >> distCoeffs.at<double>(i, 0);
        }
        // 输出的旋转向量和平移向量
        cv::Mat rvec, tvec;

        cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);
        // cv::imshow("src", src);
        // cv::waitKey(0);
        /*std::cout << "Rotation Vector (rvec):" << std::endl
                  << rvec << std::endl;
        std::cout << "Translation Vector (tvec):" << std::endl
                  << tvec << std::endl;*/
        output_quater.push_back(rvec);
        output_position.push_back(tvec);
    }
}