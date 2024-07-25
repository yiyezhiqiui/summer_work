#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>
#include <Eigen/Dense>

#include "../include/quater_pos.hpp"
#include "../include/armor.hpp"
#include "../include/picture_process.hpp"
#include "../include/client.hpp"
#include "../include/number_classifier.hpp"


quaternion pre_center_quaternion(0, 0, 0);
position pre_center_position(0, 0, 0);

Eigen::Matrix<double, 9, 1> x;
Eigen::Matrix<double, 9, 1> x_predict;
Eigen::Matrix<double, 9, 9> P;


// 四元数
#define Pi 3.141592
quaternion quaternion_new(0, 0, 0);
position position_new(0, 0, 0);

double pingfang(double x)
{
    return x * x;
}

// 将旋转向量转换为欧拉角（XYZ顺序）
cv::Vec3d rotationVectorToEulerAngles(const cv::Vec3d &rotationVector)
{
    // 将旋转向量转换为旋转矩阵
    cv::Mat rotationMatrix;
    cv::Rodrigues(rotationVector, rotationMatrix);

    // 从旋转矩阵提取欧拉角
    double sy = std::sqrt(rotationMatrix.at<double>(0, 0) * rotationMatrix.at<double>(0, 0) +
                          rotationMatrix.at<double>(1, 0) * rotationMatrix.at<double>(1, 0));

    bool singular = sy < 1e-6; // 如果sy接近0，则视为奇异

    double x, y, z;
    if (!singular)
    {
        x = std::atan2(rotationMatrix.at<double>(2, 1), rotationMatrix.at<double>(2, 2));
        y = std::atan2(-rotationMatrix.at<double>(2, 0), sy);
        z = std::atan2(rotationMatrix.at<double>(1, 0), rotationMatrix.at<double>(0, 0));
    }
    else
    {
        x = std::atan2(-rotationMatrix.at<double>(1, 2), rotationMatrix.at<double>(1, 1));
        y = std::atan2(-rotationMatrix.at<double>(2, 0), sy);
        z = 0;
    }

    return cv::Vec3d(x, y, z);
}

void picture_process(cv::Mat &frame, int sock, Process &process)
{
    // cv::imshow("test",frame);
    //  创建窗口
    cv::namedWindow("Processed Frame", cv::WINDOW_NORMAL);
    cv::resizeWindow("Processed Frame", 800, 600);

    auto time_stt = std::chrono::high_resolution_clock::now();

    // 初始化卡尔曼滤波器
    x.setZero();                                 // 初始化状态向量
    P = Eigen::Matrix<double, 9, 9>::Identity(); // 初始化协方差矩阵

    int play_count = 570;

    int continue_frame = 1;
    int count_r = 0;
    int count_dis = 0;
    int frame_count = 0;

    frame_count++;
    cv::Point2f points[6][4]; // 特征点
    std::vector<cv::Vec3d> output_position, output_quater;

    int group_cnt = 0;
    int type = 0;
    pnp(frame, points, group_cnt, output_position, output_quater, type);
    cv::Vec3d euler;
    for (const auto &rotationVector : output_quater)
    {
        euler = rotationVectorToEulerAngles(rotationVector);
    }
    cv::Mat visualization = cv::Mat::zeros(600, 800, CV_8UC3); // 创建一个黑色背景的画布

    if (output_position.size() > 0 && output_quater.size() > 0)
    {
        position position_center(output_position[0][0], output_position[0][1], output_position[0][2]);
        quaternion quaternion_center(euler[2], euler[1], euler[0]);

        std::cout << "test_original:" << output_position[0][0] << "  " << output_position[0][1] << "  " << output_position[0][2] << std::endl;
        // 得到yaw,pitch,roll
        double yaw = quaternion_center.get_yaw();
        double pitch = quaternion_center.get_pitch();
        double roll = quaternion_center.get_roll();

        // 计算角速度
        double w_yaw = (yaw - pre_center_quaternion.get_yaw()) / continue_frame;
        double w_pitch = (pitch - pre_center_quaternion.get_pitch()) / continue_frame;
        double w_roll = (roll - pre_center_quaternion.get_roll()) / continue_frame;
        double w = sqrt(pingfang(w_yaw) + pingfang(w_pitch) + pingfang(w_roll));

        // 计算线速度
        double vx = (position_center.getX() - pre_center_position.getX()) / continue_frame;
        double vy = (position_center.getY() - pre_center_position.getY()) / continue_frame;
        double vz = (position_center.getZ() - pre_center_position.getZ()) / continue_frame;
        double v = sqrt(pingfang(vx) + pingfang(vy) + pingfang(vz));

        // 计算距离
        double distance = abs(v) / abs(w);
        process.res_position = position(process.res_position.getX() + vx / v * distance, process.res_position.getY() + vy / v * distance, process.res_position.getZ());

        // 将位置传入卡尔曼函数
        kalman(process.res_position, w, distance, yaw);
        std::cout << "test_kalman:" << process.res_position << std::endl
                  << std::endl;
        transform_camera_to_Gimbal(process.res_quaternion, process.res_position, process);
        std::cout << "test_cam_to_gim:" << process.res_position;
        transform_Gimbal_to_Odom(process.res_quaternion, process.res_position, process);
        std::cout << "test_gim_to_odom:" << process.res_position;

        // 测试
        double r = sqrt(pingfang(x_predict(0) - output_position[0][0]) + pingfang(x_predict(1) - output_position[0][1]) + pingfang(x_predict(2) - output_position[0][2]));
        if (r > 0.3 && r < 0.8)
        {
            count_r++;
        }
        if (distance > 0.3 && distance < 0.8)
        {
            count_dis++;
        }

        // 将预测点画在画布上
        // cv::circle(visualization, predict_point, 5, cv::Scalar(0, 255, 0), -1);

        pre_center_quaternion = quaternion(euler[2], euler[1], euler[0]);
        pre_center_position = position(output_position[0][0], output_position[0][1], output_position[0][2]);
        continue_frame = 1;
    }
    else
    {
        continue_frame++;
        pre_center_quaternion = quaternion(0, 0, 0);
        pre_center_position = position(0, 0, 0);
    }

    // 调用处理函数处理当前帧并返回处理后的帧
    cv::Mat processedFrame = invoke_armor_visual(frame, points, group_cnt, type);
    // 显示处理后的图像
    cv::imshow("Processed Frame", processedFrame);
    // cv::imshow("Kalman Visualization", visualization);

    cv::waitKey(1);
    auto time_end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsedTime = time_end - time_stt;
    // std::cout << "Average FPS: " <<frame_count<<"  "<< 572 / elapsedTime.count() << std::endl;

    cv::destroyAllWindows();
}

void kalman(position &original_position, double w, double r, double yaw)
{
    // 状态转移矩阵
    Eigen::Matrix<double, 9, 9> A;
    A << 1, 0, 0, 1, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 1, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 1, 0, 0, 0,
        0, 0, 0, 1, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 1, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 1, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 1;

    // 观测矩阵
    Eigen::Matrix<double, 6, 9> H;
    H << 1, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 1;

    // 过程噪声协方差矩阵
    Eigen::Matrix<double, 9, 9> Q;
    Q << 1, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 1, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 1, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 1, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 1;

    // 观测噪声协方差
    Eigen::Matrix<double, 6, 6> R;
    R << 1, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0,
        0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 1;

    // 预测步骤
    x = A * x;                     // 预测状态
    P = A * P * A.transpose() + Q; // 预测协方差

    // 计算卡尔曼增益
    Eigen::Matrix<double, 9, 6> K = P * H.transpose() * (H * P * H.transpose() + R).inverse();

    // 更新步骤
    Eigen::Matrix<double, 6, 1> z;
    z << original_position.getX(), original_position.getY(), original_position.getZ(), w, r, yaw;

    x = x + K * (z - H * x);                                   // 更新状态
    P = (Eigen::Matrix<double, 9, 9>::Identity() - K * H) * P; // 更新协方差

    x_predict = A * x;
}



cv::Mat invoke_armor_visual(cv::Mat frame,cv::Point2f (*points)[4],int& group_cnt,int &type)
{
    // 初始化 src 变量
    cv::Mat src = frame;

    // 数字分类器
    std::string model_path = "../data/model/mlp.onnx";
    std::string label_path = "../data/model/label.txt";
    float confidence_threshold = 0.7;
    std::vector<std::string> ignore_classes = {"outpost", "uard", "base", "negative"};

    armor::NumberClassifier classifier(model_path, label_path, confidence_threshold, ignore_classes);


    // 提取数字
    for (int i = 0; i < group_cnt; i++)
    {
        Armor armors_tmp;
        std::vector<Armor> armorlist;

        armors_tmp.left_light.top = points[i][2];
        armors_tmp.left_light.bottom = points[i][3];
        armors_tmp.right_light.top = points[i][1];
        armors_tmp.right_light.bottom = points[i][0];

        //装甲板类型
        if(type==0)
        {
            armors_tmp.type = ArmorType::SMALL;
        }
        else
        {
            armors_tmp.type = ArmorType::LARGE;
        }

        armorlist.push_back(armors_tmp);
        classifier.ExtractNumbers(src, armorlist);

        // 分类
        classifier.Classify(armorlist);

        // 输出结果
        //for(auto arm: armorlist)
        //{
        //    std::cout << "Classification Result_test: " << arm.classification_result<< std::endl;
        //}
        /*std::cout << "Armor Number: " << armorlist[0].number << std::endl;
        std::cout << "Classification Result_test: " << armorlist[0].classification_result<< std::endl;
        std::cout << "Classification Result: " << armorlist[0].number<<": "<<armorlist[0].classification_confidence*100.0<<"%" << std::endl;
        std::cout << std::endl;*/
    }
    return src;
}
