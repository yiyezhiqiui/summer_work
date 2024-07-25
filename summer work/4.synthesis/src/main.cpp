#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>
#include <Eigen/Dense>

#include "../include/invoke.hpp"
#include "../include/quater_pos.hpp"
#include "../include/armor.hpp"

void kalman(position &original_position, double w, double r, double yaw);

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

Eigen::Matrix<double, 9, 1> x;
Eigen::Matrix<double, 9, 1> x_predict;
Eigen::Matrix<double, 9, 9> P;

int main()
{
    std::string video_path = "../data/video/2.mp4";

    cv::VideoCapture cap(video_path, cv::CAP_GSTREAMER);

    if (!cap.isOpened())
    {
        std::cerr << "Error: Could not open the video file with GStreamer." << std::endl;
        return 0;
    }

    std::cout << "Frame width: " << cap.get(cv::CAP_PROP_FRAME_WIDTH) << std::endl;
    std::cout << "Frame height: " << cap.get(cv::CAP_PROP_FRAME_HEIGHT) << std::endl;

    // 创建窗口
    cv::namedWindow("Processed Frame", cv::WINDOW_NORMAL);
    cv::resizeWindow("Processed Frame", 800, 600);

    cv::namedWindow("Kalman Visualization", cv::WINDOW_NORMAL);
    cv::resizeWindow("Kalman Visualization", 800, 600);

    cv::Mat frame;
    auto time_stt = std::chrono::high_resolution_clock::now();

    // 初始化卡尔曼滤波器
    x.setZero();                                 // 初始化状态向量
    P = Eigen::Matrix<double, 9, 9>::Identity(); // 初始化协方差矩阵

    int play_count = 570;
    cap.set(cv::CAP_PROP_POS_FRAMES, play_count);

    quaternion pre_center_quaternion(0, 0, 0);
    position pre_center_position(0, 0, 0);
    int continue_frame = 1;
    int count_r = 0;
    int count_dis = 0;
    int frame_count=0;

    while (true)
    {
        cap >> frame;

        if (frame.empty())
        {
            std::cout << "Video has ended or failed to read the frame." << std::endl;
            break;
        }
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
            // std::cout<<"yaw:"<<euler[0]<<"pitch:"<<euler[1]<<"roll:"<<euler[2]<<std::endl;
        }
        cv::Mat visualization = cv::Mat::zeros(600, 800, CV_8UC3); // 创建一个黑色背景的画布

        if (output_position.size() > 0 && output_quater.size() > 0)
        {
            std::cout<<"帧数:"<<frame_count<<std::endl;
            position position_center(output_position[0][0], output_position[0][1], output_position[0][2]);
            quaternion quaternion_center(euler[2], euler[1], euler[0]);

            // std::cout << "平移向量" << "x:" << output_position[0][0] << "y:" << output_position[0][1] << "z:" << output_position[0][2] << std::endl;
            // std::cout << "旋转向量" << "roll:" << output_quater[0][0] << "pitch:" << output_quater[0][1] << "yaw:" << output_quater[0][2] << std::endl;

            // transform_camera_to_Gimbal(quaternion_center, position_center)

            // 得到yaw,pitch,roll
            double yaw = quaternion_center.get_yaw();
            double pitch = quaternion_center.get_pitch();
            double roll = quaternion_center.get_roll();

            // 计算角速度
            double w_yaw = (yaw - pre_center_quaternion.get_yaw()) / continue_frame;
            double w_pitch = (pitch - pre_center_quaternion.get_pitch()) / continue_frame;
            double w_roll = (roll - pre_center_quaternion.get_roll()) / continue_frame;
            double w = sqrt(pingfang(w_yaw) + pingfang(w_pitch) + pingfang(w_roll));
            /*std::cout << "w:" << w << "  w_yaw:" << w_yaw << "  w_pitch:" << w_pitch << "  w_roll" << w_roll << std::endl;
            std::cout << "yaw:" << yaw << "  pitch:" << pitch << "  roll:" << roll << std::endl;
            std::cout << "pyaw:" << pre_center_quaternion.get_yaw() << "  ppitch:" << pre_center_quaternion.get_pitch() << "  proll:" << pre_center_quaternion.get_roll() << std::endl;*/
            // 计算线速度
            double vx = (position_center.getX() - pre_center_position.getX()) / continue_frame;
            double vy = (position_center.getY() - pre_center_position.getY()) / continue_frame;
            double vz = (position_center.getZ() - pre_center_position.getZ()) / continue_frame;
            double v = sqrt(pingfang(vx) + pingfang(vy) + pingfang(vz));
            /*std::cout << "x:" << position_center.getX() << "  y:" << position_center.getY() << "  z:" << position_center.getZ() << std::endl;
            std::cout << "px:" << pre_center_position.getX() << "  py:" << pre_center_position.getY() << "  pz:" << pre_center_position.getZ() << std::endl;
            std::cout << "xframe:" << continue_frame << std::endl;
            std::cout << "v" << v << "  vx:" << vx << "  vy:" << vy << "  vz:" << vz << std::endl;*/

            // 计算距离
            double distance = abs(v) / abs(w);

            position location_center(0, 0, distance);
            transform_xyz(quaternion_center, location_center, position_center.getX(), position_center.getY(), position_center.getZ());
            // 将位置传入卡尔曼函数
            kalman(location_center, w, 0.5, yaw);

            // 获取当前帧PnP解算出来的坐标并忽略z值
            cv::Point2f center_point(400.0, 300.0); // 画布的中点

            // 将当前点画在画布的中点上
            cv::circle(visualization, center_point, 5, cv::Scalar(0, 0, 255), -1);

            double draw_x = static_cast<int>(x_predict(0) * 1000) - static_cast<int>(position_center.getX() * 1000);
            double draw_y = static_cast<int>(x_predict(1) * 1000) - static_cast<int>(position_center.getY() * 1000);
            // 计算预测点相对于当前帧的坐标并忽略z值
            cv::Point2f predict_point(center_point.x + draw_x, center_point.y + draw_y);

            // 测试
            //std::cout << "预测小车中心" << x_predict(0) << "  " << x_predict(1) << "  " << x_predict(2) << std::endl;
            //std::cout << "装甲板中心" << output_position[0][0] << "  " << output_position[0][1] << "  " << output_position[0][2] << std::endl;
            std::string result;
            result = result + "装甲板坐标为(x,y,z):(" + std::to_string(x_predict(0)) + "," + std::to_string(x_predict(1)) + "," + std::to_string(x_predict(2)) + ")";
            std::cout<<result<<std::endl;
            
            double r = sqrt(pingfang(x_predict(0) - output_position[0][0]) + pingfang(x_predict(1) - output_position[0][1]) + pingfang(x_predict(2) - output_position[0][2]));
            //std::cout << "距离r" << r << std::endl;
            //std::cout << "距离distance" << distance << std::endl;
            if (r > 0.3 && r < 0.8)
            {
                count_r++;
            }
            if (distance > 0.3 && distance < 0.8)
            {
                count_dis++;
            }
            std::cout <<"r:"<< count_r <<"  distance:"<<count_dis<<"frame_count"<<frame_count<< std::endl<<std::endl;
            // 将预测点画在画布上
            cv::circle(visualization, predict_point, 5, cv::Scalar(0, 255, 0), -1);

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
        cv::imshow("Kalman Visualization", visualization);
        //  调整等待时间为 1 毫秒，以确保视频播放连贯
        if (cv::waitKey(1) == 27) // 按下ESC键退出
        {
            break;
        }
    }
    auto time_end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsedTime = time_end - time_stt;
    //std::cout << "Average FPS: " <<frame_count<<"  "<< 572 / elapsedTime.count() << std::endl;

    cap.release();
    cv::destroyAllWindows();

    return 0;
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