#include <vector>
#include <string>
#include <iostream>
#include <boost/algorithm/string.hpp>
#include <fstream>
#include <opencv2/opencv.hpp> // 包含 OpenCV 头文件

#include "../include/armor.hpp"
#include "../include/number_classifier.hpp"
#include "../include/invoke.hpp"
#include "../include/quater_pos.hpp"

// 四元数
#define Pi 3.141592
quaternion quaternion_new(0, 0, 0);
position position_new(0, 0, 0);
void invoke_quater_pos()
{
    std::string transform_way;
    std::getline(std::cin, transform_way);
    std::vector<std::string> tokens;
    boost::split(tokens, transform_way, boost::is_any_of(std::string(1, '/')));

    double yaw, pitch, roll, x, y, z;
    std::cout << "请输入测试数据(yaw,roll,pitch,x,y,z):" << std::endl;
    std::cin >> yaw >> pitch >> roll >> x >> y >> z;

    quaternion quaternion_original(yaw, pitch, roll);
    position position_original(x, y, z);
    if (tokens[1] == "Gimbal")
    {
        transform_camera_to_Gimbal(quaternion_original, position_original);
    }
    else if (tokens[1] == "Odom")
    {
        transform_camera_to_Gimbal(quaternion_original, position_original);
        quaternion quaternion_new_creat(quaternion_new.yaw, quaternion_new.pitch, quaternion_new.roll);
        transform_Gimbal_to_Odom(quaternion_new_creat, position_new);
    }
    else
    {
        transform_camera_to_Gimbal(quaternion_original, position_original);
        quaternion quaternion_new_creat(quaternion_new.yaw, quaternion_new.pitch, quaternion_new.roll);
        transform_Gimbal_to_Shooter(quaternion_new_creat, position_new);
    }
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
