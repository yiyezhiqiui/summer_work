#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp> // 包含 OpenCV 头文件

#include "../include/armor.hpp"
#include "../include/number_classifier.hpp"

cv::Mat src;          // 这里定义了 src 变量
cv::Point2f point_extern[5][4]; // 这里定义了 point 数组
int group_cnt;

// pnp 函数的声明
void pnp();

int main()
{
    // 初始化 src 变量
    int src_picture_index;
    std::cout << "请输入你要对第几张图片进行处理 (1~3): ";
    std::cin >> src_picture_index;
    // 检查用户输入是否在合理范围内
    while (src_picture_index < 0 || src_picture_index > 20)
    {
        std::cerr << "输入的图片编号超出范围！请重新输入有效编号。" << std::endl;
        std::cin >> src_picture_index; 
    }
    std::string src_picture = "../picture/" + std::to_string(src_picture_index) + ".png";
    src = cv::imread(src_picture);

    /// 数字分类器
    std::string model_path = "../model/mlp.onnx";
    std::string label_path = "../model/label.txt";
    float confidence_threshold = 1;
    std::vector<std::string> ignore_classes = {"outpost", "uard", "base", "negative"};
    armor::NumberClassifier classifier(model_path, label_path, confidence_threshold, ignore_classes);

    // 提取数字
    pnp();
    for(int i=0;i<group_cnt;i++)
    {
        Armor armors_tmp;
    std::vector<Armor> armorlist;
    armors_tmp.left_light.top = point_extern[i][1];
    armors_tmp.left_light.bottom = point_extern[i][0];
    armors_tmp.right_light.top = point_extern[i][2];
    armors_tmp.right_light.bottom = point_extern[i][3];
    armors_tmp.type = ArmorType::SMALL;
    armorlist.push_back(armors_tmp);
    classifier.ExtractNumbers(src, armorlist);

    // 分类
    classifier.Classify(armorlist);
    // 输出结果
    std::cout << "Armor Number: " << armorlist[0].number << std::endl;
    std::cout << "Classification Result: " << armorlist[0].classification_result << std::endl;
    std::cout << std::endl;
    }
    return 0;
}