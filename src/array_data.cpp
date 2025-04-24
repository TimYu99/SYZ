// array_data.cpp

#include "array_data.h"
#include <iostream>
#include <vector>
#include <vector>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <ctime>
#include <sstream>
// 全局数组定义和初始化
//const int rows = 379;
//const int cols = 401;

std::vector<Centroid> trajectory;
Centroid last_centroid;

//代码流程：
//计算帧间差异：首先，计算 frame1 和 frame2 的差异图。
//目标检测：通过二值化和形态学操作去噪后，提取目标区域的质心。
//位移判断：如果上一帧的质心存在，计算当前帧质心与上一帧的质心之间的距离，判断是否有较大位移。
//更新上一帧的质心：将当前帧的质心作为下一帧的上一帧质心，供后续使用。
//参数说明：
//frame1 和 frame2：两帧图像，表示连续的视频帧。
//no_target：标记是否检测到目标，0 表示未检测到目标，1 表示检测到目标。
//centroid：返回的目标质心，包含 x 和 y 坐标。
int process_frame_difference(cv::Mat frame1, cv::Mat frame2,int& no_target, Centroid& centroid)
{
    int large_movement_count=0;
    // 初始化
    static Centroid last_centroid = { 0, 0 };  // 记录上一次的质心
    // 计算帧间差异
    cv::Mat diff;
    cv::absdiff(frame2, frame1, diff);

    // 计算差异的统计特性
    cv::Scalar mean_diff, std_diff;
    cv::meanStdDev(diff, mean_diff, std_diff);

    // 设置动态阈值
    double threshold = mean_diff[0] + 1.5 * std_diff[0];

    // 二值化差分图像
    cv::Mat binary_image;
    cv::threshold(diff, binary_image, threshold, 255, cv::THRESH_BINARY);
    bool result = cv::imwrite("D:/ceshi/binary_image.png", binary_image);
    if (result) {
        std::cout << "binary_image图像已保存到文件夹!" << std::endl;
    }
    else {
        std::cerr << "保存图像失败!" << std::endl;
    }

    // 后处理：形态学操作去噪2025年2月13日修改
    cv::Mat cleaned_image1;
    cv::Mat se1 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    cv::morphologyEx(binary_image, cleaned_image1, cv::MORPH_OPEN, se1);
    cv::Mat cleaned_image;
    cv::Mat se = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(4, 3));
    cv::morphologyEx(binary_image, cleaned_image, cv::MORPH_OPEN, se);
    // 后处理：形态学操作去噪
    //cv::Mat cleaned_image1;
    //cv::Mat se1 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(4, 4));
    //cv::morphologyEx(binary_image, cleaned_image1, cv::MORPH_OPEN, se1);
    //cv::Mat cleaned_image;
    //cv::Mat se = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    //cv::morphologyEx(binary_image, cleaned_image, cv::MORPH_OPEN, se);
    bool result1 = cv::imwrite("D:/ceshi/cleaned_image.png", cleaned_image);
    if (result1) {
        std::cout << "cleaned_image图像已保存到文件夹!" << std::endl;
    }
    else {
        std::cerr << "保存图像失败!" << std::endl;
    }
    bool result2 = cv::imwrite("D:/ceshi/cleaned_image1.png", cleaned_image1);
    if (result2) {
        std::cout << "cleaned_image1图像已保存到文件夹!" << std::endl;
    }
    else {
        std::cerr << "保存图像失败!" << std::endl;
    }
    // 去掉小区域
    cv::Mat labeled_image;
    int num_labels = cv::connectedComponents(cleaned_image1, labeled_image);

    // 获取目标区域的质心
    std::vector<Centroid> centroids;
    for (int label = 1; label < num_labels; ++label) {  // 标签从1开始，0是背景
        cv::Moments moments = cv::moments(labeled_image == label, true);
        if (moments.m00 > 0) {  // 如果区域面积不为零
            Centroid c = { static_cast<float>(moments.m10 / moments.m00), static_cast<float>(moments.m01 / moments.m00) };
            centroids.push_back(c);
        }
    }

    // 检查是否检测到目标
    if (centroids.empty()) {
        no_target = 0;  // 没有目标检测到
        large_movement_count = 0;  // 没有目标时，位移计数清零
        return 0;
    }

    // 选取第一个质心作为目标
    centroid = centroids.front();
    no_target = 1;  // 目标已检测到
    // 保存图像
    saveImageWithTimestamp1(cleaned_image1);
    saveImageWithTimestamp(cleaned_image);
    if (last_centroid.x != 0 && last_centroid.y != 0) {  // 如果上一帧存在有效的质心
        float distance = std::sqrt(std::pow(centroid.x - last_centroid.x, 2) + std::pow(centroid.y - last_centroid.y, 2));

        // 判断是否有较大位移
        float large_movement_threshold = 5.0f;  // 5像素阈值
        if (distance > large_movement_threshold) {
            large_movement_count = 1;  // 有较大位移
        }
        else {
            large_movement_count = 0;  // 位移较小
        }
    }
    else {
        large_movement_count = 0;  // 如果是第一帧，则没有位移
    }
    // 更新上一帧的质心
    last_centroid = centroid;

    

    return 1;
}





// 打印某个位置的值函数定义
void printValueAt(int row, int col) {
    if (row >= 0 && row < rows && col >= 0 && col < cols) {
        //std::cout << "Value at (" << row << ", " << col << ") is: " << beijingtu21[row][col] << std::endl;
    }
    else {
        std::cout << "Index out of bounds." << std::endl;
    }
}
void saveImageWithTimestamp(const cv::Mat& image) {
    // 获取当前时间
    std::time_t now = std::time(nullptr);
    std::tm* now_tm = std::localtime(&now);

    // 格式化时间戳
    std::ostringstream oss;
    oss << "D:/mubiao/target_"
        << (now_tm->tm_year + 1900) << "-"
        << (now_tm->tm_mon + 1) << "-"
        << now_tm->tm_mday << "_"
        << now_tm->tm_hour << "-"
        << now_tm->tm_min << "-"
        << now_tm->tm_sec << ".png";

    std::string filename = oss.str();

    // 保存图像
    bool result2 = cv::imwrite(filename, image);
    if (result2) {
        std::cout << "图像已保存到文件: " << filename << std::endl;
    }
    else {
        std::cerr << "保存图像失败!" << std::endl;
    }
}
void saveImageWithTimestamp1(const cv::Mat& image) {
    // 获取当前时间
    std::time_t now = std::time(nullptr);
    std::tm* now_tm = std::localtime(&now);

    // 格式化时间戳
    std::ostringstream oss;
    oss << "D:/mubiao/target1_"
        << (now_tm->tm_year + 1900) << "-"
        << (now_tm->tm_mon + 1) << "-"
        << now_tm->tm_mday << "_"
        << now_tm->tm_hour << "-"
        << now_tm->tm_min << "-"
        << now_tm->tm_sec << ".png";

    std::string filename = oss.str();

    // 保存图像
    bool result2 = cv::imwrite(filename, image);
    if (result2) {
        std::cout << "图像已保存到文件: " << filename << std::endl;
    }
    else {
        std::cerr << "保存图像失败!" << std::endl;
    }
}
//// 矩阵相减函数定义subtractArrays(array1, array2, result);
//void subtractArrays(const int arr1[rows][cols], const int arr2[rows][cols], int result[rows][cols]) {
//    for (int i = 0; i < rows; ++i) {
//        for (int j = 0; j < cols; ++j) {
//            result[i][j] = arr1[i][j] - arr2[i][j];
//        }
//    }
//}
//// 统计数组里存在各区域值阈值
//std::vector<int> countIntervals(const std::vector<int>& arr) {
//    const int interval = 10000;
//    const int numIntervals = 7;
//    std::vector<int> counts(numIntervals, 0);
//
//    for (int value : arr) {
//        if (value >= 0 && value <= 65535) {
//            int index = value / interval;
//            if (index >= numIntervals) {
//                index = numIntervals - 1;
//            }
//            counts[index]++;
//        }
//    }
//
//    return counts;
//}
//struct IntervalCounts {
//    int count_0_9999;
//    int count_10000_19999;
//    int count_20000_29999;
//    int count_30000_39999;
//    int count_40000_49999;
//    int count_50000_59999;
//    int count_60000_65535;
//};
//// 函数用于计算每个区间的累计数量
//IntervalCounts accumulateCounts(const std::vector<int>& counts) {
//    IntervalCounts accumulatedCounts = { 0, 0, 0, 0, 0, 0, 0 };
//    accumulatedCounts.count_0_9999 = counts[0];
//    accumulatedCounts.count_10000_19999 = accumulatedCounts.count_0_9999 + counts[1];
//    accumulatedCounts.count_20000_29999 = accumulatedCounts.count_10000_19999 + counts[2];
//    accumulatedCounts.count_30000_39999 = accumulatedCounts.count_20000_29999 + counts[3];
//    accumulatedCounts.count_40000_49999 = accumulatedCounts.count_30000_39999 + counts[4];
//    accumulatedCounts.count_50000_59999 = accumulatedCounts.count_40000_49999 + counts[5];
//    accumulatedCounts.count_60000_65535 = accumulatedCounts.count_50000_59999 + counts[6];
//    return accumulatedCounts;
//}
//// 根据输入角度计算对应的列索引
//int getColumnIndex(double angle) {
//    if (angle < 0.0 || angle > 360.0) {
//        throw std::out_of_range("Angle must be between 0 and 360 degrees.");
//    }
//    // 每列代表0.9度
//    int colIndex = static_cast<int>(std::round(angle / 0.9));
//    if (colIndex >= cols) {
//        colIndex = cols - 1; // 防止索引越界
//    }
//    return colIndex;
//}
//
//// 获取对应角度的那一列数据
//std::vector<int> getColumnData(double angle) {
//    int colIndex = getColumnIndex(angle);
//    std::vector<int> columnData(rows);
//    for (int i = 0; i < rows; ++i) {
//        //columnData[i] = beijingtu21[i][colIndex];
//    }
//    return columnData;
//}
