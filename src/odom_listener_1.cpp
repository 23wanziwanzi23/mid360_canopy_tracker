#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <vector>
#include <cmath> // 提供 sqrt, pow, abs 等数学函数

// 全局变量，记录起始位置
bool is_first_run = true;
double start_x = 0.0;
double start_y = 0.0;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    // 1. 获取当前时刻的绝对坐标
    double current_x = msg->pose.pose.position.x;
    double current_y = msg->pose.pose.position.y;

    // 2. 锁定起点坐标
    if (is_first_run) {
        start_x = current_x;
        start_y = current_y;
        is_first_run = false;
        ROS_INFO("起点坐标已锁定: X=%.2f, Y=%.2f", start_x, start_y);
        return;
    }

    // 3. 计算各个维度的移动距离
    // X 轴移动距离：|当前 X - 起点 X|
    double delta_x = std::abs(current_x - start_x);
    
    // Y 轴移动距离：|当前 Y - 起点 Y|
    double delta_y = std::abs(current_y - start_y);
    
    // 距离原点（起点）的直线距离：欧氏距离公式
    double total_distance = std::sqrt(std::pow(current_x - start_x, 2) + std::pow(current_y - start_y, 2));

    // 4. 打印三个数据 (保留三位小数，精确到毫米)
    ROS_INFO("移动监控 -> X轴移动: %.3f m, Y轴移动: %.3f m, 直线距离: %.3f m", 
             delta_x, delta_y, total_distance);

    // 【控制逻辑示例】
    // 比如：如果要求机器横向平移 0.5 米，且总距离不超过 0.6 米：
    // if (delta_y >= 0.5 && total_distance <= 0.6) { ... }
}

// 定义一个简单的 2D 点结构体
struct Point2D {
    double x;
    double y;
};

// PCA 直线拟合函数
// 输入：截取的一小簇雷达点云 (全局坐标)
// 返回：拟合出的切线方向角 (弧度)
double calculateCanopyTangent(const std::vector<Point2D>& points) {
    if (points.size() < 2) {
        return 0.0; // 点太少，无法拟合
    }

    double sum_x = 0.0, sum_y = 0.0;
    int N = points.size();

    // 1. 求均值 (质心)
    for (const auto& p : points) {
        sum_x += p.x;
        sum_y += p.y;
    }
    double mean_x = sum_x / N;
    double mean_y = sum_y / N;

    // 2. 求协方差元素
    double cov_xx = 0.0, cov_yy = 0.0, cov_xy = 0.0;
    for (const auto& p : points) {
        double dx = p.x - mean_x;
        double dy = p.y - mean_y;
        cov_xx += dx * dx;
        cov_yy += dy * dy;
        cov_xy += dx * dy;
    }

    // 3. 计算切线角度
    // atan2 的返回值在 -pi 到 pi 之间，乘以 0.5 后在 -pi/2 到 pi/2 之间
    double tangent_angle = 0.5 * std::atan2(2.0 * cov_xy, cov_xx - cov_yy);

    return tangent_angle;
}

int main(int argc, char** argv) {

    setlocale(LC_ALL,"");
    // ros::init(argc, argv, "odom_listener_distance");
    // ros::NodeHandle nh;

    // ros::Subscriber sub = nh.subscribe("/Odometry", 10, odomCallback);

    // 假设这是雷达扫到的果树边缘的一簇点
    std::vector<Point2D> canopy_points = {
        {1.0, 2.0}, {1.1, 2.2}, {1.2, 2.3}, {1.35, 2.7}, {1.4, 2.9}
    };

    double angle_rad = calculateCanopyTangent(canopy_points);
    double angle_deg = angle_rad * 180.0 / M_PI;

    std::cout << "树冠局部切线角度 (度): " << angle_deg << std::endl;

    // ros::spin();
    return 0;
}