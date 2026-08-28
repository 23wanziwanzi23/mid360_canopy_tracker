#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
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

int main(int argc, char** argv) {
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "odom_listener_distance");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/Odometry", 10, odomCallback);

    ros::spin();
    return 0;
}