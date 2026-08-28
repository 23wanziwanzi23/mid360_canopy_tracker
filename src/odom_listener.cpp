#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h> // 提供四元数转欧拉角的数学计算
#include <cmath> // 提供 sqrt, pow, abs 等数学函数

// 全局变量，记录起始位置
bool is_first_run = true;
double start_x = 0.0;
double start_y = 0.0;

// 回调函数：每次接收到里程计数据时触发
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    
    /*****计算移动距离*****/
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
    
    
    /*****计算当前朝向*****/
    // 1. 从 msg 中提取四元数 (x, y, z, w)
    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w
    );

    // 2. 将四元数转换为 3x3 旋转矩阵，然后再提取出欧拉角 (Roll, Pitch, Yaw)
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw); // 传入引用，函数会自动将计算结果赋值给这三个变量

    // 3. 将 Yaw 角 (弧度) 转换为度数，方便调试查看
    double current_yaw_degree = yaw * 180.0 / M_PI;
    
    // 打印结果
    ROS_INFO("当前车体绝对偏航角: %.2f 度", current_yaw_degree);
    
    // 【你的状态机逻辑可以写在这里】
    // 例如：if (current_yaw_degree >= target_yaw) { 发送停止指令; }
}

int main(int argc, char** argv) {

    setlocale(LC_ALL,"");
    // 初始化 ROS 节点
    ros::init(argc, argv, "odom_listener_node");
    ros::NodeHandle nh;

    // 订阅 FAST-LIO 输出的里程计话题 (话题名视具体配置而定，通常是 /Odometry)
    ros::Subscriber sub = nh.subscribe("/Odometry", 10, odomCallback);

    // 保持程序不死，等待回调
    ros::spin();

    return 0;
}