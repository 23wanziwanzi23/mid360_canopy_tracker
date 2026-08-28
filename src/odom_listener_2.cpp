#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_datatypes.h>
#include <cmath>
#include <vector>

// 定义简单的 2D 点结构体
struct Point2D {
    double x;
    double y;
};

class CanopyTracker {
private:
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    ros::Subscriber scan_sub_;

    // 机器人状态数据
    double current_yaw_ = 0.0;
    double start_x_ = 0.0;
    double start_y_ = 0.0;
    double total_distance_ = 0.0;
    bool is_first_odom_ = true;

    // 算法参数
    const double TARGET_DISTANCE = 0.8; // 期望离树冠的作业距离 (米)
    const double Kp_dist = 0.5;         // 距离修正比例系数

public:
    CanopyTracker() {
        // 订阅里程计和 2D 激光雷达
        odom_sub_ = nh_.subscribe("/Odometry", 10, &CanopyTracker::odomCallback, this);
        scan_sub_ = nh_.subscribe("/scan_2d", 10, &CanopyTracker::scanCallback, this);
        ROS_INFO("树冠轮廓跟随节点已启动，等待数据输入...");
    }

    // PCA 拟合直线，返回切线角度 (相对于雷达坐标系)
    double calculateCanopyTangent(const std::vector<Point2D>& points, double& avg_distance) {
        if (points.size() < 5) return 0.0; // 有效点太少，不进行拟合

        double sum_x = 0.0, sum_y = 0.0, sum_dist = 0.0;
        int N = points.size();

        for (const auto& p : points) {
            sum_x += p.x;
            sum_y += p.y;
            sum_dist += std::sqrt(p.x * p.x + p.y * p.y);
        }
        double mean_x = sum_x / N;
        double mean_y = sum_y / N;
        avg_distance = sum_dist / N; // 顺便算出当前这簇点离车体的平均距离

        double cov_xx = 0.0, cov_yy = 0.0, cov_xy = 0.0;
        for (const auto& p : points) {
            double dx = p.x - mean_x;
            double dy = p.y - mean_y;
            cov_xx += dx * dx;
            cov_yy += dy * dy;
            cov_xy += dx * dy;
        }

        // 计算局部切线角度
        return 0.5 * std::atan2(2.0 * cov_xy, cov_xx - cov_yy);
    }

    // 里程计回调：负责更新全局位置和朝向
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        double current_x = msg->pose.pose.position.x;
        double current_y = msg->pose.pose.position.y;

        // 提取绝对 Yaw 角
        tf::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        );
        tf::Matrix3x3 m(q);
        double roll, pitch;
        m.getRPY(roll, pitch, current_yaw_);

        // 计算移动距离
        if (is_first_odom_) {
            start_x_ = current_x;
            start_y_ = current_y;
            is_first_odom_ = false;
        } else {
            total_distance_ = std::sqrt(std::pow(current_x - start_x_, 2) + std::pow(current_y - start_y_, 2));
        }
    }

    // 雷达回调：负责感知树冠并输出控制指令
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        std::vector<Point2D> roi_points;
        
        // 1. 提取感兴趣区域 (ROI)：只看车头右前方 0 ~ 60 度的点
        // 假设我们打算逆时针绕树，果树在机器人的右侧
        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            double r = msg->ranges[i];
            // 过滤无效噪点和太远的点
            if (std::isinf(r) || std::isnan(r) || r < 0.2 || r > 2.5) continue; 
            
            double angle = msg->angle_min + i * msg->angle_increment;
            
            // 筛选右前方的雷达点 (0 到 -60度，因为右手系中右侧角度为负)
            if (angle < 0.0 && angle > -M_PI/3.0) {
                // 极坐标转直角坐标 (相对于雷达自身)
                roi_points.push_back({r * std::cos(angle), r * std::sin(angle)});
            }
        }

        double current_tree_distance = 0.0;
        double tangent_angle = calculateCanopyTangent(roi_points, current_tree_distance);

        if (roi_points.size() >= 5) {
            // 2. 距离修正 (PD控制里的 P)
            // 如果离树太远 (error > 0)，需要向树的方向靠拢；反之则远离。
            double distance_error = current_tree_distance - TARGET_DISTANCE;
            double correction_angle = Kp_dist * distance_error;

            // 3. 计算最终下发给底盘的相对转向角
            // 基础切线角度 + 距离修正角度
            double target_steering_angle = tangent_angle + correction_angle;

            ROS_INFO("当前累计移动: %.2fm | 树冠距离: %.2fm | 拟合切线: %.1f° | 修正航向: %.1f°",
                     total_distance_, 
                     current_tree_distance, 
                     tangent_angle * 180.0 / M_PI,
                     target_steering_angle * 180.0 / M_PI);
                     
            // 【整合你的状态机】
            // 如果发现 total_distance_ 达到要求，且 current_yaw_ 变化了 60度
            // 就可以在这里下发 cmd_vel 停车，并切换到震动采摘的准备状态
        } else {
            ROS_WARN("视野内未发现有效树冠点云，维持当前航向或停车...");
        }
    }
};

int main(int argc, char** argv) {
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "canopy_tracker_node");
    
    // 实例化对象，ROS 会自动接管后台回调循环
    CanopyTracker tracker;
    
    ros::spin();
    return 0;
}