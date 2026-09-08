#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_datatypes.h>
// 1. 新增：引入可视化标记的头文件
#include <visualization_msgs/Marker.h>
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
    // 2. 新增：定义一个 Marker 发布者
    ros::Publisher marker_pub_;

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

        // 3. 新增：初始化发布者，话题名为 /steering_vector
        marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/steering_vector", 10);

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
        
        // ==========================================
        // 切线角度范围窗口限制 [-80°, -20°]
        // ==========================================
        // 1. 将弧度转为度数，方便进行直观判断
        double tangent_deg = tangent_angle * 180.0 / M_PI;
        
        // 2. 判断逻辑：如果不在 -80° 到 -20° 之间，强制归零
        if (tangent_deg <= -20.0 && tangent_deg >= -80.0) {
            // 角度合法，什么都不做，保留原有的 tangent_angle
        } else {
            // 角度越界，切线角度强制设为 0 弧度
            tangent_angle = 0.0; 
            tangent_deg = 0.0; // 同步清零，为了下面终端打印显示正确
        }
        // ==========================================

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
        
            
            // ==========================================
            // 4. 新增：构建并发布 RViz 可视化箭头
            // ==========================================
            visualization_msgs::Marker arrow;
            // 坐标系必须和你的雷达点云一致，通常是 livox_frame
            arrow.header.frame_id = "livox_frame"; 
            arrow.header.stamp = ros::Time::now();
            arrow.ns = "canopy_tracker";
            arrow.id = 0;
            // 形状设为箭头
            arrow.type = visualization_msgs::Marker::ARROW;
            arrow.action = visualization_msgs::Marker::ADD;

            // 箭头的起点和终点
            geometry_msgs::Point p_start, p_end;
            // 起点设定在机器人中心 (雷达原点 0,0,0)
            p_start.x = 0.0; p_start.y = 0.0; p_start.z = 0.0;
            // 终点沿着计算出的航向角延伸 (假设箭头画 2 米长，方便观看)
            p_end.x = 2.0 * std::cos(target_steering_angle);
            p_end.y = 2.0 * std::sin(target_steering_angle);
            p_end.z = 0.0;

            arrow.points.push_back(p_start);
            arrow.points.push_back(p_end);

            // 箭头的尺寸设置 (轴宽, 箭头宽, 箭头长)
            arrow.scale.x = 0.05; 
            arrow.scale.y = 0.15; 
            arrow.scale.z = 0.1;

            // 箭头的颜色 (RGBA 格式，这里设为亮绿色)
            arrow.color.r = 0.0;
            arrow.color.g = 1.0;
            arrow.color.b = 0.0;
            arrow.color.a = 1.0; // Alpha必须为1，否则是完全透明的！

            // 存活时间，设为 0.2 秒后自动消失 (防止残影)
            arrow.lifetime = ros::Duration(0.2); 

            // 发布这个箭头！
            marker_pub_.publish(arrow);
            // ==========================================

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