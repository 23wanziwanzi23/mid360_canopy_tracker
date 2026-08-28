#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>

#include <algorithm>
#include <cmath>
#include <string>

/*
 * simple_chassis_sim_node
 * -----------------------
 * 这是一个“轻量二维底盘仿真节点”，不依赖 Gazebo。
 *
 * 它的作用是模拟一个履带式/差速式底盘：
 *   1. 订阅 /cmd_vel，接收控制节点发来的线速度和角速度。
 *   2. 用简单运动学模型积分机器人位姿。
 *   3. 发布 /Odometry，让控制节点以为自己正在读真实里程计。
 *   4. 发布 odom -> base_link 和 base_link -> livox_frame 的 TF。
 *   5. 发布一个蓝色箭头 Marker，在 RViz 中显示机器人位置和朝向。
 *
 * 注意：
 *   - 这里不模拟履带打滑、惯性、地面摩擦，只验证上层控制逻辑。
 *   - ROS 中 angular.z > 0 表示左转，angular.z < 0 表示右转。
 */
class SimpleChassisSim {
public:
    SimpleChassisSim() : pnh_("~") {
        loadParameters();

        // 订阅控制节点发布的速度指令。你的 odom_listener_5_dipan.cpp 会发布这个话题。
        cmd_sub_ = nh_.subscribe(cmd_vel_topic_, 10, &SimpleChassisSim::cmdCallback, this);

        // 发布模拟里程计。控制节点会订阅这个话题。
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>(odom_topic_, 10);

        // 发布 RViz 中用于显示小车的 Marker。
        robot_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/sim_robot_marker", 1);

        last_time_ = ros::Time::now();
        last_cmd_time_ = ros::Time(0);

        ROS_INFO("simple_chassis_sim started: x=%.2f y=%.2f yaw=%.1f deg",
                 x_, y_, init_yaw_deg_);
    }

    void spin() {
        ros::Rate loop(rate_);
        while (ros::ok()) {
            ros::spinOnce();
            update();
            loop.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber cmd_sub_;
    ros::Publisher odom_pub_;
    ros::Publisher robot_marker_pub_;
    tf::TransformBroadcaster tf_broadcaster_;

    // 话题和坐标系名称。默认值与 odom_listener_5_dipan.cpp 对齐。
    std::string odom_topic_;
    std::string cmd_vel_topic_;
    std::string odom_frame_;
    std::string base_frame_;
    std::string laser_frame_;

    // 机器人二维位姿，单位分别是 m、m、rad。
    double x_ = 0.0;
    double y_ = 0.0;
    double yaw_ = 0.0;

    // 仿真运行参数。
    double init_yaw_deg_ = 0.0;
    double laser_yaw_offset_deg_ = -90.0;
    double laser_yaw_offset_rad_ = -M_PI / 2.0;
    double rate_ = 50.0;
    double cmd_timeout_ = 0.5;
    double max_linear_speed_ = 0.4;
    double max_angular_speed_ = 0.8;

    // 保存最近一次收到的 /cmd_vel。
    geometry_msgs::Twist latest_cmd_;
    ros::Time last_cmd_time_;
    ros::Time last_time_;

    void loadParameters() {
        pnh_.param("odom_topic", odom_topic_, std::string("/Odometry"));
        pnh_.param("cmd_vel_topic", cmd_vel_topic_, std::string("/cmd_vel"));
        pnh_.param("odom_frame", odom_frame_, std::string("odom"));
        pnh_.param("base_frame", base_frame_, std::string("base_link"));
        pnh_.param("laser_frame", laser_frame_, std::string("livox_frame"));
        pnh_.param("rate", rate_, 50.0);
        pnh_.param("cmd_timeout", cmd_timeout_, 0.5);
        pnh_.param("max_linear_speed", max_linear_speed_, 0.4);
        pnh_.param("max_angular_speed", max_angular_speed_, 0.8);

        /*
         * 初始位姿很重要。
         *
         * 默认树冠中心在 (0, 0)，机器人从 (1.6, 0) 附近出发。
         * init_yaw_deg = -90 表示车头沿树冠切线方向朝下。
         * laser_yaw_offset_deg = -90 表示雷达朝车体右侧看，此时雷达正前方对着树冠。
         */
        pnh_.param("init_x", x_, 1.6);
        pnh_.param("init_y", y_, 0.0);
        pnh_.param("init_yaw_deg", init_yaw_deg_, -90.0);
        pnh_.param("laser_yaw_offset_deg", laser_yaw_offset_deg_, -90.0);
        yaw_ = init_yaw_deg_ * M_PI / 180.0;
        laser_yaw_offset_rad_ = laser_yaw_offset_deg_ * M_PI / 180.0;
    }

    static double clamp(double value, double low, double high) {
        return std::max(low, std::min(value, high));
    }

    static double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    void cmdCallback(const geometry_msgs::Twist::ConstPtr& msg) {
        /*
         * 真实底盘一般会有限速，这里也做一个限幅。
         * 这样即使控制节点参数设置过大，仿真小车也不会瞬间飞出去。
         */
        latest_cmd_ = *msg;
        latest_cmd_.linear.x = clamp(latest_cmd_.linear.x, -max_linear_speed_, max_linear_speed_);
        latest_cmd_.angular.z = clamp(latest_cmd_.angular.z, -max_angular_speed_, max_angular_speed_);
        last_cmd_time_ = ros::Time::now();
    }

    void update() {
        const ros::Time now = ros::Time::now();
        const double dt = std::max(0.0, (now - last_time_).toSec());
        last_time_ = now;

        double v = 0.0;
        double w = 0.0;

        /*
         * 如果超过 cmd_timeout 秒没有收到新的 /cmd_vel，就自动停车。
         * 这是仿真实车安全逻辑：控制节点异常退出时，底盘不应继续沿旧指令运动。
         */
        if (!last_cmd_time_.isZero() && (now - last_cmd_time_).toSec() <= cmd_timeout_) {
            v = latest_cmd_.linear.x;
            w = latest_cmd_.angular.z;
        }

        /*
         * 差速/履带底盘的简化二维运动学模型：
         *   x_dot   = v * cos(yaw)
         *   y_dot   = v * sin(yaw)
         *   yaw_dot = w
         *
         * 这里的 v 和 w 就来自 /cmd_vel。
         */
        x_ += v * std::cos(yaw_) * dt;
        y_ += v * std::sin(yaw_) * dt;
        yaw_ = normalizeAngle(yaw_ + w * dt);

        publishOdom(now, v, w);
        publishRobotMarker(now);
    }

    void publishOdom(const ros::Time& stamp, double v, double w) {
        const geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(yaw_);

        // 发布 odom -> base_link，方便 RViz 和激光仿真节点使用统一坐标关系。
        geometry_msgs::TransformStamped base_tf;
        base_tf.header.stamp = stamp;
        base_tf.header.frame_id = odom_frame_;
        base_tf.child_frame_id = base_frame_;
        base_tf.transform.translation.x = x_;
        base_tf.transform.translation.y = y_;
        base_tf.transform.translation.z = 0.0;
        base_tf.transform.rotation = q;
        tf_broadcaster_.sendTransform(base_tf);

        /*
         * 雷达坐标系相对车体坐标系的安装角。
         *
         * 你的实际安装方式是：
         *   - 履带底盘侧面对着树冠；
         *   - Mid-360 雷达正面对着树冠。
         *
         * 在 ROS base_link 中，x 轴通常表示车头前方，y 轴左侧为正。
         * 如果树在车体右侧，雷达正面要朝右，那么 livox_frame 相对 base_link
         * 需要绕 z 轴旋转 -90 度。
         */
        geometry_msgs::TransformStamped laser_tf;
        laser_tf.header.stamp = stamp;
        laser_tf.header.frame_id = base_frame_;
        laser_tf.child_frame_id = laser_frame_;
        laser_tf.transform.translation.x = 0.0;
        laser_tf.transform.translation.y = 0.0;
        laser_tf.transform.translation.z = 0.0;
        laser_tf.transform.rotation = tf::createQuaternionMsgFromYaw(laser_yaw_offset_rad_);
        tf_broadcaster_.sendTransform(laser_tf);

        nav_msgs::Odometry odom;
        odom.header.stamp = stamp;
        odom.header.frame_id = odom_frame_;
        odom.child_frame_id = base_frame_;
        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = q;
        odom.twist.twist.linear.x = v;
        odom.twist.twist.angular.z = w;
        odom_pub_.publish(odom);
    }

    void publishRobotMarker(const ros::Time& stamp) {
        visualization_msgs::Marker marker;
        marker.header.stamp = stamp;
        marker.header.frame_id = odom_frame_;
        marker.ns = "simple_chassis_sim";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;

        // 蓝色箭头的位置就是机器人当前位置，箭头方向就是机器人车头方向。
        marker.pose.position.x = x_;
        marker.pose.position.y = y_;
        marker.pose.position.z = 0.05;
        marker.pose.orientation = tf::createQuaternionMsgFromYaw(yaw_);

        // scale.x 是箭头长度，scale.y/scale.z 控制箭头粗细。
        marker.scale.x = 0.6;
        marker.scale.y = 0.28;
        marker.scale.z = 0.18;
        marker.color.r = 0.1;
        marker.color.g = 0.45;
        marker.color.b = 1.0;
        marker.color.a = 1.0;
        marker.lifetime = ros::Duration(0.2);
        robot_marker_pub_.publish(marker);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "simple_chassis_sim_node");
    SimpleChassisSim sim;
    sim.spin();
    return 0;
}
