#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <string>
#include <vector>

/*
 * tree_laserscan_sim_node
 * -----------------------
 * 这是一个“树冠二维激光雷达仿真节点”，不依赖 Gazebo。
 *
 * 它的作用是：
 *   1. 订阅 /Odometry，知道机器人当前在 odom 坐标系中的位置和朝向。
 *   2. 把树冠二维投影简化成一个圆。
 *   3. 从机器人雷达位置向四周发射二维射线。
 *   4. 计算每条射线是否打到树冠圆。
 *   5. 生成 sensor_msgs/LaserScan，发布为 /scan_2d。
 *   6. 发布绿色树冠圆盘和绿色雷达命中点，方便 RViz 可视化。
 *
 * 为什么用圆表示树冠？
 *   你的控制代码目前只需要二维 LaserScan 中的树冠边界点。
 *   用圆形树冠投影可以先验证“识别树冠 -> 拟合切线 -> 绕树运动”的闭环逻辑。
 *   后续如果圆形跑通，可以再把圆改成椭圆、不规则边界、缺口边界或带噪声边界。
 */
class TreeLaserScanSim {
public:
    TreeLaserScanSim() : pnh_("~"), rng_(std::random_device{}()) {
        loadParameters();

        // 订阅底盘仿真节点发布的里程计。
        odom_sub_ = nh_.subscribe(odom_topic_, 10, &TreeLaserScanSim::odomCallback, this);

        // 发布模拟出来的二维激光雷达数据。控制节点会订阅这个话题。
        scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>(scan_topic_, 10);

        // 发布 RViz 可视化 Marker。
        canopy_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/sim_canopy_marker", 1);
        canopy_points_pub_ = nh_.advertise<visualization_msgs::Marker>("/sim_canopy_points", 1);

        ROS_INFO("tree_laserscan_sim started: tree=(%.2f, %.2f), canopy_radius=%.2f",
                 tree_x_, tree_y_, canopy_radius_);
    }

    void spin() {
        ros::Rate loop(rate_);
        while (ros::ok()) {
            ros::spinOnce();
            publishScanAndMarkers();
            loop.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber odom_sub_;
    ros::Publisher scan_pub_;
    ros::Publisher canopy_marker_pub_;
    ros::Publisher canopy_points_pub_;

    // 话题和坐标系名称。laser_frame 要和控制节点里的 laser_frame 参数一致。
    std::string odom_topic_;
    std::string scan_topic_;
    std::string odom_frame_;
    std::string laser_frame_;

    // 机器人当前位姿。这里从 /Odometry 更新。
    double robot_x_ = 1.6;
    double robot_y_ = 0.0;
    double robot_yaw_ = -90.0 * M_PI / 180.0;
    bool has_odom_ = false;

    // 树冠圆参数：圆心在 odom 坐标系下，半径表示树冠二维投影半径。
    double tree_x_ = 0.0;
    double tree_y_ = 0.0;
    double canopy_radius_ = 0.65;

    // LaserScan 参数。
    double range_min_ = 0.2;
    double range_max_ = 3.5;
    double angle_min_deg_ = -180.0;
    double angle_max_deg_ = 180.0;
    double angle_increment_deg_ = 0.5;
    double angle_min_ = -M_PI;
    double angle_max_ = M_PI;
    double angle_increment_ = 0.5 * M_PI / 180.0;
    double laser_yaw_offset_deg_ = -90.0;
    double laser_yaw_offset_rad_ = -M_PI / 2.0;
    double noise_stddev_ = 0.01;
    double rate_ = 15.0;

    // 用于给模拟测距添加高斯噪声。
    std::mt19937 rng_;
    std::normal_distribution<double> noise_dist_;

    void loadParameters() {
        pnh_.param("odom_topic", odom_topic_, std::string("/Odometry"));
        pnh_.param("scan_topic", scan_topic_, std::string("/scan_2d"));
        pnh_.param("odom_frame", odom_frame_, std::string("odom"));
        pnh_.param("laser_frame", laser_frame_, std::string("livox_frame"));
        pnh_.param("rate", rate_, 15.0);

        pnh_.param("tree_x", tree_x_, 0.0);
        pnh_.param("tree_y", tree_y_, 0.0);
        pnh_.param("canopy_radius", canopy_radius_, 0.65);
        pnh_.param("range_min", range_min_, 0.2);
        pnh_.param("range_max", range_max_, 3.5);
        pnh_.param("angle_min_deg", angle_min_deg_, -180.0);
        pnh_.param("angle_max_deg", angle_max_deg_, 180.0);
        pnh_.param("angle_increment_deg", angle_increment_deg_, 0.5);
        pnh_.param("laser_yaw_offset_deg", laser_yaw_offset_deg_, -90.0);
        pnh_.param("noise_stddev", noise_stddev_, 0.01);

        angle_min_ = angle_min_deg_ * M_PI / 180.0;
        angle_max_ = angle_max_deg_ * M_PI / 180.0;
        angle_increment_ = std::max(0.05 * M_PI / 180.0, angle_increment_deg_ * M_PI / 180.0);
        laser_yaw_offset_rad_ = laser_yaw_offset_deg_ * M_PI / 180.0;
        canopy_radius_ = std::max(0.05, canopy_radius_);
        range_max_ = std::max(range_min_ + 0.1, range_max_);

        noise_dist_ = std::normal_distribution<double>(0.0, std::max(0.0, noise_stddev_));
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        robot_x_ = msg->pose.pose.position.x;
        robot_y_ = msg->pose.pose.position.y;

        tf::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        );
        tf::Matrix3x3 m(q);
        double roll = 0.0;
        double pitch = 0.0;
        m.getRPY(roll, pitch, robot_yaw_);
        has_odom_ = true;
    }

    void publishScanAndMarkers() {
        if (!has_odom_) {
            ROS_WARN_THROTTLE(1.0, "tree_laserscan_sim waiting for odometry");
            return;
        }

        const ros::Time stamp = ros::Time::now();

        sensor_msgs::LaserScan scan;
        scan.header.stamp = stamp;
        scan.header.frame_id = laser_frame_;
        scan.angle_min = angle_min_;
        scan.angle_max = angle_max_;
        scan.angle_increment = angle_increment_;
        scan.time_increment = 0.0;
        scan.scan_time = 1.0 / std::max(1.0, rate_);
        scan.range_min = range_min_;
        scan.range_max = range_max_;

        const int beam_count = static_cast<int>(std::floor((angle_max_ - angle_min_) / angle_increment_)) + 1;
        scan.ranges.assign(beam_count, std::numeric_limits<float>::infinity());

        visualization_msgs::Marker hit_points;
        makeHitPointsMarker(stamp, hit_points);

        /*
         * 对每一束激光做“射线与圆的相交计算”。
         *
         * angle_base:
         *   雷达坐标系下的射线角度，0 度表示车头前方。
         *
         * angle_world:
         *   odom 世界坐标系下的射线角度。
         *
         * 如果射线打中树冠圆，就把最近交点距离写入 scan.ranges[i]。
         * 如果没有打中，就保持 infinity，表示这束激光没有障碍物。
         */
        for (int i = 0; i < beam_count; ++i) {
            const double angle_base = angle_min_ + static_cast<double>(i) * angle_increment_;
            /*
             * angle_base 是雷达坐标系下的角度。
             * robot_yaw_ 是车体 base_link 在 odom 下的航向角。
             * laser_yaw_offset_rad_ 是雷达相对车体的安装角。
             *
             * 因此射线在 odom 世界坐标系下的方向为：
             *   车体航向 + 雷达安装角 + 雷达束角
             */
            const double angle_world = robot_yaw_ + laser_yaw_offset_rad_ + angle_base;
            const double dx = std::cos(angle_world);
            const double dy = std::sin(angle_world);

            double hit_range = 0.0;
            if (!rayCircleIntersection(robot_x_, robot_y_, dx, dy, hit_range)) {
                continue;
            }

            hit_range += noise_dist_(rng_);
            if (hit_range < range_min_ || hit_range > range_max_) {
                continue;
            }

            scan.ranges[i] = static_cast<float>(hit_range);

            geometry_msgs::Point p;
            p.x = hit_range * std::cos(angle_base);
            p.y = hit_range * std::sin(angle_base);
            p.z = 0.0;
            hit_points.points.push_back(p);
        }

        scan_pub_.publish(scan);
        canopy_points_pub_.publish(hit_points);
        publishCanopyMarker(stamp);
    }

    bool rayCircleIntersection(double ox, double oy, double dx, double dy, double& hit_range) const {
        /*
         * 射线方程：
         *   P(t) = O + t * D, t > 0
         *
         * 圆方程：
         *   (x - tree_x)^2 + (y - tree_y)^2 = canopy_radius^2
         *
         * 把射线方程代入圆方程，会得到关于 t 的二次方程。
         * 若判别式 < 0，说明射线没有打到圆。
         * 若有两个交点，取 t 更小且大于 0 的那个，即离雷达最近的命中点。
         */
        const double fx = ox - tree_x_;
        const double fy = oy - tree_y_;

        const double b = 2.0 * (fx * dx + fy * dy);
        const double c = fx * fx + fy * fy - canopy_radius_ * canopy_radius_;
        const double discriminant = b * b - 4.0 * c;
        if (discriminant < 0.0) {
            return false;
        }

        const double sqrt_disc = std::sqrt(discriminant);
        const double t1 = (-b - sqrt_disc) / 2.0;
        const double t2 = (-b + sqrt_disc) / 2.0;

        const double eps = 1e-6;
        if (t1 > eps) {
            hit_range = t1;
            return true;
        }
        if (t2 > eps) {
            hit_range = t2;
            return true;
        }
        return false;
    }

    void makeHitPointsMarker(const ros::Time& stamp, visualization_msgs::Marker& marker) const {
        marker.header.stamp = stamp;
        marker.header.frame_id = laser_frame_;
        marker.ns = "tree_laserscan_sim";
        marker.id = 1;
        marker.type = visualization_msgs::Marker::POINTS;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.035;
        marker.scale.y = 0.035;
        marker.color.r = 0.0;
        marker.color.g = 0.8;
        marker.color.b = 0.1;
        marker.color.a = 1.0;
        marker.lifetime = ros::Duration(0.2);
    }

    void publishCanopyMarker(const ros::Time& stamp) {
        visualization_msgs::Marker marker;
        marker.header.stamp = stamp;
        marker.header.frame_id = odom_frame_;
        marker.ns = "tree_laserscan_sim";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;

        // RViz 中显示成一个很矮的绿色圆柱，用它代表树冠的二维投影。
        marker.pose.position.x = tree_x_;
        marker.pose.position.y = tree_y_;
        marker.pose.position.z = 0.02;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 2.0 * canopy_radius_;
        marker.scale.y = 2.0 * canopy_radius_;
        marker.scale.z = 0.04;
        marker.color.r = 0.0;
        marker.color.g = 0.55;
        marker.color.b = 0.1;
        marker.color.a = 0.35;
        marker.lifetime = ros::Duration(0.2);
        canopy_marker_pub_.publish(marker);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "tree_laserscan_sim_node");
    TreeLaserScanSim sim;
    sim.spin();
    return 0;
}
