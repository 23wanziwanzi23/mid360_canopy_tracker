#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <string>
#include <vector>

/*
 * tree_laserscan_sim_node_v2
 * --------------------------
 * 高级二维树冠 LaserScan 仿真节点。
 *
 * 和简单版 tree_laserscan_sim_node.cpp 的区别：
 *   1. 树冠不再只能是标准圆，可以是椭圆。
 *   2. 树冠边界可以有随机起伏，用来模拟真实树叶边界不平整。
 *   3. 树冠可以设置缺口，用来模拟雷达只看到部分树冠。
 *   4. 可以添加相邻树，检查控制节点会不会误跟踪旁边的树。
 *   5. 可以添加离群点，模拟杂草、枝叶飞点、果园杂物等干扰。
 *
 * 实现方式：
 *   这个节点不是做严格的射线与几何体碰撞，而是先生成树冠边界点云，
 *   再把这些点投影到 LaserScan 的角度 bin 里。
 *
 * 这种方式非常适合算法压力测试：
 *   - 你可以直接控制边界点的形状、缺口、噪声和干扰；
 *   - 生成速度快；
 *   - RViz 里也能直观看到控制节点到底在追踪哪些点。
 */

struct CanopyConfig {
    double x = 0.0;
    double y = 0.0;
    double radius_x = 0.70;
    double radius_y = 0.55;
    double yaw = 0.0;
    double waviness_amplitude = 0.04;
    int waviness_frequency = 5;
    double radial_noise_stddev = 0.01;
    bool enable_gap = false;
    double gap_center = 0.0;
    double gap_width = 0.0;
    bool enabled = true;
};

class TreeLaserScanSimV2 {
public:
    TreeLaserScanSimV2() : pnh_("~"), rng_(std::random_device{}()) {
        loadParameters();

        odom_sub_ = nh_.subscribe(odom_topic_, 10, &TreeLaserScanSimV2::odomCallback, this);
        scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>(scan_topic_, 10);
        canopy_markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/sim_canopy_markers", 1);
        hit_points_pub_ = nh_.advertise<visualization_msgs::Marker>("/sim_canopy_points", 1);
        outlier_points_pub_ = nh_.advertise<visualization_msgs::Marker>("/sim_outlier_points", 1);

        ROS_INFO("tree_laserscan_sim_v2 started: target=(%.2f, %.2f), neighbor_enabled=%s",
                 target_canopy_.x, target_canopy_.y, neighbor_canopy_.enabled ? "true" : "false");
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
    ros::Publisher canopy_markers_pub_;
    ros::Publisher hit_points_pub_;
    ros::Publisher outlier_points_pub_;

    std::string odom_topic_;
    std::string scan_topic_;
    std::string odom_frame_;
    std::string laser_frame_;

    double robot_x_ = 1.6;
    double robot_y_ = 0.0;
    double robot_yaw_ = -90.0 * M_PI / 180.0;
    bool has_odom_ = false;

    double rate_ = 15.0;
    double range_min_ = 0.2;
    double range_max_ = 4.0;
    double angle_min_ = -M_PI;
    double angle_max_ = M_PI;
    double angle_increment_ = 0.5 * M_PI / 180.0;
    double angle_min_deg_ = -180.0;
    double angle_max_deg_ = 180.0;
    double angle_increment_deg_ = 0.5;
    double laser_yaw_offset_deg_ = -90.0;
    double laser_yaw_offset_rad_ = -M_PI / 2.0;

    int boundary_point_count_ = 720;
    int outlier_count_ = 0;
    double outlier_min_range_ = 0.4;
    double outlier_max_range_ = 2.5;
    double outlier_angle_min_ = -60.0 * M_PI / 180.0;
    double outlier_angle_max_ = 60.0 * M_PI / 180.0;

    CanopyConfig target_canopy_;
    CanopyConfig neighbor_canopy_;

    std::mt19937 rng_;
    std::normal_distribution<double> unit_noise_{0.0, 1.0};

    void loadParameters() {
        pnh_.param("odom_topic", odom_topic_, std::string("/Odometry"));
        pnh_.param("scan_topic", scan_topic_, std::string("/scan_2d"));
        pnh_.param("odom_frame", odom_frame_, std::string("odom"));
        pnh_.param("laser_frame", laser_frame_, std::string("livox_frame"));
        pnh_.param("rate", rate_, 15.0);

        pnh_.param("range_min", range_min_, 0.2);
        pnh_.param("range_max", range_max_, 4.0);
        pnh_.param("angle_min_deg", angle_min_deg_, -180.0);
        pnh_.param("angle_max_deg", angle_max_deg_, 180.0);
        pnh_.param("angle_increment_deg", angle_increment_deg_, 0.5);
        pnh_.param("laser_yaw_offset_deg", laser_yaw_offset_deg_, -90.0);

        pnh_.param("boundary_point_count", boundary_point_count_, 720);
        pnh_.param("outlier_count", outlier_count_, 0);
        pnh_.param("outlier_min_range", outlier_min_range_, 0.4);
        pnh_.param("outlier_max_range", outlier_max_range_, 2.5);

        double outlier_angle_min_deg = -60.0;
        double outlier_angle_max_deg = 60.0;
        pnh_.param("outlier_angle_min_deg", outlier_angle_min_deg, outlier_angle_min_deg);
        pnh_.param("outlier_angle_max_deg", outlier_angle_max_deg, outlier_angle_max_deg);
        outlier_angle_min_ = outlier_angle_min_deg * M_PI / 180.0;
        outlier_angle_max_ = outlier_angle_max_deg * M_PI / 180.0;

        loadCanopyParameters("target", target_canopy_, true);
        loadCanopyParameters("neighbor", neighbor_canopy_, false);

        angle_min_ = angle_min_deg_ * M_PI / 180.0;
        angle_max_ = angle_max_deg_ * M_PI / 180.0;
        angle_increment_ = std::max(0.05 * M_PI / 180.0, angle_increment_deg_ * M_PI / 180.0);
        laser_yaw_offset_rad_ = laser_yaw_offset_deg_ * M_PI / 180.0;
        boundary_point_count_ = std::max(36, boundary_point_count_);
        range_max_ = std::max(range_min_ + 0.1, range_max_);
        outlier_count_ = std::max(0, outlier_count_);
        outlier_max_range_ = std::max(outlier_min_range_ + 0.1, outlier_max_range_);
    }

    void loadCanopyParameters(const std::string& prefix, CanopyConfig& cfg, bool default_enabled) {
        pnh_.param(prefix + "_enabled", cfg.enabled, default_enabled);
        pnh_.param(prefix + "_x", cfg.x, cfg.x);
        pnh_.param(prefix + "_y", cfg.y, cfg.y);
        pnh_.param(prefix + "_radius_x", cfg.radius_x, cfg.radius_x);
        pnh_.param(prefix + "_radius_y", cfg.radius_y, cfg.radius_y);

        double yaw_deg = cfg.yaw * 180.0 / M_PI;
        pnh_.param(prefix + "_yaw_deg", yaw_deg, yaw_deg);
        cfg.yaw = yaw_deg * M_PI / 180.0;

        pnh_.param(prefix + "_waviness_amplitude", cfg.waviness_amplitude, cfg.waviness_amplitude);
        pnh_.param(prefix + "_waviness_frequency", cfg.waviness_frequency, cfg.waviness_frequency);
        pnh_.param(prefix + "_radial_noise_stddev", cfg.radial_noise_stddev, cfg.radial_noise_stddev);
        pnh_.param(prefix + "_enable_gap", cfg.enable_gap, cfg.enable_gap);

        double gap_center_deg = cfg.gap_center * 180.0 / M_PI;
        double gap_width_deg = cfg.gap_width * 180.0 / M_PI;
        pnh_.param(prefix + "_gap_center_deg", gap_center_deg, gap_center_deg);
        pnh_.param(prefix + "_gap_width_deg", gap_width_deg, gap_width_deg);
        cfg.gap_center = gap_center_deg * M_PI / 180.0;
        cfg.gap_width = gap_width_deg * M_PI / 180.0;

        cfg.radius_x = std::max(0.05, cfg.radius_x);
        cfg.radius_y = std::max(0.05, cfg.radius_y);
        cfg.waviness_amplitude = std::max(0.0, cfg.waviness_amplitude);
        cfg.waviness_frequency = std::max(0, cfg.waviness_frequency);
        cfg.radial_noise_stddev = std::max(0.0, cfg.radial_noise_stddev);
        cfg.gap_width = std::max(0.0, std::min(2.0 * M_PI, cfg.gap_width));
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
            ROS_WARN_THROTTLE(1.0, "tree_laserscan_sim_v2 waiting for odometry");
            return;
        }

        const ros::Time stamp = ros::Time::now();

        sensor_msgs::LaserScan scan;
        fillScanHeader(stamp, scan);

        visualization_msgs::Marker hit_points;
        makePointsMarker(stamp, laser_frame_, "advanced_canopy_hits", 0, 0.0, 0.85, 0.1, hit_points);

        visualization_msgs::Marker outlier_points;
        makePointsMarker(stamp, laser_frame_, "advanced_outliers", 0, 1.0, 0.25, 0.0, outlier_points);

        visualization_msgs::MarkerArray canopy_markers;

        projectCanopy(target_canopy_, 0, scan, hit_points, canopy_markers, stamp);
        projectCanopy(neighbor_canopy_, 1, scan, hit_points, canopy_markers, stamp);
        addOutliers(scan, outlier_points);

        scan_pub_.publish(scan);
        hit_points_pub_.publish(hit_points);
        outlier_points_pub_.publish(outlier_points);
        canopy_markers_pub_.publish(canopy_markers);
    }

    void fillScanHeader(const ros::Time& stamp, sensor_msgs::LaserScan& scan) const {
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
    }

    void projectCanopy(const CanopyConfig& cfg,
                       int marker_id,
                       sensor_msgs::LaserScan& scan,
                       visualization_msgs::Marker& hit_points,
                       visualization_msgs::MarkerArray& markers,
                       const ros::Time& stamp) {
        if (!cfg.enabled) return;

        visualization_msgs::Marker boundary_marker;
        makeBoundaryMarker(stamp, cfg, marker_id, boundary_marker);

        for (int i = 0; i < boundary_point_count_; ++i) {
            const double theta = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(boundary_point_count_);
            if (isInGap(theta, cfg)) {
                continue;
            }

            geometry_msgs::Point point_odom;
            sampleCanopyBoundaryPoint(cfg, theta, point_odom);
            boundary_marker.points.push_back(point_odom);

            geometry_msgs::Point point_laser;
            if (!transformOdomPointToLaser(point_odom, point_laser)) {
                continue;
            }

            const double range = std::hypot(point_laser.x, point_laser.y);
            const double angle = std::atan2(point_laser.y, point_laser.x);
            if (range < range_min_ || range > range_max_ || angle < angle_min_ || angle > angle_max_) {
                continue;
            }

            const int beam_index = static_cast<int>(std::round((angle - angle_min_) / angle_increment_));
            if (beam_index < 0 || beam_index >= static_cast<int>(scan.ranges.size())) {
                continue;
            }

            if (!std::isfinite(scan.ranges[beam_index]) || range < scan.ranges[beam_index]) {
                scan.ranges[beam_index] = static_cast<float>(range);
            }

            hit_points.points.push_back(point_laser);
        }

        // LINE_STRIP 要闭合，所以把第一个点再追加一次。
        if (!boundary_marker.points.empty()) {
            boundary_marker.points.push_back(boundary_marker.points.front());
        }
        markers.markers.push_back(boundary_marker);
    }

    bool isInGap(double theta, const CanopyConfig& cfg) const {
        if (!cfg.enable_gap || cfg.gap_width <= 0.0) return false;

        const double diff = normalizeAngle(theta - cfg.gap_center);
        return std::fabs(diff) < cfg.gap_width / 2.0;
    }

    void sampleCanopyBoundaryPoint(const CanopyConfig& cfg, double theta, geometry_msgs::Point& point) {
        /*
         * 先生成一个椭圆边界点：
         *   local_x = radius_x * cos(theta)
         *   local_y = radius_y * sin(theta)
         *
         * 再叠加两个扰动：
         *   1. waviness：确定性的正弦起伏，保证每次运行形状大体一致；
         *   2. radial_noise：随机径向噪声，模拟树叶边界抖动。
         */
        const double wave = cfg.waviness_amplitude * std::sin(cfg.waviness_frequency * theta);
        const double noise = cfg.radial_noise_stddev * unit_noise_(rng_);
        const double scale = std::max(0.2, 1.0 + wave + noise);

        const double local_x = cfg.radius_x * scale * std::cos(theta);
        const double local_y = cfg.radius_y * scale * std::sin(theta);

        const double cos_yaw = std::cos(cfg.yaw);
        const double sin_yaw = std::sin(cfg.yaw);
        point.x = cfg.x + cos_yaw * local_x - sin_yaw * local_y;
        point.y = cfg.y + sin_yaw * local_x + cos_yaw * local_y;
        point.z = 0.0;
    }

    bool transformOdomPointToLaser(const geometry_msgs::Point& point_odom, geometry_msgs::Point& point_laser) const {
        const double dx = point_odom.x - robot_x_;
        const double dy = point_odom.y - robot_y_;
        const double laser_yaw_odom = robot_yaw_ + laser_yaw_offset_rad_;

        // odom -> laser 是绕 z 轴反向旋转 laser_yaw_odom。
        const double cos_yaw = std::cos(laser_yaw_odom);
        const double sin_yaw = std::sin(laser_yaw_odom);
        point_laser.x = cos_yaw * dx + sin_yaw * dy;
        point_laser.y = -sin_yaw * dx + cos_yaw * dy;
        point_laser.z = 0.0;
        return true;
    }

    void addOutliers(sensor_msgs::LaserScan& scan, visualization_msgs::Marker& outlier_points) {
        if (outlier_count_ <= 0) return;

        std::uniform_real_distribution<double> angle_dist(outlier_angle_min_, outlier_angle_max_);
        std::uniform_real_distribution<double> range_dist(outlier_min_range_, outlier_max_range_);

        for (int i = 0; i < outlier_count_; ++i) {
            const double angle = angle_dist(rng_);
            const double range = range_dist(rng_);
            if (angle < angle_min_ || angle > angle_max_) {
                continue;
            }

            const int beam_index = static_cast<int>(std::round((angle - angle_min_) / angle_increment_));
            if (beam_index < 0 || beam_index >= static_cast<int>(scan.ranges.size())) {
                continue;
            }

            if (!std::isfinite(scan.ranges[beam_index]) || range < scan.ranges[beam_index]) {
                scan.ranges[beam_index] = static_cast<float>(range);
            }

            geometry_msgs::Point p;
            p.x = range * std::cos(angle);
            p.y = range * std::sin(angle);
            p.z = 0.0;
            outlier_points.points.push_back(p);
        }
    }

    void makeBoundaryMarker(const ros::Time& stamp,
                            const CanopyConfig& cfg,
                            int id,
                            visualization_msgs::Marker& marker) const {
        marker.header.stamp = stamp;
        marker.header.frame_id = odom_frame_;
        marker.ns = "advanced_canopy_boundary";
        marker.id = id;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.035;
        marker.pose.orientation.w = 1.0;
        marker.color.a = cfg.enabled ? 1.0 : 0.0;

        if (id == 0) {
            marker.color.r = 0.0;
            marker.color.g = 0.75;
            marker.color.b = 0.1;
        } else {
            marker.color.r = 0.95;
            marker.color.g = 0.75;
            marker.color.b = 0.1;
        }

        marker.lifetime = ros::Duration(0.3);
    }

    void makePointsMarker(const ros::Time& stamp,
                          const std::string& frame_id,
                          const std::string& ns,
                          int id,
                          double r,
                          double g,
                          double b,
                          visualization_msgs::Marker& marker) const {
        marker.header.stamp = stamp;
        marker.header.frame_id = frame_id;
        marker.ns = ns;
        marker.id = id;
        marker.type = visualization_msgs::Marker::POINTS;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.035;
        marker.scale.y = 0.035;
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 1.0;
        marker.lifetime = ros::Duration(0.3);
    }

    static double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "tree_laserscan_sim_node_v2");
    TreeLaserScanSimV2 sim;
    sim.spin();
    return 0;
}


