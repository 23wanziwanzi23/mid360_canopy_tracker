#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

struct Point2D {
    double x;
    double y;
};

class CanopyTracker {
private:
    /**
     * @brief 运动状态枚举类,状态机
     *
     * 用于表示目标相对于树冠（canopy）的运动模式。
     */
    enum class MotionState {
        WORK_PAUSE,
        ALIGN_TANGENT,
        MOVE_AROUND_CANOPY,
        FACE_CANOPY,
        STOPPED
    };

    // ===================== ROS 通信接口 =====================
    ros::NodeHandle nh_;                ///< 公共节点句柄，用于订阅/发布全局命名空间的话题
    ros::NodeHandle pnh_;               ///< 私有节点句柄（"~"），用于访问本节点的私有参数和话题
    ros::Subscriber odom_sub_;          ///< 里程计数据订阅者
    ros::Subscriber scan_sub_;          ///< 激光扫描数据订阅者
    ros::Publisher  marker_pub_;        ///< RViz 可视化标记发布者
    ros::Publisher  cmd_vel_pub_;       ///< 速度指令（geometry_msgs/Twist）发布者

    // ===================== 机器人位姿状态 =====================
    double current_x_ = 0.0;            ///< 机器人当前世界坐标系 x 坐标（来自里程计）
    double current_y_ = 0.0;            ///< 机器人当前世界坐标系 y 坐标
    double current_yaw_ = 0.0;          ///< 机器人当前偏航角（弧度）
    bool   has_odom_ = false;           ///< 是否已成功接收到里程计数据

    // ===================== 移动片段记录 =====================
    double segment_start_x_ = 0.0;      ///< 当前移动片段的起始 x 坐标
    double segment_start_y_ = 0.0;      ///< 当前移动片段的起始 y 坐标
    bool   segment_start_set_ = false;  ///< 移动片段起点是否已设定

    // ===================== 树木位置估计 =====================
    double estimated_tree_x_ = 0.0;     ///< 估计的树干/树冠中心 x 坐标
    double estimated_tree_y_ = 0.0;     ///< 估计的树干/树冠中心 y 坐标
    bool   has_tree_estimate_ = false;  ///< 是否已获得有效的树木位置估计

    // ===================== 绕树运动控制 =====================
    double last_orbit_angle_ = 0.0;         ///< 上一控制周期的绕树角度（弧度），用于计算增量
    double accumulated_orbit_angle_ = 0.0;  ///< 累计绕树旋转角度（弧度），判断是否完成一圈

    // ===================== 总体状态与时间 =====================
    MotionState state_ = MotionState::WORK_PAUSE;
    ros::Time state_enter_time_;            ///< 最后一次接收到有效激光扫描数据的时间戳
    ros::Time last_valid_scan_time_;

   // ===================== 运动控制参数 =====================
    double target_distance_ = 0.8;                      ///< 期望与树冠保持的距离 (m)
    double step_distance_ = 0.5;                        ///< 沿树冠切线移动的步长 (m)
    double linear_speed_ = 0.12;                        ///< 直线移动时的线速度 (m/s)
    double k_dist_ = 0.5;                               ///< 距离误差的比例增益（用于线速度调整）
    double k_turn_move_ = 1.2;                          ///< 绕树移动时转向的比例增益
    double k_turn_align_ = 1.5;                         ///< 朝向树冠对齐时的转向比例增益
    double max_angular_speed_ = 0.5;                    ///< 最大允许角速度 (rad/s)
    double align_tolerance_rad_ = 5.0 * M_PI / 180.0;  ///< 朝向对齐容差，5° 转弧度 (~0.0873 rad)
    double scan_timeout_ = 0.5;                         ///< 激光扫描数据超时判定时间 (s)

    // ===================== 滤波与约束参数 =====================
    double tree_filter_alpha_ = 0.15;                   ///< 树木位置估计的低通滤波系数 (0~1)，越小越平滑
    double min_orbit_delta_rad_ = 0.2 * M_PI / 180.0;  ///< 累计绕树角度的最小增量阈值 (~0.0035 rad)，避免微小扰动
    bool   clockwise_ = true;                           ///< 绕树运动方向：true 为顺时针，false 为逆时针

    // ===================== 话题与坐标系配置 =====================
    std::string cmd_vel_topic_ = "/cmd_vel";            ///< 速度指令发布话题名称
    std::string odom_topic_ = "/Odometry";              ///< 里程计数据订阅话题名称
    std::string scan_topic_ = "/scan_2d";               ///< 激光扫描数据订阅话题名称
    std::string laser_frame_ = "livox_frame";           ///< 激光雷达坐标系名称，用于坐标变换

    // ===================== 切线方向滑动窗口滤波 =====================
    std::vector<double> tangent_history_;   ///< 历史切线方向角度队列（先进先出），用于滑动窗口平滑
    size_t filter_window_size_ = 5;         ///< 滑动窗口大小，即保留最近 N 次切线方向测量值
    bool   is_window_active_ = false;       ///< 窗口滤波器是否激活（当队列长度达到窗口大小时通常置为 true）

public:
    /**
     * @brief 构造函数，初始化节点并建立所有 ROS 通信接口
     *
     * 使用私有命名空间初始化句柄，从参数服务器加载配置，
     * 订阅里程计与激光扫描话题，发布速度指令与可视化标记。
     */
    CanopyTracker() : pnh_("~") {
        loadParameters();

        odom_sub_ = nh_.subscribe(odom_topic_, 10, &CanopyTracker::odomCallback, this);
        scan_sub_ = nh_.subscribe(scan_topic_, 10, &CanopyTracker::scanCallback, this);
        marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/steering_vector", 10);
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 10);
        state_enter_time_ = ros::Time::now();

        // ROS_INFO("canopy_tracker started. cmd_vel_topic=%s step_distance=%.2fm target_distance=%.2fm work_pause=%.2fs direction=%s",
        //          cmd_vel_topic_.c_str(), step_distance_, target_distance_, work_pause_duration_,
        //          clockwise_ ? "clockwise" : "counterclockwise");
        ROS_INFO("树冠跟踪器已启动。速度话题=%s 步长=%.2fm 目标距离=%.2fm 作业停顿时间=%.2fs 绕行方向=%s",
                 cmd_vel_topic_.c_str(), step_distance_, target_distance_, work_pause_duration_,
                 clockwise_ ? "顺时针" : "逆时针");
    }

    ~CanopyTracker() {
        publishStop();
    }

private:
    void loadParameters() {
        pnh_.param("target_distance", target_distance_, target_distance_);
        pnh_.param("step_distance", step_distance_, step_distance_);
        pnh_.param("linear_speed", linear_speed_, linear_speed_);
        pnh_.param("k_dist", k_dist_, k_dist_);
        pnh_.param("k_turn_move", k_turn_move_, k_turn_move_);
        pnh_.param("k_turn_align", k_turn_align_, k_turn_align_);
        pnh_.param("k_bearing", k_bearing_, k_bearing_);
        pnh_.param("max_angular_speed", max_angular_speed_, max_angular_speed_);
        pnh_.param("align_tolerance_deg", align_tolerance_rad_, align_tolerance_rad_ * 180.0 / M_PI);
        align_tolerance_rad_ *= M_PI / 180.0;
        pnh_.param("scan_timeout", scan_timeout_, scan_timeout_);
        pnh_.param("work_pause_duration", work_pause_duration_, work_pause_duration_);
        pnh_.param("tree_filter_alpha", tree_filter_alpha_, tree_filter_alpha_);
        pnh_.param("min_orbit_delta_deg", min_orbit_delta_rad_, min_orbit_delta_rad_ * 180.0 / M_PI);
        min_orbit_delta_rad_ *= M_PI / 180.0;
        pnh_.param("roi_angle_limit_deg", roi_angle_limit_rad_, roi_angle_limit_rad_ * 180.0 / M_PI);
        roi_angle_limit_rad_ *= M_PI / 180.0;
        pnh_.param("canopy_radius_for_orbit", canopy_radius_for_orbit_, canopy_radius_for_orbit_);
        pnh_.param("laser_yaw_offset_deg", laser_yaw_offset_rad_, laser_yaw_offset_rad_ * 180.0 / M_PI);
        laser_yaw_offset_rad_ *= M_PI / 180.0;
        pnh_.param("clockwise", clockwise_, clockwise_);
        pnh_.param("cmd_vel_topic", cmd_vel_topic_, cmd_vel_topic_);
        pnh_.param("odom_topic", odom_topic_, odom_topic_);
        pnh_.param("scan_topic", scan_topic_, scan_topic_);
        pnh_.param("laser_frame", laser_frame_, laser_frame_);

        int filter_window = static_cast<int>(filter_window_size_);
        pnh_.param("filter_window_size", filter_window, filter_window);
        filter_window_size_ = static_cast<size_t>(std::max(1, filter_window));

        step_distance_ = std::max(0.05, step_distance_);
        linear_speed_ = std::max(0.0, linear_speed_);
        max_angular_speed_ = std::max(0.05, max_angular_speed_);
        work_pause_duration_ = std::max(0.0, work_pause_duration_);
        tree_filter_alpha_ = clamp(tree_filter_alpha_, 0.01, 1.0);
        min_orbit_delta_rad_ = clamp(min_orbit_delta_rad_, 0.0, 5.0 * M_PI / 180.0);
        roi_angle_limit_rad_ = clamp(roi_angle_limit_rad_, 20.0 * M_PI / 180.0, M_PI);
        canopy_radius_for_orbit_ = std::max(0.0, canopy_radius_for_orbit_);
    }

    static double clamp(double value, double low, double high) {
        return std::max(low, std::min(value, high));
    }

    static double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    double calculateCanopyTangent(const std::vector<Point2D>& points, double& avg_distance, double& bearing) {
        if (points.size() < 5) return 0.0;

        double sum_x = 0.0;
        double sum_y = 0.0;
        double sum_dist = 0.0;
        const int n = static_cast<int>(points.size());

        for (const auto& p : points) {
            sum_x += p.x;
            sum_y += p.y;
            sum_dist += std::sqrt(p.x * p.x + p.y * p.y);
        }

        const double mean_x = sum_x / n;
        const double mean_y = sum_y / n;
        avg_distance = sum_dist / n;
        bearing = std::atan2(mean_y, mean_x);

        double cov_xx = 0.0;
        double cov_yy = 0.0;
        double cov_xy = 0.0;
        for (const auto& p : points) {
            const double dx = p.x - mean_x;
            const double dy = p.y - mean_y;
            cov_xx += dx * dx;
            cov_yy += dy * dy;
            cov_xy += dx * dy;
        }

        return 0.5 * std::atan2(2.0 * cov_xy, cov_xx - cov_yy);
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;

        tf::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        );
        tf::Matrix3x3 m(q);
        double roll = 0.0;
        double pitch = 0.0;
        m.getRPY(roll, pitch, current_yaw_);

        if (!has_odom_) {
            segment_start_x_ = current_x_;
            segment_start_y_ = current_y_;
            segment_start_set_ = true;
            has_odom_ = true;
        }
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        if (!has_odom_) {
            publishStop();
            ROS_WARN_THROTTLE(1.0, "waiting for odometry before sending chassis commands");
            return;
        }

        std::vector<Point2D> roi_points;
        extractRoiPoints(msg, roi_points);

        if (roi_points.size() < 5) {
            if ((ros::Time::now() - last_valid_scan_time_).toSec() > scan_timeout_) {
                publishStop();
                ROS_WARN_THROTTLE(1.0, "no valid canopy points, stopping chassis");
            }
            return;
        }

        last_valid_scan_time_ = ros::Time::now();

        double current_tree_distance = 0.0;
        double canopy_bearing = 0.0;
        double tangent_angle = calculateCanopyTangent(roi_points, current_tree_distance, canopy_bearing);
        tangent_angle = filterTangentAngle(tangent_angle);

        updateOrbitProgress();
        updateTreeEstimate(current_tree_distance, canopy_bearing);

        if (std::fabs(accumulated_orbit_angle_) >= 2.0 * M_PI) {
            state_ = MotionState::STOPPED;
        }

        if (state_ == MotionState::STOPPED) {
            publishStop();
            // ROS_INFO_THROTTLE(1.0, "orbit complete, chassis stopped. accumulated_orbit=%.1f deg",
            //                   accumulated_orbit_angle_ * 180.0 / M_PI);
            ROS_INFO_THROTTLE(1.0, "绕行完成，底盘已停止。累计绕行角度=%.1f 度",
                              accumulated_orbit_angle_ * 180.0 / M_PI);
            publishMarker(0.0);
            return;
        }

        if (state_ == MotionState::WORK_PAUSE) {
            controlWorkPause();
            return;
        }

        if (state_ == MotionState::ALIGN_TANGENT) {
            controlAlignTangent(canopy_bearing);
            return;
        }

        if (!segment_start_set_) {
            resetSegmentStart();
        }

        const double segment_distance = std::hypot(current_x_ - segment_start_x_, current_y_ - segment_start_y_);

        if (state_ == MotionState::MOVE_AROUND_CANOPY && segment_distance >= step_distance_) {
            state_ = MotionState::FACE_CANOPY;
            state_enter_time_ = ros::Time::now();
            publishStop();
            // ROS_INFO("step finished: %.2fm, turning to face canopy", segment_distance);
            ROS_INFO("步长移动完成：%.2fm，正在转向面朝树冠", segment_distance);
            return;
        }

        if (state_ == MotionState::FACE_CANOPY) {
            controlFaceCanopy(canopy_bearing);
            return;
        }

        controlMoveAroundCanopy(tangent_angle, canopy_bearing, current_tree_distance);
    }

    void extractRoiPoints(const sensor_msgs::LaserScan::ConstPtr& msg, std::vector<Point2D>& roi_points) const {
        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            const double r = msg->ranges[i];
            if (std::isinf(r) || std::isnan(r) || r < 0.2 || r > 2.5) continue;

            const double angle = msg->angle_min + static_cast<double>(i) * msg->angle_increment;

            /*
             * 你的实际安装方式是“雷达正对树冠，底盘侧面对树冠”。
             * 因此树冠点应该出现在雷达正前方，而不是车体右前方。
             *
             * 这里取雷达正前方一个对称扇区：
             *   -roi_angle_limit_rad_ ~ +roi_angle_limit_rad_
             *
             * launch 中默认 roi_angle_limit_deg=120，仿真初期范围稍大一些，
             * 方便先确认状态机和运动方向。实车调试时可以逐渐收窄。
             */
            if (std::fabs(angle) < roi_angle_limit_rad_) {
                roi_points.push_back({r * std::cos(angle), r * std::sin(angle)});
            }
        }
    }

    double filterTangentAngle(double tangent_angle) {
        double tangent_deg = tangent_angle * 180.0 / M_PI;
        const double lower_limit = clockwise_ ? -80.0 : 20.0;
        const double upper_limit = clockwise_ ? -20.0 : 80.0;
        const double buffer = 4.0;

        const bool in_hard_window = clockwise_
            ? (tangent_deg <= upper_limit && tangent_deg >= lower_limit)
            : (tangent_deg >= lower_limit && tangent_deg <= upper_limit);

        if (!in_hard_window) {
            tangent_angle = 0.0;
            tangent_deg = 0.0;
        }

        tangent_history_.push_back(tangent_angle);
        if (tangent_history_.size() > filter_window_size_) {
            tangent_history_.erase(tangent_history_.begin());
        }

        double sum = 0.0;
        for (double val : tangent_history_) {
            sum += val;
        }
        tangent_angle = sum / tangent_history_.size();
        tangent_deg = tangent_angle * 180.0 / M_PI;

        if (is_window_active_) {
            const bool out_relaxed_window = clockwise_
                ? (tangent_deg > upper_limit + buffer || tangent_deg < lower_limit - buffer)
                : (tangent_deg < lower_limit - buffer || tangent_deg > upper_limit + buffer);
            if (out_relaxed_window) {
                is_window_active_ = false;
            }
        } else {
            const bool enter_strict_window = clockwise_
                ? (tangent_deg <= upper_limit - buffer && tangent_deg >= lower_limit + buffer)
                : (tangent_deg >= lower_limit + buffer && tangent_deg <= upper_limit - buffer);
            if (enter_strict_window) {
                is_window_active_ = true;
            }
        }

        return is_window_active_ ? tangent_angle : 0.0;
    }

    void updateTreeEstimate(double distance, double bearing) {
        /*
         * distance 是雷达到“树冠边界”的距离，不是雷达到树心的距离。
         * 在当前圆形树冠仿真中，树心大约在边界点后方 canopy_radius_for_orbit_ 处。
         *
         * 如果后续真实树冠半径未知，可以先把 canopy_radius_for_orbit 设小一些，
         * 或者改成由点云聚类/圆拟合估计树心。
         */
        const double center_distance = distance + canopy_radius_for_orbit_;
        const double tree_x_base = center_distance * std::cos(bearing);
        const double tree_y_base = center_distance * std::sin(bearing);

        /*
         * tree_x_base/tree_y_base 其实是雷达 livox_frame 下的坐标。
         * 转到 odom 时，必须使用：
         *   车体航向 current_yaw_ + 雷达相对车体安装角 laser_yaw_offset_rad_
         *
         * 默认 laser_yaw_offset_deg=-90：
         *   车体右侧是雷达正前方，符合“底盘侧面对树冠，雷达正对树冠”。
         */
        const double laser_yaw_odom = current_yaw_ + laser_yaw_offset_rad_;
        const double tree_x_odom = current_x_ + std::cos(laser_yaw_odom) * tree_x_base - std::sin(laser_yaw_odom) * tree_y_base;
        const double tree_y_odom = current_y_ + std::sin(laser_yaw_odom) * tree_x_base + std::cos(laser_yaw_odom) * tree_y_base;

        if (!has_tree_estimate_) {
            estimated_tree_x_ = tree_x_odom;
            estimated_tree_y_ = tree_y_odom;
            last_orbit_angle_ = std::atan2(current_y_ - estimated_tree_y_, current_x_ - estimated_tree_x_);
            accumulated_orbit_angle_ = 0.0;
            has_tree_estimate_ = true;
            return;
        }

        estimated_tree_x_ = (1.0 - tree_filter_alpha_) * estimated_tree_x_ + tree_filter_alpha_ * tree_x_odom;
        estimated_tree_y_ = (1.0 - tree_filter_alpha_) * estimated_tree_y_ + tree_filter_alpha_ * tree_y_odom;
    }

    void updateOrbitProgress() {
        if (!has_tree_estimate_) return;

        const double orbit_angle = std::atan2(current_y_ - estimated_tree_y_, current_x_ - estimated_tree_x_);
        double delta = normalizeAngle(orbit_angle - last_orbit_angle_);
        if (std::fabs(delta) < min_orbit_delta_rad_) {
            return;
        }

        if (clockwise_) {
            if (delta > 0.0) delta = 0.0;
        } else {
            if (delta < 0.0) delta = 0.0;
        }

        accumulated_orbit_angle_ += delta;
        last_orbit_angle_ = orbit_angle;
    }

    void controlMoveAroundCanopy(double tangent_angle, double canopy_bearing, double current_tree_distance) {
        /*
         * 绕树移动控制。
         *
         * 这里不再直接把 PCA 拟合出来的 tangent_angle 当作唯一控制量。
         * 原因是 PCA 拟合直线只有“方向轴”，没有明确的前后方向，存在 180 度二义性；
         * 而且在 FACE_CANOPY 后车头正对树冠，如果立刻前进，会先向树冠方向冲。
         *
         * 更稳定的做法：
         *   - 顺时针绕树：希望树冠一直位于机器人右侧，即 bearing 约为 -90 度。
         *   - 逆时针绕树：希望树冠一直位于机器人左侧，即 bearing 约为 +90 度。
         *
         * canopy_bearing 是树冠相对车头的角度：
         *   0 度表示树在正前方；
         *   正角度表示树在左侧；
         *   负角度表示树在右侧。
         */
        const double desired_bearing = 0.0;
        const double bearing_error = normalizeAngle(canopy_bearing - desired_bearing);

        const double distance_error = current_tree_distance - target_distance_;
        const double correction_angle = clockwise_ ? -k_dist_ * distance_error : k_dist_ * distance_error;
        const double target_steering_angle = normalizeAngle(k_bearing_ * bearing_error + correction_angle);

        geometry_msgs::Twist cmd;
        cmd.linear.x = linear_speed_;
        cmd.angular.z = clamp(k_turn_move_ * target_steering_angle, -max_angular_speed_, max_angular_speed_);
        cmd_vel_pub_.publish(cmd);

        publishMarker(target_steering_angle);

        // ROS_INFO_THROTTLE(0.5,
        //                   "MOVE step=%.2f/%.2fm tree_dist=%.2fm bearing_laser=%.1f deg desired=%.1f deg tangent=%.1f deg steer=%.1f deg cmd(v=%.2f,w=%.2f) orbit=%.1f deg",
        //                   std::hypot(current_x_ - segment_start_x_, current_y_ - segment_start_y_),
        //                   step_distance_,
        //                   current_tree_distance,
        //                   canopy_bearing * 180.0 / M_PI,
        //                   desired_bearing * 180.0 / M_PI,
        //                   tangent_angle * 180.0 / M_PI,
        //                   target_steering_angle * 180.0 / M_PI,
        //                   cmd.linear.x,
        //                   cmd.angular.z,
        //                   accumulated_orbit_angle_ * 180.0 / M_PI);
        ROS_INFO_THROTTLE(0.5,
                          "移动 步进=%.2f/%.2fm 树距=%.2fm 雷达方位角=%.1f 度 期望角=%.1f 度 切线角=%.1f 度 转向角=%.1f 度 控制指令(v=%.2f,w=%.2f) 累计绕行=%.1f 度",
                          std::hypot(current_x_ - segment_start_x_, current_y_ - segment_start_y_),
                          step_distance_,
                          current_tree_distance,
                          canopy_bearing * 180.0 / M_PI,
                          desired_bearing * 180.0 / M_PI,
                          tangent_angle * 180.0 / M_PI,
                          target_steering_angle * 180.0 / M_PI,
                          cmd.linear.x,
                          cmd.angular.z,
                          accumulated_orbit_angle_ * 180.0 / M_PI);
    }

    void controlWorkPause() {
        /*
         * WORK_PAUSE 用来模拟“采收/作业停顿”。
         *
         * 在这个状态中：
         *   1. 持续发布零速度，保证仿真小车停住。
         *   2. 等待 work_pause_duration_ 秒。
         *   3. 时间到后记录新一段移动的起点，并切换到 MOVE_AROUND_CANOPY。
         *
         * 这样你在 RViz 中能明显看到：
         *   停 1 秒 -> 移动一段 -> 转向正对树冠 -> 停 1 秒 -> 继续移动。
         */
        publishStop();

        const double elapsed = (ros::Time::now() - state_enter_time_).toSec();
        // ROS_INFO_THROTTLE(0.5, "WORK_PAUSE %.2f/%.2fs, chassis stopped for simulated harvesting",
        //                   elapsed, work_pause_duration_);
        ROS_INFO_THROTTLE(0.5, "作业停顿 %.2f/%.2fs，底盘已停止以进行模拟采摘",
                          elapsed, work_pause_duration_);

        if (elapsed >= work_pause_duration_) {
            state_ = MotionState::ALIGN_TANGENT;
            state_enter_time_ = ros::Time::now();
            // ROS_INFO("work pause finished, aligning to tangent direction");
            ROS_INFO("作业停顿结束，正在对齐切线方向");
        }
    }

    void controlAlignTangent(double canopy_bearing) {
        /*
         * 切线对准状态。
         *
         * WORK_PAUSE 结束时，机器人通常是正对树冠的。
         * 如果此时直接 linear.x > 0，机器人会朝树冠前进，不会沿树冠绕行。
         *
         * 因此移动前先原地转向到“绕树切线方向”：
         *   - 顺时针：树应该在车体右侧，目标 bearing = -90 度。
         *   - 逆时针：树应该在车体左侧，目标 bearing = +90 度。
         */
        const double desired_bearing = 0.0;
        const double yaw_error = normalizeAngle(canopy_bearing - desired_bearing);

        if (std::fabs(yaw_error) <= align_tolerance_rad_) {
            publishStop();
            resetSegmentStart();
            state_ = MotionState::MOVE_AROUND_CANOPY;
            state_enter_time_ = ros::Time::now();
            // ROS_INFO("tangent aligned, starting move segment");
            ROS_INFO("切线方向已对齐，开始移动段");
            return;
        }

        geometry_msgs::Twist cmd;
        cmd.angular.z = clamp(k_turn_align_ * yaw_error, -max_angular_speed_, max_angular_speed_);
        cmd_vel_pub_.publish(cmd);

        publishMarker(yaw_error);
        // ROS_INFO_THROTTLE(0.5, "ALIGN_TANGENT bearing=%.1f deg desired=%.1f deg cmd(w=%.2f)",
        //                   canopy_bearing * 180.0 / M_PI,
        //                   desired_bearing * 180.0 / M_PI,
        //                   cmd.angular.z);
        ROS_INFO_THROTTLE(0.5, "对齐切线 方位角=%.1f 度 期望角=%.1f 度 控制指令(w=%.2f)",
                          canopy_bearing * 180.0 / M_PI,
                          desired_bearing * 180.0 / M_PI,
                          cmd.angular.z);
    }

    void controlFaceCanopy(double canopy_bearing) {
        const double yaw_error = normalizeAngle(canopy_bearing);

        if (std::fabs(yaw_error) <= align_tolerance_rad_) {
            publishStop();
            state_ = MotionState::WORK_PAUSE;
            state_enter_time_ = ros::Time::now();
            // ROS_INFO("canopy aligned, entering work pause");
            ROS_INFO("已对齐树冠，进入作业停顿");
            return;
        }

        geometry_msgs::Twist cmd;
        cmd.angular.z = clamp(k_turn_align_ * yaw_error, -max_angular_speed_, max_angular_speed_);
        cmd_vel_pub_.publish(cmd);

        publishMarker(canopy_bearing);
        // ROS_INFO_THROTTLE(0.5, "FACE yaw_error=%.1f deg cmd(w=%.2f)",
        //                   yaw_error * 180.0 / M_PI, cmd.angular.z);
        ROS_INFO_THROTTLE(0.5, "朝向调整 航向误差=%.1f 度 控制指令(w=%.2f)",
                          yaw_error * 180.0 / M_PI, cmd.angular.z);
    }

    void resetSegmentStart() {
        segment_start_x_ = current_x_;
        segment_start_y_ = current_y_;
        segment_start_set_ = true;
    }

    void publishStop() {
        geometry_msgs::Twist cmd;
        cmd_vel_pub_.publish(cmd);
    }

    void publishMarker(double heading_angle) {
        visualization_msgs::Marker arrow;
        arrow.header.frame_id = laser_frame_;
        arrow.header.stamp = ros::Time::now();
        arrow.ns = "canopy_tracker";
        arrow.id = 0;
        arrow.type = visualization_msgs::Marker::ARROW;
        arrow.action = visualization_msgs::Marker::ADD;

        geometry_msgs::Point p_start;
        geometry_msgs::Point p_end;
        p_start.x = 0.0;
        p_start.y = 0.0;
        p_start.z = 0.0;
        p_end.x = 2.0 * std::cos(heading_angle);
        p_end.y = 2.0 * std::sin(heading_angle);
        p_end.z = 0.0;

        arrow.points.push_back(p_start);
        arrow.points.push_back(p_end);
        arrow.scale.x = 0.05;
        arrow.scale.y = 0.15;
        arrow.scale.z = 0.1;
        arrow.color.r = 0.0;
        arrow.color.g = 1.0;
        arrow.color.b = 0.0;
        arrow.color.a = 1.0;
        arrow.lifetime = ros::Duration(0.2);

        marker_pub_.publish(arrow);
    }
};

int main(int argc, char** argv) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "canopy_tracker_node");

    CanopyTracker tracker;
    ros::spin();
    return 0;
}