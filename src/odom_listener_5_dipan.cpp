#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/String.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>

#include <algorithm>
#include <clocale>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

/**
 * @brief 二维点结构体。
 *
 * 坐标系：
 *   - 默认表示雷达坐标系 livox_frame 下的点。
 *   - x 轴：雷达正前方。
 *   - y 轴：雷达左侧为正，右侧为负。
 */
struct Point2D {
    double x;  ///< 点的 x 坐标，单位：m。
    double y;  ///< 点的 y 坐标，单位：m。
};

/**
 * @brief PCA 直线拟合结果。
 */
struct LineFitResult {
    bool valid = false;        ///< 是否拟合成功。点数不足时为 false。
    double line_angle = 0.0;   ///< 拟合直线方向角，单位：rad，坐标系：雷达坐标系。
    double avg_distance = 0.0; ///< ROI 点云到雷达的平均距离，单位：m。
    double surface_distance = 0.0; ///< ROI 较近点的稳健距离，表示雷达到树冠表面的距离，单位：m。
    double bearing = 0.0;      ///< ROI 点云质心相对雷达正前方的方位角，单位：rad。
    double centroid_x = 0.0;   ///< ROI 点云质心 x 坐标，单位：m，坐标系：雷达坐标系。
    double centroid_y = 0.0;   ///< ROI 点云质心 y 坐标，单位：m，坐标系：雷达坐标系。
    size_t point_count = 0;    ///< 参与拟合的点数量。
};

/**
 * @brief 树冠跟踪与底盘控制节点。
 *
 * 输入话题：
 *   - /Odometry：机器人里程计，提供当前位置和车体航向角。
 *   - /scan_2d：二维雷达扫描，提供树冠边界点。
 *
 * 输出话题：
 *   - /cmd_vel：底盘速度控制指令。
 *   - /steering_vector：RViz 中显示目标航向的箭头。
 *   - /heading_roi_points：RViz 中显示右侧航向 ROI 点。
 *   - /facing_roi_points：RViz 中显示前方正对判断 ROI 点。
 *   - /heading_roi_region：RViz 中用蓝色半透明扇形显示航向检测有效范围。
 *   - /facing_roi_region：RViz 中用橙色半透明扇形显示正对检测有效范围。
 *   - /heading_fit_line：RViz 中显示航向 ROI 的 PCA 拟合线。
 *   - /facing_fit_line：RViz 中显示正对判断 ROI 的 PCA 拟合线。
 *   - /canopy_tracker_state：发布当前状态机名称，消息类型 std_msgs/String。
 *
 * 当前点云处理逻辑：
 *   1. 航向 ROI 的点云拟合局部树冠切线，作为正常移动时的主要航向依据。
 *   2. 正对 ROI 同时测量小车右侧的树冠表面距离，移动中持续闭环到目标距离。
 *   3. 固定树心只用于统计绕行进度，并在航向点云短时丢失时临时兜底。
 */
class CanopyTracker {
private:
    /**
     * @brief 底盘运动状态机。
     */
    enum class MotionState {
        WORK_PAUSE,          ///< 停车等待，模拟采收/作业停顿。
        ALIGN_TANGENT,       ///< 作业后先对齐切线方向，准备移动。
        MOVE_AROUND_CANOPY,  ///< 沿树冠边界移动一小段距离。
        FACE_CANOPY,         ///< 移动结束后调整姿态，让机器正对树冠。
        STOPPED              ///< 绕树一圈完成，停止运动。
    };

    ros::NodeHandle nh_;   ///< 全局 NodeHandle，用于普通话题订阅/发布。
    ros::NodeHandle pnh_;  ///< 私有 NodeHandle，用于读取 ~private 参数。

    ros::Subscriber odom_sub_; ///< 里程计订阅器。
    ros::Subscriber scan_sub_; ///< 二维雷达订阅器。

    ros::Publisher cmd_vel_pub_;            ///< /cmd_vel 发布器。
    ros::Publisher steering_marker_pub_;    ///< /steering_vector 发布器。
    ros::Publisher heading_roi_marker_pub_; ///< /heading_roi_points 发布器。
    ros::Publisher facing_roi_marker_pub_;  ///< /facing_roi_points 发布器。
    ros::Publisher heading_roi_region_pub_; ///< /heading_roi_region 航向有效范围发布器。
    ros::Publisher facing_roi_region_pub_;  ///< /facing_roi_region 正对有效范围发布器。
    ros::Publisher heading_fit_marker_pub_; ///< /heading_fit_line 发布器。
    ros::Publisher facing_fit_marker_pub_;  ///< /facing_fit_line 发布器。
    ros::Publisher state_pub_;              ///< /canopy_tracker_state 状态名称发布器。

    double current_x_ = 0.0;   ///< 当前机器人 x 坐标，单位：m，坐标系：odom。
    double current_y_ = 0.0;   ///< 当前机器人 y 坐标，单位：m，坐标系：odom。
    double current_yaw_ = 0.0; ///< 当前机器人航向角 yaw，单位：rad，坐标系：odom。
    bool has_odom_ = false;    ///< 是否已经收到过里程计。

    double orbit_start_x_ = 0.0;   ///< 绕行起点 x，取第一帧有效里程计位置，单位：m。
    double orbit_start_y_ = 0.0;   ///< 绕行起点 y，取第一帧有效里程计位置，单位：m。
    bool orbit_start_set_ = false; ///< 是否已经记录绕行起点。

    double segment_start_x_ = 0.0;   ///< 当前移动小段的起点 x，单位：m。
    double segment_start_y_ = 0.0;   ///< 当前移动小段的起点 y，单位：m。
    bool segment_start_set_ = false; ///< 当前移动小段起点是否有效。

    double estimated_tree_x_ = 0.0;        ///< 估计树心 x，单位：m，坐标系：odom。
    double estimated_tree_y_ = 0.0;        ///< 估计树心 y，单位：m，坐标系：odom。
    bool has_tree_estimate_ = false;       ///< 是否已经获得树心估计。
    double last_orbit_angle_ = 0.0;        ///< 上一次机器人相对树心的极角，单位：rad。
    double accumulated_orbit_angle_ = 0.0; ///< 累计绕树角度，单位：rad。

    MotionState state_ = MotionState::WORK_PAUSE; ///< 当前运动状态。
    ros::Time state_enter_time_;                  ///< 进入当前状态的时间。
    ros::Time last_valid_scan_time_;              ///< 最近一次检测到有效树冠点的时间。
    ros::Time last_valid_heading_time_;           ///< 最近一次航向 ROI 拟合有效的时间。

    double target_distance_ = 0.3;     ///< 绕行时小车右侧到树冠表面的目标距离，单位：m。
    double distance_tolerance_ = 0.05; ///< 分段结束时允许的右侧距离误差，单位：m。
    double step_distance_ = 0.5;       ///< 每次移动的小段距离，单位：m。
    double linear_speed_ = 0.12;       ///< 移动时线速度，单位：m/s。
    double k_dist_ = 0.5;              ///< 距离误差修正比例系数。
    double k_turn_move_ = 1.2;         ///< 移动状态下目标航向角到角速度的比例系数。
    double k_orbit_heading_ = 0.8;     ///< 几何切向航向误差到角速度的比例系数。
    double k_bearing_move_ = 0.8;      ///< 移动时树冠质心方位误差到角速度的比例系数。
    double k_distance_move_ = 0.8;     ///< 移动时右侧树冠距离误差到角速度的比例系数。
    double k_turn_align_ = 1.5;        ///< 原地对准状态下角度误差到角速度的比例系数。
    double k_orbit_feedforward_ = 1.0; ///< 绕树前馈角速度系数，0 表示关闭前馈。
    double max_angular_speed_ = 0.5;   ///< 最大角速度限幅，单位：rad/s。
    double facing_line_weight_ = 0.6;  ///< 正对误差中 PCA 线角误差的权重，范围 0~1。
    double facing_bearing_weight_ = 0.4; ///< 正对误差中质心方位误差的权重，范围 0~1。

    double align_tolerance_rad_ = 5.0 * M_PI / 180.0;     ///< 对准允许误差，单位：rad。
    double scan_timeout_ = 0.5;                           ///< 雷达目标丢失超时时间，单位：s。
    double work_pause_duration_ = 1.0;                    ///< 每个作业点停车等待时间，单位：s。
    double tree_filter_alpha_ = 0.15;                     ///< 树心低通滤波系数，范围 0~1。
    double max_orbit_delta_rad_ = 15.0 * M_PI / 180.0;    ///< 单帧允许的最大绕行极角变化，超出视为里程计跳变。
    double return_distance_threshold_ = 0.5;              ///< 完成一圈时允许的起点直线距离，单位：m。
    double heading_fallback_timeout_ = 0.5;               ///< 航向拟合失效后允许几何切线兜底的时间，单位：s。
    double max_heading_change_rad_ = 5.0 * M_PI / 180.0; ///< 航向拟合单帧最大变化，单位：rad。
    double canopy_radius_for_orbit_ = 0.65;               ///< 用于树心估计的树冠半径近似值，单位：m。
    double laser_yaw_offset_rad_ = -M_PI / 2.0;           ///< 雷达相对 base_link 的安装角，单位：rad。
    double laser_x_offset_ = 0.0;                         ///< 雷达相对 base_link 的前向位置，单位：m。
    double laser_y_offset_ = 0.0;                         ///< 雷达相对 base_link 的左向位置，单位：m。
    bool clockwise_ = true;                               ///< 是否顺时针绕树。true 为顺时针，false 为逆时针。
    bool lock_tree_estimate_ = true;                      ///< 首次估计树心后是否锁定，防止树心漂移造成假绕行。

    double heading_roi_min_rad_ = -120.0 * M_PI / 180.0; ///< 航向 ROI 最小角，单位：rad。
    double heading_roi_max_rad_ = -20.0 * M_PI / 180.0; ///< 航向 ROI 最大角，单位：rad。
    double facing_roi_min_rad_ = -30.0 * M_PI / 180.0;  ///< 正对判断 ROI 最小角，单位：rad。
    double facing_roi_max_rad_ = 30.0 * M_PI / 180.0;   ///< 正对判断 ROI 最大角，单位：rad。

    std::string cmd_vel_topic_ = "/cmd_vel";  ///< 底盘速度控制话题名。
    std::string odom_topic_ = "/Odometry";    ///< 里程计话题名。
    std::string scan_topic_ = "/scan_2d";     ///< 二维雷达话题名。
    std::string laser_frame_ = "livox_frame"; ///< 雷达坐标系名称。

    std::vector<double> heading_history_; ///< 航向角滑动平均缓存。
    size_t filter_window_size_ = 5;       ///< 航向角滑动平均窗口大小。

public:
    /**
     * @brief 构造函数。
     *
     * 作用：
     *   - 读取 ROS 参数；
     *   - 初始化订阅器；
     *   - 初始化发布器；
     *   - 进入初始 WORK_PAUSE 状态。
     */
    CanopyTracker() : pnh_("~") {
        loadParameters();

        odom_sub_ = nh_.subscribe(odom_topic_, 10, &CanopyTracker::odomCallback, this);
        scan_sub_ = nh_.subscribe(scan_topic_, 10, &CanopyTracker::scanCallback, this);

        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 10);
        steering_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/steering_vector", 10);
        heading_roi_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/heading_roi_points", 10);
        facing_roi_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/facing_roi_points", 10);
        heading_roi_region_pub_ = nh_.advertise<visualization_msgs::Marker>("/heading_roi_region", 10);
        facing_roi_region_pub_ = nh_.advertise<visualization_msgs::Marker>("/facing_roi_region", 10);
        heading_fit_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/heading_fit_line", 10);
        facing_fit_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/facing_fit_line", 10);
        // latch=true：即使调试工具稍后连接，也能立即看到最近一次状态。
        state_pub_ = nh_.advertise<std_msgs::String>("/canopy_tracker_state", 1, true);

        state_enter_time_ = ros::Time::now();
        publishState();

        ROS_INFO("树冠跟踪控制节点已启动：单段距离=%.2f 米，目标树冠距离=%.2f 米，作业停顿=%.2f 秒，绕行方向=%s",
                 step_distance_, target_distance_, work_pause_duration_,
                 clockwise_ ? "顺时针" : "逆时针");
    }

    /**
     * @brief 析构函数。
     *
     * 作用：
     *   节点退出时发布一次零速度，降低底盘继续运动的风险。
     */
    ~CanopyTracker() {
        publishStop();
    }

private:
    /**
     * @brief 从 ROS 参数服务器读取控制参数。
     *
     * 参数来源：
     *   rosrun/roslaunch 中以私有参数形式传入，例如 _step_distance:=0.3。
     *
     * @return 无。
     */
    void loadParameters() {
        pnh_.param("target_distance", target_distance_, target_distance_);
        pnh_.param("distance_tolerance", distance_tolerance_, distance_tolerance_);
        pnh_.param("step_distance", step_distance_, step_distance_);
        pnh_.param("linear_speed", linear_speed_, linear_speed_);
        pnh_.param("k_dist", k_dist_, k_dist_);
        pnh_.param("k_turn_move", k_turn_move_, k_turn_move_);
        pnh_.param("k_orbit_heading", k_orbit_heading_, k_orbit_heading_);
        pnh_.param("k_bearing_move", k_bearing_move_, k_bearing_move_);
        pnh_.param("k_distance_move", k_distance_move_, k_distance_move_);
        pnh_.param("k_turn_align", k_turn_align_, k_turn_align_);
        pnh_.param("k_orbit_feedforward", k_orbit_feedforward_, k_orbit_feedforward_);
        pnh_.param("max_angular_speed", max_angular_speed_, max_angular_speed_);
        pnh_.param("facing_line_weight", facing_line_weight_, facing_line_weight_);
        pnh_.param("facing_bearing_weight", facing_bearing_weight_, facing_bearing_weight_);
        pnh_.param("scan_timeout", scan_timeout_, scan_timeout_);
        pnh_.param("work_pause_duration", work_pause_duration_, work_pause_duration_);
        pnh_.param("tree_filter_alpha", tree_filter_alpha_, tree_filter_alpha_);
        pnh_.param("return_distance_threshold", return_distance_threshold_, return_distance_threshold_);
        pnh_.param("heading_fallback_timeout", heading_fallback_timeout_, heading_fallback_timeout_);
        pnh_.param("canopy_radius_for_orbit", canopy_radius_for_orbit_, canopy_radius_for_orbit_);
        pnh_.param("clockwise", clockwise_, clockwise_);
        pnh_.param("lock_tree_estimate", lock_tree_estimate_, lock_tree_estimate_);
        pnh_.param("cmd_vel_topic", cmd_vel_topic_, cmd_vel_topic_);
        pnh_.param("odom_topic", odom_topic_, odom_topic_);
        pnh_.param("scan_topic", scan_topic_, scan_topic_);
        pnh_.param("laser_frame", laser_frame_, laser_frame_);
        pnh_.param("laser_x_offset", laser_x_offset_, laser_x_offset_);
        pnh_.param("laser_y_offset", laser_y_offset_, laser_y_offset_);

        double align_tolerance_deg = align_tolerance_rad_ * 180.0 / M_PI;
        double max_orbit_delta_deg = max_orbit_delta_rad_ * 180.0 / M_PI;
        double max_heading_change_deg = max_heading_change_rad_ * 180.0 / M_PI;
        double laser_yaw_offset_deg = laser_yaw_offset_rad_ * 180.0 / M_PI;
        double heading_roi_min_deg = heading_roi_min_rad_ * 180.0 / M_PI;
        double heading_roi_max_deg = heading_roi_max_rad_ * 180.0 / M_PI;
        double facing_roi_min_deg = facing_roi_min_rad_ * 180.0 / M_PI;
        double facing_roi_max_deg = facing_roi_max_rad_ * 180.0 / M_PI;

        pnh_.param("align_tolerance_deg", align_tolerance_deg, align_tolerance_deg);
        pnh_.param("max_orbit_delta_deg", max_orbit_delta_deg, max_orbit_delta_deg);
        pnh_.param("max_heading_change_deg", max_heading_change_deg, max_heading_change_deg);
        pnh_.param("laser_yaw_offset_deg", laser_yaw_offset_deg, laser_yaw_offset_deg);
        pnh_.param("heading_roi_min_deg", heading_roi_min_deg, heading_roi_min_deg);
        pnh_.param("heading_roi_max_deg", heading_roi_max_deg, heading_roi_max_deg);
        pnh_.param("facing_roi_min_deg", facing_roi_min_deg, facing_roi_min_deg);
        pnh_.param("facing_roi_max_deg", facing_roi_max_deg, facing_roi_max_deg);

        align_tolerance_rad_ = align_tolerance_deg * M_PI / 180.0;
        max_orbit_delta_rad_ = max_orbit_delta_deg * M_PI / 180.0;
        max_heading_change_rad_ = max_heading_change_deg * M_PI / 180.0;
        laser_yaw_offset_rad_ = laser_yaw_offset_deg * M_PI / 180.0;
        heading_roi_min_rad_ = heading_roi_min_deg * M_PI / 180.0;
        heading_roi_max_rad_ = heading_roi_max_deg * M_PI / 180.0;
        facing_roi_min_rad_ = facing_roi_min_deg * M_PI / 180.0;
        facing_roi_max_rad_ = facing_roi_max_deg * M_PI / 180.0;

        int filter_window = static_cast<int>(filter_window_size_);
        pnh_.param("filter_window_size", filter_window, filter_window);
        filter_window_size_ = static_cast<size_t>(std::max(1, filter_window));

        step_distance_ = std::max(0.05, step_distance_);
        linear_speed_ = std::max(0.0, linear_speed_);
        target_distance_ = std::max(0.05, target_distance_);
        distance_tolerance_ = clamp(distance_tolerance_, 0.01, target_distance_);
        max_angular_speed_ = std::max(0.05, max_angular_speed_);
        work_pause_duration_ = std::max(0.0, work_pause_duration_);
        tree_filter_alpha_ = clamp(tree_filter_alpha_, 0.01, 1.0);
        max_orbit_delta_rad_ = clamp(max_orbit_delta_rad_, 1.0 * M_PI / 180.0,
                                     90.0 * M_PI / 180.0);
        return_distance_threshold_ = std::max(0.05, return_distance_threshold_);
        heading_fallback_timeout_ = std::max(0.0, heading_fallback_timeout_);
        max_heading_change_rad_ = clamp(max_heading_change_rad_, 0.5 * M_PI / 180.0,
                                        45.0 * M_PI / 180.0);
        canopy_radius_for_orbit_ = std::max(0.0, canopy_radius_for_orbit_);
        k_orbit_feedforward_ = clamp(k_orbit_feedforward_, 0.0, 2.0);

        // 两个权重只表达相对占比。归一化后总和恒为 1，避免改变控制器整体增益。
        facing_line_weight_ = std::max(0.0, facing_line_weight_);
        facing_bearing_weight_ = std::max(0.0, facing_bearing_weight_);
        const double facing_weight_sum = facing_line_weight_ + facing_bearing_weight_;
        if (facing_weight_sum < 1e-6) {
            ROS_WARN("正对判断的两个误差权重均为零，已恢复默认权重 0.6 和 0.4");
            facing_line_weight_ = 0.6;
            facing_bearing_weight_ = 0.4;
        } else {
            facing_line_weight_ /= facing_weight_sum;
            facing_bearing_weight_ /= facing_weight_sum;
        }
    }

    /**
     * @brief 将数值限制在指定范围内。
     *
     * @param value 输入值。
     * @param low 允许的最小值。
     * @param high 允许的最大值。
     * @return 限幅后的值。
     */
    static double clamp(double value, double low, double high) {
        return std::max(low, std::min(value, high));
    }

    /**
     * @brief 将角度归一化到 [-pi, pi]。
     *
     * @param angle 输入角度，单位：rad。
     * @return 归一化后的角度，单位：rad。
     */
    static double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    /**
     * @brief 处理 PCA 拟合直线的 180 度二义性。
     *
     * @param line_angle PCA 输出的直线角度，单位：rad。
     * @param reference_angle 期望接近的参考角，单位：rad。
     * @return 与 reference_angle 更接近的等价直线角度，单位：rad。
     */
    static double chooseClosestLineAngle(double line_angle, double reference_angle) {
        double best_angle = normalizeAngle(line_angle);
        double best_error = std::fabs(normalizeAngle(best_angle - reference_angle));

        const double candidates[2] = {
            normalizeAngle(line_angle + M_PI),
            normalizeAngle(line_angle - M_PI)
        };

        for (double candidate : candidates) {
            const double error = std::fabs(normalizeAngle(candidate - reference_angle));
            if (error < best_error) {
                best_error = error;
                best_angle = candidate;
            }
        }

        return best_angle;
    }

    /**
     * @brief 对点云进行 PCA 直线拟合。
     *
     * @param points 输入点云，坐标系：雷达坐标系。
     * @param reference_angle 用于消除 180 度二义性的参考角，单位：rad。
     * @return 直线拟合结果。点数少于 5 时 valid=false。
     */
    LineFitResult fitLineByPca(const std::vector<Point2D>& points, double reference_angle) const {
        LineFitResult result;
        result.point_count = points.size();
        if (points.size() < 5) {
            return result;
        }

        double sum_x = 0.0;
        double sum_y = 0.0;
        double sum_dist = 0.0;
        std::vector<double> distances;
        distances.reserve(points.size());

        for (const auto& p : points) {
            const double distance = std::hypot(p.x, p.y);
            sum_x += p.x;
            sum_y += p.y;
            sum_dist += distance;
            distances.push_back(distance);
        }

        const double mean_x = sum_x / static_cast<double>(points.size());
        const double mean_y = sum_y / static_cast<double>(points.size());

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

        const double raw_line_angle = 0.5 * std::atan2(2.0 * cov_xy, cov_xx - cov_yy);

        result.valid = true;
        result.line_angle = chooseClosestLineAngle(raw_line_angle, reference_angle);
        result.avg_distance = sum_dist / static_cast<double>(points.size());
        // 采用第 5 百分位距离，而不是整个弧段的平均距离。它相当于取一小组
        // 较近点的稳健代表值：比单个最小值抗噪，同时接近雷达到树冠的净间距。
        std::sort(distances.begin(), distances.end());
        const size_t surface_index = static_cast<size_t>(0.05 * (distances.size() - 1));
        result.surface_distance = distances[surface_index];
        result.bearing = std::atan2(mean_y, mean_x);
        result.centroid_x = mean_x;
        result.centroid_y = mean_y;
        return result;
    }

    /**
     * @brief 里程计回调函数。
     *
     * @param msg nav_msgs/Odometry 消息，提供机器人在 odom 坐标系下的位置和姿态。
     * @return 无。
     */
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
            // 不假定 Fast-LIO 一定从数值上的 (0, 0) 起步。记录第一帧位置后，
            // 后续统一计算相对起点距离，仿真和实车使用同一套完成条件。
            orbit_start_x_ = current_x_;
            orbit_start_y_ = current_y_;
            orbit_start_set_ = true;
            resetSegmentStart();
            has_odom_ = true;
            ROS_INFO("已记录绕行起点：x=%.3f 米，y=%.3f 米", orbit_start_x_, orbit_start_y_);
            return;
        }

        // 绕行角度来自里程计位置，因此在里程计回调中更新，避免其结果受
        // 雷达扫描频率影响。停止后不再继续改变最终统计值。
        if (state_ != MotionState::STOPPED) {
            updateOrbitProgress();
        }
    }

    /**
     * @brief 二维雷达回调函数。
     *
     * @param msg sensor_msgs/LaserScan 消息，坐标系通常为 livox_frame。
     * @return 无。
     */
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        if (!has_odom_) {
            publishStop();
            ROS_WARN_THROTTLE(1.0, "尚未收到里程计，等待里程计后再发送底盘控制指令");
            return;
        }

        std::vector<Point2D> heading_points;
        std::vector<Point2D> facing_points;
        extractRoiPoints(msg, heading_roi_min_rad_, heading_roi_max_rad_, heading_points);
        extractRoiPoints(msg, facing_roi_min_rad_, facing_roi_max_rad_, facing_points);

        // 扇形的内外半径直接采用当前 LaserScan 消息的有效测距范围，
        // 因此图像同时表达“允许的角度范围”和“允许的距离范围”。
        publishRoiRegion(heading_roi_min_rad_, heading_roi_max_rad_,
                         msg->range_min, msg->range_max,
                         heading_roi_region_pub_, "heading_roi_region",
                         0.0, 0.35, 1.0, 0.18, 0.01);
        publishRoiRegion(facing_roi_min_rad_, facing_roi_max_rad_,
                         msg->range_min, msg->range_max,
                         facing_roi_region_pub_, "facing_roi_region",
                         1.0, 0.45, 0.0, 0.22, 0.02);

        publishRoiPoints(heading_points, heading_roi_marker_pub_, "heading_roi_points", 0.0, 0.4, 1.0);
        publishRoiPoints(facing_points, facing_roi_marker_pub_, "facing_roi_points", 1.0, 0.2, 0.0);

        if (heading_points.size() < 5 && facing_points.size() < 5) {
            if ((ros::Time::now() - last_valid_scan_time_).toSec() > scan_timeout_) {
                publishStop();
                ROS_WARN_THROTTLE(1.0, "雷达检测区域内均无有效树冠点，底盘已停车");
            }
            return;
        }

        last_valid_scan_time_ = ros::Time::now();

        // PCA 只能得到一条无方向直线。车体 x 轴在雷达坐标系中的方向为
        // -laser_yaw_offset；以它作为参考才能选中“向前走”的切线方向，
        // 避免误选相反方向后持续输出最大角速度。
        const double heading_reference = normalizeAngle(-laser_yaw_offset_rad_);
        LineFitResult heading_fit = fitLineByPca(heading_points, heading_reference);
        if (heading_fit.valid) {
            // 宽角度 ROI 覆盖弯曲树冠时，PCA 第一主轴可能沿弧面的深度方向，
            // 而不是沿边界切向。第一主轴和其垂线中，更接近车辆当前前进
            // 方向的候选才作为局部切线；后续滤波继续保证方向连续。
            const double perpendicular = chooseClosestLineAngle(
                heading_fit.line_angle + M_PI / 2.0, heading_reference);
            const double primary_error = std::fabs(
                normalizeAngle(heading_fit.line_angle - heading_reference));
            const double perpendicular_error = std::fabs(
                normalizeAngle(perpendicular - heading_reference));
            if (perpendicular_error < primary_error) {
                heading_fit.line_angle = perpendicular;
            }
        }
        if (heading_fit.valid) {
            last_valid_heading_time_ = ros::Time::now();
        }

        const double facing_reference = M_PI / 2.0;
        const LineFitResult facing_fit = fitLineByPca(facing_points, facing_reference);

        publishFittedLine(heading_fit, heading_fit_marker_pub_, "heading_fit_line", 0.0, 0.8, 1.0);
        publishFittedLine(facing_fit, facing_fit_marker_pub_, "facing_fit_line", 1.0, 0.0, 0.8);

        if (state_ == MotionState::MOVE_AROUND_CANOPY && !heading_fit.valid) {
            ROS_WARN_THROTTLE(1.0, "航向检测区域内的有效点过少，正在检查是否允许短时几何兜底");
        }

        const bool normal_state_needs_facing =
            state_ == MotionState::WORK_PAUSE ||
            state_ == MotionState::ALIGN_TANGENT ||
            state_ == MotionState::MOVE_AROUND_CANOPY ||
            state_ == MotionState::FACE_CANOPY;
        if (normal_state_needs_facing && !facing_fit.valid) {
            publishStop();
            ROS_WARN_THROTTLE(1.0, "正对检测区域内的有效点过少，无法判断机器是否正对树冠");
            return;
        }

        const LineFitResult& tree_fit_for_estimate = facing_fit.valid
                                                    ? facing_fit
                                                    : heading_fit;
        updateTreeEstimate(tree_fit_for_estimate.surface_distance, tree_fit_for_estimate.bearing);

        const double start_distance = distanceToOrbitStart();
        const double directed_progress = directedOrbitProgress();
        if (state_ != MotionState::STOPPED &&
            directed_progress >= 2.0 * M_PI &&
            start_distance <= return_distance_threshold_) {
            setState(MotionState::STOPPED);
            publishStop();
            ROS_INFO("已完成一圈绕树运动：净累计角度=%.1f 度，距起点=%.2f 米，底盘已停车",
                     directed_progress * 180.0 / M_PI,
                     start_distance);
        } else if (state_ != MotionState::STOPPED &&
                   directed_progress >= 2.0 * M_PI) {
            ROS_INFO_THROTTLE(1.0,
                              "累计绕行角度已达到 %.1f 度，但距起点仍有 %.2f 米（要求不大于 %.2f 米），继续绕行",
                              directed_progress * 180.0 / M_PI,
                              start_distance,
                              return_distance_threshold_);
        }

        if (state_ == MotionState::STOPPED) {
            publishStop();
            publishSteeringMarker(0.0);
            return;
        }

        if (state_ == MotionState::WORK_PAUSE) {
            controlWorkPause(facing_fit);
            return;
        }

        if (state_ == MotionState::ALIGN_TANGENT) {
            controlAlignTangent(facing_fit);
            return;
        }

        if (!segment_start_set_) {
            resetSegmentStart();
        }

        const double segment_distance = std::hypot(current_x_ - segment_start_x_,
                                                   current_y_ - segment_start_y_);

        if (state_ == MotionState::MOVE_AROUND_CANOPY && segment_distance >= step_distance_) {
            const double distance_error = facing_fit.surface_distance - target_distance_;
            if (std::fabs(distance_error) <= distance_tolerance_) {
                setState(MotionState::FACE_CANOPY);
                publishStop();
                ROS_INFO("本段移动完成：已移动 %.2f 米，右侧树冠距离=%.2f 米，开始正对树冠",
                         segment_distance,
                         facing_fit.surface_distance);
                return;
            }

            ROS_INFO_THROTTLE(0.5,
                              "本段里程已达到 %.2f 米，但右侧树冠距离=%.2f 米，继续边移动边调整到 %.2f±%.2f 米",
                              segment_distance,
                              facing_fit.surface_distance,
                              target_distance_,
                              distance_tolerance_);
        }

        if (state_ == MotionState::FACE_CANOPY) {
            controlFaceCanopy(facing_fit);
            return;
        }

        controlMoveAroundCanopy(heading_fit, facing_fit);
    }

    /**
     * @brief 从 LaserScan 中提取指定角度范围内的有效点。
     *
     * @param msg 输入 LaserScan。
     * @param roi_min_rad ROI 起始角，单位：rad，坐标系：雷达坐标系。
     * @param roi_max_rad ROI 结束角，单位：rad，坐标系：雷达坐标系。
     * @param roi_points 输出点集，坐标系：雷达坐标系。
     * @return 无。
     */
    void extractRoiPoints(const sensor_msgs::LaserScan::ConstPtr& msg,
                          double roi_min_rad,
                          double roi_max_rad,
                          std::vector<Point2D>& roi_points) const {
        const double min_angle = std::min(roi_min_rad, roi_max_rad);
        const double max_angle = std::max(roi_min_rad, roi_max_rad);

        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            const double r = msg->ranges[i];
            if (std::isinf(r) || std::isnan(r) || r < msg->range_min || r > msg->range_max) {
                continue;
            }

            const double angle = msg->angle_min + static_cast<double>(i) * msg->angle_increment;
            if (angle < min_angle || angle > max_angle) {
                continue;
            }

            roi_points.push_back({r * std::cos(angle), r * std::sin(angle)});
        }
    }

    /**
     * @brief 对航向 ROI 的拟合线做滑动平均滤波。
     *
     * @param heading_angle 雷达坐标系下的航向直线角，单位：rad。
     * @return 滤波后的航向直线角，单位：rad。
     */
    double filterHeadingAngle(double heading_angle) {
        // PCA 拟合线具有 180 度二义性。先让新结果靠近上一帧结果，再限制
        // 单帧变化量，防止少量异常点让目标航向突然翻转或大幅跳变。
        if (!heading_history_.empty()) {
            heading_angle = chooseClosestLineAngle(heading_angle, heading_history_.back());
            const double delta = normalizeAngle(heading_angle - heading_history_.back());
            heading_angle = normalizeAngle(heading_history_.back()
                                         + clamp(delta,
                                                 -max_heading_change_rad_,
                                                 max_heading_change_rad_));
        }

        heading_history_.push_back(heading_angle);
        if (heading_history_.size() > filter_window_size_) {
            heading_history_.erase(heading_history_.begin());
        }

        double sin_sum = 0.0;
        double cos_sum = 0.0;
        for (double angle : heading_history_) {
            sin_sum += std::sin(angle);
            cos_sum += std::cos(angle);
        }

        return std::atan2(sin_sum, cos_sum);
    }

    /**
     * @brief 根据雷达测得的树冠边界估计树心位置。
     *
     * @param distance 雷达到树冠边界的平均距离，单位：m。
     * @param bearing 树冠边界点质心在雷达坐标系中的方位角，单位：rad。
     * @return 无。
     */
    void updateTreeEstimate(double distance, double bearing) {
        const double center_distance = distance + canopy_radius_for_orbit_;
        const double tree_x_laser = center_distance * std::cos(bearing);
        const double tree_y_laser = center_distance * std::sin(bearing);

        const double laser_yaw_odom = current_yaw_ + laser_yaw_offset_rad_;
        const double laser_x_odom = current_x_
                                  + std::cos(current_yaw_) * laser_x_offset_
                                  - std::sin(current_yaw_) * laser_y_offset_;
        const double laser_y_odom = current_y_
                                  + std::sin(current_yaw_) * laser_x_offset_
                                  + std::cos(current_yaw_) * laser_y_offset_;
        const double tree_x_odom = laser_x_odom + std::cos(laser_yaw_odom) * tree_x_laser
                                                 - std::sin(laser_yaw_odom) * tree_y_laser;
        const double tree_y_odom = laser_y_odom + std::sin(laser_yaw_odom) * tree_x_laser
                                                 + std::cos(laser_yaw_odom) * tree_y_laser;

        if (!has_tree_estimate_) {
            estimated_tree_x_ = tree_x_odom;
            estimated_tree_y_ = tree_y_odom;
            last_orbit_angle_ = std::atan2(current_y_ - estimated_tree_y_,
                                           current_x_ - estimated_tree_x_);
            accumulated_orbit_angle_ = 0.0;
            has_tree_estimate_ = true;
            return;
        }

        // 一圈完成判断必须围绕同一个参考中心。对当前单树仿真，首次正对观测
        // 已足以得到稳定树心；锁定后可避免局部树冠半径变化让树心随车漂移。
        if (lock_tree_estimate_) {
            return;
        }

        estimated_tree_x_ = (1.0 - tree_filter_alpha_) * estimated_tree_x_
                          + tree_filter_alpha_ * tree_x_odom;
        estimated_tree_y_ = (1.0 - tree_filter_alpha_) * estimated_tree_y_
                          + tree_filter_alpha_ * tree_y_odom;
    }

    /**
     * @brief 更新机器人已经绕树转过的累计角度。
     *
     * @return 无。
     */
    void updateOrbitProgress() {
        if (!has_tree_estimate_) {
            return;
        }

        const double orbit_angle = std::atan2(current_y_ - estimated_tree_y_,
                                              current_x_ - estimated_tree_x_);
        const double delta = normalizeAngle(orbit_angle - last_orbit_angle_);
        last_orbit_angle_ = orbit_angle;

        // 顺向和逆向变化必须全部累计，这样里程计在同一区域来回抖动时能够
        // 相互抵消。旧逻辑只保留期望方向的变化，会把零均值噪声积成一整圈。
        if (std::fabs(delta) > max_orbit_delta_rad_) {
            ROS_WARN_THROTTLE(1.0,
                              "里程计绕行极角单次跳变 %.1f 度，超过 %.1f 度限制，本次不累计",
                              delta * 180.0 / M_PI,
                              max_orbit_delta_rad_ * 180.0 / M_PI);
            return;
        }

        accumulated_orbit_angle_ += delta;
    }

    /**
     * @brief 计算当前里程计位置到绕行起点的直线距离。
     * @return 距离，单位：m；尚未记录起点时返回无穷大。
     */
    double distanceToOrbitStart() const {
        if (!orbit_start_set_) {
            return std::numeric_limits<double>::infinity();
        }
        return std::hypot(current_x_ - orbit_start_x_, current_y_ - orbit_start_y_);
    }

    /**
     * @brief 将有符号净累计角度转换成设定绕行方向上的正进度。
     * @return 顺时针时返回负累计角的相反数，逆时针时返回正累计角。
     */
    double directedOrbitProgress() const {
        return clockwise_ ? -accumulated_orbit_angle_ : accumulated_orbit_angle_;
    }

    /**
     * @brief 控制 WORK_PAUSE 状态。
     *
     * @param facing_fit 正前方 ROI 的拟合结果，用于日志显示当前正对情况。
     * @return 无。
     */
    void controlWorkPause(const LineFitResult& facing_fit) {
        publishStop();

        const double elapsed = (ros::Time::now() - state_enter_time_).toSec();
        ROS_INFO_THROTTLE(0.5,
                          "作业停顿：已等待 %.2f/%.2f 秒，正对拟合线角=%.1f 度，质心方位角=%.1f 度",
                          elapsed,
                          work_pause_duration_,
                          facing_fit.line_angle * 180.0 / M_PI,
                          facing_fit.bearing * 180.0 / M_PI);

        if (elapsed >= work_pause_duration_) {
            setState(MotionState::ALIGN_TANGENT);
            ROS_INFO("作业停顿结束，开始检查移动前的树冠朝向");
        }
    }

    /**
     * @brief 控制作业后进入移动前的对齐状态。
     *
     * @param facing_fit 正前方 ROI 的拟合结果。
     * @return 无。
     */
    void controlAlignTangent(const LineFitResult& facing_fit) {
        const double yaw_error = calculateFacingError(facing_fit);

        if (std::fabs(yaw_error) <= align_tolerance_rad_) {
            publishStop();
            resetSegmentStart();
            heading_history_.clear();
            setState(MotionState::MOVE_AROUND_CANOPY);
            ROS_INFO("树冠朝向已对准，开始新一段绕树移动");
            return;
        }

        geometry_msgs::Twist cmd;
        cmd.angular.z = clamp(k_turn_align_ * yaw_error, -max_angular_speed_, max_angular_speed_);
        cmd_vel_pub_.publish(cmd);

        publishSteeringMarker(facing_fit.line_angle);
        ROS_INFO_THROTTLE(0.5,
                          "移动前对准：拟合线角=%.1f 度，质心方位角=%.1f 度，融合误差=%.1f 度，角速度指令=%.3f 弧度每秒",
                          facing_fit.line_angle * 180.0 / M_PI,
                          facing_fit.bearing * 180.0 / M_PI,
                          yaw_error * 180.0 / M_PI,
                          cmd.angular.z);
    }

    /**
     * @brief 控制 MOVE_AROUND_CANOPY 状态。
     *
     * @param heading_fit 航向 ROI 的拟合结果，正常情况下提供局部树冠切线。
     * @param facing_fit 正对 ROI 的拟合结果，提供树冠方位和表面距离。
     * @return 无。
     */
    void controlMoveAroundCanopy(const LineFitResult& heading_fit,
                                 const LineFitResult& facing_fit) {
        if (!has_tree_estimate_) {
            publishStop();
            ROS_WARN_THROTTLE(1.0, "尚未建立树心估计，绕树移动已停止");
            return;
        }

        const ros::Time now = ros::Time::now();
        const double dx = current_x_ - estimated_tree_x_;
        const double dy = current_y_ - estimated_tree_y_;
        const double current_orbit_radius = std::max(0.10, std::hypot(dx, dy));
        const double radial_angle = std::atan2(dy, dx);
        const double orbit_sign = clockwise_ ? -1.0 : 1.0;

        const bool using_lidar_heading = heading_fit.valid;
        double heading_error = 0.0;
        double target_heading_laser = 0.0;
        // 基础曲率只规定顺/逆时针以及大致转弯量，不提供目标航向。
        // 真正的下一步方向仍由雷达局部切线 heading_error 决定。
        const double nominal_orbit_radius = std::max(
            0.10, canopy_radius_for_orbit_ + target_distance_);
        double feedforward_w = orbit_sign * k_orbit_feedforward_
                             * linear_speed_ / nominal_orbit_radius;

        if (using_lidar_heading) {
            // 拟合线位于雷达坐标系。加上雷达相对底盘的安装角后，得到目标
            // 切线相对当前车体前方的误差；它直接决定下一小段的移动方向。
            target_heading_laser = filterHeadingAngle(heading_fit.line_angle);
            heading_error = normalizeAngle(laser_yaw_offset_rad_ + target_heading_laser);
        } else {
            const double invalid_duration = last_valid_heading_time_.isZero()
                                          ? (now - state_enter_time_).toSec()
                                          : (now - last_valid_heading_time_).toSec();
            if (invalid_duration > heading_fallback_timeout_) {
                publishStop();
                publishSteeringMarker(0.0);
                ROS_WARN_THROTTLE(1.0,
                                  "航向点云已连续无效 %.2f 秒，超过 %.2f 秒兜底时限，底盘已停车",
                                  invalid_duration,
                                  heading_fallback_timeout_);
                return;
            }

            // 雷达航向短时丢失时才使用树心几何切线，避免一两帧缺点导致急停。
            const double desired_yaw = normalizeAngle(radial_angle + orbit_sign * M_PI / 2.0);
            heading_error = normalizeAngle(desired_yaw - current_yaw_);
            target_heading_laser = normalizeAngle(desired_yaw
                                                - current_yaw_
                                                - laser_yaw_offset_rad_);
            // 兜底阶段才使用固定树心测得的当前半径，使短时盲行曲率连续。
            feedforward_w = orbit_sign * k_orbit_feedforward_
                          * linear_speed_ / current_orbit_radius;
        }

        // bearing > 0 表示树冠在雷达左侧，底盘应左转；bearing < 0 则右转。
        const double bearing_error = normalizeAngle(facing_fit.bearing);
        const double distance_error = facing_fit.surface_distance - target_distance_;

        // 雷达有效时，局部拟合切线是主航向；几何兜底时使用独立的较温和增益。
        // 每一项单独限幅，避免局部异常树冠点让角速度突然饱和。
        const double heading_gain = using_lidar_heading ? k_turn_move_ : k_orbit_heading_;
        double heading_w = clamp(heading_gain * heading_error, -0.06, 0.06);
        double bearing_w = clamp(k_bearing_move_ * bearing_error, -0.05, 0.05);
        const double distance_w = clamp(orbit_sign * k_distance_move_ * distance_error,
                                        -0.08, 0.08);

        // 当右侧净距超出完成容差时，距离闭环拥有更高优先级，防止局部切线
        // 和质心方位项与靠近/远离修正相互抵消。回到容差带后立即恢复完整
        // 的雷达局部切线跟随。
        if (std::fabs(distance_error) > distance_tolerance_) {
            heading_w *= 0.35;
            bearing_w *= 0.35;
        }

        geometry_msgs::Twist cmd;
        // 朝向或树冠方位误差增大时适当减速，但仍保留线速度，避免只转不走。
        const double control_error = std::max(std::fabs(heading_error), std::fabs(bearing_error));
        const double heading_speed_scale = clamp(std::cos(control_error), 0.25, 1.0);
        // 偏离 0.30 m 较多时降低前进速度，为横向转向修正留出距离；仍保持
        // 最低 40% 线速度，避免退化为原地打转。
        const double distance_speed_scale = clamp(
            1.0 - std::fabs(distance_error) / std::max(0.10, target_distance_),
            0.40, 1.0);
        const double speed_scale = std::min(heading_speed_scale, distance_speed_scale);
        cmd.linear.x = linear_speed_ * speed_scale;
        cmd.angular.z = clamp(feedforward_w + heading_w + bearing_w + distance_w,
                              -max_angular_speed_,
                              max_angular_speed_);
        cmd_vel_pub_.publish(cmd);

        publishSteeringMarker(target_heading_laser);

        ROS_INFO_THROTTLE(0.5,
                          "绕树移动：航向来源=%s，本段距离=%.2f/%.2f 米，航向误差=%.1f 度，质心方位误差=%.1f 度，表面距离=%.2f 米，距起点=%.2f 米，角速度分量（前馈=%.3f，航向=%.3f，方位=%.3f，距离=%.3f），速度指令（线速度=%.2f 米每秒，角速度=%.3f 弧度每秒），净累计绕行=%.1f 度",
                          using_lidar_heading ? "雷达局部切线" : "短时几何兜底",
                          std::hypot(current_x_ - segment_start_x_, current_y_ - segment_start_y_),
                          step_distance_,
                          heading_error * 180.0 / M_PI,
                          bearing_error * 180.0 / M_PI,
                          facing_fit.surface_distance,
                          distanceToOrbitStart(),
                          feedforward_w,
                          heading_w,
                          bearing_w,
                          distance_w,
                          cmd.linear.x,
                          cmd.angular.z,
                          directedOrbitProgress() * 180.0 / M_PI);
    }

    /**
     * @brief 控制 FACE_CANOPY 状态。
     *
     * @param facing_fit 正前方 ROI 的拟合结果。
     * @return 无。
     */
    void controlFaceCanopy(const LineFitResult& facing_fit) {
        const double yaw_error = calculateFacingError(facing_fit);

        if (std::fabs(yaw_error) <= align_tolerance_rad_) {
            publishStop();
            setState(MotionState::WORK_PAUSE);
            ROS_INFO("机器已重新正对树冠，进入作业停顿状态");
            return;
        }

        geometry_msgs::Twist cmd;
        cmd.angular.z = clamp(k_turn_align_ * yaw_error, -max_angular_speed_, max_angular_speed_);
        cmd_vel_pub_.publish(cmd);

        publishSteeringMarker(facing_fit.line_angle);
        ROS_INFO_THROTTLE(0.5,
                          "正对树冠调整：拟合线角=%.1f 度，质心方位角=%.1f 度，融合误差=%.1f 度，角速度指令=%.3f 弧度每秒",
                          facing_fit.line_angle * 180.0 / M_PI,
                          facing_fit.bearing * 180.0 / M_PI,
                          yaw_error * 180.0 / M_PI,
                          cmd.angular.z);
    }

    /**
     * @brief 融合前方 ROI 的拟合线角和质心方位，计算“是否正对树冠”的误差。
     *
     * 判断依据：
     *   当雷达正对树冠时，正前方 ROI 扫到的树冠边界通常近似横向展开，
     *   因此拟合线在雷达坐标系下应接近 +90 deg 或 -90 deg；同时，ROI
     *   质心方位 bearing 应接近 0 deg。两种信息加权融合后，局部枝叶造成的
     *   单一 PCA 线角波动不容易直接触发错误转向。
     *
     * @param facing_fit 正前方 ROI 的拟合结果。
     * @return 对准误差，单位：rad。绝对值越小，表示越接近正对树冠。
     */
    double calculateFacingError(const LineFitResult& facing_fit) const {
        const double target_facing_line = M_PI / 2.0;
        const double line_angle = chooseClosestLineAngle(facing_fit.line_angle, target_facing_line);
        const double line_error = normalizeAngle(line_angle - target_facing_line);
        const double bearing_error = normalizeAngle(facing_fit.bearing);
        return normalizeAngle(facing_line_weight_ * line_error
                            + facing_bearing_weight_ * bearing_error);
    }

    /**
     * @brief 将状态枚举转换为便于日志和话题调试的固定字符串。
     *
     * @param state 状态机枚举值。
     * @return 状态名称。返回字符串的生命周期覆盖整个程序运行期。
     */
    static const char* stateName(MotionState state) {
        switch (state) {
            case MotionState::WORK_PAUSE: return "WORK_PAUSE";
            case MotionState::ALIGN_TANGENT: return "ALIGN_TANGENT";
            case MotionState::MOVE_AROUND_CANOPY: return "MOVE_AROUND_CANOPY";
            case MotionState::FACE_CANOPY: return "FACE_CANOPY";
            case MotionState::STOPPED: return "STOPPED";
        }
        return "UNKNOWN";
    }

    /**
     * @brief 将状态枚举转换为中文名称，仅用于终端日志。
     *
     * 状态话题仍然使用 stateName() 返回的固定英文标识，以保持接口兼容；
     * 终端日志则使用本函数返回的中文名称，方便直接观察状态变化。
     *
     * @param state 状态机枚举值。
     * @return 中文状态名称。返回字符串的生命周期覆盖整个程序运行期。
     */
    static const char* stateChineseName(MotionState state) {
        switch (state) {
            case MotionState::WORK_PAUSE: return "作业停顿";
            case MotionState::ALIGN_TANGENT: return "移动前对准";
            case MotionState::MOVE_AROUND_CANOPY: return "绕树移动";
            case MotionState::FACE_CANOPY: return "正对树冠调整";
            case MotionState::STOPPED: return "停止";
        }
        return "未知状态";
    }

    /**
     * @brief 发布当前状态机名称。
     *
     * @return 无。
     */
    void publishState() {
        std_msgs::String msg;
        msg.data = stateName(state_);
        state_pub_.publish(msg);
    }

    /**
     * @brief 完成状态转换，并同步更新时间与状态调试话题。
     *
     * @param new_state 即将进入的新状态。
     * @return 无。若状态未变化，不重复更新时间和发布消息。
     */
    void setState(MotionState new_state) {
        if (state_ == new_state) {
            return;
        }
        const char* old_name = stateChineseName(state_);
        state_ = new_state;
        state_enter_time_ = ros::Time::now();
        publishState();
        ROS_INFO("状态转换：%s -> %s", old_name, stateChineseName(state_));
    }

    /**
     * @brief 记录当前移动小段的起点。
     *
     * @return 无。
     */
    void resetSegmentStart() {
        segment_start_x_ = current_x_;
        segment_start_y_ = current_y_;
        segment_start_set_ = true;
    }

    /**
     * @brief 发布零速度，让底盘停止。
     *
     * @return 无。
     */
    void publishStop() {
        geometry_msgs::Twist cmd;
        cmd_vel_pub_.publish(cmd);
    }

    /**
     * @brief 在 RViz 中显示某个 ROI 点集。
     *
     * @param points ROI 点集，坐标系：雷达坐标系。
     * @param publisher Marker 发布器。
     * @param ns Marker 命名空间。
     * @param r 红色通道，范围 0~1。
     * @param g 绿色通道，范围 0~1。
     * @param b 蓝色通道，范围 0~1。
     * @return 无。
     */
    void publishRoiPoints(const std::vector<Point2D>& points,
                          ros::Publisher& publisher,
                          const std::string& ns,
                          double r,
                          double g,
                          double b) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = laser_frame_;
        marker.header.stamp = ros::Time::now();
        marker.ns = ns;
        marker.id = 0;
        marker.type = visualization_msgs::Marker::POINTS;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.04;
        marker.scale.y = 0.04;
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 1.0;
        marker.lifetime = ros::Duration(0.2);

        for (const auto& point : points) {
            geometry_msgs::Point p;
            p.x = point.x;
            p.y = point.y;
            p.z = 0.0;
            marker.points.push_back(p);
        }

        publisher.publish(marker);
    }

    /**
     * @brief 在 RViz 中用半透明扇形显示一个雷达检测区域。
     *
     * 扇形使用 TRIANGLE_LIST 拼接而成。每个小扇片由内圆弧和外圆弧之间
     * 的两个三角形组成，所以既能显示角度边界，也能显示 LaserScan 的
     * 最小、最大有效测距。Marker 随 livox_frame 一起运动。
     *
     * @param roi_min_rad 检测区域最小角，单位：rad，雷达坐标系。
     * @param roi_max_rad 检测区域最大角，单位：rad，雷达坐标系。
     * @param range_min 有效测距下限，单位：m。
     * @param range_max 有效测距上限，单位：m。
     * @param publisher Marker 发布器。
     * @param ns Marker 命名空间。
     * @param r 红色通道，范围 0~1。
     * @param g 绿色通道，范围 0~1。
     * @param b 蓝色通道，范围 0~1。
     * @param alpha 透明度，范围 0~1。
     * @param z_height 扇形相对雷达坐标系原点的显示高度，单位：m。
     * @return 无。
     */
    void publishRoiRegion(double roi_min_rad,
                          double roi_max_rad,
                          double range_min,
                          double range_max,
                          ros::Publisher& publisher,
                          const std::string& ns,
                          double r,
                          double g,
                          double b,
                          double alpha,
                          double z_height) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = laser_frame_;
        marker.header.stamp = ros::Time::now();
        marker.ns = ns;
        marker.id = 0;
        marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        // TRIANGLE_LIST 的点已经使用真实米制坐标，因此缩放必须保持为 1。
        // 如果不显式赋值，Marker 的默认缩放为 0，RViz 会报警并拒绝绘制。
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = alpha;
        marker.lifetime = ros::Duration(0.2);

        const double min_angle = std::min(roi_min_rad, roi_max_rad);
        const double max_angle = std::max(roi_min_rad, roi_max_rad);
        const double inner_radius = std::max(0.0, range_min);
        const double outer_radius = std::max(inner_radius, range_max);

        // 每个扇片最大约 3 deg，使圆弧在 RViz 中看起来足够平滑。
        const int segment_count = std::max(
            1, static_cast<int>(std::ceil((max_angle - min_angle) / (3.0 * M_PI / 180.0))));

        const auto makePoint = [z_height](double radius, double angle) {
            geometry_msgs::Point point;
            point.x = radius * std::cos(angle);
            point.y = radius * std::sin(angle);
            point.z = z_height;
            return point;
        };

        for (int i = 0; i < segment_count; ++i) {
            const double ratio0 = static_cast<double>(i) / segment_count;
            const double ratio1 = static_cast<double>(i + 1) / segment_count;
            const double angle0 = min_angle + (max_angle - min_angle) * ratio0;
            const double angle1 = min_angle + (max_angle - min_angle) * ratio1;

            const geometry_msgs::Point inner0 = makePoint(inner_radius, angle0);
            const geometry_msgs::Point outer0 = makePoint(outer_radius, angle0);
            const geometry_msgs::Point inner1 = makePoint(inner_radius, angle1);
            const geometry_msgs::Point outer1 = makePoint(outer_radius, angle1);

            marker.points.push_back(inner0);
            marker.points.push_back(outer0);
            marker.points.push_back(outer1);
            marker.points.push_back(inner0);
            marker.points.push_back(outer1);
            marker.points.push_back(inner1);
        }

        publisher.publish(marker);
    }

    /**
     * @brief 在 RViz 中画出穿过 ROI 质心的 PCA 拟合线。
     *
     * @param fit PCA 拟合结果；无效结果会删除上一次残留 Marker。
     * @param publisher Marker 发布器。
     * @param ns Marker 命名空间。
     * @param r 红色通道，范围 0~1。
     * @param g 绿色通道，范围 0~1。
     * @param b 蓝色通道，范围 0~1。
     * @return 无。
     */
    void publishFittedLine(const LineFitResult& fit,
                           ros::Publisher& publisher,
                           const std::string& ns,
                           double r,
                           double g,
                           double b) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = laser_frame_;
        marker.header.stamp = ros::Time::now();
        marker.ns = ns;
        marker.id = 0;

        if (!fit.valid) {
            marker.action = visualization_msgs::Marker::DELETE;
            publisher.publish(marker);
            return;
        }

        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.035;
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 1.0;
        marker.lifetime = ros::Duration(0.2);

        // 线段总长 1.2 m，以点云质心为中心，方向取 PCA 主方向。
        const double half_length = 0.6;
        const double dx = half_length * std::cos(fit.line_angle);
        const double dy = half_length * std::sin(fit.line_angle);

        geometry_msgs::Point start;
        start.x = fit.centroid_x - dx;
        start.y = fit.centroid_y - dy;
        start.z = 0.03;
        geometry_msgs::Point end;
        end.x = fit.centroid_x + dx;
        end.y = fit.centroid_y + dy;
        end.z = 0.03;
        marker.points.push_back(start);
        marker.points.push_back(end);
        publisher.publish(marker);
    }

    /**
     * @brief 在 RViz 中显示目标方向箭头。
     *
     * @param heading_angle 箭头方向角，单位：rad，坐标系：雷达坐标系。
     * @return 无。
     */
    void publishSteeringMarker(double heading_angle) {
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

        steering_marker_pub_.publish(arrow);
    }
};

/**
 * @brief ROS 节点入口函数。
 *
 * @param argc 命令行参数数量。
 * @param argv 命令行参数数组。
 * @return 程序退出码。正常退出返回 0。
 */
int main(int argc, char** argv) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "canopy_tracker_node");

    CanopyTracker tracker;
    ros::spin();
    return 0;
}
