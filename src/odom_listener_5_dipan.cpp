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
     * @brief 运动状态枚举类，状态机
     *
     * 用于表示目标相对于树冠（canopy）的运动模式。
     */
    enum class MotionState {
        WORK_PAUSE,        ///< 作业停顿（模拟采收或喷药等动作）
        ALIGN_TANGENT,     ///< 对齐切线方向（原地调整朝向，使树冠位于侧面）
        MOVE_AROUND_CANOPY,///< 围绕树冠移动
        FACE_CANOPY,       ///< 面向树冠（原地调整朝向，使树冠位于正前方）
        STOPPED            ///< 停止所有运动
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
    MotionState state_ = MotionState::WORK_PAUSE;  ///< 当前运动状态，初始为作业停顿
    ros::Time state_enter_time_;                  ///< 进入当前状态的时刻（用于计时）
    ros::Time last_valid_scan_time_;              ///< 最后一次接收到有效激光扫描数据的时间戳

    // ===================== 运动控制参数 =====================
    double target_distance_ = 0.8;                      ///< 期望与树冠保持的距离 (m)
    double step_distance_ = 0.5;                        ///< 沿树冠切线移动的步长 (m)
    double linear_speed_ = 0.12;                        ///< 直线移动时的线速度 (m/s)
    double k_dist_ = 0.5;                               ///< 距离误差的比例增益（用于转向修正）
    double k_turn_move_ = 1.2;                          ///< 绕树移动时转向的比例增益
    double k_turn_align_ = 1.5;                         ///< 朝向对齐（面朝树冠/切线）时的转向比例增益
    double k_bearing_ = 0.8;                            ///< 树冠方位角误差的比例增益（用于绕树移动时的侧向保持）
    double max_angular_speed_ = 0.5;                    ///< 最大允许角速度 (rad/s)
    double align_tolerance_rad_ = 5.0 * M_PI / 180.0;  ///< 朝向对齐容差，5° 转弧度 (~0.0873 rad)
    double scan_timeout_ = 0.5;                         ///< 激光扫描数据超时判定时间 (s)
    double work_pause_duration_ = 1.0;                  ///< 作业停顿持续时间 (s)，用于模拟采收/喷药等动作

    // ===================== 滤波与约束参数 =====================
    double tree_filter_alpha_ = 0.15;                   ///< 树木位置估计的低通滤波系数 (0~1)，越小越平滑
    double min_orbit_delta_rad_ = 0.2 * M_PI / 180.0;  ///< 累计绕树角度的最小增量阈值 (~0.0035 rad)，避免微小扰动
    double roi_angle_limit_rad_ = 120.0 * M_PI / 180.0; ///< 感兴趣区域（ROI）的角度范围（弧度），从激光雷达正前方对称展开
    double canopy_radius_for_orbit_ = 0.3;              ///< 树冠半径估计值 (m)，用于将树冠边界距离转换为树心距离
    double laser_yaw_offset_rad_ = -90.0 * M_PI / 180.0;///< 激光雷达相对于车体的安装偏航角 (rad)，负值表示雷达正对车体右侧
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
        // 从 ROS 参数服务器加载运行参数（距离、速度、增益等）
        loadParameters();

        // 订阅传感器数据，队列长度 10，绑定回调函数
        odom_sub_ = nh_.subscribe(odom_topic_, 10, &CanopyTracker::odomCallback, this);
        scan_sub_ = nh_.subscribe(scan_topic_, 10, &CanopyTracker::scanCallback, this);

        // 发布可视化引导向量标记，用于 RViz 显示
        marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/steering_vector", 10);
        // 发布机器人速度指令
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 10);

        // 初始化状态进入时间
        state_enter_time_ = ros::Time::now();

        // 打印启动信息，包含关键参数
        ROS_INFO("树冠跟踪器已启动。速度话题=%s 步长=%.2fm 目标距离=%.2fm 作业停顿时间=%.2fs 绕行方向=%s",
                 cmd_vel_topic_.c_str(), step_distance_, target_distance_, work_pause_duration_,
                 clockwise_ ? "顺时针" : "逆时针");
    }

    /**
     * @brief 析构函数，节点退出前发送停止指令，保证机器人安全停止
     */
    ~CanopyTracker() {
        publishStop();  // 发布零线速度和零角速度，确保机器人停止
    }

private:
    /**
     * @brief 从 ROS 私有参数服务器加载配置参数，并进行范围约束与单位转换
     *
     * 参数名若存在则读取，否则保留默认值。
     * 角度类参数以度为单位读入，内部转换为弧度存储。
     * 对速度、距离、滤波系数等施加合理边界，避免异常值。
     */
    void loadParameters() {
        // ---- 基础运动控制参数 ----
        pnh_.param("target_distance", target_distance_, target_distance_);                // 目标树冠距离 (m)
        pnh_.param("step_distance", step_distance_, step_distance_);                      // 步长 (m)
        pnh_.param("linear_speed", linear_speed_, linear_speed_);                         // 线速度 (m/s)
        pnh_.param("k_dist", k_dist_, k_dist_);                                           // 距离误差比例增益
        pnh_.param("k_turn_move", k_turn_move_, k_turn_move_);                            // 绕树转向增益
        pnh_.param("k_turn_align", k_turn_align_, k_turn_align_);                         // 对齐转向增益
        pnh_.param("k_bearing", k_bearing_, k_bearing_);                                  // 树冠方位角误差增益
        pnh_.param("max_angular_speed", max_angular_speed_, max_angular_speed_);          // 最大角速度 (rad/s)

        // 对齐容差：以度读入，转换为弧度保存
        pnh_.param("align_tolerance_deg", align_tolerance_rad_, align_tolerance_rad_ * 180.0 / M_PI);
        align_tolerance_rad_ *= M_PI / 180.0;                                             // 度 → 弧度

        pnh_.param("scan_timeout", scan_timeout_, scan_timeout_);                         // 扫描超时 (s)
        pnh_.param("work_pause_duration", work_pause_duration_, work_pause_duration_);    // 作业停顿时长 (s)

        // ---- 滤波与约束参数 ----
        pnh_.param("tree_filter_alpha", tree_filter_alpha_, tree_filter_alpha_);          // 树木位置低通滤波系数

        // 最小绕树角度增量：以度读入，转换为弧度保存
        pnh_.param("min_orbit_delta_deg", min_orbit_delta_rad_, min_orbit_delta_rad_ * 180.0 / M_PI);
        min_orbit_delta_rad_ *= M_PI / 180.0;                                             // 度 → 弧度

        // ROI 角度范围：以度读入，转换为弧度保存
        pnh_.param("roi_angle_limit_deg", roi_angle_limit_rad_, roi_angle_limit_rad_ * 180.0 / M_PI);
        roi_angle_limit_rad_ *= M_PI / 180.0;                                             // 度 → 弧度

        // 树冠半径估计值 (m)
        pnh_.param("canopy_radius_for_orbit", canopy_radius_for_orbit_, canopy_radius_for_orbit_);

        // 激光雷达安装偏航角：以度读入，转换为弧度保存
        pnh_.param("laser_yaw_offset_deg", laser_yaw_offset_rad_, laser_yaw_offset_rad_ * 180.0 / M_PI);
        laser_yaw_offset_rad_ *= M_PI / 180.0;                                            // 度 → 弧度

        pnh_.param("clockwise", clockwise_, clockwise_);                                  // 绕树方向

        // ---- 话题与坐标系名称 ----
        pnh_.param("cmd_vel_topic", cmd_vel_topic_, cmd_vel_topic_);
        pnh_.param("odom_topic", odom_topic_, odom_topic_);
        pnh_.param("scan_topic", scan_topic_, scan_topic_);
        pnh_.param("laser_frame", laser_frame_, laser_frame_);

        // ---- 滑动窗口滤波参数 ----
        int filter_window = static_cast<int>(filter_window_size_);                        // 临时以 int 读取
        pnh_.param("filter_window_size", filter_window, filter_window);
        filter_window_size_ = static_cast<size_t>(std::max(1, filter_window));            // 确保窗口至少为 1

        // ---- 应用硬约束，防止异常值 ----
        step_distance_     = std::max(0.05, step_distance_);                              // 步长下限 5cm
        linear_speed_      = std::max(0.0, linear_speed_);                                // 线速度非负
        max_angular_speed_ = std::max(0.05, max_angular_speed_);                          // 最大角速度下限 0.05 rad/s
        work_pause_duration_ = std::max(0.0, work_pause_duration_);                       // 作业停顿时间非负
        tree_filter_alpha_ = clamp(tree_filter_alpha_, 0.01, 1.0);                        // 滤波系数限幅 0.01~1.0
        min_orbit_delta_rad_ = clamp(min_orbit_delta_rad_, 0.0, 5.0 * M_PI / 180.0);     // 角度增量上限 5°
        roi_angle_limit_rad_ = clamp(roi_angle_limit_rad_, 20.0 * M_PI / 180.0, M_PI);   // ROI 角度限幅 20°~180°
        canopy_radius_for_orbit_ = std::max(0.0, canopy_radius_for_orbit_);               // 树冠半径非负
    }

    /**
     * @brief 将值限定在 [low, high] 区间内
     * @param value 原始值
     * @param low   下限
     * @param high  上限
     * @return 限幅后的值
     */
    static double clamp(double value, double low, double high) {
        return std::max(low, std::min(value, high));
    }

    /**
     * @brief 将角度归一化到 [-π, π) 范围内
     * @param angle 原始角度 (rad)
     * @return 归一化后的角度 (rad)
     */
    static double normalizeAngle(double angle) {
        while (angle > M_PI)  angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    /**
     * @brief 根据树冠点云计算近似切线方向、平均距离与方位角
     *
     * 使用主成分分析（PCA）拟合点云的分布方向，返回的切线角度
     * 为主轴方向角度（在 [-π/2, π/2) 范围内）。
     *
     * @param[in]  points       属于树冠的二维点集（激光坐标系）
     * @param[out] avg_distance  点集到激光原点的平均距离 (m)
     * @param[out] bearing       点集中心相对于激光原点的方位角 (rad, [-π, π))
     * @return 树冠轮廓的切线方向角 (rad, 相对于激光坐标系 x 轴)
     */
    double calculateCanopyTangent(const std::vector<Point2D>& points,
                                  double& avg_distance,
                                  double& bearing) {
        // 点数不足，无法可靠估计，返回 0
        if (points.size() < 5) return 0.0;

        double sum_x = 0.0;
        double sum_y = 0.0;
        double sum_dist = 0.0;
        const int n = static_cast<int>(points.size());

        // 累加各点的坐标与距离
        for (const auto& p : points) {
            sum_x += p.x;
            sum_y += p.y;
            sum_dist += std::sqrt(p.x * p.x + p.y * p.y);
        }

        const double mean_x = sum_x / n;      // 点集中心 x
        const double mean_y = sum_y / n;      // 点集中心 y
        avg_distance = sum_dist / n;          // 平均距离
        bearing = std::atan2(mean_y, mean_x); // 中心方位角

        // 计算协方差矩阵元素
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

        // 主方向角 = 0.5 * atan2(2*cov_xy, cov_xx - cov_yy)
        return 0.5 * std::atan2(2.0 * cov_xy, cov_xx - cov_yy);
    }

    /**
     * @brief 里程计数据回调，更新机器人位姿并初始化移动起点
     * @param msg 里程计消息（包含世界坐标系下的位置与姿态四元数）
     */
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        // 提取平面位置
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;

        // 将四元数转换为欧拉角，提取偏航角
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

        // 初次获得里程计时，将当前位置设为当前片段的起点
        if (!has_odom_) {
            segment_start_x_ = current_x_;
            segment_start_y_ = current_y_;
            segment_start_set_ = true;
            has_odom_ = true;
        }
    }

    /**
     * @brief 激光扫描数据回调，执行核心控制逻辑
     *
     * 处理流程：
     *   - 提取感兴趣区域内的树冠点
     *   - 估计切线方向并进行滤波
     *   - 更新绕树进度与树木位置估计
     *   - 根据状态机执行对应动作（作业停顿、切线对齐、绕树移动、面向树冠、停止）
     *
     * @param msg LaserScan 消息
     */
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        // 未收到里程计数据前，不发送运动指令
        if (!has_odom_) {
            publishStop();
            ROS_WARN_THROTTLE(1.0, "waiting for odometry before sending chassis commands");
            return;
        }

        std::vector<Point2D> roi_points; // 输出直角坐标系下的树冠点集 (x 向前，y 向左，单位：米)
        extractRoiPoints(msg, roi_points);

        // 有效点太少则视为丢失目标，超时后停止运动
        if (roi_points.size() < 5) {
            if ((ros::Time::now() - last_valid_scan_time_).toSec() > scan_timeout_) {
                publishStop();
                ROS_WARN_THROTTLE(1.0, "no valid canopy points, stopping chassis");
            }
            return;
        }

        last_valid_scan_time_ = ros::Time::now();

        // 计算树冠切线方向与平均距离、方位
        double current_tree_distance = 0.0;
        double canopy_bearing = 0.0;
        double tangent_angle = calculateCanopyTangent(roi_points, current_tree_distance, canopy_bearing);
        tangent_angle = filterTangentAngle(tangent_angle);   // 滑动窗口平滑

        updateOrbitProgress();                               // 更新绕树角度累计
        updateTreeEstimate(current_tree_distance, canopy_bearing); // 更新树木中心位置估计

        // 检查是否已绕树一周，是则切换为停止状态
        if (std::fabs(accumulated_orbit_angle_) >= 2.0 * M_PI) {
            state_ = MotionState::STOPPED;
        }

        if (state_ == MotionState::STOPPED) {
            publishStop();
            ROS_INFO_THROTTLE(1.0, "绕行完成，底盘已停止。累计绕行角度=%.1f 度",
                              accumulated_orbit_angle_ * 180.0 / M_PI);
            publishMarker(0.0);
            return;
        }

        // 作业停顿状态：模拟采收/喷药，保持停止，计时结束后转入切线对齐
        if (state_ == MotionState::WORK_PAUSE) {
            controlWorkPause();
            return;
        }

        // 切线对齐状态：原地旋转使树冠位于侧面，以便开始绕树移动
        if (state_ == MotionState::ALIGN_TANGENT) {
            controlAlignTangent(canopy_bearing);
            return;
        }

        // 确保移动片段起点有效（异常情况恢复）
        if (!segment_start_set_) {
            resetSegmentStart();
        }

        // 计算从片段起点到当前位置的移动距离
        const double segment_distance = std::hypot(current_x_ - segment_start_x_,
                                                   current_y_ - segment_start_y_);

        // 状态切换：完成一个步长后转入“面向树冠”状态
        if (state_ == MotionState::MOVE_AROUND_CANOPY && segment_distance >= step_distance_) {
            state_ = MotionState::FACE_CANOPY;
            state_enter_time_ = ros::Time::now();
            publishStop();  // 先停止再调整朝向
            ROS_INFO("步长移动完成：%.2fm，正在转向面朝树冠", segment_distance);
            return;
        }

        // 执行当前状态对应的控制逻辑
        if (state_ == MotionState::FACE_CANOPY) {
            controlFaceCanopy(canopy_bearing);          // 调整机器人朝向树冠
            return;
        }

        // 绕树移动状态
        controlMoveAroundCanopy(tangent_angle, canopy_bearing, current_tree_distance);
    }

    /**
     * @brief 从激光扫描中提取感兴趣区域（ROI）内的树冠点
     *
     * 根据实际安装方式，激光雷达正对树冠，因此取雷达正前方一个对称扇区：
     *   -roi_angle_limit_rad_ ~ +roi_angle_limit_rad_
     *
     * 这种设计使树冠点稳定出现在雷达正前方，为后续切线估计和方位角计算提供高质量点集。
     *
     * @param msg        输入激光扫描数据
     * @param roi_points 输出直角坐标系下的树冠点集 (x 向前，y 向左，单位：米)
     */
    void extractRoiPoints(const sensor_msgs::LaserScan::ConstPtr& msg,
                          std::vector<Point2D>& roi_points) const {
        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            const double r = msg->ranges[i];
            // 过滤无效测量值：无限远、NaN、过近（<0.2m 可能为机器人自身）或过远（>2.5m 超出树冠范围）
            if (std::isinf(r) || std::isnan(r) || r < 0.2 || r > 2.5) continue;

            // 计算当前测量点的角度（弧度，坐标系通常为：x 向前，y 向左，角度从 x 轴逆时针为正）
            const double angle = msg->angle_min + static_cast<double>(i) * msg->angle_increment;

            // 选取雷达正前方对称扇区内的点
            if (std::fabs(angle) < roi_angle_limit_rad_) {
                // 极坐标 → 直角坐标：x = r·cos(θ), y = r·sin(θ)
                roi_points.push_back({r * std::cos(angle), r * std::sin(angle)});
            }
        }
    }

    /**
     * @brief 对切线角度进行限幅、滑动窗口平滑和滞后激活滤波
     *
     * 该滤波器的设计目标是：抑制噪声引起的切线方向抖动，并在角度严重偏离
     * 预期范围时及时切断输出（返回 0），防止错误引导。
     *
     * 滤波流程：
     *   1. 硬窗口限幅（单位：度）：
     *      - 顺时针：仅接受 [-80°, -20°] 内的角度
     *      - 逆时针：仅接受 [20°, 80°] 内的角度
     *      若原始角度超出硬窗口，强制置零（视为无效）。
     *
     *   2. 滑动窗口平均：将（可能已置零的）角度存入历史队列，取队列均值，
     *      得到平滑后的切线角度 tangent_angle。
     *
     *   3. 滞回激活管理（is_window_active_）：
     *      引入缓冲量 buffer（默认 4°），避免在窗口边界频繁切换激活状态。
     *      - 若当前处于**激活**状态：只有当平滑角度超出 `[hard_limit ∓ buffer]`
     *        宽松窗口时，才退出激活（视为丢失切线）。
     *      - 若当前处于**非激活**状态：只有当平滑角度进入 `[hard_limit ± buffer]`
     *        严格窗口时，才激活输出。
     *
     *   4. 最终输出：若 is_window_active_ 为 true，返回平滑后的角度；
     *      否则返回 0.0（表示未锁定有效切线方向）。
     *
     * @param tangent_angle 原始切线角度（弧度，来自 PCA 估计，范围为 [-π/2, π/2)）
     * @return 滤波后的切线角度（弧度），若未激活则返回 0.0
     */
    double filterTangentAngle(double tangent_angle) {
        // 转换为度，便于后续直观阈值设置
        double tangent_deg = tangent_angle * 180.0 / M_PI;

        // 硬窗口边界（单位：度）
        const double lower_limit = clockwise_ ? -80.0 : 20.0;
        const double upper_limit = clockwise_ ? -20.0 : 80.0;
        const double buffer = 4.0;  // 滞回缓冲带宽度

        // 检查原始角度是否位于硬窗口内
        const bool in_hard_window = clockwise_
            ? (tangent_deg <= upper_limit && tangent_deg >= lower_limit)
            : (tangent_deg >= lower_limit && tangent_deg <= upper_limit);

        // 超出硬窗口时，将当前角度置为 0（视为无效）
        if (!in_hard_window) {
            tangent_angle = 0.0;
            tangent_deg = 0.0;
        }

        // ---- 滑动窗口平均 ----
        tangent_history_.push_back(tangent_angle);
        if (tangent_history_.size() > filter_window_size_) {
            tangent_history_.erase(tangent_history_.begin());  // 保持队列长度
        }

        double sum = 0.0;
        for (double val : tangent_history_) {
            sum += val;
        }
        tangent_angle = sum / tangent_history_.size();          // 均值平滑
        tangent_deg = tangent_angle * 180.0 / M_PI;             // 同步更新角度度数

        // ---- 滞回激活状态更新 ----
        if (is_window_active_) {
            // 处于激活状态时，检查是否超出宽松窗口
            const bool out_relaxed_window = clockwise_
                ? (tangent_deg > upper_limit + buffer || tangent_deg < lower_limit - buffer)
                : (tangent_deg < lower_limit - buffer || tangent_deg > upper_limit + buffer);
            if (out_relaxed_window) {
                is_window_active_ = false;   // 退出激活
            }
        } else {
            // 处于非激活状态时，检查是否进入严格窗口
            const bool enter_strict_window = clockwise_
                ? (tangent_deg <= upper_limit - buffer && tangent_deg >= lower_limit + buffer)
                : (tangent_deg >= lower_limit + buffer && tangent_deg <= upper_limit - buffer);
            if (enter_strict_window) {
                is_window_active_ = true;    // 激活
            }
        }

        // 返回结果：激活时输出平滑角度，否则输出 0
        return is_window_active_ ? tangent_angle : 0.0;
    }

    /**
     * @brief 更新树中心在世界坐标系下的估计位置
     *
     * 利用当前帧激光数据提供的相对距离与方位角，结合机器人里程计位姿，
     * 将树木位置投影到全局坐标系，并通过指数滑动平均（低通滤波）更新全局估计。
     * 首次调用时直接初始化，并重置绕树角度记录。
     *
     * 注意：distance 为雷达到树冠边界的距离，需加上 canopy_radius_for_orbit_
     * 才能得到雷达到树心的距离。坐标转换时需考虑激光雷达安装偏航角
     * laser_yaw_offset_rad_。
     *
     * @param distance 树冠点云平均距离（激光原点至树冠表面的距离，m）
     * @param bearing  树冠中心方位角（激光坐标系下，rad）
     */
    void updateTreeEstimate(double distance, double bearing) {
        // 将树冠边界距离修正为树心距离（假设树冠近似圆形）
        const double center_distance = distance + canopy_radius_for_orbit_;
        const double tree_x_base = center_distance * std::cos(bearing);
        const double tree_y_base = center_distance * std::sin(bearing);

        // 将激光坐标系下的树心坐标转换到世界坐标系（odom 系）
        // 注意：需使用激光雷达相对于车体的安装偏航角 laser_yaw_offset_rad_
        const double laser_yaw_odom = current_yaw_ + laser_yaw_offset_rad_;
        const double tree_x_odom = current_x_ + std::cos(laser_yaw_odom) * tree_x_base
                                            - std::sin(laser_yaw_odom) * tree_y_base;
        const double tree_y_odom = current_y_ + std::sin(laser_yaw_odom) * tree_x_base
                                            + std::cos(laser_yaw_odom) * tree_y_base;

        // 首次估计：直接赋值，并初始化绕树角度记录
        if (!has_tree_estimate_) {
            estimated_tree_x_ = tree_x_odom;
            estimated_tree_y_ = tree_y_odom;
            // 记录当前机器人相对于树中心的角度，作为绕树起始角度
            last_orbit_angle_ = std::atan2(current_y_ - estimated_tree_y_,
                                           current_x_ - estimated_tree_x_);
            accumulated_orbit_angle_ = 0.0;
            has_tree_estimate_ = true;
            return;
        }

        // 指数滑动平均滤波：使树木位置估计平滑，避免单帧噪声导致目标跳动
        estimated_tree_x_ = (1.0 - tree_filter_alpha_) * estimated_tree_x_
                            + tree_filter_alpha_ * tree_x_odom;
        estimated_tree_y_ = (1.0 - tree_filter_alpha_) * estimated_tree_y_
                            + tree_filter_alpha_ * tree_y_odom;
    }

    /**
     * @brief 更新绕树累积角度进度
     *
     * 根据当前机器人位置与估计的树木中心，计算相对于上一周期的方位角变化量，
     * 经方向过滤和最小变化阈值限制后，累加到 accumulated_orbit_angle_ 中。
     *
     * 方向过滤规则：
     *   - 顺时针绕树 (clockwise_ = true) ：仅允许角度负向变化（机器人围绕树中心顺时针转动）
     *   - 逆时针绕树 (clockwise_ = false)：仅允许角度正向变化
     *   若 delta 符号与预期方向相反，则将其置 0，防止往复摆动计入累积。
     *
     * 最小变化阈值 min_orbit_delta_rad_ ：避免因定位噪声引起微小角度波动干扰累积。
     */
    void updateOrbitProgress() {
        if (!has_tree_estimate_) return;

        // 当前机器人相对树中心的方位角（世界坐标系）
        const double orbit_angle = std::atan2(current_y_ - estimated_tree_y_,
                                              current_x_ - estimated_tree_x_);

        // 计算角度增量，并归一化到 [-π, π]
        double delta = normalizeAngle(orbit_angle - last_orbit_angle_);

        // 增量过小则忽略，避免定位噪声污染累积值
        if (std::fabs(delta) < min_orbit_delta_rad_) {
            return;
        }

        // 方向约束：仅允许符合绕树方向的角度变化
        if (clockwise_) {
            // 顺时针：角度 delta 应为负（角度值减小）
            if (delta > 0.0) delta = 0.0;
        } else {
            // 逆时针：角度 delta 应为正（角度值增大）
            if (delta < 0.0) delta = 0.0;
        }

        // 累积绕树角度并更新上一时刻角度
        accumulated_orbit_angle_ += delta;
        last_orbit_angle_ = orbit_angle;
    }

    /**
     * @brief 控制机器人沿树冠切线方向移动，同时保持与树冠的期望距离
     *
     * 核心思路：
     *   - 固定线速度 linear_speed_ 作为前进动力。
     *   - 使用树冠方位角误差 bearing_error 和距离误差 distance_error 计算
     *     转向角 target_steering_angle，使机器人保持在树冠侧面并维持期望距离。
     *   - 通过比例控制器产生角速度，实现沿树冠绕行。
     *
     * @param tangent_angle        树冠轮廓的切线方向（机器人坐标系，rad，PCA 估计结果，仅用于日志显示）
     * @param canopy_bearing       树冠中心在激光坐标系下的方位角 (rad)
     * @param current_tree_distance 当前激光雷达测量到的树冠平均距离（m）
     */
    void controlMoveAroundCanopy(double tangent_angle, double canopy_bearing, double current_tree_distance) {
        // 期望树冠位于机器人正前方（bearing = 0），实际控制中希望树冠在侧面，但此处用 bearing_error 进行横向调整
        const double desired_bearing = 0.0;
        const double bearing_error = normalizeAngle(canopy_bearing - desired_bearing);

        // 距离误差：正值表示离树冠太远，需要靠近；负值表示太近，需要远离
        const double distance_error = current_tree_distance - target_distance_;

        // 校正角度：符号由绕树方向和距离误差共同决定
        const double correction_angle = clockwise_ ? -k_dist_ * distance_error
                                                    :  k_dist_ * distance_error;

        // 合成期望的机器人转向角度（机器人坐标系下的目标航向偏差）
        const double target_steering_angle = normalizeAngle(k_bearing_ * bearing_error + correction_angle);

        // 构造速度指令
        geometry_msgs::Twist cmd;
        cmd.linear.x = linear_speed_;                        // 固定线速度前进
        // 角速度由目标转向角经过比例控制并限幅得到
        cmd.angular.z = clamp(k_turn_move_ * target_steering_angle,
                              -max_angular_speed_, max_angular_speed_);
        cmd_vel_pub_.publish(cmd);

        // 发布可视化标记（引导向量）
        publishMarker(target_steering_angle);

        // 输出调试信息
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

    /**
     * @brief 作业停顿状态控制
     *
     * 在该状态中持续发布零速度使机器人停止，模拟采收或喷药等作业动作。
     * 当停顿时间达到 work_pause_duration_ 后，切换到 ALIGN_TANGENT 状态，
     * 准备进行切线方向对齐。
     */
    void controlWorkPause() {
        publishStop();

        const double elapsed = (ros::Time::now() - state_enter_time_).toSec();
        ROS_INFO_THROTTLE(0.5, "作业停顿 %.2f/%.2fs，底盘已停止以进行模拟采摘",
                          elapsed, work_pause_duration_);

        if (elapsed >= work_pause_duration_) {
            state_ = MotionState::ALIGN_TANGENT;
            state_enter_time_ = ros::Time::now();
            ROS_INFO("作业停顿结束，正在对齐切线方向");
        }
    }

    /**
     * @brief 切线对准状态控制
     *
     * 通过原地旋转使树冠位于机器人侧面（期望 bearing 为 0，即正前方），
     * 为后续绕树移动做好准备。当朝向误差小于容差时，重置移动片段起点，
     * 并切换到 MOVE_AROUND_CANOPY 状态。
     *
     * @param canopy_bearing 树冠中心在激光坐标系中的方位角 (rad)
     */
    void controlAlignTangent(double canopy_bearing) {
        // 期望树冠位于正前方（bearing = 0），后续绕树时通过距离误差调整保持侧面距离
        const double desired_bearing = 0.0;
        const double yaw_error = normalizeAngle(canopy_bearing - desired_bearing);

        // 偏差足够小：认为对齐完成，停止运动，切换到绕树移动状态
        if (std::fabs(yaw_error) <= align_tolerance_rad_) {
            publishStop();
            resetSegmentStart();
            state_ = MotionState::MOVE_AROUND_CANOPY;
            state_enter_time_ = ros::Time::now();
            ROS_INFO("切线方向已对齐，开始移动段");
            return;
        }

        // 比例控制产生角速度，并限制在最大角速度范围内
        geometry_msgs::Twist cmd;
        cmd.angular.z = clamp(k_turn_align_ * yaw_error, -max_angular_speed_, max_angular_speed_);
        cmd_vel_pub_.publish(cmd);

        publishMarker(yaw_error);
        ROS_INFO_THROTTLE(0.5, "对齐切线 方位角=%.1f 度 期望角=%.1f 度 控制指令(w=%.2f)",
                          canopy_bearing * 180.0 / M_PI,
                          desired_bearing * 180.0 / M_PI,
                          cmd.angular.z);
    }

    /**
     * @brief 控制机器人原地旋转，使其朝向树冠中心
     *
     * 以树冠方位角作为目标偏航，通过比例控制器计算角速度。
     * 当朝向误差小于容差时，切换到作业停顿状态，并记录进入时间。
     *
     * @param canopy_bearing 树冠中心在激光坐标系中的方位角 (rad，此处直接作为偏航误差)
     *                       注意：实际使用中应补偿机器人当前偏航角，但此处简化处理。
     */
    void controlFaceCanopy(double canopy_bearing) {
        // 当前朝向与目标树冠方向的偏差（此处假设 canopy_bearing 即为相对偏航误差）
        const double yaw_error = normalizeAngle(canopy_bearing);

        // 偏差足够小：认为对齐完成，停止运动，进入作业停顿
        if (std::fabs(yaw_error) <= align_tolerance_rad_) {
            publishStop();
            state_ = MotionState::WORK_PAUSE;
            state_enter_time_ = ros::Time::now();
            ROS_INFO("已对齐树冠，进入作业停顿");
            return;
        }

        // 比例控制产生角速度，并限制在最大角速度范围内
        geometry_msgs::Twist cmd;
        cmd.angular.z = clamp(k_turn_align_ * yaw_error, -max_angular_speed_, max_angular_speed_);
        cmd_vel_pub_.publish(cmd);

        // 发布可视化引导向量（用于 RViz 调试）
        publishMarker(canopy_bearing);
        ROS_INFO_THROTTLE(0.5, "朝向调整 航向误差=%.1f 度 控制指令(w=%.2f)",
                          yaw_error * 180.0 / M_PI, cmd.angular.z);
    }

    /**
     * @brief 将当前移动片段的起点重置为机器人当前位置
     *
     * 在每次开始新的“绕树移动”阶段前调用，用于后续计算步长移动距离。
     */
    void resetSegmentStart() {
        segment_start_x_ = current_x_;
        segment_start_y_ = current_y_;
        segment_start_set_ = true;
    }

    /**
     * @brief 发布零线速度和零角速度，使机器人停止所有运动
     */
    void publishStop() {
        geometry_msgs::Twist cmd;  // 默认所有字段为 0
        cmd_vel_pub_.publish(cmd);
    }

    /**
     * @brief 发布 RViz 可视化箭头，指示当前期望的转向方向
     *
     * 在激光雷达坐标系（laser_frame_）下，从原点出发绘制一条长度为 2.0 米的绿色箭头，
     * 方向由 heading_angle 指定（弧度，0 表示正前方）。箭头生命周期为 0.2 秒，
     * 若无持续发布将自动消失，用于实时显示调试时的目标航向。
     *
     * @param heading_angle 期望的转向角度（机器人坐标系，rad），0 为正前方
     */
    void publishMarker(double heading_angle) {
        visualization_msgs::Marker arrow;
        arrow.header.frame_id = laser_frame_;     // 箭头所属坐标系
        arrow.header.stamp = ros::Time::now();    // 时间戳
        arrow.ns = "canopy_tracker";              // 命名空间，避免标记冲突
        arrow.id = 0;                             // 固定 ID，同一 ID 更新时会覆盖
        arrow.type = visualization_msgs::Marker::ARROW;   // 箭头类型
        arrow.action = visualization_msgs::Marker::ADD;   // 添加/更新标记

        // 箭头起止点：起点在激光原点，终点根据角度计算
        geometry_msgs::Point p_start;
        geometry_msgs::Point p_end;
        p_start.x = 0.0;
        p_start.y = 0.0;
        p_start.z = 0.0;
        p_end.x = 2.0 * std::cos(heading_angle);   // 箭头长度 2.0 米
        p_end.y = 2.0 * std::sin(heading_angle);
        p_end.z = 0.0;

        arrow.points.push_back(p_start);
        arrow.points.push_back(p_end);

        // 箭头尺寸：杆粗 0.05m，箭头宽 0.15m，箭头高 0.1m
        arrow.scale.x = 0.05;
        arrow.scale.y = 0.15;
        arrow.scale.z = 0.1;

        // 颜色：绿色，不透明
        arrow.color.r = 0.0;
        arrow.color.g = 1.0;
        arrow.color.b = 0.0;
        arrow.color.a = 1.0;

        // 生命周期：0.2 秒后标记自动消失（若不再发布）
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


