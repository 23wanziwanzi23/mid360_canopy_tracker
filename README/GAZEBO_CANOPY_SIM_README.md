# Gazebo 树冠绕行仿真入门

这一步的目标不是做复杂真实农机，而是把你已经在二维仿真中跑通的控制闭环搬到 Gazebo：

```text
Gazebo 小车
  -> 发布 /Odometry

Gazebo 侧向 2D 激光雷达
  -> 发布 /scan_2d

你的 odom_listener_5_dipan
  -> 发布 /cmd_vel

Gazebo 差速驱动插件
  -> 根据 /cmd_vel 推动车辆运动
```

## 1. 你需要知道的 Gazebo 基本概念

先只记这几个：

```text
world 文件
  Gazebo 场景，例如地面、光源、树冠障碍物。

URDF/xacro 文件
  机器人模型，例如车体、轮子、雷达、坐标系、Gazebo 插件。

gazebo_ros_diff_drive
  Gazebo 的差速底盘插件，订阅 /cmd_vel，发布 /Odometry。

gazebo_ros_laser
  Gazebo 的二维激光插件，发布 sensor_msgs/LaserScan。

spawn_model
  把 robot_description 中的机器人生成到 Gazebo 世界里。
```

## 2. 新增文件

这次新增：

```text
urdf/tracked_canopy_robot.urdf.xacro
worlds/single_canopy.world
launch/gazebo_canopy_sim.launch
GAZEBO_CANOPY_SIM_README.md
```

复制到你的 ROS 包中：

```bash
mkdir -p ~/fast_lio_ws/src/mid360_processor/urdf
mkdir -p ~/fast_lio_ws/src/mid360_processor/worlds
mkdir -p ~/fast_lio_ws/src/mid360_processor/launch
```

放置位置：

```text
tracked_canopy_robot.urdf.xacro -> ~/fast_lio_ws/src/mid360_processor/urdf/tracked_canopy_robot.urdf.xacro
single_canopy.world             -> ~/fast_lio_ws/src/mid360_processor/worlds/single_canopy.world
gazebo_canopy_sim.launch        -> ~/fast_lio_ws/src/mid360_processor/launch/gazebo_canopy_sim.launch
```

## 3. 确认依赖

在 Ubuntu/ROS 里确认这些包存在：

```bash
rospack find gazebo_ros
rospack find xacro
rospack find robot_state_publisher
```

如果缺少，按你的 ROS 版本安装。比如 ROS Noetic：

```bash
sudo apt install ros-noetic-gazebo-ros ros-noetic-xacro ros-noetic-robot-state-publisher
```

ROS Melodic 则是：

```bash
sudo apt install ros-melodic-gazebo-ros ros-melodic-xacro ros-melodic-robot-state-publisher
```

## 4. package.xml 建议补充

```xml
<depend>gazebo_ros</depend>
<depend>xacro</depend>
<depend>robot_state_publisher</depend>
```

你的控制节点已有这些依赖也要保留：

```xml
<depend>roscpp</depend>
<depend>sensor_msgs</depend>
<depend>nav_msgs</depend>
<depend>geometry_msgs</depend>
<depend>tf</depend>
<depend>visualization_msgs</depend>
```

## 5. CMakeLists.txt

Gazebo 的 world、launch、urdf 文件不需要编译。

你只需要保证你的控制节点已经能编译：

```cmake
add_executable(odom_listener_5_dipan src/odom_listener_5_dipan.cpp)

target_link_libraries(odom_listener_5_dipan
  ${catkin_LIBRARIES}
)
```

如果你前面的二维仿真节点也保留，可以继续保留它们。

## 6. 编译

```bash
cd ~/fast_lio_ws
catkin_make
source devel/setup.bash
```

## 7. 启动 Gazebo 仿真

```bash
roslaunch mid360_processor gazebo_canopy_sim.launch
```

如果电脑性能一般，可以先关闭 Gazebo GUI：

```bash
roslaunch mid360_processor gazebo_canopy_sim.launch gui:=false
```

如果想先检查模型是否生成，不让仿真动：

```bash
roslaunch mid360_processor gazebo_canopy_sim.launch paused:=true
```

## 8. RViz 检查

新开终端：

```bash
source ~/fast_lio_ws/devel/setup.bash
rviz
```

Fixed Frame：

```text
odom
```

添加：

```text
TF
RobotModel
Odometry    /Odometry
LaserScan   /scan_2d
Marker      /steering_vector
```

你应该看到：

```text
蓝色/黑色小车模型
侧向安装的绿色雷达
LaserScan 从车体右侧扫向树冠
/steering_vector 箭头
```

## 9. 话题检查

```bash
rostopic list
```

重点应该有：

```text
/cmd_vel
/Odometry
/scan_2d
/tf
/joint_states
```

检查雷达：

```bash
rostopic hz /scan_2d
rostopic echo /scan_2d --noarr
```

检查里程计：

```bash
rostopic echo /Odometry --noarr
```

检查控制输出：

```bash
rostopic echo /cmd_vel
```

## 10. 第一次实验预期

默认初始条件：

```text
树冠在世界原点 (0, 0)
机器人在 (1.70, 0)
机器人车头朝 -Y
雷达朝车体右侧，也就是朝树冠
```

正常状态流程：

```text
WORK_PAUSE 停 1 秒
ALIGN_TANGENT 确认雷达正对树冠
MOVE 沿树冠切线走 step_distance
FACE 调整雷达重新正对树冠
WORK_PAUSE 再停 1 秒
重复直到绕满一圈
```

## 11. 常见问题

### Gazebo 打不开或很卡

先用：

```bash
roslaunch mid360_processor gazebo_canopy_sim.launch gui:=false
```

只用 RViz 看数据。

### 找不到 gazebo_ros

```bash
rospack find gazebo_ros
```

如果找不到，说明 Gazebo ROS 插件没装。

### 找不到 xacro

```bash
rospack find xacro
```

找不到就安装对应 ROS 版本的 xacro。

### 没有 /scan_2d

检查：

```bash
rostopic list | grep scan
```

如果没有，优先看 Gazebo 终端有没有 `libgazebo_ros_laser.so` 加载错误。

### 小车不动

检查：

```bash
rostopic echo /cmd_vel
```

如果 `/cmd_vel` 有速度但 Gazebo 小车不动，可能是 diff_drive 插件或轮子 joint 配置问题。

如果 `/cmd_vel` 一直是 0，说明控制节点没有识别到树冠，继续检查 `/scan_2d`。

### 小车方向不对

当前模型假设：

```text
base_link x：车头前进方向
livox_frame x：雷达正前方
livox_frame 相对 base_link 旋转 -90 度
```

如果你真实安装是雷达朝车体左侧，应把 launch 参数改成：

```bash
roslaunch mid360_processor gazebo_canopy_sim.launch laser_yaw_offset_deg:=90.0
```

同时 URDF 里的 `livox_joint` 也需要改成 `+1.57079632679`，这属于后续左侧安装版本。

## 12. 下一步

如果这个 Gazebo 最小仿真跑通，下一步再做：

```text
1. 树冠圆柱 -> 不规则树冠模型
2. 单棵树 -> 多棵树果园行
3. 差速外观履带 -> 更真实的履带模型
4. 理想平地 -> 果园地形起伏
5. 直接 /cmd_vel -> 接入你的 H7 通信节点做半实物测试
```
