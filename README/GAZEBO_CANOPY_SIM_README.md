# Gazebo 树冠绕行仿真

该仿真直接复用 `odom_listener_5_dipan` 的树冠跟随状态机：Gazebo 发布 `/Odometry`
和 `/scan_2d`，控制节点发布 `/cmd_vel`。

## 仿真条件

- 车辆俯视尺寸约 `1.60 m × 1.00 m`；
- 雷达安装在车体右侧，相对车头旋转 `-90°`；
- 激光扫描为 `360°`、`1800` 点、`10 Hz`，量程 `0.10–50.00 m`；
- 树冠由多个重叠叶簇构成，俯视轮廓约 `6 m × 5 m`；
- 车辆从树冠东侧出发，雷达到树冠的初始净距离约 `0.50 m`；
- 默认目标距离为 `0.50 ± 0.05 m`，每段移动 `0.50 m`。

履带、车轮和车体保留实际比例外形。为了专注验证上层树冠跟随算法，
`/cmd_vel` 由 Gazebo 平面运动插件执行，车辆外形不参与地面碰撞，不模拟
履带与地面的复杂滑移摩擦。机器人根链接关闭重力，所有外形固定成一个
刚性平面模型；树冠碰撞边界仍保留给雷达扫描。

## 编译与启动

```bash
cd ~/fast_lio_ws
catkin_make --pkg mid360_processor
source devel/setup.bash
roslaunch mid360_processor gazebo_canopy_sim.launch
```

该 launch 默认同时打开 Gazebo 和 RViz。如果电脑性能有限：

```bash
roslaunch mid360_processor gazebo_canopy_sim.launch gui:=false
```

只看 Gazebo、不启动 RViz：

```bash
roslaunch mid360_processor gazebo_canopy_sim.launch launch_rviz:=false
```

暂停在初始状态检查模型：

```bash
roslaunch mid360_processor gazebo_canopy_sim.launch paused:=true
```

Gazebo 默认不画出 1800 条激光射线，但 `/scan_2d` 始终正常发布。需要检查射线时：

```bash
roslaunch mid360_processor gazebo_canopy_sim.launch visualize_laser:=true
```

## 正常现象

程序依次执行“作业停顿 → 移动前对准 → 绕树移动 → 正对树冠调整”。
绕树移动时，小车一边前进一边用右侧雷达跟随局部树冠，并将表面距离保持在
`target_distance`附近。默认每走完 `0.50 m` 停一次。

关键话题：

```text
/scan_2d          Gazebo 二维激光
/Odometry         Gazebo 里程计
/cmd_vel          控制节点速度指令
/steering_vector  转向目标可视化
```

## 参数在哪里改

日常调参优先修改 `launch/gazebo_canopy_sim.launch`：

- `robot_x` / `robot_y` / `robot_yaw`：初始位置和朝向；
- `target_distance`：右侧雷达与树冠的目标净距离；
- `step_distance`：两次作业停顿之间的行驶距离；
- `linear_speed` / `max_angular_speed`：速度上限；
- `heading_roi_min_deg` / `heading_roi_max_deg`：蓝色航向 ROI 扇形；
- `facing_roi_min_deg` / `facing_roi_max_deg`：橙色正对 ROI 扇形；
- `return_distance_threshold`：完成一圈时允许的返回起点距离。

雷达扫描点数、频率和量程在 `urdf/tracked_canopy_robot.urdf.xacro` 的
`side_lidar` 中修改；树冠大小和形状在 `worlds/single_canopy.world` 中修改。

## 快速检查

```bash
rostopic hz /scan_2d
rostopic echo -n 1 /Odometry --noarr
rostopic echo -n 1 /cmd_vel
```
