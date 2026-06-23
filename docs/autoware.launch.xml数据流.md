# WTB `autoware.launch.xml` 实车数据流

## 1. 正式启动命令

以后 WTB 实车统一使用官方入口启动：

```bash
source /opt/ros/humble/setup.bash
source ~/autoware/install/setup.bash
source ~/Wtbcar_autoware_nav2/install/setup.bash
ros2 launch autoware_launch autoware.launch.xml \
  map_path:=/home/eisa/autoware_map/maps/wtb_map1 \
  vehicle_model:=wtb_vehicle \
  sensor_model:=wtb_sensor_kit \
  data_path:=/home/eisa/autoware_data
```

运行前提：

- `source ~/Wtbcar_autoware_nav2/install/setup.bash` 会自动补齐：
  - `ROS_DOMAIN_ID=88`
  - `LD_LIBRARY_PATH` 中的 `/opt/acados/lib`
- 这样可以避免 ROS 2 默认 domain 串网，也能保证 `path_optimizer` 的 `acados` 依赖可加载。

## 2. 启动层次

```text
autoware.launch.xml
├── global_params.launch.py
├── pointcloud_container.launch.py
├── tier4_vehicle_component
├── tier4_system_component
├── tier4_map_component
├── tier4_sensing_component
├── tier4_localization_component
├── tier4_perception_component
├── tier4_planning_component
├── tier4_control_component
├── tier4_autoware_api_component
└── rviz2
```

WTB 现在不再额外包一层 `wtb.launch.xml` 主入口，而是直接让官方 `autoware.launch.xml` 识别：

- `vehicle_model:=wtb_vehicle`
- `sensor_model:=wtb_sensor_kit`

## 3. 主数据流

### 3.1 传感器层

```text
雷神 C16 LiDAR
  -> lslidar_driver_node
  -> /sensing/lidar/pointcloud_raw
  -> pointcloud_xyzirt_to_xyzirc
  -> /sensing/lidar/concatenated/pointcloud

FDILink IMU
  -> ahrs_driver_node
  -> /imu
  -> relay
  -> /sensing/imu/imu_data

车辆速度反馈
  /vehicle/status/velocity_status
  -> autoware_vehicle_velocity_converter
  -> /sensing/vehicle_velocity_converter/twist_with_covariance
```

修复后的关键点：

- IMU `frame_id` 现在使用 `base_link`
- IMU 已补充非零协方差
- `wtb_car` 发布的 `/car_odom` 也已补充非零协方差

### 3.2 定位层

```text
/sensing/lidar/concatenated/pointcloud
  -> crop_box_filter_measurement_range
  -> voxel_grid_downsample_filter
  -> random_downsample_filter
  -> /localization/util/downsample/pointcloud

/localization/util/downsample/pointcloud
  + /map/pointcloud_map
  -> autoware_ndt_scan_matcher
  -> /localization/pose_estimator/pose_with_covariance

/localization/pose_estimator/pose_with_covariance
  + /localization/twist_estimator/twist_with_covariance
  -> autoware_ekf_localizer
  -> /localization/kinematic_state
  -> /localization/pose_twist_fusion_filter/pose
  -> TF: map -> base_link
```

### 3.3 规划层

```text
/map/vector_map + /localization/kinematic_state
  -> mission_planner
  -> /planning/mission_planning/route

route + pose + perception
  -> behavior_path_planner
  -> behavior_velocity_planner
  -> motion_velocity_planner
  -> velocity_smoother
  -> /planning/scenario_planning/trajectory
  -> trajectory_relay
  -> /planning/trajectory
```

### 3.4 控制层

```text
/planning/trajectory + /localization/kinematic_state
  -> trajectory_follower
  -> /control/trajectory_follower/control_cmd
  -> vehicle_cmd_gate
  -> /control/command/control_cmd
```

### 3.5 WTB 底盘适配层

```text
/control/control_mode_request
  -> wtb_vehicle_interface
  -> /vehicle/status/control_mode

/control/command/control_cmd
  -> wtb_vehicle_interface
  -> /twist_cmd
  -> wtb_car
  -> CAN
  -> 底盘 ECU

底盘反馈
  -> can_bridge
  -> wtb_car
  -> /car_odom
  -> /wtb_car_message
  -> wtb_vehicle_interface
  -> /vehicle/status/velocity_status
  -> /vehicle/status/steering_status
  -> /vehicle/status/gear_status
```

## 4. 当前 WTB 关键适配点

### 4.1 LiDAR 点云格式适配

- 原始点云：`PointXYZIRT`
- Autoware NDT 期望：`PointXYZIRC`
- 适配节点：`pointcloud_xyzirt_to_xyzirc`

### 4.2 IMU 话题适配

- 驱动输出：`/imu`
- Autoware 输入：`/sensing/imu/imu_data`
- 适配方式：`topic_tools relay`

### 4.3 交通灯心跳占位

WTB 当前没有真实交通灯识别链，因此补了：

```text
traffic_light_heartbeat.py
  -> /perception/traffic_light_recognition/traffic_signals
```

作用：

- 避免系统监控长期把 `traffic_signals` 当成硬错误
- 降低 `perception` 相关假故障对自动驾驶可用性的干扰

### 4.4 控制模式接管

`wtb_vehicle_interface` 已补齐：

- `/control/control_mode_request` 服务
- `/vehicle/status/control_mode` 状态发布
- 只有进入 Autoware 控制模式后，`/control/command/control_cmd` 才会下发到底盘

## 5. 本次排障后系统行为变化

### 5.1 为什么 LLT 不再刷屏

之前：

- IMU 协方差为零或接近零
- `/car_odom` 协方差也过小
- EKF 在融合 pose/twist 时更容易出现数值退化
- 最终表现为 LLT 分解报错反复刷屏

现在：

- IMU 协方差已显式配置为非零
- 里程计协方差也补成非零
- EKF 数值条件改善
- LLT 刷屏问题消失

### 5.2 为什么 `Auto` 的关键不在 RViz

`Auto` 是否可点，核心取决于这几条链路是否同时正常：

- 定位是否完成初始化
- fail-safe 是否处于 `NORMAL`
- duplicated-node checker 是否正常
- 控制模式服务和状态链路是否完整

也就是说，`Auto` 按钮只是最终表象，不是根因。

### 5.3 为什么官方 `autoware.launch.xml` 现在可以直接作为主入口

因为本次已经把 WTB 运行所需的关键差异前移到了更底层：

- `wtb_vehicle` / `wtb_sensor_kit` 模型本身可被官方入口识别
- 运行时 env hook 已自动设置 `ROS_DOMAIN_ID=88` 和 `acados` 路径
- 控制模式服务补齐
- LiDAR / IMU / 底盘协方差与状态链修复
- 交通灯占位心跳补齐

因此现在直接使用官方启动命令即可正常进入地图、定位、规划、控制流程。

## 6. 常看话题

```text
/sensing/lidar/pointcloud_raw
/sensing/lidar/concatenated/pointcloud
/sensing/imu/imu_data
/car_odom
/localization/kinematic_state
/planning/mission_planning/route
/planning/trajectory
/control/trajectory_follower/control_cmd
/control/command/control_cmd
/vehicle/status/velocity_status
/vehicle/status/control_mode
/system/fail_safe/mrm_state
```

## 7. 相关文档

- [`Autoware启动排障与修复复盘.md`](/home/eisa/Wtbcar_autoware_nav2/src/docs/Autoware启动排障与修复复盘.md)
- [`Wtbcar_autoware_nav2实车Autoware部署.md`](/home/eisa/Wtbcar_autoware_nav2/src/docs/Wtbcar_autoware_nav2实车Autoware部署.md)
- [`autoware_map_workflow.md`](/home/eisa/Wtbcar_autoware_nav2/src/docs/autoware_map_workflow.md)

## 8. 避让绕障模块调优指南

### 8.1 先确认当前默认状态

基于当前这套 Autoware 安装，复杂场景里“避障不理想”有两个很关键的默认值需要先确认：

- 规划默认预设文件 [`default_preset.yaml`](/home/eisa/autoware/install/autoware_launch/share/autoware_launch/config/planning/preset/default_preset.yaml:9)
  - `launch_static_obstacle_avoidance: "true"`
  - `launch_avoidance_by_lane_change_module: "true"`
  - `launch_dynamic_obstacle_avoidance: "false"`
- 感知默认 launch 文件 [`tier4_perception_component.launch.xml`](/home/eisa/autoware/install/autoware_launch/share/autoware_launch/launch/components/tier4_perception_component.launch.xml:56)
  - 注释里保留了 `centerpoint`
  - 实际生效默认值是 `lidar_detection_model:=clustering`

这意味着：

- 你现在很可能主要依赖的是“静态避障 + 绕障式变道 + 障碍停/减速”
- 真正的 `dynamic_obstacle_avoidance` 可能压根没启用
- 感知默认也很可能还是传统聚类，而不是 CenterPoint

### 8.2 参数文件加载链

规划侧避障参数最终由 [`tier4_planning_component.launch.xml`](/home/eisa/autoware/install/autoware_launch/share/autoware_launch/launch/components/tier4_planning_component.launch.xml:28) 和 [`behavior_planning.launch.xml`](/home/eisa/autoware/install/tier4_planning_launch/share/tier4_planning_launch/launch/scenario_planning/lane_driving/behavior_planning/behavior_planning.launch.xml:256) 注入 `behavior_path_planner`：

- 静态避障：
  [`static_obstacle_avoidance.param.yaml`](/home/eisa/autoware/install/autoware_launch/share/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/static_obstacle_avoidance.param.yaml)
- 动态避障：
  [`dynamic_obstacle_avoidance.param.yaml`](/home/eisa/autoware/install/autoware_launch/share/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/dynamic_obstacle_avoidance.param.yaml)
- 绕障式变道：
  [`avoidance_by_lane_change.param.yaml`](/home/eisa/autoware/install/autoware_launch/share/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/avoidance_by_lane_change/avoidance_by_lane_change.param.yaml)

最终轨迹还会继续受到以下文件二次约束：

- [`path_optimizer.param.yaml`](/home/eisa/autoware/install/autoware_launch/share/autoware_launch/config/planning/scenario_planning/lane_driving/motion_planning/autoware_path_optimizer/path_optimizer.param.yaml)
- [`obstacle_stop.param.yaml`](/home/eisa/autoware/install/autoware_launch/share/autoware_launch/config/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/obstacle_stop.param.yaml)
- [`obstacle_slow_down.param.yaml`](/home/eisa/autoware/install/autoware_launch/share/autoware_launch/config/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/obstacle_slow_down.param.yaml)

所以调参建议一定按下面顺序做：

1. 先确认模块是否真的启用。
2. 再调 `behavior_path_planner` 的避障判定和横向偏移。
3. 最后再看 `obstacle_stop` / `obstacle_slow_down` / `path_optimizer` 有没有把结果改得更保守。

### 8.3 第一优先级：把动态避障真正打开

当前默认预设里 [`default_preset.yaml`](/home/eisa/autoware/install/autoware_launch/share/autoware_launch/config/planning/preset/default_preset.yaml:16) 把 `launch_dynamic_obstacle_avoidance` 设成了 `false`。

建议：

- 临时验证：直接把该值改成 `true`
- 长期做法：自定义一个新的 preset，比如 `wtb_dynamic_preset.yaml`，然后启动时使用 `planning_module_preset:=wtb_dynamic`

如果不先打开这个开关，你会看到下面这些典型现象：

- 动态目标主要靠 `obstacle_stop` / `obstacle_slow_down` 被动减速
- 对 cut-in、对向会车、横穿目标的横向避让不积极
- 看起来像“识别到了，但就是不绕”

### 8.4 静态避障 `static_obstacle_avoidance.param.yaml`

#### 8.4.1 横向安全边距

关键项见：

- `target_object.*.lateral_margin.soft_margin` [static_obstacle_avoidance.param.yaml:26](/home/eisa/autoware/install/autoware_launch/share/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/static_obstacle_avoidance.param.yaml:26)
- `target_object.*.lateral_margin.hard_margin` [static_obstacle_avoidance.param.yaml:27](/home/eisa/autoware/install/autoware_launch/share/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/static_obstacle_avoidance.param.yaml:27)
- `target_object.*.lateral_margin.hard_margin_for_parked_vehicle` [static_obstacle_avoidance.param.yaml:28](/home/eisa/autoware/install/autoware_launch/share/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/static_obstacle_avoidance.param.yaml:28)

调参逻辑：

- 绕障路径过长、横向绕得太大：
  - 先减小 `soft_margin`
  - 再视情况减小 `hard_margin`
- 车辆贴障太近、绕过时不放心：
  - 先增大 `hard_margin`
  - 对行人/自行车优先增大 `soft_margin`

建议起步值：

- 车类目标：`soft_margin` 从 `0.5` 试到 `0.3 ~ 0.4`
- 行人/自行车：尽量保留更大边距，不建议一开始就降

#### 8.4.2 触发距离和检测区域

关键项见：

- `object_check_goal_distance` [static_obstacle_avoidance.param.yaml:125](/home/eisa/autoware/install/autoware_launch/share/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/static_obstacle_avoidance.param.yaml:125)
- `detection_area.min_forward_distance` [static_obstacle_avoidance.param.yaml:133](/home/eisa/autoware/install/autoware_launch/share/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/static_obstacle_avoidance.param.yaml:133)
- `detection_area.max_forward_distance` [static_obstacle_avoidance.param.yaml:134](/home/eisa/autoware/install/autoware_launch/share/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/static_obstacle_avoidance.param.yaml:134)

调参逻辑：

- 避障触发过晚：
  - 增大 `min_forward_distance`
  - 增大 `max_forward_distance`
  - 同时增大下面的 `min_prepare_time`
- 避障很早就开始，导致绕行过长：
  - 先减小 `max_forward_distance`
  - 再减小 `object_check_goal_distance`

实车建议：

- 城市/园区低速：`max_forward_distance` 可以先从 `150` 收到 `80 ~ 100`
- 如果你已经确认触发偏晚，再把 `min_forward_distance` 从 `50` 提到 `60 ~ 80`

#### 8.4.3 模糊静止车辆策略

当前默认很保守：

- `avoidance_for_ambiguous_vehicle.policy: "manual"` [static_obstacle_avoidance.param.yaml:155](/home/eisa/autoware/install/autoware_launch/share/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/static_obstacle_avoidance.param.yaml:155)
- `avoidance_for_parking_violation_vehicle.policy: "ignore"`
- `avoidance_for_close_vehicle.policy: "ignore"`

这会直接导致：

- 路边半停半并线车辆看起来“识别到了，但不主动绕”
- 可疑停车车辆经常等人工确认或直接忽略

建议：

- 园区/封闭道路调试：`ambiguous_vehicle.policy` 先改成 `auto`
- 路边违停很多的场景：把 `parking_violation_vehicle.policy` 从 `ignore` 试到 `auto`
- 狭窄场景需要主动贴边绕行时：再考虑把 `close_vehicle.policy` 改成 `auto`

#### 8.4.4 避障准备时间和横向执行能力

关键项见：

- `soft_drivable_bound_margin` [static_obstacle_avoidance.param.yaml:248](/home/eisa/autoware/install/autoware_launch/share/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/static_obstacle_avoidance.param.yaml:248)
- `min_prepare_time` [static_obstacle_avoidance.param.yaml:259](/home/eisa/autoware/install/autoware_launch/share/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/static_obstacle_avoidance.param.yaml:259)
- `max_prepare_time` [static_obstacle_avoidance.param.yaml:260](/home/eisa/autoware/install/autoware_launch/share/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/static_obstacle_avoidance.param.yaml:260)
- `min_slow_down_speed` [static_obstacle_avoidance.param.yaml:262](/home/eisa/autoware/install/autoware_launch/share/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/static_obstacle_avoidance.param.yaml:262)

调参逻辑：

- 触发太晚：
  - 增大 `min_prepare_time` / `max_prepare_time`
- 车辆为了绕障减速过多：
  - 检查 `min_slow_down_speed`
  - 同时检查后级 `obstacle_slow_down`
- 轨迹被车道边界压得太死：
  - 适当减小 `soft_drivable_bound_margin`

#### 8.4.5 路径生成方式

关键项：

- `path_generation_method: "shift_line_base"` [static_obstacle_avoidance.param.yaml:361](/home/eisa/autoware/install/autoware_launch/share/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/autoware_behavior_path_static_obstacle_avoidance_module/static_obstacle_avoidance.param.yaml:361)

建议：

- 简单单障碍、强调可解释性：保留 `shift_line_base`
- 障碍物多、需要连续 S 形绕行：试 `both`
- 如果你想让 `path_optimizer` 更多参与复杂几何避障：试 `optimization_base` 或 `both`

注意：

- `optimization_base` 通常能让路径更自然
- 但 RTC/人工审批粒度会变弱，不如 `shift_line_base` 直观

### 8.5 动态避障 `dynamic_obstacle_avoidance.param.yaml`

#### 8.5.1 目标筛选范围

关键项见：

- `min_obj_lat_offset_to_ego_path` [dynamic_obstacle_avoidance.param.yaml:25](/home/eisa/autoware/install/autoware_launch/share/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/dynamic_obstacle_avoidance.param.yaml:25)
- `max_obj_lat_offset_to_ego_path` [dynamic_obstacle_avoidance.param.yaml:26](/home/eisa/autoware/install/autoware_launch/share/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/dynamic_obstacle_avoidance.param.yaml:26)

调参逻辑：

- 旁侧 cut-in 车辆总是太晚才被纳入：
  - 增大 `max_obj_lat_offset_to_ego_path`，先试 `1.0 -> 1.5`
- 系统对相邻车道目标过于敏感：
  - 反过来减小这个值

#### 8.5.2 cut-in / cut-out 识别灵敏度

关键项：

- `cut_in_object.min_time_to_start_cut_in` [dynamic_obstacle_avoidance.param.yaml:29](/home/eisa/autoware/install/autoware_launch/share/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/dynamic_obstacle_avoidance.param.yaml:29)

建议：

- 对并线车反应过慢：把 `1.0` 先试到 `0.5 ~ 0.7`
- 如果误报很多：再往回收

#### 8.5.3 动态绕行横向偏移能力

关键项：

- `expand_drivable_area` [dynamic_obstacle_avoidance.param.yaml:54](/home/eisa/autoware/install/autoware_launch/share/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/dynamic_obstacle_avoidance.param.yaml:54)
- `polygon_generation_method` [dynamic_obstacle_avoidance.param.yaml:55](/home/eisa/autoware/install/autoware_launch/share/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/dynamic_obstacle_avoidance.param.yaml:55)
- `lat_offset_from_obstacle` [dynamic_obstacle_avoidance.param.yaml:59](/home/eisa/autoware/install/autoware_launch/share/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/dynamic_obstacle_avoidance.param.yaml:59)
- `margin_distance_around_pedestrian` [dynamic_obstacle_avoidance.param.yaml:60](/home/eisa/autoware/install/autoware_launch/share/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/dynamic_obstacle_avoidance.param.yaml:60)
- `max_lat_offset_to_avoid` [dynamic_obstacle_avoidance.param.yaml:64](/home/eisa/autoware/install/autoware_launch/share/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/dynamic_obstacle_avoidance.param.yaml:64)

调参逻辑：

- 根本不愿意横向绕动态目标：
  - 先把 `max_lat_offset_to_avoid` 从 `0.5` 提到 `0.7 ~ 1.0`
  - 狭窄道路再考虑 `expand_drivable_area: true`
- 路径太保守、离动态目标太远：
  - 适当减小 `lat_offset_from_obstacle`
- 行人避让不够稳：
  - 增大 `margin_distance_around_pedestrian`

#### 8.5.4 横向机动时机

关键项：

- `max_ego_lat_acc` [dynamic_obstacle_avoidance.param.yaml:68](/home/eisa/autoware/install/autoware_launch/share/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/dynamic_obstacle_avoidance.param.yaml:68)
- `max_ego_lat_jerk` [dynamic_obstacle_avoidance.param.yaml:69](/home/eisa/autoware/install/autoware_launch/share/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/dynamic_obstacle_avoidance.param.yaml:69)
- `delay_time_ego_shift` [dynamic_obstacle_avoidance.param.yaml:70](/home/eisa/autoware/install/autoware_launch/share/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/autoware_behavior_path_dynamic_obstacle_avoidance_module/dynamic_obstacle_avoidance.param.yaml:70)

调参逻辑：

- 避让动作启动太晚：
  - 先把 `delay_time_ego_shift` 从 `1.0` 降到 `0.5`
  - 再小幅增大 `max_ego_lat_acc` / `max_ego_lat_jerk`
- 动作太激进、乘坐感差：
  - 反向调回

### 8.6 绕障式变道 `avoidance_by_lane_change.param.yaml`

这个模块更适合“前方障碍需要借相邻车道整体绕过”，不是细粒度贴障绕行。

关键项：

- `execute_object_longitudinal_margin` [avoidance_by_lane_change.param.yaml:4](/home/eisa/autoware/install/autoware_launch/share/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/avoidance_by_lane_change/avoidance_by_lane_change.param.yaml:4)
- `execute_only_when_lane_change_finish_before_object` [avoidance_by_lane_change.param.yaml:5](/home/eisa/autoware/install/autoware_launch/share/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/avoidance_by_lane_change/avoidance_by_lane_change.param.yaml:5)
- `target_type.bicycle/pedestrian` 默认关闭 [avoidance_by_lane_change.param.yaml:93](/home/eisa/autoware/install/autoware_launch/share/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/avoidance_by_lane_change/avoidance_by_lane_change.param.yaml:93)

建议：

- 如果它太早发起整车道绕行：
  - 把 `execute_object_longitudinal_margin` 从 `80` 收到 `40 ~ 60`
- 如果你希望只有在能完整变道后才执行：
  - 把 `execute_only_when_lane_change_finish_before_object` 改成 `true`

### 8.7 后级轨迹约束：为什么“已经想绕，但结果还是保守”

#### 8.7.1 `obstacle_stop.param.yaml`

关键项：

- `stop_margin` [obstacle_stop.param.yaml:9](/home/eisa/autoware/install/autoware_launch/share/autoware_launch/config/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/obstacle_stop.param.yaml:9)
- `terminal_stop_margin` [obstacle_stop.param.yaml:10](/home/eisa/autoware/install/autoware_launch/share/autoware_launch/config/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/obstacle_stop.param.yaml:10)
- `min_behavior_stop_margin` [obstacle_stop.param.yaml:11](/home/eisa/autoware/install/autoware_launch/share/autoware_launch/config/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/obstacle_stop.param.yaml:11)
- `crossing_obstacle.collision_time_margin` [obstacle_stop.param.yaml:105](/home/eisa/autoware/install/autoware_launch/share/autoware_launch/config/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/obstacle_stop.param.yaml:105)

调参逻辑：

- 老是过早刹停，像不愿意绕：
  - 适当减小 `stop_margin`
- 对横穿目标反应偏晚：
  - 增大 `collision_time_margin`

#### 8.7.2 `obstacle_slow_down.param.yaml`

关键项：

- `moving_object_speed_threshold` [obstacle_slow_down.param.yaml:42](/home/eisa/autoware/install/autoware_launch/share/autoware_launch/config/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/obstacle_slow_down.param.yaml:42)
- `min_lat_margin` [obstacle_slow_down.param.yaml:60](/home/eisa/autoware/install/autoware_launch/share/autoware_launch/config/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/obstacle_slow_down.param.yaml:60)
- `max_lat_margin` [obstacle_slow_down.param.yaml:61](/home/eisa/autoware/install/autoware_launch/share/autoware_launch/config/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/obstacle_slow_down.param.yaml:61)
- `successive_num_to_entry_slow_down_condition` [obstacle_slow_down.param.yaml:65](/home/eisa/autoware/install/autoware_launch/share/autoware_launch/config/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/obstacle_slow_down.param.yaml:65)

调参逻辑：

- 慢车/行人触发减速偏晚：
  - 把 `successive_num_to_entry_slow_down_condition` 从 `5` 试到 `3`
  - 必要时减小 `moving_object_speed_threshold`
- 很容易误触发减速：
  - 恢复到 `5`
  - 或适当减小 `max_lat_margin`

#### 8.7.3 `path_optimizer.param.yaml`

如果 `behavior_path_planner` 已经给出了可绕的候选，但最终轨迹还是过于贴中心线或外扩不足，要检查：

- `mpt.clearance.soft_clearance_from_road`
- `mpt.avoidance.min_drivable_width`
- `mpt.avoidance.max_avoidance_cost`

常见现象：

- `min_drivable_width` 太大，优化器会宁可不挤过去
- `soft_clearance_from_road` 太保守，轨迹会更“胖”

### 8.8 推荐实车调参顺序

建议按这个顺序做，每次只改 2 到 4 个参数：

1. 先把 `launch_dynamic_obstacle_avoidance` 打开。
2. 静态避障先调：
   - `soft_margin`
   - `max_forward_distance`
   - `min_prepare_time`
   - `ambiguous_vehicle.policy`
3. 动态避障再调：
   - `max_obj_lat_offset_to_ego_path`
   - `min_time_to_start_cut_in`
   - `max_lat_offset_to_avoid`
   - `delay_time_ego_shift`
4. 如果还是“看到目标但只会刹车”，再调：
   - `obstacle_stop.stop_margin`
   - `obstacle_slow_down.successive_num_to_entry_slow_down_condition`
5. 如果还是“路径能绕但形状不好”，最后再调 `path_optimizer`

### 8.9 建议的第一组起步修改

如果你的典型问题是“绕障偏长 + 动态目标反应慢”，建议第一轮先试下面这一组：

- `default_preset.yaml`
  - `launch_dynamic_obstacle_avoidance: "true"`
- `static_obstacle_avoidance.param.yaml`
  - 车类 `soft_margin: 0.5 -> 0.35`
  - `max_forward_distance: 150 -> 90`
  - `min_prepare_time: 1.0 -> 1.5`
  - `avoidance_for_ambiguous_vehicle.policy: "manual" -> "auto"`
- `dynamic_obstacle_avoidance.param.yaml`
  - `max_obj_lat_offset_to_ego_path: 1.0 -> 1.5`
  - `cut_in_object.min_time_to_start_cut_in: 1.0 -> 0.6`
  - `max_lat_offset_to_avoid: 0.5 -> 0.8`
  - `delay_time_ego_shift: 1.0 -> 0.5`
- `obstacle_slow_down.param.yaml`
  - `successive_num_to_entry_slow_down_condition: 5 -> 3`

## 9. `tier4_perception_component.launch.xml` 中 `lidar_detection_model` 解析

### 9.1 先纠正一个概念

你提到“雷达感知模型”，但在当前 launch 里真正控制的是 LiDAR 检测模型，不是 radar 模型。

对应文件：

- [`tier4_perception_component.launch.xml`](/home/eisa/autoware/install/autoware_launch/share/autoware_launch/launch/components/tier4_perception_component.launch.xml:56)
- [`perception.launch.xml`](/home/eisa/autoware/install/tier4_perception_launch/share/tier4_perception_launch/launch/perception.launch.xml)
- [`detection.launch.xml`](/home/eisa/autoware/install/tier4_perception_launch/share/tier4_perception_launch/launch/object_recognition/detection/detection.launch.xml:18)

当前可选 `lidar_detection_model_type` 包括：

- `clustering`
- `centerpoint`
- `pointpainting`
- `transfusion`
- `bevfusion`
- `apollo`

### 9.2 当前默认值的重要结论

当前安装里 [`tier4_perception_component.launch.xml`](/home/eisa/autoware/install/autoware_launch/share/autoware_launch/launch/components/tier4_perception_component.launch.xml:56) 实际默认值是：

```xml
<arg name="lidar_detection_model" default="clustering" />
```

也就是说，如果你没有额外覆写，感知主检测大概率不是 CenterPoint。

另一个细节是：

- `perception.launch.xml` 会把 `lidar_detection_model` 按 `type/name` 拆分
- 如果只有 `centerpoint` 没有显式模型名，那么 [`lidar_dnn_detector.launch.xml`](/home/eisa/autoware/install/tier4_perception_launch/share/tier4_perception_launch/launch/object_recognition/detection/detector/lidar_dnn_detector.launch.xml:112) 默认会落到 `centerpoint_tiny`

所以：

- `centerpoint` 等价于“类型是 centerpoint，模型名留空”
- 真正想强制用完整模型，应该用 `centerpoint/centerpoint`
- 只写 `centerpoint` 时，多半实际跑的是 `centerpoint_tiny`

### 9.3 `clustering` 的底层原理

对应链路：

- [`lidar_rule_detector.launch.xml`](/home/eisa/autoware/install/tier4_perception_launch/share/tier4_perception_launch/launch/object_recognition/detection/detector/lidar_rule_detector.launch.xml:23)
- [`voxel_grid_based_euclidean_cluster.param.yaml`](/home/eisa/autoware/install/autoware_launch/share/autoware_launch/config/perception/object_recognition/detection/clustering/voxel_grid_based_euclidean_cluster.param.yaml:3)

处理逻辑大致是：

1. 先用地面分割得到障碍点云
2. 再做 voxel grid + Euclidean clustering
3. 再经 `shape_estimation` 估计尺寸/朝向
4. 最后变成 `DetectedObjects`

核心参数：

- `tolerance: 0.7`
- `voxel_leaf_size: 0.3`
- `min_cluster_size: 10`
- `max_cluster_size: 3000`
- `use_height: false`

优点：

- 不依赖深度模型和 ONNX/TensorRT 文件
- 对近距离、大目标、规则静态障碍物比较稳
- 算力开销相对小，部署简单

缺点：

- 本质上只是在点云几何上聚类，没有语义理解
- 远距离稀疏点云容易碎裂或漏检
- 动态目标的朝向、类别、边界框质量通常不如深度学习模型
- 很依赖前级地面分割和点云质量

它很适合：

- 先把整车跑通
- GPU 紧张
- 场景以低速静态障碍为主

它不太适合：

- 希望更早识别 cut-in、会车、横穿目标
- 需要更稳定的类别和速度语义
- 远距离动态目标是主要矛盾

### 9.4 `centerpoint` 的底层原理

对应文件：

- [`centerpoint.param.yaml`](/home/eisa/autoware/install/autoware_launch/share/autoware_launch/config/perception/object_recognition/detection/lidar_model/centerpoint.param.yaml:16)
- [`centerpoint_common.param.yaml`](/home/eisa/autoware/install/autoware_launch/share/autoware_launch/config/perception/object_recognition/detection/lidar_model/centerpoint_common.param.yaml)
- [`lidar_dnn_detector.launch.xml`](/home/eisa/autoware/install/tier4_perception_launch/share/tier4_perception_launch/launch/object_recognition/detection/detector/lidar_dnn_detector.launch.xml:112)

处理逻辑大致是：

1. 把点云体素化
2. 用 voxel encoder + backbone/head 做 BEV 检测
3. 通过 NMS 输出 3D 框、类别、朝向
4. 再交给后续 validator / tracker / prediction

当前参数里可以直接看到：

- `cloud_capacity: 2000000`
- `circle_nms_dist_threshold: 0.5`
- `iou_nms_threshold: 0.1`
- `densification_params.num_past_frames: 1`

优点：

- 对车辆、行人、两轮车的检测和分类通常明显好于 clustering
- 朝向和框体更稳定，更利于跟踪与预测
- 对动态目标更友好，通常能更早给出可用于规划的目标

缺点：

- 依赖模型文件、TensorRT 和 GPU
- 受训练域影响，换激光雷达、换场景后可能需要重新验证
- 如果模型版本不对、点云分布差异大，也会误检或漏检

### 9.5 `clustering` 与 `centerpoint` 的实战对比

如果你的问题重点是：

- 绕障路径过长
- 无法有效识别动态障碍物
- 避障触发偏晚

那么在大多数情况下：

- `clustering` 更可能是瓶颈
- `centerpoint` 更值得优先尝试

对比总结：

- `clustering`
  - 优势：轻量、稳、易部署
  - 劣势：动态目标语义弱，远距离和稀疏目标效果一般
- `centerpoint`
  - 优势：动态目标检测、分类、朝向通常更强
  - 劣势：更吃模型和算力

### 9.6 为什么切到 `centerpoint` 后不一定完全替代 `clustering`

从 [`detection.launch.xml`](/home/eisa/autoware/install/tier4_perception_launch/share/tier4_perception_launch/launch/object_recognition/detection/detection.launch.xml:169) 可以看到：

- 在 `lidar` 模式下，同时打开了 `lidar_dnn` 和 `lidar_rule`
- 在 `camera_lidar_fusion` 模式下，同时打开了 `camera_lidar` 检测和 `lidar_dnn`

这意味着：

- `centerpoint` 往往是“新增一条 ML 检测链”
- 不是简单把 `clustering` 彻底关掉
- 你在 RViz 里要同时看 ML 输出、validation 输出、tracking 输出，不能只盯最终 `/perception/object_recognition/objects`

### 9.7 其他模型怎么理解

#### `pointpainting`

对应文件：

- [`pointpainting.param.yaml`](/home/eisa/autoware/install/autoware_launch/share/autoware_launch/config/perception/object_recognition/detection/lidar_model/pointpainting.param.yaml:15)

特点：

- 本质上是“相机语义 + LiDAR 检测”的联合方式
- 更依赖相机标定、图像质量和多传感器同步
- 如果你的相机链稳定，往往比纯 LiDAR `centerpoint` 更擅长类别语义

适合：

- 多相机可用、标定好
- 想提升车辆/VRU 分类稳定性

#### `transfusion`

对应文件：

- [`transfusion.param.yaml`](/home/eisa/autoware/install/autoware_launch/share/autoware_launch/config/perception/object_recognition/detection/lidar_model/transfusion.param.yaml:12)

特点：

- Transformer 风格 3D 检测
- 也带时序 densification
- 通常比 `centerpoint_tiny` 更重，但全局关系建模更强

适合：

- GPU 裕量更大
- 希望进一步提升复杂动态场景检测质量

#### `bevfusion`

特点：

- 更强的 BEV 多模态融合路线
- 对相机质量和标定更敏感
- 算力需求通常也更高

适合：

- 多摄像头稳定可用
- 追求更高的综合检测性能

### 9.8 什么时候应该切模型

建议如下：

1. 如果你现在还是 `clustering`，而主要抱怨是“动态目标识别差、避障太晚”，优先切到 `centerpoint/centerpoint` 或至少 `centerpoint`。
2. 如果 GPU 紧张，先试 `centerpoint`，因为它大概率会实际使用 `centerpoint_tiny`，验证成本较低。
3. 如果相机标定和同步很稳定，且希望进一步提升类别语义，再试 `pointpainting` 或 `bevfusion`。
4. 如果 `centerpoint` 已经够好，就不要急着上更重的 `transfusion/bevfusion`，先把 tracker、prediction、dynamic avoidance 调顺。

### 9.9 动态目标效果不佳时，感知侧还要一起看

除了切模型，下面几个文件也会直接影响动态绕障是否“看得见、跟得上、敢预测”：

- 验证器：
  [`obstacle_pointcloud_based_validator.param.yaml`](/home/eisa/autoware/install/autoware_launch/share/autoware_launch/config/perception/object_recognition/detection/detected_object_validation/obstacle_pointcloud_based_validator.param.yaml:10)
  - `validate_max_distance_m: 70.0`
  - `using_2d_validator: true`
- 跟踪器：
  [`multi_object_tracker_node.param.yaml`](/home/eisa/autoware/install/autoware_launch/share/autoware_launch/config/perception/object_recognition/tracking/multi_object_tracker/multi_object_tracker_node.param.yaml:21)
  - `publish_rate: 10.0`
  - `enable_delay_compensation: true`
  - `enable_unknown_object_velocity_estimation: true`
- 预测器：
  [`map_based_prediction.param.yaml`](/home/eisa/autoware/install/autoware_launch/share/autoware_launch/config/perception/object_recognition/prediction/map_based_prediction.param.yaml:13)
  - `min_velocity_for_map_based_prediction: 1.0`

实车建议：

- 慢速 cut-in、缓慢横穿目标预测不积极时，把 `min_velocity_for_map_based_prediction` 试到 `0.3 ~ 0.5`
- 远距离对象经常被 validator 删掉时，检查 `validate_max_distance_m`

### 9.10 建议的验证顺序

每改完一轮参数，按下面顺序看话题：

1. `ML/聚类检测`
   - `/perception/object_recognition/detection/centerpoint/objects`
   - `/perception/object_recognition/detection/clustering/objects`
2. `验证后目标`
   - `/perception/object_recognition/detection/centerpoint/validation/objects`
3. `跟踪输出`
   - `/perception/object_recognition/tracking/objects`
4. `规划最终输入`
   - `/perception/object_recognition/objects`
5. `行为规划调试`
   - `/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/info/static_obstacle_avoidance`
   - `/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/info/dynamic_obstacle_avoidance`
   - `/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/debug/static_obstacle_avoidance`

如果你愿意，下一步可以直接在 `wtb_autoware_bringup` 里做一套“WTB 动态避障 + CenterPoint”专用 preset 和 param overlay，这样就不用每次去改 `~/autoware/install` 下的文件。
