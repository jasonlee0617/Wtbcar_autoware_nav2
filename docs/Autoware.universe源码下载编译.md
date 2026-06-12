## 1.3 工控机编译：清理旧环境并重新编译

适用于 WVCSC 工控机 `/home/eisa/autoware` 已经编译过，但中途出现过卡死、中断、CMake 缓存污染、包残缺安装等情况。

### 1.3.1 清理旧编译产物

只删除编译产物，不删除源码：

```bash
cd /home/eisa/autoware

# 确认没有正在编译的进程
ps -ef | grep -E "colcon|cmake|gmake|make|ninja|cc1plus" | grep -v grep

# 清理旧 build/install/log
rm -rf build install log
```

不要删除：

```text
/home/eisa/autoware/src
/home/eisa/autoware/repositories
/home/eisa/autoware_data
```

### 1.3.2 检查 swap

工控机编译 Autoware 容易因为内存压力导致桌面卡住或编译中断。建议至少保证 30G 以上总内存加 swap。

```bash
free -h
swapon --show
```

如果还没有额外 swapfile，可以追加 16G：

```bash
sudo fallocate -l 16G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
free -h
```

如果 `/swapfile` 已经存在，不要重复创建，只检查：

```bash
swapon --show
free -h
```

### 1.3.3 工控机正式编译命令

按当前要求，工控机也使用下面这个主编译命令：

```
git clone https://github.com/autowarefoundation/autoware.git 
cd autoware
git checkout 1.8.0
ansible-playbook autoware.dev_env.install_dev_env --skip-tags nvidia
cd autoware 
mkdir -p src 
vcs import src < repositories/autoware.repos
sudo apt update && sudo apt upgrade rosdep update rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### 1.3.4 编译完成验证

```
cd /home/eisa/autoware
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 pkg prefix autoware_launch
ros2 pkg prefix tier4_localization_launch
ros2 pkg prefix tier4_perception_launch
ros2 pkg prefix autoware_path_optimizer
```


官方 planning simulator 参数验证：

```
ros2 launch autoware_launch planning_simulator.launch.xml --show-args
```

官方示例运行：

```bash
ros2 launch autoware_launch planning_simulator.launch.xml \
  map_path:=$HOME/autoware_data/maps/sample-map-planning \
  vehicle_model:=sample_vehicle \
  sensor_model:=sample_sensor_kit
```

WVCSC 工作区运行前 source 顺序：

```bash
cd /home/eisa/Wtbcar_autoware_nav2
source /opt/ros/humble/setup.bash
source /home/eisa/autoware/install/setup.bash
source install/setup.bash
```

然后再启动 WVCSC 实车链，例如：

```bash
ros2 launch wvcsc_autoware_bringup hybrid_real_vehicle.launch.xml \
  map_path:=/home/eisa/autoware_data/maps/wvcsc_map \
  rviz:=true
