# laser_copilot

一些基于激光的辅助飞行的小工具/算法

仓库中大部分的节点都支持 compose

## 依赖

- periphery: 串口通信 https://github.com/vsergeev/c-periphery.git
- px4_msgs: 飞控通信 https://github.com/PX4/px4_msgs/tree/release/1.14
- mavros_msg: 飞控通信 https://github.com/mavlink/mavros.git
- livox_ros_driver2: 读取 livox mid360 激光数据 https://github.com/HopeCollector/livox_ros_driver2.git


## 文件结构

- laser_copilot_applications: ros2 相关的 cpp、python 文件，并提供启动单个节点的 launch 文件
- laser_copilot_bringup: 用于统合多个节点启动

## 节点

- obj_dist: 订阅激光消息，并生成一圈 72 个距离信息，用于避障
- safe_fly_controller: 接受外部发送的目标点，并进行避障飞行
- traj_replayer: 重播飞行轨迹，仅支持 `/fmu/out/vehicle_odometry`
- visualization_helper: 如果需要在仿真界面可视化的观察，需要启动这个节点以完善 tf 变换树
- **NO USE** tracker: 使用 python 实现的建议路径追踪方法，不好用，不要用
- odom2csv: 配合 tracker 使用，用于将 `nav_msgs/msg/Odometry` 转换为 csv 文件，现在没用了

## 启动脚本

每个节点都有其单独启动脚本，此外在 `laser_copilot_bringup` 项目中还有统合其他节点的启动脚本

- livox_driver.launch.py: 用于启动 livox 激光驱动的 compose 版本，需要 livox_ros_driver2 的支持
- offboard_mode.launch.py: 启动 safe_fly 节点和提供避障信息的 obj_dist 节点，并根据其他参数选择是否启动 visualization_helper 等功能
