# laser_copilot

一些基于激光的辅助飞行的小工具/算法

## obj_dist

根据 px4 的[防碰撞功能](https://github.com/mavlink/mavros/blob/3e1adc7ec10c8b7dd819b90e259d5cd521c46228/mavros_extras/src/plugins/obstacle_distance.cpp#L86)功能编写，基于原始激光数据，计算最近的一圈激光点的距离，并发布成为 `sensor_msgs::LaserScan`
