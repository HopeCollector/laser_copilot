#!/usr/bin/env python3

import os
import rclpy
import rclpy.qos as qos
from rclpy.node import Node
from mavros_msgs.msg import PositionTarget
from mavros_msgs.srv import SetMode
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from typing import List


class tracker(Node):
    sp: PositionTarget
    cur_pos: Odometry
    sp_list: List[PositionTarget]

    def __init__(self):
        super().__init__("tracker")
        self.pub = self.create_publisher(
            PositionTarget, "/mavros/setpoint_raw/local", 10
        )
        self.sub_odom = self.create_subscription(
            Odometry,
            "/mavros/local_position/odom",
            self.odom_cb,
            qos.qos_profile_sensor_data,
        )
        self.timer = self.create_timer(0.05, self.timer_cb)
        self.declare_parameter("setpoint_file_name", "")
        filename = str(self.get_parameter("setpoint_file_name").value)
        self.sp_list = csv2targets(filename)
        self.cur_pos = Odometry()
        self.sp: PositionTarget = None
        self.update_sp()
        if self.switch_to_offboard_mode():
            self.get_logger().info("ready to takeoff")
        else:
            exit()

    def switch_to_offboard_mode(self) -> bool:
        client = self.create_client(SetMode, '/mavros/set_mode')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/mavros/set_mode service not avaliable')
        req = SetMode.Request()
        req.custom_mode = 'OFFBOARD'
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def is_ok_go_next(self):
        dx2 = (self.cur_pos.pose.pose.position.x - self.sp.position.x) ** 2
        dy2 = (self.cur_pos.pose.pose.position.y - self.sp.position.y) ** 2
        dz2 = (self.cur_pos.pose.pose.position.z - self.sp.position.z) ** 2
        return dx2 + dy2 + dz2 <= 1

    def update_sp(self):
        if len(self.sp_list) == 0:
            return
        if self.sp != None and not self.is_ok_go_next():
            return
        self.sp = self.sp_list.pop(0)
        print(f"go to next position {self.sp.position}")

    ## publish setpoint
    def timer_cb(self):
        if self.sp == None:
            return
        self.sp.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        self.pub.publish(self.sp)

    ## update current position
    def odom_cb(self, msg: Odometry):
        self.cur_pos = msg
        self.update_sp()


def main(args=None):
    rclpy.init(args=args)
    node = tracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


def xyz2target(*args) -> PositionTarget:
    xyz = []
    if len(args) == 1:
        xyz = args[0]
    elif len(args) == 3:
        xyz = args
    p = Point()
    p.x = xyz[0]
    p.y = xyz[1]
    p.z = xyz[2]
    tgt = PositionTarget()
    tgt.position = p
    return tgt


def str2target(s: str) -> PositionTarget:
    pos = [float(x) for x in s.split(",")]
    tgt = xyz2target(pos)
    return tgt


def csv2targets(file: str) -> List[PositionTarget]:
    ret = []
    with open(file, "r") as f:
        for line in f.readlines():
            ret.append(str2target(line))
    return ret


if __name__ == "__main__":
    main()
