#!/usr/bin/env python3

import math
import numpy as np
import rclpy
import rclpy.qos as qos
from rclpy.node import Node
from rclpy.time import Time
from mavros_msgs.msg import PositionTarget
from mavros_msgs.srv import SetMode
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from visualization_msgs.msg import Marker
from functools import reduce


class tracker(Node):
    def __init__(self):
        super().__init__("tracker")
        self.cur_pos = Odometry()
        self.sp: PositionTarget = None
        self.sp_list: list[PositionTarget] = []
        self.filename: str = ""
        self.is_pub_visual_msg: bool = True
        self.track_distance: float = 0.5
        self.max_speed = 2.0
        self.speed_ratio = 0.8
        self.load_param()
        self.sp_list = csv2targets(self.filename)
        if self.is_pub_visual_msg:
            self.init_visual_support()
        self.pub_ctl = self.create_publisher(
            PositionTarget, "/mavros/setpoint_raw/local", 10
        )
        self.sub_odom = self.create_subscription(
            Odometry,
            "/mavros/local_position/odom",
            self.odom_cb,
            qos.qos_profile_sensor_data,
        )
        self.timer = self.create_timer(0.05, self.timer_cb)
        if self.switch_to_offboard_mode():
            self.get_logger().info("ready to takeoff")
            self.update_sp()
            self.publish_path()
        else:
            exit()

    def load_param(self):
        self.declare_parameter("setpoint_file_name", "")
        self.declare_parameter("enable_visual_msg", True)
        self.declare_parameter("track_distance", 0.5)
        self.declare_parameter("max_speed", 2.0)
        self.filename = str(self.get_parameter("setpoint_file_name").value)
        self.is_pub_visual_msg = bool(self.get_parameter("enable_visual_msg").value)
        self.track_distance = float(self.get_parameter("track_distance").value)
        self.max_speed = float(self.get_parameter("max_speed").value)

    def init_visual_support(self):
        # path that drone will follow, only need pub once
        self.pub_path_target = self.create_publisher(Marker, "out/path/target", 10)
        msg = Marker()
        msg.header.frame_id = "odom"
        msg.header.stamp = Time().to_msg()
        msg.ns = "laser_copilot_vmsg"
        msg.id = 0
        msg.type = Marker.LINE_STRIP
        msg.action = Marker.ADD
        msg.scale.x = 0.05
        msg.color.a = msg.color.g = 1.0
        for sp in self.sp_list:
            msg.points.append(sp.position)
        self.vmsg_path_tgt = msg
        # path the drone really fly
        self.pub_path_real = self.create_publisher(Path, "out/path/real", 10)
        self.vmsg_path_real = Path()
        self.vmsg_path_real.header.frame_id = "odom"
        # point the drone is going to
        self.pub_sp_target = self.create_publisher(Marker, "out/setpoint/target", 10)
        msg = Marker()
        msg.header.frame_id = "odom"
        msg.ns = "laser_copilot_vmsg"
        msg.id = 1
        msg.type = Marker.CUBE
        msg.scale.x = msg.scale.y = msg.scale.z = 0.5
        msg.color.a = msg.color.r = 1.0
        self.vmsg_sp_tgt = msg
        # speed vector, the longer the faster
        self.pub_sp_arrow = self.create_publisher(Marker, "out/setpoint/velocity", 10)
        msg = Marker()
        msg.header.frame_id = "odom"
        msg.ns = "laser_copilot_vmsg"
        msg.id = 2
        msg.type = Marker.ARROW
        msg.scale.x = 0.1
        msg.scale.y = 0.2
        msg.color.a = msg.color.r = msg.color.g = 1.0
        self.vmsg_sp_arrow = msg

    def switch_to_offboard_mode(self) -> bool:
        client = self.create_client(SetMode, "/mavros/set_mode")
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("/mavros/set_mode service not avaliable")
        req = SetMode.Request()
        req.custom_mode = "OFFBOARD"
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def is_ok_go_next(self):
        dx2 = (self.cur_pos.pose.pose.position.x - self.sp.position.x) ** 2
        dy2 = (self.cur_pos.pose.pose.position.y - self.sp.position.y) ** 2
        dz2 = (self.cur_pos.pose.pose.position.z - self.sp.position.z) ** 2
        return dx2 + dy2 + dz2 <= self.track_distance

    def update_sp(self):
        if len(self.sp_list) == 0:
            return
        if self.sp != None and not self.is_ok_go_next():
            return
        while len(self.sp_list) > 0 and self.sp_list[0].position.z < 0.3:
            self.sp_list.pop(0)
        if len(self.sp_list) > 0:
            self.sp = self.sp_list.pop(0)
        print(f"going to {self.sp.position}")

    ## publish setpoint
    def timer_cb(self):
        if self.sp == None:
            return
        p = self.cur_pos.pose.pose.position
        cur_pos = np.array([p.x, p.y, p.z])
        p = self.sp.position
        sp_pos = np.array([p.x, p.y, p.z])
        vel = sp_pos - cur_pos
        vel = (sp_pos - cur_pos) * self.max_speed / self.track_distance
        msg = PositionTarget()
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        msg.position = self.sp.position
        msg.velocity.x = self.speed_ratio * vel[0] + (1 - self.speed_ratio) * self.sp.velocity.x
        msg.velocity.y = self.speed_ratio * vel[1] + (1 - self.speed_ratio) * self.sp.velocity.y
        msg.velocity.z = self.speed_ratio * vel[2] + (1 - self.speed_ratio) * self.sp.velocity.z
        msg.yaw = self.sp.yaw
        self.sp.velocity = msg.velocity
        self.pub_ctl.publish(msg)
        self.publish_setpoint()

    ## update current position
    def odom_cb(self, msg: Odometry):
        self.cur_pos = msg
        self.update_sp()
        self.publish_path()

    def publish_path(self):
        if not self.is_pub_visual_msg:
            return
        p = PoseStamped()
        p.header.frame_id = "odom"
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose = self.cur_pos.pose.pose
        self.vmsg_path_real.poses.append(p)
        self.vmsg_path_real.header.stamp = p.header.stamp
        self.pub_path_real.publish(self.vmsg_path_real)
        self.pub_path_target.publish(self.vmsg_path_tgt)

    def publish_setpoint(self):
        if not self.is_pub_visual_msg:
            return
        # delete previous point
        self.vmsg_sp_tgt.header.stamp = self.get_clock().now().to_msg()
        # self.vmsg_sp_tgt.action = Marker.DELETE
        # self.pub_sp_target.publish(self.vmsg_sp_tgt)
        # add new point
        self.vmsg_sp_tgt.action = Marker.ADD
        self.vmsg_sp_tgt.pose.position = self.sp.position
        self.pub_sp_target.publish(self.vmsg_sp_tgt)
        # delete previous arrow
        self.vmsg_sp_arrow.header.stamp = self.get_clock().now().to_msg()
        # self.vmsg_sp_arrow.action = Marker.DELETE
        # self.pub_sp_arrow.publish(self.vmsg_sp_arrow)
        # add new arrow
        self.vmsg_sp_arrow.action = Marker.ADD
        self.vmsg_sp_arrow.points = [self.cur_pos.pose.pose.position, self.sp.position]
        self.pub_sp_arrow.publish(self.vmsg_sp_arrow)


def dist(a: Point, b: Point):
    return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)


def str2target(s: str) -> PositionTarget:
    pos = [float(x) for x in s.split(",")]
    p = Point()
    p.x = pos[0]
    p.y = pos[1]
    p.z = pos[2]
    tgt = PositionTarget()
    tgt.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
    tgt.position = p
    tgt.yaw = pos[3] / 180.0 * math.pi
    return tgt


def csv2targets(file: str) -> list[PositionTarget]:
    ret = []
    with open(file, "r") as f:
        for line in f.readlines():
            ret.append(str2target(line))
    return ret


def main(args=None):
    rclpy.init(args=args)
    node = tracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
