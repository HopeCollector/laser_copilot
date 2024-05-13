#!/usr/bin/env python3

import oak_utils as oak
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import CompressedImage
import numpy as np
from typing import Dict, Optional, Tuple
import cv2
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs as tf_cvt
from pyquaternion import Quaternion
import math
import sys

ODOM_FRAME = "odom"
BASE_FRAME = "base_link"
OAK_FRAME = "oak_link"
DEG2RAD = math.pi / 180.0
CAM_INSTALL_DEG = 36.0
CAM_INSTALL_RAD = CAM_INSTALL_DEG * DEG2RAD
CAM_INSTALL_ROT_AXIS = [0, 1, 0]
CAM_INSTALL_Q = Quaternion(axis=CAM_INSTALL_ROT_AXIS, angle=CAM_INSTALL_RAD)
CAM_INSTALL_TX = 0.0
CAM_INSTALL_TY = 0.0
CAM_INSTALL_TZ = 0.0
MIN_FELLOW_DIST = 3.0

tf_msg = TransformStamped()
tf_msg.header.frame_id = BASE_FRAME
tf_msg.child_frame_id = OAK_FRAME
tf_msg.transform.translation.x = CAM_INSTALL_TX
tf_msg.transform.translation.y = CAM_INSTALL_TY
tf_msg.transform.translation.z = CAM_INSTALL_TZ
tf_msg.transform.rotation.x = CAM_INSTALL_Q.x
tf_msg.transform.rotation.y = CAM_INSTALL_Q.y
tf_msg.transform.rotation.z = CAM_INSTALL_Q.z
tf_msg.transform.rotation.w = CAM_INSTALL_Q.w


class people_tracker(Node):
    def __init__(self):
        super().__init__("people_tracker")
        self.__id: int = 0
        self.__pub_img = self.create_publisher(
            CompressedImage, "out/image", 3
        )
        self.__pub_tgt = self.create_publisher(PoseStamped, "out/goal", 3)
        self.declare_parameter("track_people_id", 0)
        # Preventing oak module from reading ros parameters, causing startup failure
        sys.argv = sys.argv[0]
        self.__cam = oak.oak_camera(self.cb_oak)
        self.__cam.start()
        self.__tf_camera_to_body = StaticTransformBroadcaster(self)
        self.__tf_buffer = Buffer()
        self.__tf_listener = TransformListener(self.__tf_buffer, self)
        self.__timer_100hz = self.create_timer(0.01, self.__timer_cb_100hz)
        self.__timer_1hz = self.create_timer(1.0, self.__timer_cb_1hz)

    def __timer_cb_100hz(self):
        if not self.__cam.is_ok():
            self.get_logger().warning("oak camera is about to closed")
            return
        self.__cam.poll()

    def __timer_cb_1hz(self):
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        self.__tf_camera_to_body.sendTransform(tf_msg)
        pass

    def __publish_img(self, img: np.ndarray):
        result, encimg = cv2.imencode(".jpeg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 10])
        if result:
            img_msg = CompressedImage()
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.format = "jpeg"
            img_msg.data = encimg.tobytes()
            self.__pub_img.publish(img_msg)

    def __to_odom_frame(self, pos: PoseStamped) -> PoseStamped:
        try:
            t = self.__tf_buffer.lookup_transform(ODOM_FRAME, pos.header.frame_id, Time())
            return tf_cvt.do_transform_pose_stamped(pos, t)
        except TransformException as e:
            self.get_logger().error(
                f"failed to transform from {pos.header.frame_id} to {ODOM_FRAME}: {e}"
            )
            return None

    def __publish_target(self, obj: oak.TrackedObj):
        p = np.array(obj.pos_xyz)
        norm_target = np.linalg.norm([-p[0], -p[1], 0])
        target = p + MIN_FELLOW_DIST * norm_target
        _, _, yaw = obj.pos_ball
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = OAK_FRAME
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.position.x = target[0]
        pose_msg.pose.position.y = target[1]
        pose_msg.pose.position.z = target[2]
        q = Quaternion(axis=[0, 0, 1], angle=yaw)
        pose_msg.pose.orientation.x = q.x
        pose_msg.pose.orientation.y = q.y
        pose_msg.pose.orientation.z = q.z
        pose_msg.pose.orientation.w = q.w
        pose_msg = self.__to_odom_frame(pose_msg)
        if pose_msg is not None:
            self.__pub_tgt.publish(pose_msg)

    def cb_oak(self, pkgs: Dict[int, oak.TrackedObj], img: Optional[np.ndarray]):
        if not rclpy.ok() or img is None:
            return
        self.__publish_img(img)
        self.__id = int(self.get_parameter("track_people_id").value)
        if self.__id in pkgs:
            self.__publish_target(pkgs[self.__id])

    def destory_node(self):
        self.__cam.__del__()
        super().destroy_node()


if __name__ == "__main__":
    rclpy.init(args=sys.argv)
    node = people_tracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
