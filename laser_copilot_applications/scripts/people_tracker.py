#!/usr/bin/env python3

import oak_utils as oak
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped, TransformStamped
from foxglove_msgs.msg import ImageAnnotations, PointsAnnotation, TextAnnotation, Point2, Color
from sensor_msgs.msg import CompressedImage
import numpy as np
from typing import Dict, Optional
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

color_map: Dict[int, Color] = {}

def create_bbox_msg(obj: oak.TrackedObj, id: int, stamp: Time) -> PointsAnnotation:
    mkr = PointsAnnotation()
    mkr.timestamp = stamp.to_msg()
    mkr.type = PointsAnnotation.LINE_LOOP
    mkr.outline_color = color_map[id]
    mkr.thickness = 5.0
    bbox = [float(x) for x in obj.bbox]
    mkr.points = [
        Point2(x=bbox[0], y=bbox[1]),
        Point2(x=bbox[2], y=bbox[1]),
        Point2(x=bbox[2], y=bbox[3]),
        Point2(x=bbox[0], y=bbox[3]),
    ]
    return mkr

def create_text_msg(obj: oak.TrackedObj, id: int, stamp: Time) -> TextAnnotation:
    txt = TextAnnotation()
    txt.timestamp = stamp.to_msg()
    x, y, _, _ = obj.bbox
    txt.font_size = 20.0
    txt.position = Point2(x=float(x), y=float(y)+2*txt.font_size)
    txt.text = f"id: {id}\n{obj.distance:.2f}m"
    txt.text_color = Color(r=0.0, g=0.0, b=0.0, a=1.0)
    txt.background_color = Color(r=1.0, g=1.0, b=1.0, a=0.7)
    return txt

class people_tracker(Node):
    def __init__(self):
        super().__init__("people_tracker")
        self.__id: int = 0
        self.__pub_img = self.create_publisher(
            CompressedImage, "out/image", 3
        )
        self.__pub_annotation = self.create_publisher(ImageAnnotations, "out/annotations", 3)
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

    def __publish_img(self, img: np.ndarray, stamp: Time):
        result, encimg = cv2.imencode(".jpeg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 10])
        if result:
            img_msg = CompressedImage()
            img_msg.header.stamp = stamp.to_msg()
            img_msg.header.frame_id = OAK_FRAME
            img_msg.format = "jpeg"
            img_msg.data = encimg.tobytes()
            self.__pub_img.publish(img_msg)

    def __publish_annotations(self, objs: Dict[int, oak.TrackedObj], stamp: Time):
        msg = ImageAnnotations()
        for id, obj in objs.items():
            if id not in color_map:
                color = np.random.choice(range(256), size=3) / 255.0
                color_map[id] = Color(a=1.0, r=color[0], g=color[1], b=color[2])
            try:
                msg.points.append(create_bbox_msg(obj, id, stamp))
                msg.texts.append(create_text_msg(obj, id, stamp))
            except Exception as e:
                self.get_logger().error(f"failed to publish annotation: {e}")
        self.__pub_annotation.publish(msg)

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
        stamp = self.get_clock().now()
        self.__publish_img(img, stamp)
        self.__publish_annotations(pkgs, stamp)
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
