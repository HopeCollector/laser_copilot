#!/usr/bin/env python3

import rclpy
import os
import sys
from math import atan2
from pathlib import Path
from rclpy.node import Node
from rosbags.highlevel import AnyReader
from scipy.spatial.transform import Rotation


def cvt_on_fly():
    pass


def cvt_from_bag(filename: str, topicname: str, outfilename: str):
    poses = ''
    with AnyReader([Path(filename)]) as reader:
        connections = [x for x in reader.connections if x.topic == topicname]
        for connection, timestamp, rawdata in reader.messages(connections=connections):
            msg = reader.deserialize(rawdata, connection.msgtype)
            pos = msg.pose.pose.position
            ori = msg.pose.pose.orientation
            ori = Rotation.from_quat([ori.x, ori.y, ori.z, ori.w])
            roll, pitch, yaw = ori.as_euler('xyz', degrees=True)
            poses += f'{pos.x},{pos.y},{pos.z},{yaw}\n'
    with open(outfilename, 'w') as out:
        out.write(poses)


def main(argv):
    if len(argv) == 4:
        cvt_from_bag(argv[1], argv[2], argv[3])
    else:
        cvt_on_fly()


if __name__ == "__main__":
    main(sys.argv)
