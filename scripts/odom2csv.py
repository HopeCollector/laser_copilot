#!/usr/bin/env python3

import rclpy
import os
import sys
from pathlib import Path
from rclpy.node import Node
from rosbags.highlevel import AnyReader


def cvt_on_fly():
    pass


def cvt_from_bag(filename: str, topicname: str, outfilename: str):
    poses = ''
    with AnyReader([Path(filename)]) as reader:
        connections = [x for x in reader.connections if x.topic == topicname]
        for connection, timestamp, rawdata in reader.messages(connections=connections):
            msg = reader.deserialize(rawdata, connection.msgtype)
            pos = msg.pose.pose.position
            poses += f'{pos.x},{pos.y},{pos.z}\n'
    with open(outfilename, 'w') as out:
        out.write(poses)


def main(argv):
    if len(argv) == 4:
        cvt_from_bag(argv[1], argv[2], argv[3])
    else:
        cvt_on_fly()


if __name__ == "__main__":
    main(sys.argv)
