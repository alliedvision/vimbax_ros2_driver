# Copyright 2024 Allied Vision Technologies GmbH. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
import cv2
import cv_bridge

from asyncio import Future

import argparse

import signal

from sensor_msgs.msg import Image


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("node_name")

    (args, rosargs) = parser.parse_known_args()

    rclpy.init(args=rosargs)

    node = Node("_stream_opencv")

    bridge = cv_bridge.CvBridge()

    stop_future = Future()

    def signal_handler(signum, frame):
        if signum == signal.SIGINT:
            stop_future.set_result(None)

    signal.signal(signal.SIGINT, signal_handler)

    def on_frame(msg: Image):
        mat = bridge.imgmsg_to_cv2(msg, 'rgb8')
        cv2.imshow("frame", mat)
        if cv2.waitKey(1) == 0x1B:
            stop_future.set_result(None)

    node.create_subscription(Image, f"{args.node_name}/image_raw", on_frame, 10)

    rclpy.spin_until_future_complete(node, stop_future)
