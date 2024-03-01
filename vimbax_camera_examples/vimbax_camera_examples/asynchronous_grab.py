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
import rclpy.executors
import signal

import argparse

from asyncio import Future

from sensor_msgs.msg import Image

last_frame_id = -1
frames_recv = 0
frames_missing = 0


def main():
    stop_future = Future()

    def signal_handler(signum, frame):
        if signum == signal.SIGINT:
            stop_future.set_result(None)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    parser = argparse.ArgumentParser()
    parser.add_argument("node_name")
    parser.add_argument("-i", "--info", action="store_true", help="Show frame infos")
    parser.add_argument("-c", "--count", type=int, default=0, help="Frame count until stop stream")

    (args, rosargs) = parser.parse_known_args()

    rclpy.init(args=rosargs)

    node = Node("_asynchronous_grab")

    def on_frame(msg: Image):
        global last_frame_id
        global frames_recv
        global frames_missing
        frame_id = int(msg.header.frame_id)
        missing = frame_id - last_frame_id - 1

        if frames_recv > 0:
            frames_missing += missing
        if args.info:
            if missing > 0:
                print(f"{missing} frame missing!")
            print(f"Frame id {msg.header.frame_id} Size {msg.width}x{msg.height} "
                  + f"Format {msg.encoding}")
        else:
            print(".", end='', flush=True)

        frames_recv += 1
        last_frame_id = frame_id
        if args.count > 0 and frames_recv >= args.count:
            stop_future.set_result(None)

    node.create_subscription(Image, f"{args.node_name}/image_raw", on_frame, 10)

    rclpy.spin_until_future_complete(node, stop_future)

    print(f"Received frames {frames_recv}")
    print(f"Missing frames {frames_missing}")
