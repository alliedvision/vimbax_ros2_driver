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
from rclpy.qos_event import SubscriptionEventCallbacks
import rclpy.executors
import signal

import argparse

from asyncio import Future

from sensor_msgs.msg import Image

frames_recv = 0
lost_frames = 0


def main():
    stop_future = Future()

    def signal_handler(signum, frame):
        if signum == signal.SIGINT:
            stop_future.set_result(None)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    parser = argparse.ArgumentParser()
    parser.add_argument("node_namespace")
    parser.add_argument("-i", "--info", action="store_true", help="Show frame infos")
    parser.add_argument("-c", "--count", type=int, default=0, help="Frame count until stop stream")

    (args, rosargs) = parser.parse_known_args()

    rclpy.init(args=rosargs)

    node = Node("_asynchronous_grab")

    def on_frame(msg: Image):
        global frames_recv

        if args.info:
            print(f"Frame id {msg.header.frame_id} Size {msg.width}x{msg.height} "
                  + f"Format {msg.encoding}")
        else:
            print(".", end='', flush=True)

        frames_recv += 1
        if args.count > 0 and frames_recv >= args.count:
            stop_future.set_result(None)

    def on_message_lost(message_lost_status):
        global lost_frames
        print(f"Dropped {message_lost_status.total_count_change} frames")
        lost_frames = lost_frames + message_lost_status.total_count_change

    event_callbacks = SubscriptionEventCallbacks(message_lost=on_message_lost)
    node.create_subscription(Image, f"{args.node_namespace}/image_raw", on_frame, 10,
                             event_callbacks=event_callbacks)

    rclpy.spin_until_future_complete(node, stop_future)

    print(f"\nReceived frames {frames_recv}")
    print(f"Dropped {lost_frames}/{lost_frames + frames_recv} frames")
