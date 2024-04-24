# Copyright (c) 2024 Allied Vision Technologies GmbH. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Allied Vision Technologies GmbH nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


import rclpy
from rclpy.node import Node
from rclpy.qos_event import SubscriptionEventCallbacks
import rclpy.executors
import signal
from .helper import build_topic_path

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

    node = Node("vimbax_asynchronous_grab_example")

    def on_frame(msg: Image):
        global frames_recv

        if args.info:
            print(
                f"Frame id {msg.header.frame_id} Size {msg.width}x{msg.height} "
                + f"Format {msg.encoding}"
            )
        else:
            print(".", end="", flush=True)

        frames_recv += 1
        if args.count > 0 and frames_recv >= args.count:
            stop_future.set_result(None)

    # Build topic path from namespace and topic name
    topic: str = build_topic_path(args.node_namespace, 'image_raw')

    def on_message_lost(message_lost_status):
        global lost_frames
        print(f"Dropped {message_lost_status.total_count_change} frames")
        lost_frames = lost_frames + message_lost_status.total_count_change

    event_callbacks = SubscriptionEventCallbacks(message_lost=on_message_lost)
    node.create_subscription(Image, topic, on_frame, 10,
                             event_callbacks=event_callbacks)

    rclpy.spin_until_future_complete(node, stop_future)

    if not args.info:
        print()
    print(f"Received frames {frames_recv}")
    print(f"Dropped {lost_frames}/{lost_frames + frames_recv} frames")
