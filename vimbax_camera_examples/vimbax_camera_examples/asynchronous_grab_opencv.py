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

    