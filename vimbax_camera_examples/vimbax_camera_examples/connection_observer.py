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
import vimbax_camera_msgs.srv
import argparse
from .helper import build_topic_path

from time import sleep


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("node_namespace")

    (args, rosargs) = parser.parse_known_args()

    rclpy.init(args=rosargs)

    node = Node("_connected_get")

    service_type = vimbax_camera_msgs.srv.ConnectionStatus

    # Build topic path from namespace and topic name
    topic: str = build_topic_path(args.node_namespace, '/connected')

    client = node.create_client(service_type, topic)

    if not client.wait_for_service(120.0):
        print("Service got not ready in time")
        exit(1)

    request = service_type.Request()
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    response = future.result()

    was_connected = response.connected

    while rclpy.ok():
        request = service_type.Request()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future)

        response = future.result()

        if response.connected and not was_connected:
            print("Camera connect!")
        elif not response.connected and was_connected:
            print("Camera disconnect!")

        was_connected = response.connected

        sleep(0.5)
