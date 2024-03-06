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


import pytest

import rclpy
import rclpy.node
import rclpy.executors

import launch_pytest
import launch

from launch_ros.actions import Node

from threading import Thread

import queue
import time

from sensor_msgs.msg import Image

camera_test_node_name = "vimbax_camera_pytest"


class TestNode(rclpy.node.Node):
    __test__ = False

    def __init__(self, name="_test_node"):
        rclpy.node.Node.__init__(self, name)
        self.image_queue = queue.Queue()

        self.ros_spin_thread = Thread(target=lambda node: rclpy.spin(node), args=(self, ))
        self.ros_spin_thread.start()

    def subscribe_image_raw(self):

        def callback(image):
            self.image_queue.put(image)

        self.image_subscribtion = self.create_subscription(
            Image, f"{camera_test_node_name}/image_raw", callback, 10
            )

    def clear_queue(self):
        while not self.image_queue.empty():
            self.image_queue.get()

    def unsubscribe_image_raw(self):
        assert self.destroy_subscription(self.image_subscribtion)
        time.sleep(0.1)
        self.clear_queue()

    def wait_for_frame(self, timeout: float) -> Image:
        return self.image_queue.get(block=True, timeout=timeout)

    def call_service_sync(self, service, request):
        return service.call(request)


@launch_pytest.fixture
def vimbax_camera_node():
    return launch.LaunchDescription([
        Node(
            package='vimbax_camera',
            # namespace='avt_vimbax',
            executable='vimbax_camera_node',
            name=camera_test_node_name
        ),
        # Tell launch when to start the test
        # If no ReadyToTest action is added, one will be appended automatically.
        launch_pytest.actions.ReadyToTest()
    ])


@pytest.fixture
def test_node():
    rclpy.init()
    yield TestNode()
    rclpy.shutdown()
