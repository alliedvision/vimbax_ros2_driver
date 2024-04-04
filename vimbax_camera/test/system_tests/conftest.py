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

import launch_pytest
import launch

from launch.actions import ExecuteProcess

from launch_ros.actions import Node

from threading import Thread

import queue
import random
import string
import threading

from sensor_msgs.msg import Image

from vimbax_camera_msgs.srv import FeatureEnumSet
from vimbax_camera_msgs.srv import FeatureCommandRun


class TestNode(rclpy.node.Node):
    __test__ = False

    def __init__(self, name, camera_node_name, timeout_sec: float = 60.0):
        rclpy.node.Node.__init__(self, name)
        self.image_queue = queue.Queue()
        self._camera_node_name = camera_node_name
        self.__shutdown_future = rclpy.Future()
        self.__executor = rclpy.executors.SingleThreadedExecutor()
        self._rcl_timeout_sec = float(timeout_sec)

        # According to https://github.com/ros2/rclpy/issues/255 destroy_subscription
        # is threadsave now but we still get the same error without the try except
        def spin_thread():
            try:
                while rclpy.ok() and not self.__shutdown_future.done():
                    rclpy.spin_once(self, timeout_sec=0.1, executor=self.__executor)
            except KeyboardInterrupt:
                pass
            except rclpy.executors.ExternalShutdownException:
                pass

        self.ros_spin_thread = Thread(target=spin_thread)
        self.ros_spin_thread.start()

    def __del__(self):
        self.__shutdown_future.set_result(None)
        self.ros_spin_thread.join()

    def subscribe_image_raw(self):

        def callback(image):
            self.image_queue.put(image)

        self.image_subscribtion = self.create_subscription(
            Image, f"{self._camera_node_name}/image_raw", callback, 10
        )

    def clear_queue(self):
        while not self.image_queue.empty():
            self.image_queue.get()

    def unsubscribe_image_raw(self):
        def destroy_subscription() -> bool:
            return self.destroy_subscription(self.image_subscribtion)

        fut: rclpy.Future = self.__executor.create_task(destroy_subscription)
        done_ev = threading.Event()

        def unblock(future):
            nonlocal done_ev
            done_ev.set()

        fut.add_done_callback(unblock)

        if not fut.done():
            done_ev.wait(self._rcl_timeout_sec)

        assert fut.result()

        self.clear_queue()

    def wait_for_frame(self, timeout: float = None) -> Image:
        if timeout is None:
            timeout = self._rcl_timeout_sec
        return self.image_queue.get(block=True, timeout=timeout)

    def call_service_sync(self, service, request, timeout_sec=None):
        if timeout_sec is None:
            timeout_sec = self._rcl_timeout_sec
        event = threading.Event()

        def unblock(future):
            nonlocal event
            event.set()

        future = service.call_async(request)
        future.add_done_callback(unblock)

        if not future.done():
            event.wait(timeout_sec)

        assert future.done(), f"{type(service.srv_type).__name__} call did not complete!"

        return future.result()

    def camera_node_name(self):
        return self._camera_node_name


@pytest.fixture
def node_test_id():
    return "".join(random.choices(string.ascii_lowercase + string.digits, k=8))


@pytest.fixture
def camera_test_node_name(node_test_id):
    return f"vimbax_camera_pytest_{node_test_id}"


@launch_pytest.fixture
def vimbax_camera_node(camera_test_node_name):
    return launch.LaunchDescription(
        [
            ExecuteProcess(
                cmd=["ros2", "node", "list", "--all"],
                shell=True,
                output="both",
            ),
            Node(
                package="vimbax_camera",
                namespace=camera_test_node_name,
                executable="vimbax_camera_node",
                name=camera_test_node_name,
            ),
            # Tell launch when to start the test
            # If no ReadyToTest action is added, one will be appended automatically.
            launch_pytest.actions.ReadyToTest(),
        ]
    )


@pytest.fixture
def test_node(node_test_id, camera_test_node_name):
    if not rclpy.ok():
        rclpy.init()
    test_node = TestNode(f"_test_node_{node_test_id}", camera_test_node_name, timeout_sec=120.0)
    enum_set_client = test_node.create_client(
        FeatureEnumSet, f"{camera_test_node_name}/features/enum_set"
    )
    command_run_client = test_node.create_client(
        FeatureCommandRun, f"{camera_test_node_name}/features/command_run"
    )
    enum_set_client.wait_for_service(120)
    command_run_client.wait_for_service(120)

    test_node.call_service_sync(
        enum_set_client,
        FeatureEnumSet.Request(feature_name="UserSetSelector", value="UserSetDefault"),
    )

    # High timeout: Real cameras need long time to load userset
    test_node.call_service_sync(
        command_run_client, FeatureCommandRun.Request(feature_name="UserSetLoad")
    )

    yield test_node

    rclpy.shutdown()
