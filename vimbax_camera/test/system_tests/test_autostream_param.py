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
from rclpy.service import Service
from rclpy.subscription import Subscription
from rclpy import Future
import launch_pytest
import launch
from launch_ros import actions
from test_helper import check_error
from conftest import TestNode

from vimbax_camera_msgs.srv import StreamStartStop
from vimbax_camera_msgs.srv import Status
from sensor_msgs.msg import Image

import time


# Fixture to launch the vimbax_camera_node
@launch_pytest.fixture
def camera_node_with_autostream(camera_test_node_name):
    """Launch the vimbax_camera_node."""
    return launch.LaunchDescription(
        [
            actions.Node(
                package="vimbax_camera",
                executable="vimbax_camera_node",
                name=camera_test_node_name,
                namespace=camera_test_node_name,
                parameters=[{"autostream": 1}],
            ),
            # Tell launch when to start the test
            # If no ReadyToTest action is added, one will be appended automatically.
            launch_pytest.actions.ReadyToTest(),
        ]
    )


@launch_pytest.fixture
def camera_node_without_autostream(camera_test_node_name):
    """Launch the vimbax_camera_node."""
    return launch.LaunchDescription(
        [
            actions.Node(
                package="vimbax_camera",
                executable="vimbax_camera_node",
                name=camera_test_node_name,
                namespace=camera_test_node_name,
                parameters=[{"autostream": 0}],
            ),
            # Tell launch when to start the test
            # If no ReadyToTest action is added, one will be appended automatically.
            launch_pytest.actions.ReadyToTest(),
        ]
    )


class StreamAutostreamTestNode(TestNode):
    """Custom ROS2 Node to make testing easier."""

    def __init__(self, name: str, cam_node_name: str, timeout_sec: float = 10.0):
        super().__init__(name, cam_node_name)
        self.__camera_node_name = cam_node_name
        self.__rcl_timeout_sec = float(timeout_sec)
        self.__stream_start_srv: Service = self.create_client(
            srv_type=StreamStartStop, srv_name=f"/{cam_node_name}/stream_start"
        )
        self.__stream_stop_srv: Service = self.create_client(
            srv_type=StreamStartStop, srv_name=f"/{cam_node_name}/stream_stop"
        )
        self.__status_srv: Service = self.create_client(
            srv_type=Status, srv_name=f"/{cam_node_name}/status"
        )
        self.__image_future: Future = Future()
        self.__image_sub: Subscription = None

        assert self.__stream_start_srv.wait_for_service(timeout_sec=self.__rcl_timeout_sec)
        assert self.__stream_stop_srv.wait_for_service(timeout_sec=self.__rcl_timeout_sec)
        assert self.__status_srv.wait_for_service(timeout_sec=self.__rcl_timeout_sec)

    def stop_stream(self) -> StreamStartStop.Response:
        return self.call_service_sync(self.__stream_stop_srv, StreamStartStop.Request())

    def start_stream(self) -> StreamStartStop.Response:
        return self.call_service_sync(self.__stream_start_srv, StreamStartStop.Request())

    def is_streaming(self) -> bool:
        res: Status.Response = self.call_service_sync(self.__status_srv, Status.Request())
        check_error(res.error)
        return res.streaming

    def get_latest_image(self) -> Image:
        self.clear_queue()
        try:
            return self.wait_for_frame(self.__rcl_timeout_sec)
        except Exception:
            return None

    def wait_until_streaming_is(self, expected: bool, timeout_sec: float = None) -> bool:
        if timeout_sec is None:
            timeout_sec = self.__rcl_timeout_sec

        t0 = time.time()

        while (time.time() - t0) <= timeout_sec:
            val = self.is_streaming()
            if val == expected:
                return True
        return False


@pytest.fixture(autouse=True)
def init_and_shutdown_ros():
    rclpy.init()
    yield
    rclpy.shutdown()


# Verify that node.is_streaming works as intended
@pytest.mark.launch(fixture=camera_node_without_autostream)
def test_streaming_status_attribute(launch_context, camera_test_node_name, node_test_id):
    node = StreamAutostreamTestNode(f"_test_node_{node_test_id}", camera_test_node_name)

    node.subscribe_image_raw()

    assert node.wait_until_streaming_is(False)
    img = node.get_latest_image()
    assert img is None

    check_error(node.start_stream().error)

    assert node.wait_until_streaming_is(True)
    img = node.get_latest_image()
    assert img is not None

    check_error(node.stop_stream().error)

    assert node.wait_until_streaming_is(False)
    img = node.get_latest_image()
    assert img is None


# Verify node starts automatically streaming when autostream is enabled
@pytest.mark.launch(fixture=camera_node_with_autostream)
def test_autostream_enabled(launch_context, camera_test_node_name, node_test_id):

    # Detecting the graph change can take quite a lot of time therefore timeout needs to be large
    node = StreamAutostreamTestNode(f"_test_node_{node_test_id}", camera_test_node_name)

    assert node.wait_until_streaming_is(False)

    node.subscribe_image_raw()

    # The node needs some time to detect the graph change and start the camera stream
    # node.is_streaming() leeds to race conditions because the service can be called
    # while the streaming starts. Therefore wait for images with the timeout of the node
    img = node.get_latest_image()
    assert img is not None

    node.unsubscribe_image_raw()

    # The camera should stop streaming
    assert node.wait_until_streaming_is(False)


# Verify node keeps streaming when one of multiple subs unsubscribes
@pytest.mark.launch(fixture=camera_node_with_autostream)
def test_autostream_enabled_multiple_subscribers(
    launch_context, camera_test_node_name, node_test_id
):

    # Detecting the graph change can take quite a lot of time therefore timeout needs to be large
    node = StreamAutostreamTestNode(f"_test_node_{node_test_id}", camera_test_node_name)

    assert node.wait_until_streaming_is(False)

    node.subscribe_image_raw()

    # The node needs some time to detect the graph change and start the camera stream
    # node.is_streaming() leeds to race conditions because the service can be called
    # while the streaming starts. Therefore wait for images with the timeout of the node
    img = node.get_latest_image()
    assert img is not None
    assert node.wait_until_streaming_is(True)

    def discard(msg):
        pass

    second_sub = node.create_subscription(
        Image,
        f"/{camera_test_node_name}/image_raw",
        discard,
        0,
    )

    assert second_sub is not None

    img = node.get_latest_image()
    assert img is not None

    assert node.wait_until_streaming_is(True)

    node.unsubscribe_image_raw()

    assert node.wait_until_streaming_is(True)

    assert node.destroy_subscription(second_sub)

    # The camera should stop streaming
    assert node.wait_until_streaming_is(False)


# Verify node starts streaming when unsubscribing and subscribing multiple times
@pytest.mark.launch(fixture=camera_node_with_autostream)
def test_autostream_enabled_sub_unsub_repeat(launch_context, camera_test_node_name, node_test_id):

    node = StreamAutostreamTestNode(f"_test_node_{node_test_id}", camera_test_node_name)

    for i in range(10):
        node.subscribe_image_raw()

        # The node needs some time to detect the graph change and start the camera stream
        # node.is_streaming() leeds to race conditions because the service can be called
        # while the streaming starts. Therefore wait for images with the timeout of the node
        img = node.get_latest_image()
        assert img is not None, f"No image received in iteration {i}"

        node.unsubscribe_image_raw()

        assert node.wait_until_streaming_is(False), f"Node did not stop streaming in iteration {i}"


# Verify that the node starts streaming after the StreamStart service is called
@pytest.mark.launch(fixture=camera_node_without_autostream)
def test_autostream_disabled(launch_context, camera_test_node_name, node_test_id):

    node = StreamAutostreamTestNode(
        f"_test_node_{node_test_id}", camera_test_node_name, timeout_sec=5.0
    )

    assert node.wait_until_streaming_is(False)

    node.subscribe_image_raw()

    # The node should not start streaming images automatically
    assert node.wait_until_streaming_is(False)

    check_error(node.start_stream().error)

    # Now the node should stream
    assert node.wait_until_streaming_is(True)

    check_error(node.stop_stream().error)

    assert node.wait_until_streaming_is(False)

    node.unsubscribe_image_raw()

    assert node.wait_until_streaming_is(False)


# Verify that streaming works when subscribing and unsubscibing
@pytest.mark.launch(fixture=camera_node_without_autostream)
def test_autostream_disabled_sub_unsub_repeat(launch_context, camera_test_node_name, node_test_id):

    node = StreamAutostreamTestNode(f"_test_node_{node_test_id}", camera_test_node_name)

    for _ in range(10):
        node.subscribe_image_raw()

        # The node should not start streaming images automatically
        assert node.wait_until_streaming_is(False)

        check_error(node.start_stream().error)

        # Now the node should stream
        assert node.wait_until_streaming_is(True)
        img = node.get_latest_image()
        assert img is not None

        check_error(node.stop_stream().error)

        # Now the node should again not be streaming
        assert node.wait_until_streaming_is(False)

        node.unsubscribe_image_raw()


# Verify that streaming continues when unsubscribing and resubscibing
@pytest.mark.launch(fixture=camera_node_without_autostream)
def test_autostream_disabled_continue_stream_after_unsub(
    launch_context, camera_test_node_name, node_test_id
):

    node = StreamAutostreamTestNode(f"_test_node_{node_test_id}", camera_test_node_name)

    check_error(node.start_stream().error)

    assert node.wait_until_streaming_is(True)

    node.subscribe_image_raw()

    assert node.wait_until_streaming_is(True)

    node.unsubscribe_image_raw()

    for i in range(10):

        assert node.wait_until_streaming_is(True), f"Node stopped streaming in iteration {i}"

        node.subscribe_image_raw()

        assert node.wait_until_streaming_is(True), f"Node stopped streaming in iteration {i}"

        node.unsubscribe_image_raw()

        assert node.wait_until_streaming_is(True), f"Node stopped streaming in iteration {i}"
