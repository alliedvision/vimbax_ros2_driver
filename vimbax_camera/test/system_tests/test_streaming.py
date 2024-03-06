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

import time

from vimbax_camera_msgs.srv import StreamStartStop
from vimbax_camera_msgs.srv import Status

from conftest import vimbax_camera_node, camera_test_node_name, TestNode

from test_helper import check_error

@pytest.mark.launch(fixture=vimbax_camera_node)
def test_stream_start_stop_services(test_node: TestNode, launch_context):
    start_service = test_node.create_client(
        StreamStartStop, f"/{camera_test_node_name}/stream_start"
        )
    assert start_service.wait_for_service(10)
    stop_service = test_node.create_client(
        StreamStartStop, f"/{camera_test_node_name}/stream_stop"
        )
    assert stop_service.wait_for_service(10)


@pytest.mark.launch(fixture=vimbax_camera_node)
def test_stream_auto_stream_start_stop(test_node: TestNode, launch_context):
    status_service = test_node.create_client(Status, f"/{camera_test_node_name}/status")
    assert status_service.wait_for_service(10)

    status = test_node.call_service_sync(status_service, Status.Request())
    assert not status.streaming
    test_node.subscribe_image_raw()

    assert test_node.wait_for_frame(5.0)
    status = test_node.call_service_sync(status_service, Status.Request())
    assert status.streaming
    test_node.unsubscribe_image_raw()
    time.sleep(2.0)
    assert test_node.image_queue.empty()

    status = test_node.call_service_sync(status_service, Status.Request())
    assert not status.streaming


@pytest.mark.launch(fixture=vimbax_camera_node)
def test_streaming(test_node: TestNode, launch_context):
    test_node.subscribe_image_raw()
    assert test_node.wait_for_frame(5.0)
    frame_count = 0
    for i in range(100):
        assert test_node.wait_for_frame(0.5)
        frame_count += 1

    assert frame_count == 100

    test_node.unsubscribe_image_raw()


@pytest.mark.launch(fixture=vimbax_camera_node)
def test_stream_manual_start_stop(test_node: TestNode, launch_context):
    status_service = test_node.create_client(Status, f"/{camera_test_node_name}/status")
    assert status_service.wait_for_service(10)

    start_service = test_node.create_client(
        StreamStartStop, f"/{camera_test_node_name}/stream_start"
        )
    assert start_service.wait_for_service(1)
    stop_service = test_node.create_client(
        StreamStartStop, f"/{camera_test_node_name}/stream_stop"
        )
    assert stop_service.wait_for_service(1)

    status = test_node.call_service_sync(status_service, Status.Request())
    assert not status.streaming
    test_node.subscribe_image_raw()

    assert test_node.wait_for_frame(5.0)

    status = test_node.call_service_sync(status_service, Status.Request())
    assert status.streaming

    stop_result = stop_service.call(StreamStartStop.Request())
    check_error(stop_result.error)
    test_node.clear_queue()
    time.sleep(2.0)
    assert test_node.image_queue.empty()

    status = test_node.call_service_sync(status_service, Status.Request())
    assert not status.streaming

    start_result = start_service.call(StreamStartStop.Request())
    check_error(start_result.error)

    assert test_node.wait_for_frame(5.0)

    status = test_node.call_service_sync(status_service, Status.Request())
    assert status.streaming

    test_node.unsubscribe_image_raw()
    time.sleep(2.0)
    assert test_node.image_queue.empty()

    status = test_node.call_service_sync(status_service, Status.Request())
    assert not status.streaming
