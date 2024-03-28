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

from threading import Event

from vimbax_camera_msgs.srv import FeatureEnumInfoGet
from vimbax_camera_msgs.srv import FeatureCommandRun

from vimbax_camera_msgs.msg import EventData

from vimbax_camera_events.event_subscriber import EventSubscriber

from conftest import vimbax_camera_node, TestNode


@pytest.mark.launch(fixture=vimbax_camera_node)
def test_genicam_events(test_node: TestNode, launch_context):
    enum_info_service = test_node.create_client(
        FeatureEnumInfoGet, f"{test_node.camera_node_name()}/features/enum_info_get"
    )
    assert enum_info_service.wait_for_service(10)
    command_run_service = test_node.create_client(
        FeatureCommandRun, f"{test_node.camera_node_name()}/features/command_run"
    )
    assert command_run_service.wait_for_service(10)

    enum_info_request = FeatureEnumInfoGet.Request(feature_name="EventSelector")
    enum_info_response = enum_info_service.call(enum_info_request)

    if "Test" not in enum_info_response.available_values:
        pytest.skip("Test event not supported")

    subscriber = EventSubscriber(EventData, test_node, f"{test_node.camera_node_name()}/events")

    on_event_event = Event()

    def on_event(data):
        on_event_event.set()

    subscriber.subscribe_event("Test", on_event)
    future = command_run_service.call_async(
        FeatureCommandRun.Request(feature_name="TestEventGenerate")
    )
    rclpy.spin_until_future_complete(node=test_node, future=future, timeout_sec=10.0)
    assert future.done(), "Service call did not complete in time"
    run_result = future.result()

    assert run_result.error.code == 0
    assert on_event_event.wait(1)
