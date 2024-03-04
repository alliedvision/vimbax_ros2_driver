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
import rclpy.node
import rclpy.executors

import pytest


import launch_pytest
import launch
from launch_ros.actions import Node

from threading import Thread
from threading import Event

from vimbax_camera_msgs.srv import FeatureEnumInfoGet
from vimbax_camera_msgs.srv import FeatureCommandRun

from vimbax_camera_msgs.msg import EventData

from vimbax_camera_events.event_subscriber import EventSubscriber


camera_test_node_name = "vimbax_camera_pytest"


class TestNode(rclpy.node.Node):
    __test__ = False

    def __init__(self, name="_test_node"):
        rclpy.node.Node.__init__(self, name)

        self.ros_spin_thread = Thread(target=lambda node: rclpy.spin(node), args=(self, ))
        self.ros_spin_thread.start()


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
    return TestNode()


@pytest.mark.launch(fixture=vimbax_camera_node)
def test_genicam_events(test_node: TestNode, launch_context):
    try: 
        enum_info_service = test_node.create_client(
            FeatureEnumInfoGet, f"{camera_test_node_name}/features/enum_info_get"
        )
        assert enum_info_service.wait_for_service(10)
        command_run_service = test_node.create_client(
            FeatureCommandRun, f"{camera_test_node_name}/features/command_run"
        )
        assert command_run_service.wait_for_service(10)

        enum_info_request = FeatureEnumInfoGet.Request(feature_name="EventSelector")
        enum_info_response = enum_info_service.call(enum_info_request)

        
        if "Test" not in enum_info_response.available_values:
            pytest.skip("Test event not supported")

        subscriber = EventSubscriber(EventData, test_node, f"{camera_test_node_name}/events")

        on_event_event = Event()

        def on_event(data):
            on_event_event.set()

        subscribtion = subscriber.subscribe_event("Test", on_event)
        run_result = command_run_service.call(
            FeatureCommandRun.Request(feature_name="TestEventGenerate")
        )
        assert run_result.error.code == 0
        assert on_event_event.wait(1)
       
        
    finally:
        rclpy.shutdown()
