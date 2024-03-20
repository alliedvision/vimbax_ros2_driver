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
import launch_pytest
import launch
from launch_ros.actions import Node
from conftest import call_service, assert_clean_shutdown
from vimbax_camera_msgs.srv import FeatureCommandRun
from launch.actions.shutdown_action import Shutdown
import logging
import re

LOGGER = logging.getLogger(__name__)


@pytest.fixture
def camera_node_action(camera_test_node_name):
    return Node(
        package="vimbax_camera",
        executable="vimbax_camera_node",
        namespace=camera_test_node_name,
        name=camera_test_node_name,
    )


@launch_pytest.fixture
def event_viewer_node(camera_node_action, camera_test_node_name):
    return launch.LaunchDescription(
        [
            Node(
                package="vimbax_camera_examples",
                executable="event_viewer",
                arguments=[f"/{camera_test_node_name}", "Test"],
                cached_output=True,
            ),
            camera_node_action,
            launch_pytest.actions.ReadyToTest(),
        ]
    )


@pytest.mark.launch(fixture=event_viewer_node)
def test_event_viewer(launch_context, camera_test_node_name, event_viewer_node, launch_service):

    action: Node = event_viewer_node.describe_sub_entities()[0]

    call_service(
        FeatureCommandRun,
        f"/{camera_test_node_name}/features/command_run",
        FeatureCommandRun.Request(feature_name="TestEventGenerate"),
    )

    launch_service.shutdown()

    assert_clean_shutdown(launch_context, action)

    lines: List[str] = action.get_stdout().strip().split(sep='\n')
    
    assert 3 == len(lines), "The example should output 3 lines!"
    assert "Got event Test" == lines[0].strip()

    assert re.match('EventTestTimestamp: [0-9]+', lines[1].strip()) is not None

    assert "EventTest: 20479" == lines[2].strip()
