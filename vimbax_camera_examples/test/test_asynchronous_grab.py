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
from conftest import assert_clean_shutdown
import logging

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
def async_grab_node(camera_node_action, camera_test_node_name):
    return launch.LaunchDescription(
        [
            Node(
                package="vimbax_camera_examples",
                executable="asynchronous_grab",
                arguments=["-c", "1", f"/{camera_test_node_name}"],
                cached_output=True,
            ),
            camera_node_action,
            launch_pytest.actions.ReadyToTest(),
        ]
    )


@pytest.mark.launch(fixture=async_grab_node)
def test_event_viewer(launch_context, camera_test_node_name, async_grab_node):

    action: Node = async_grab_node.describe_sub_entities()[0]

    assert_clean_shutdown(launch_context, action)

    expected = ".\nReceived frames 1\nDropped 0/1 frames"

    assert expected == action.get_stdout().strip(), "The output should match the expected!"
