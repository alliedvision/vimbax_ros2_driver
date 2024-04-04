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
from vimbax_camera_msgs.srv import Status
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
def status_get_node(camera_node_action, camera_test_node_name):
    return launch.LaunchDescription(
        [
            Node(
                package="vimbax_camera_examples",
                executable="status_get",
                arguments=[f"/{camera_test_node_name}"],
                cached_output=True,
            ),
            camera_node_action,
            launch_pytest.actions.ReadyToTest(),
        ]
    )


@pytest.mark.launch(fixture=status_get_node)
def test_status_get(launch_context, camera_test_node_name, status_get_node, launch_service):

    action: Node = status_get_node.describe_sub_entities()[0]

    res: Status.Response() = call_service(
        Status,
        f"/{camera_test_node_name}/status",
        Status.Request(),
    )

    assert res is not None

    expected: str = f"Received status {res}"

    assert_clean_shutdown(launch_context, action)
    
    lines = action.get_stdout().strip().splitlines()
    assert lines[-1] == expected
