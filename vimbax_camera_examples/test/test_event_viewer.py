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
from vimbax_camera_msgs.srv import StreamStartStop
from launch_pytest.tools.process import wait_for_output_sync
from typing import List
import logging
import re
import time

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
                arguments=[f"/{camera_test_node_name}", "AcquisitionStart"],
                cached_output=True,
            ),
            camera_node_action,
            launch_pytest.actions.ReadyToTest(),
        ]
    )


@pytest.mark.launch(fixture=event_viewer_node)
def test_event_viewer(launch_context, camera_test_node_name, event_viewer_node, launch_service):

    action: Node = event_viewer_node.describe_sub_entities()[0]

    time.sleep(5.0)

    call_service(
        StreamStartStop,
        f"/{camera_test_node_name}/stream_start",
        StreamStartStop.Request(),
    )

    time.sleep(5.0)

    launch_service.shutdown()

    assert_clean_shutdown(launch_context, action)

    lines: List[str] = action.get_stdout().strip().split(sep="\n")
    
    # There has to be at least 1 line of output
    # The simtl camera does not produce extra event info -> only one output line
    assert 0 < len(lines)
