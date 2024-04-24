# Copyright (c) 2024 Allied Vision Technologies GmbH. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Allied Vision Technologies GmbH nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

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
