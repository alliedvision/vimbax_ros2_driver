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
from vimbax_camera_msgs.srv import FeatureEnumGet, FeatureEnumInfoGet, FeaturesListGet
import logging
from typing import List

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
def feature_get_node(camera_node_action, camera_test_node_name):
    return launch.LaunchDescription(
        [
            Node(
                package="vimbax_camera_examples",
                executable="feature_get",
                arguments=[f"/{camera_test_node_name}", "Enum", "PixelFormat"],
                cached_output=True,
            ),
            camera_node_action,
            launch_pytest.actions.ReadyToTest(),
        ]
    )


@pytest.mark.launch(fixture=feature_get_node)
def test_feature_get(launch_context, camera_test_node_name, feature_get_node):

    action = feature_get_node.describe_sub_entities()[0]

    service_name: str = f"/{camera_test_node_name}/features/enum_get"
    res: FeatureEnumGet.Response = call_service(
        FeatureEnumGet, service_name, FeatureEnumGet.Request(feature_name="PixelFormat")
    )
    expected: str = f"PixelFormat: {res.value}"

    assert_clean_shutdown(launch_context, action)

    lines: List = action.get_stdout().strip().splitlines()
    assert expected == lines[-1]


@launch_pytest.fixture
def feature_info_get_node(camera_node_action, camera_test_node_name):
    return launch.LaunchDescription(
        [
            Node(
                package="vimbax_camera_examples",
                executable="feature_info_get",
                arguments=[f"/{camera_test_node_name}", "Enum", "AcquisitionMode"],
                cached_output=True,
            ),
            camera_node_action,
            launch_pytest.actions.ReadyToTest(),
        ]
    )


@pytest.mark.launch(fixture=feature_info_get_node)
def test_feature_info_get(launch_context, camera_test_node_name, feature_info_get_node):

    action = feature_info_get_node.describe_sub_entities()[0]

    service_name: str = f"/{camera_test_node_name}/features/enum_info_get"
    res: FeatureEnumInfoGet.Response = call_service(
        FeatureEnumInfoGet,
        service_name,
        FeatureEnumInfoGet.Request(feature_name="AcquisitionMode"),
    )
    expected: str = f"all: {res.possible_values} available: {res.available_values}"

    assert_clean_shutdown(launch_context, action)

    lines: List = action.get_stdout().strip().splitlines()
    assert expected == lines[-1]


@launch_pytest.fixture
def feature_set_node(camera_node_action, camera_test_node_name):
    return launch.LaunchDescription(
        [
            Node(
                package="vimbax_camera_examples",
                executable="feature_set",
                arguments=[f"/{camera_test_node_name}", "Enum", "AcquisitionMode", "Continuous"],
                cached_output=True,
            ),
            camera_node_action,
            launch_pytest.actions.ReadyToTest(),
        ]
    )


@pytest.mark.launch(fixture=feature_set_node)
def test_feature_set(launch_context, camera_test_node_name, feature_set_node):

    action = feature_set_node.describe_sub_entities()[0]

    expected = "Changed feature AcquisitionMode to Continuous"

    assert_clean_shutdown(launch_context, action)

    lines: List = action.get_stdout().strip().splitlines()
    assert expected == lines[-1]


@launch_pytest.fixture
def execute_command_node(camera_node_action, camera_test_node_name):
    return launch.LaunchDescription(
        [
            Node(
                package="vimbax_camera_examples",
                executable="feature_command_execute",
                arguments=[f"/{camera_test_node_name}", "CounterReset"],
                cached_output=True,
            ),
            camera_node_action,
            launch_pytest.actions.ReadyToTest(),
        ]
    )


@pytest.mark.launch(fixture=execute_command_node)
def test_execute_command(launch_context, camera_test_node_name, execute_command_node):

    action = execute_command_node.describe_sub_entities()[0]

    expected = "Successfully executed command CounterReset"

    assert_clean_shutdown(launch_context, action)

    lines: List = action.get_stdout().strip().splitlines()
    assert expected == lines[-1]


@launch_pytest.fixture
def list_features_node(camera_node_action, camera_test_node_name):
    return launch.LaunchDescription(
        [
            Node(
                package="vimbax_camera_examples",
                executable="list_features",
                arguments=[f"/{camera_test_node_name}"],
                cached_output=True,
            ),
            camera_node_action,
            launch_pytest.actions.ReadyToTest(),
        ]
    )


@pytest.mark.launch(fixture=list_features_node)
def test_list_features(launch_context, camera_test_node_name, list_features_node):

    action = list_features_node.describe_sub_entities()[0]

    assert_clean_shutdown(launch_context, action)

    lines: List[str] = action.get_stdout().strip().splitlines()

    assert 0 < len(lines), "The output should contain more than 0 lines!"

    feature_names: List[str] = list(filter(lambda x: x.strip().startswith("name:"), lines))
    feature_names_set = set([x.strip().split()[1] for x in feature_names])

    res: FeaturesListGet.Result = call_service(
        FeaturesListGet, f"{camera_test_node_name}/features/list_get", FeaturesListGet.Request()
    )

    diff = set(res.feature_list).difference(feature_names_set)
    assert 0 == len(diff), "The example should ouput all features!"
