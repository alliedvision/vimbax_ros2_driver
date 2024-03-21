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
from vimbax_camera_msgs.srv import FeatureEnumGet, FeatureEnumInfoGet
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

    assert expected == action.get_stdout().strip()


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

    assert expected == action.get_stdout().strip()


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

    assert expected == action.get_stdout().strip()


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

    assert expected == action.get_stdout().strip()
