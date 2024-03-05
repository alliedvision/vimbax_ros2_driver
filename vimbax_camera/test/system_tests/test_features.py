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

import random
import string

from enum import Enum

from vimbax_camera_msgs.msg import Error

from vimbax_camera_msgs.srv import FeaturesListGet
from vimbax_camera_msgs.srv import FeatureInfoQuery
from vimbax_camera_msgs.msg import FeatureInfo

from vimbax_camera_msgs.srv import FeatureAccessModeGet
from vimbax_camera_msgs.srv import FeatureIntInfoGet
from vimbax_camera_msgs.srv import FeatureFloatInfoGet
from vimbax_camera_msgs.srv import FeatureStringInfoGet
from vimbax_camera_msgs.srv import FeatureRawInfoGet
from vimbax_camera_msgs.srv import FeatureEnumInfoGet

from vimbax_camera_msgs.srv import FeatureIntGet
from vimbax_camera_msgs.srv import FeatureIntSet

from vimbax_camera_msgs.srv import FeatureFloatGet
from vimbax_camera_msgs.srv import FeatureFloatSet

from vimbax_camera_msgs.srv import FeatureEnumGet
from vimbax_camera_msgs.srv import FeatureEnumSet

from vimbax_camera_msgs.srv import FeatureBoolGet
from vimbax_camera_msgs.srv import FeatureBoolSet

from vimbax_camera_msgs.srv import FeatureStringGet
from vimbax_camera_msgs.srv import FeatureStringSet

from vimbax_camera_msgs.srv import FeatureRawGet

from conftest import vimbax_camera_node, camera_test_node_name, TestNode


class FeatureDataType(Enum):
    INT = 1
    FLOAT = 2
    ENUM = 3
    STRING = 4
    BOOL = 5
    COMMAND = 6
    RAW = 7
    NONE = 8


sfnc_namespaces = [
    "Standard",
    "Custom"
]


def check_error(error: Error):
    assert error.code == 0, f"Unexpected error {error.code} ({error.text})"


def check_feature_info(feature_info: FeatureInfo):
    assert feature_info.name != ""
    assert feature_info.category != ""
    assert feature_info.display_name != ""
    assert feature_info.sfnc_namespace in sfnc_namespaces
    assert feature_info.data_type > 0 and feature_info.data_type < 9


def ensure_access_mode(service, name, readable: bool = True, writeable: bool = True) -> bool:
    response = service.call(FeatureAccessModeGet.Request(feature_name=name))

    check_error(response.error)

    if readable and not response.is_readable:
        return False
    if writeable and not response.is_writeable:
        return False

    return True


@pytest.mark.launch(fixture=vimbax_camera_node)
def test_feature_list(test_node: TestNode, launch_context):
    feature_list_service = test_node.create_client(
        FeaturesListGet, f"/{camera_test_node_name}/features/list_get")
    assert feature_list_service.wait_for_service(10)

    response = feature_list_service.call(FeaturesListGet.Request())

    check_error(response.error)
    assert len(response.feature_list) > 0


@pytest.mark.launch(fixture=vimbax_camera_node)
def test_feature_list_query(test_node: TestNode, launch_context):
    feature_list_service = test_node.create_client(
        FeaturesListGet, f"/{camera_test_node_name}/features/list_get")
    assert feature_list_service.wait_for_service(10)
    feature_info_query_service = test_node.create_client(
        FeatureInfoQuery, f"/{camera_test_node_name}/feature_info_query")
    assert feature_info_query_service.wait_for_service(10)

    response = feature_list_service.call(FeaturesListGet.Request())

    check_error(response.error)
    assert len(response.feature_list) > 0

    for feature in response.feature_list:
        feature_info_request = FeatureInfoQuery.Request()
        feature_info_request.feature_names.append(feature)
        feature_info = feature_info_query_service.call(feature_info_request)

        check_error(feature_info.error)
        assert len(feature_info.feature_info) == 1
        assert feature_info.feature_info[0].name == feature
        check_feature_info(feature_info.feature_info[0])


@pytest.mark.launch(fixture=vimbax_camera_node)
def test_feature_list_query_all(test_node: TestNode, launch_context):
    feature_list_service = test_node.create_client(
        FeaturesListGet, f"/{camera_test_node_name}/features/list_get")
    assert feature_list_service.wait_for_service(10)
    feature_info_query_service = test_node.create_client(
        FeatureInfoQuery, f"/{camera_test_node_name}/feature_info_query")
    assert feature_info_query_service.wait_for_service(10)

    response = feature_list_service.call(FeaturesListGet.Request())

    check_error(response.error)
    assert len(response.feature_list) > 0

    feature_info_request = FeatureInfoQuery.Request()
    feature_info_request.feature_names = response.feature_list
    feature_info = feature_info_query_service.call(feature_info_request)

    check_error(feature_info.error)
    assert len(feature_info.feature_info) == len(response.feature_list)
    for info in feature_info.feature_info:
        check_feature_info(info)


@pytest.mark.launch(fixture=vimbax_camera_node)
def test_feature_list_is_query_empty(test_node: TestNode, launch_context):
    feature_list_service = test_node.create_client(
        FeaturesListGet, f"/{camera_test_node_name}/features/list_get")
    assert feature_list_service.wait_for_service(10)
    feature_info_query_service = test_node.create_client(
        FeatureInfoQuery, f"/{camera_test_node_name}/feature_info_query")
    assert feature_info_query_service.wait_for_service(10)

    response = feature_list_service.call(FeaturesListGet.Request())

    check_error(response.error)
    assert len(response.feature_list) > 0

    feature_info_request = FeatureInfoQuery.Request()
    feature_info = feature_info_query_service.call(feature_info_request)

    check_error(feature_info.error)
    assert len(feature_info.feature_info) == len(response.feature_list)
    feature_names = [info.name for info in feature_info.feature_info]
    assert feature_names == response.feature_list


@pytest.mark.launch(fixture=vimbax_camera_node)
def test_feature_get_access_mode(test_node: TestNode, launch_context):
    feature_list_service = test_node.create_client(
        FeaturesListGet, f"/{camera_test_node_name}/features/list_get")
    assert feature_list_service.wait_for_service(10)
    feature_access_mode_service = test_node.create_client(
        FeatureAccessModeGet, f"/{camera_test_node_name}/features/access_mode_get")
    assert feature_access_mode_service.wait_for_service(10)

    response = feature_list_service.call(FeaturesListGet.Request())

    check_error(response.error)
    assert len(response.feature_list) > 0

    for feature_name in response.feature_list:
        access_mode_request = FeatureAccessModeGet.Request()
        access_mode_request.feature_name = feature_name
        access_mode_response = feature_access_mode_service.call(access_mode_request)
        check_error(access_mode_response.error)


@pytest.mark.launch(fixture=vimbax_camera_node)
def test_feature_type_info_get(test_node: TestNode, launch_context):
    feature_info_query_service = test_node.create_client(
        FeatureInfoQuery, f"/{camera_test_node_name}/feature_info_query")
    assert feature_info_query_service.wait_for_service(10)
    feature_access_mode_service = test_node.create_client(
        FeatureAccessModeGet, f"/{camera_test_node_name}/features/access_mode_get")
    assert feature_access_mode_service.wait_for_service(10)
    feature_int_info_service = test_node.create_client(
        FeatureIntInfoGet, f"/{camera_test_node_name}/features/int_info_get")
    assert feature_int_info_service.wait_for_service(10)
    feature_float_info_service = test_node.create_client(
        FeatureFloatInfoGet, f"/{camera_test_node_name}/features/float_info_get")
    assert feature_float_info_service.wait_for_service(10)
    feature_enum_info_service = test_node.create_client(
        FeatureEnumInfoGet, f"/{camera_test_node_name}/features/enum_info_get")
    assert feature_enum_info_service.wait_for_service(10)
    feature_string_info_service = test_node.create_client(
        FeatureStringInfoGet, f"/{camera_test_node_name}/features/string_info_get")
    assert feature_string_info_service.wait_for_service(10)
    feature_raw_info_service = test_node.create_client(
        FeatureRawInfoGet, f"/{camera_test_node_name}/features/raw_info_get")
    assert feature_raw_info_service.wait_for_service(10)

    feature_info_response = feature_info_query_service.call(FeatureInfoQuery.Request())

    check_error(feature_info_response.error)
    assert len(feature_info_response.feature_info) > 0

    available_features = [info for info in feature_info_response.feature_info
                          if ensure_access_mode(
                              feature_access_mode_service, info.name, writeable=False
                          )]

    for feature_info in available_features:

        if feature_info.data_type == FeatureDataType.INT.value:
            int_info_request = FeatureIntInfoGet.Request(feature_name=feature_info.name)
            int_info_response = feature_int_info_service.call(int_info_request)
            check_error(int_info_response.error)
            assert int_info_response.min <= int_info_response.max
        elif feature_info.data_type == FeatureDataType.FLOAT.value:
            float_info_request = FeatureFloatInfoGet.Request(feature_name=feature_info.name)
            float_info_response = feature_float_info_service.call(float_info_request)
            check_error(float_info_response.error)
            assert float_info_response.min <= float_info_response.max
        elif feature_info.data_type == FeatureDataType.ENUM.value:
            enum_info_request = FeatureEnumInfoGet.Request(feature_name=feature_info.name)
            enum_info_response = feature_enum_info_service.call(enum_info_request)
            check_error(enum_info_response.error)
            for option in enum_info_response.available_values:
                assert option in enum_info_response.possible_values
        elif feature_info.data_type == FeatureDataType.STRING.value:
            string_info_request = FeatureStringInfoGet.Request(feature_name=feature_info.name)
            string_info_response = feature_string_info_service.call(string_info_request)
            check_error(string_info_response.error)
            assert string_info_response.max_length > 0
        elif feature_info.data_type == FeatureDataType.RAW.value:
            raw_info_request = FeatureRawInfoGet.Request(feature_name=feature_info.name)
            raw_info_response = feature_raw_info_service.call(raw_info_request)
            check_error(raw_info_response.error)
            assert raw_info_response.max_length > 0


@pytest.mark.launch(fixture=vimbax_camera_node)
def test_feature_int_get(test_node: TestNode, launch_context):
    feature_info_query_service = test_node.create_client(
        FeatureInfoQuery, f"/{camera_test_node_name}/feature_info_query")
    assert feature_info_query_service.wait_for_service(10)
    feature_access_mode_service = test_node.create_client(
        FeatureAccessModeGet, f"/{camera_test_node_name}/features/access_mode_get")
    assert feature_access_mode_service.wait_for_service(10)
    feature_int_get_service = test_node.create_client(
        FeatureIntGet, f"/{camera_test_node_name}/features/int_get")
    assert feature_int_get_service.wait_for_service(10)

    feature_info_response = feature_info_query_service.call(FeatureInfoQuery.Request())

    check_error(feature_info_response.error)
    assert len(feature_info_response.feature_info) > 0

    available_features = [info for info in feature_info_response.feature_info
                          if info.data_type == FeatureDataType.INT.value and
                          ensure_access_mode(
                              feature_access_mode_service, info.name, writeable=False
                          )]

    for feature_info in available_features:
        response = feature_int_get_service.call(
            FeatureIntGet.Request(feature_name=feature_info.name))
        check_error(response.error)


@pytest.mark.launch(fixture=vimbax_camera_node)
def test_feature_int_set(test_node: TestNode, launch_context):
    features_ignore = ["TestPendingAck", "CustomModuleRegData"]

    feature_info_query_service = test_node.create_client(
        FeatureInfoQuery, f"/{camera_test_node_name}/feature_info_query")
    assert feature_info_query_service.wait_for_service(10)
    feature_access_mode_service = test_node.create_client(
        FeatureAccessModeGet, f"/{camera_test_node_name}/features/access_mode_get")
    assert feature_access_mode_service.wait_for_service(10)
    feature_int_get_service = test_node.create_client(
        FeatureIntGet, f"/{camera_test_node_name}/features/int_get")
    assert feature_int_get_service.wait_for_service(10)
    feature_int_set_service = test_node.create_client(
        FeatureIntSet, f"/{camera_test_node_name}/features/int_set")
    assert feature_int_set_service.wait_for_service(10)
    feature_int_info_service = test_node.create_client(
        FeatureIntInfoGet, f"/{camera_test_node_name}/features/int_info_get")
    assert feature_int_info_service.wait_for_service(10)

    feature_info_response = feature_info_query_service.call(FeatureInfoQuery.Request())

    check_error(feature_info_response.error)
    assert len(feature_info_response.feature_info) > 0

    available_features = [info for info in feature_info_response.feature_info
                          if info.data_type == FeatureDataType.INT.value and
                          ensure_access_mode(feature_access_mode_service, info.name)]

    for feature_info in available_features:
        if feature_info.name in features_ignore:
            continue

        int_get_response = feature_int_get_service.call(
            FeatureIntGet.Request(feature_name=feature_info.name))
        check_error(int_get_response.error)

        int_info_response = feature_int_info_service.call(
            FeatureIntInfoGet.Request(feature_name=feature_info.name))
        check_error(int_info_response.error)

        default_value = int_get_response.value

        for value in [int_info_response.min, int_info_response.max, default_value]:
            print(f"Setting value to {value}")
            set_request = FeatureIntSet.Request(feature_name=feature_info.name)
            set_request.value = value

            set_response = feature_int_set_service.call(set_request)
            check_error(set_response.error)

            get_set_response = feature_int_get_service.call(
                FeatureIntGet.Request(feature_name=feature_info.name))
            check_error(get_set_response.error)
            assert get_set_response.value == value


@pytest.mark.launch(fixture=vimbax_camera_node)
def test_feature_float_get(test_node: TestNode, launch_context):
    feature_info_query_service = test_node.create_client(
        FeatureInfoQuery, f"/{camera_test_node_name}/feature_info_query")
    assert feature_info_query_service.wait_for_service(10)
    feature_access_mode_service = test_node.create_client(
        FeatureAccessModeGet, f"/{camera_test_node_name}/features/access_mode_get")
    assert feature_access_mode_service.wait_for_service(10)
    feature_float_get_service = test_node.create_client(
        FeatureFloatGet, f"/{camera_test_node_name}/features/float_get")
    assert feature_float_get_service.wait_for_service(10)

    feature_info_response = feature_info_query_service.call(FeatureInfoQuery.Request())

    check_error(feature_info_response.error)
    assert len(feature_info_response.feature_info) > 0

    available_features = [info for info in feature_info_response.feature_info
                          if info.data_type == FeatureDataType.FLOAT.value and
                          ensure_access_mode(
                              feature_access_mode_service, info.name, writeable=False
                          )]

    for feature_info in available_features:
        response = feature_float_get_service.call(
            FeatureFloatGet.Request(feature_name=feature_info.name))
        check_error(response.error)


@pytest.mark.launch(fixture=vimbax_camera_node)
def test_feature_float_set(test_node: TestNode, launch_context):
    features_ignore = [
        "LvWatchdogTimerDuration", "Gain"
    ]

    feature_info_query_service = test_node.create_client(
        FeatureInfoQuery, f"/{camera_test_node_name}/feature_info_query")
    assert feature_info_query_service.wait_for_service(10)
    feature_access_mode_service = test_node.create_client(
        FeatureAccessModeGet, f"/{camera_test_node_name}/features/access_mode_get")
    assert feature_access_mode_service.wait_for_service(10)
    feature_float_get_service = test_node.create_client(
        FeatureFloatGet, f"/{camera_test_node_name}/features/float_get")
    assert feature_float_get_service.wait_for_service(10)
    feature_float_set_service = test_node.create_client(
        FeatureFloatSet, f"/{camera_test_node_name}/features/float_set")
    assert feature_float_set_service.wait_for_service(10)
    feature_float_info_service = test_node.create_client(
        FeatureFloatInfoGet, f"/{camera_test_node_name}/features/float_info_get")
    assert feature_float_info_service.wait_for_service(10)

    feature_info_response = feature_info_query_service.call(FeatureInfoQuery.Request())

    check_error(feature_info_response.error)
    assert len(feature_info_response.feature_info) > 0

    available_features = [info for info in feature_info_response.feature_info
                          if info.data_type == FeatureDataType.FLOAT.value and
                          ensure_access_mode(feature_access_mode_service, info.name)]

    for feature_info in available_features:
        if feature_info.name in features_ignore:
            continue

        float_get_response = feature_float_get_service.call(
            FeatureFloatGet.Request(feature_name=feature_info.name))
        check_error(float_get_response.error)

        float_info_response = feature_float_info_service.call(
            FeatureFloatInfoGet.Request(feature_name=feature_info.name))
        check_error(float_info_response.error)

        default_value = float_get_response.value

        for value in [float_info_response.min, float_info_response.max, default_value]:
            print(f"Setting value to {value}")
            set_request = FeatureFloatSet.Request(feature_name=feature_info.name)
            set_request.value = value

            set_response = feature_float_set_service.call(set_request)
            check_error(set_response.error)

            get_set_response = feature_float_get_service.call(
                FeatureFloatGet.Request(feature_name=feature_info.name))
            check_error(get_set_response.error)
            assert get_set_response.value == value


@pytest.mark.launch(fixture=vimbax_camera_node)
def test_feature_enum_get(test_node: TestNode, launch_context):
    feature_info_query_service = test_node.create_client(
        FeatureInfoQuery, f"/{camera_test_node_name}/feature_info_query")
    assert feature_info_query_service.wait_for_service(10)
    feature_access_mode_service = test_node.create_client(
        FeatureAccessModeGet, f"/{camera_test_node_name}/features/access_mode_get")
    assert feature_access_mode_service.wait_for_service(10)
    feature_enum_get_service = test_node.create_client(
        FeatureEnumGet, f"/{camera_test_node_name}/features/enum_get")
    assert feature_enum_get_service.wait_for_service(10)

    feature_info_response = feature_info_query_service.call(FeatureInfoQuery.Request())

    check_error(feature_info_response.error)
    assert len(feature_info_response.feature_info) > 0

    available_features = [info for info in feature_info_response.feature_info
                          if info.data_type == FeatureDataType.ENUM.value and
                          ensure_access_mode(
                              feature_access_mode_service, info.name, writeable=False
                          )]

    for feature_info in available_features:
        response = feature_enum_get_service.call(
            FeatureEnumGet.Request(feature_name=feature_info.name))
        check_error(response.error)


@pytest.mark.launch(fixture=vimbax_camera_node)
def test_feature_enum_set(test_node: TestNode, launch_context):
    features_ignore = [
        "TestPattern", "ExposureActiveMode", "DevicePowerSavingMode", "BalanceWhiteAuto"
    ]

    feature_info_query_service = test_node.create_client(
        FeatureInfoQuery, f"/{camera_test_node_name}/feature_info_query")
    assert feature_info_query_service.wait_for_service(10)
    feature_access_mode_service = test_node.create_client(
        FeatureAccessModeGet, f"/{camera_test_node_name}/features/access_mode_get")
    assert feature_access_mode_service.wait_for_service(10)
    feature_enum_get_service = test_node.create_client(
        FeatureEnumGet, f"/{camera_test_node_name}/features/enum_get")
    assert feature_enum_get_service.wait_for_service(10)
    feature_enum_set_service = test_node.create_client(
        FeatureEnumSet, f"/{camera_test_node_name}/features/enum_set")
    assert feature_enum_set_service.wait_for_service(10)
    feature_enum_info_service = test_node.create_client(
        FeatureEnumInfoGet, f"/{camera_test_node_name}/features/enum_info_get")
    assert feature_enum_info_service.wait_for_service(10)

    feature_info_response = feature_info_query_service.call(FeatureInfoQuery.Request())

    check_error(feature_info_response.error)
    assert len(feature_info_response.feature_info) > 0

    available_features = [info for info in feature_info_response.feature_info
                          if info.data_type == FeatureDataType.ENUM.value and
                          ensure_access_mode(feature_access_mode_service, info.name)]

    for feature_info in available_features:
        if feature_info.name in features_ignore:
            continue

        enum_get_response = feature_enum_get_service.call(
            FeatureEnumGet.Request(feature_name=feature_info.name))
        check_error(enum_get_response.error)

        enum_info_response = feature_enum_info_service.call(
            FeatureEnumInfoGet.Request(feature_name=feature_info.name))
        check_error(enum_info_response.error)

        default_value = enum_get_response.value

        for value in enum_info_response.available_values + [default_value]:
            print(f"Setting value to {value}")
            set_request = FeatureEnumSet.Request(feature_name=feature_info.name)
            set_request.value = value

            set_response = feature_enum_set_service.call(set_request)
            check_error(set_response.error)

            get_set_response = feature_enum_get_service.call(
                FeatureEnumGet.Request(feature_name=feature_info.name))
            check_error(get_set_response.error)
            assert get_set_response.value == value


@pytest.mark.launch(fixture=vimbax_camera_node)
def test_feature_bool_get(test_node: TestNode, launch_context):
    feature_info_query_service = test_node.create_client(
        FeatureInfoQuery, f"/{camera_test_node_name}/feature_info_query")
    assert feature_info_query_service.wait_for_service(10)
    feature_access_mode_service = test_node.create_client(
        FeatureAccessModeGet, f"/{camera_test_node_name}/features/access_mode_get")
    assert feature_access_mode_service.wait_for_service(10)
    feature_bool_get_service = test_node.create_client(
        FeatureBoolGet, f"/{camera_test_node_name}/features/bool_get")
    assert feature_bool_get_service.wait_for_service(10)

    feature_info_response = feature_info_query_service.call(FeatureInfoQuery.Request())

    check_error(feature_info_response.error)
    assert len(feature_info_response.feature_info) > 0

    available_features = [info for info in feature_info_response.feature_info
                          if info.data_type == FeatureDataType.BOOL.value and
                          ensure_access_mode(
                              feature_access_mode_service, info.name, writeable=False
                          )]

    for feature_info in available_features:
        response = feature_bool_get_service.call(
            FeatureBoolGet.Request(feature_name=feature_info.name))
        check_error(response.error)


@pytest.mark.launch(fixture=vimbax_camera_node)
def test_feature_bool_set(test_node: TestNode, launch_context):
    features_ignore = ["TestPattern", "ExposureActiveMode", "DevicePowerSavingMode"]

    feature_info_query_service = test_node.create_client(
        FeatureInfoQuery, f"/{camera_test_node_name}/feature_info_query")
    assert feature_info_query_service.wait_for_service(10)
    feature_access_mode_service = test_node.create_client(
        FeatureAccessModeGet, f"/{camera_test_node_name}/features/access_mode_get")
    assert feature_access_mode_service.wait_for_service(10)
    feature_bool_get_service = test_node.create_client(
        FeatureBoolGet, f"/{camera_test_node_name}/features/bool_get")
    assert feature_bool_get_service.wait_for_service(10)
    feature_bool_set_service = test_node.create_client(
        FeatureBoolSet, f"/{camera_test_node_name}/features/bool_set")
    assert feature_bool_set_service.wait_for_service(10)

    feature_info_response = feature_info_query_service.call(FeatureInfoQuery.Request())

    check_error(feature_info_response.error)
    assert len(feature_info_response.feature_info) > 0

    available_features = [info for info in feature_info_response.feature_info
                          if info.data_type == FeatureDataType.BOOL.value and
                          ensure_access_mode(feature_access_mode_service, info.name)]

    for feature_info in available_features:
        if feature_info.name in features_ignore:
            continue

        default_get_response = feature_bool_get_service.call(
            FeatureBoolGet.Request(feature_name=feature_info.name))
        check_error(default_get_response.error)

        default_value = default_get_response.value

        for value in [True, False, default_value]:
            print(f"Setting value to {value}")
            set_request = FeatureBoolSet.Request(feature_name=feature_info.name)
            set_request.value = value

            set_response = feature_bool_set_service.call(set_request)
            check_error(set_response.error)

            get_set_response = feature_bool_get_service.call(
                FeatureBoolGet.Request(feature_name=feature_info.name))
            check_error(get_set_response.error)


@pytest.mark.launch(fixture=vimbax_camera_node)
def test_feature_string_get(test_node: TestNode, launch_context):
    feature_info_query_service = test_node.create_client(
        FeatureInfoQuery, f"/{camera_test_node_name}/feature_info_query")
    assert feature_info_query_service.wait_for_service(10)
    feature_access_mode_service = test_node.create_client(
        FeatureAccessModeGet, f"/{camera_test_node_name}/features/access_mode_get")
    assert feature_access_mode_service.wait_for_service(10)
    feature_string_get_service = test_node.create_client(
        FeatureStringGet, f"/{camera_test_node_name}/features/string_get")
    assert feature_string_get_service.wait_for_service(10)

    feature_info_response = feature_info_query_service.call(FeatureInfoQuery.Request())

    check_error(feature_info_response.error)
    assert len(feature_info_response.feature_info) > 0

    available_features = [info for info in feature_info_response.feature_info
                          if info.data_type == FeatureDataType.STRING.value and
                          ensure_access_mode(
                              feature_access_mode_service, info.name, writeable=False
                          )]

    for feature_info in available_features:
        response = feature_string_get_service.call(
            FeatureStringGet.Request(feature_name=feature_info.name))
        check_error(response.error)


@pytest.mark.launch(fixture=vimbax_camera_node)
def test_feature_string_set(test_node: TestNode, launch_context):
    feature_info_query_service = test_node.create_client(
        FeatureInfoQuery, f"/{camera_test_node_name}/feature_info_query")
    assert feature_info_query_service.wait_for_service(10)
    feature_access_mode_service = test_node.create_client(
        FeatureAccessModeGet, f"/{camera_test_node_name}/features/access_mode_get")
    assert feature_access_mode_service.wait_for_service(10)
    feature_string_get_service = test_node.create_client(
        FeatureStringGet, f"/{camera_test_node_name}/features/string_get")
    assert feature_string_get_service.wait_for_service(10)
    feature_string_set_service = test_node.create_client(
        FeatureStringSet, f"/{camera_test_node_name}/features/string_set")
    feature_string_set_service.wait_for_service(10)
    feature_string_info_service = test_node.create_client(
        FeatureStringInfoGet, f"/{camera_test_node_name}/features/string_info_get")
    feature_string_info_service.wait_for_service(10)

    feature_info_response = feature_info_query_service.call(FeatureInfoQuery.Request())

    check_error(feature_info_response.error)
    assert len(feature_info_response.feature_info) > 0

    available_features = [info for info in feature_info_response.feature_info
                          if info.data_type == FeatureDataType.STRING.value and
                          ensure_access_mode(feature_access_mode_service, info.name)]

    for feature_info in available_features:
        default_response = feature_string_get_service.call(
            FeatureStringGet.Request(feature_name=feature_info.name))
        check_error(default_response.error)

        info_response = feature_string_info_service.call(
            FeatureStringInfoGet.Request(feature_name=feature_info.name))
        check_error(info_response.error)

        single_char_str = ''.join(
            random.choices(string.ascii_letters + string.digits, k=1))
        max_length_str = ''.join(
            random.choices(string.ascii_letters + string.digits, k=info_response.max_length))

        for value in [single_char_str, max_length_str, default_response.value]:
            print(f"Setting string to value {value}")
            set_request = FeatureStringSet.Request(feature_name=feature_info.name)
            set_request.value = value
            set_response = feature_string_set_service.call(set_request)

            check_error(set_response.error)

            get_set_response = feature_string_get_service.call(
                FeatureStringGet.Request(feature_name=feature_info.name))

            check_error(get_set_response.error)
            assert get_set_response.value == value


@pytest.mark.launch(fixture=vimbax_camera_node)
def test_feature_raw_get(test_node: TestNode, launch_context):
    feature_info_query_service = test_node.create_client(
        FeatureInfoQuery, f"/{camera_test_node_name}/feature_info_query")
    assert feature_info_query_service.wait_for_service(10)
    feature_access_mode_service = test_node.create_client(
        FeatureAccessModeGet, f"/{camera_test_node_name}/features/access_mode_get")
    assert feature_access_mode_service.wait_for_service(10)
    feature_raw_get_service = test_node.create_client(
        FeatureRawGet, f"/{camera_test_node_name}/features/raw_get")
    assert feature_raw_get_service.wait_for_service(10)

    feature_info_response = feature_info_query_service.call(FeatureInfoQuery.Request())

    check_error(feature_info_response.error)
    assert len(feature_info_response.feature_info) > 0

    available_features = [info for info in feature_info_response.feature_info
                          if info.data_type == FeatureDataType.RAW.value and
                          ensure_access_mode(
                              feature_access_mode_service, info.name, writeable=False
                          )]

    for feature_info in available_features:
        response = feature_raw_get_service.call(
            FeatureRawGet.Request(feature_name=feature_info.name)
            )
        check_error(response.error)
        assert len(response.buffer) > 0
        assert len(response.buffer) == response.buffer_size
