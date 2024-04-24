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

import random
import string


from vimbax_camera_msgs.srv import FeaturesListGet
from vimbax_camera_msgs.srv import FeatureInfoQuery

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
from vimbax_camera_msgs.msg import FeatureModule

from conftest import vimbax_camera_node, TestNode

from test_helper import check_error, check_feature_info, ensure_access_mode, FeatureDataType

MODULE_LIST = [
    FeatureModule(id=FeatureModule.MODULE_REMOTE_DEVICE),
    FeatureModule(id=FeatureModule.MODULE_SYSTEM),
    FeatureModule(id=FeatureModule.MODULE_INTERFACE),
    FeatureModule(id=FeatureModule.MODULE_LOCAL_DEVICE),
    FeatureModule(id=FeatureModule.MODULE_STREAM),
]

MODULE_NAME_LIST = [
    "MODULE_REMOTE_DEVICE",
    "MODULE_SYSTEM",
    "MODULE_INTERFACE",
    "MODULE_LOCAL_DEVICE",
    "MODULE_STREAM",
]


@pytest.mark.launch(fixture=vimbax_camera_node)
@pytest.mark.parametrize("module", MODULE_LIST, ids=MODULE_NAME_LIST)
def test_feature_list(test_node: TestNode, launch_context, module: FeatureModule):
    feature_list_service = test_node.create_client(
        FeaturesListGet, f"/{test_node.camera_node_name()}/features/list_get"
    )
    assert feature_list_service.wait_for_service(10)

    req = FeaturesListGet.Request(feature_module=module)
    response = test_node.call_service_sync(feature_list_service, req)

    check_error(response.error)
    assert len(response.feature_list) > 0


@pytest.mark.launch(fixture=vimbax_camera_node)
@pytest.mark.parametrize("module", MODULE_LIST, ids=MODULE_NAME_LIST)
def test_feature_list_query(test_node: TestNode, launch_context, module: FeatureModule):
    feature_list_service = test_node.create_client(
        FeaturesListGet, f"/{test_node.camera_node_name()}/features/list_get"
    )
    assert feature_list_service.wait_for_service(10)
    feature_info_query_service = test_node.create_client(
        FeatureInfoQuery, f"/{test_node.camera_node_name()}/feature_info_query"
    )
    assert feature_info_query_service.wait_for_service(10)

    response = test_node.call_service_sync(
        feature_list_service, FeaturesListGet.Request(feature_module=module)
    )

    check_error(response.error)
    assert len(response.feature_list) > 0

    for feature in response.feature_list:
        feature_info_request = FeatureInfoQuery.Request(feature_module=module)
        feature_info_request.feature_names.append(feature)
        feature_info = test_node.call_service_sync(
            feature_info_query_service, feature_info_request
        )

        check_error(feature_info.error)
        assert len(feature_info.feature_info) == 1
        assert feature_info.feature_info[0].name == feature
        check_feature_info(feature_info.feature_info[0])


@pytest.mark.launch(fixture=vimbax_camera_node)
@pytest.mark.parametrize("module", MODULE_LIST, ids=MODULE_NAME_LIST)
def test_feature_list_query_all(test_node: TestNode, launch_context, module: FeatureModule):
    feature_list_service = test_node.create_client(
        FeaturesListGet, f"/{test_node.camera_node_name()}/features/list_get"
    )
    assert feature_list_service.wait_for_service(10)
    feature_info_query_service = test_node.create_client(
        FeatureInfoQuery, f"/{test_node.camera_node_name()}/feature_info_query"
    )
    assert feature_info_query_service.wait_for_service(10)

    response = test_node.call_service_sync(
        feature_list_service, FeaturesListGet.Request(feature_module=module)
    )

    check_error(response.error)
    assert len(response.feature_list) > 0

    feature_info_request = FeatureInfoQuery.Request(feature_module=module)
    feature_info_request.feature_names = response.feature_list
    feature_info = test_node.call_service_sync(feature_info_query_service, feature_info_request)

    check_error(feature_info.error)
    assert len(feature_info.feature_info) == len(response.feature_list)
    for info in feature_info.feature_info:
        check_feature_info(info)


@pytest.mark.launch(fixture=vimbax_camera_node)
@pytest.mark.parametrize("module", MODULE_LIST, ids=MODULE_NAME_LIST)
def test_feature_list_is_query_empty(test_node: TestNode, launch_context, module: FeatureModule):
    feature_list_service = test_node.create_client(
        FeaturesListGet, f"/{test_node.camera_node_name()}/features/list_get"
    )
    assert feature_list_service.wait_for_service(10)
    feature_info_query_service = test_node.create_client(
        FeatureInfoQuery, f"/{test_node.camera_node_name()}/feature_info_query"
    )
    assert feature_info_query_service.wait_for_service(10)

    response = test_node.call_service_sync(
        feature_list_service, FeaturesListGet.Request(feature_module=module)
    )

    check_error(response.error)
    assert len(response.feature_list) > 0

    feature_info_request = FeatureInfoQuery.Request(feature_module=module)
    feature_info = test_node.call_service_sync(feature_info_query_service, feature_info_request)

    check_error(feature_info.error)
    assert len(feature_info.feature_info) == len(response.feature_list)
    feature_names = [info.name for info in feature_info.feature_info]
    assert feature_names == response.feature_list


@pytest.mark.launch(fixture=vimbax_camera_node)
@pytest.mark.parametrize("module", MODULE_LIST, ids=MODULE_NAME_LIST)
def test_feature_get_access_mode(test_node: TestNode, launch_context, module: FeatureModule):
    feature_list_service = test_node.create_client(
        FeaturesListGet, f"/{test_node.camera_node_name()}/features/list_get"
    )
    assert feature_list_service.wait_for_service(10)
    feature_access_mode_service = test_node.create_client(
        FeatureAccessModeGet, f"/{test_node.camera_node_name()}/features/access_mode_get"
    )
    assert feature_access_mode_service.wait_for_service(10)

    response = test_node.call_service_sync(
        feature_list_service, FeaturesListGet.Request(feature_module=module)
    )

    check_error(response.error)
    assert len(response.feature_list) > 0

    for feature_name in response.feature_list:
        access_mode_request = FeatureAccessModeGet.Request(feature_module=module)
        access_mode_request.feature_name = feature_name
        access_mode_response = test_node.call_service_sync(
            feature_access_mode_service, access_mode_request
        )
        check_error(access_mode_response.error)


@pytest.mark.launch(fixture=vimbax_camera_node)
@pytest.mark.parametrize("module", MODULE_LIST, ids=MODULE_NAME_LIST)
def test_feature_type_info_get(test_node: TestNode, launch_context, module: FeatureModule):
    feature_info_query_service = test_node.create_client(
        FeatureInfoQuery, f"/{test_node.camera_node_name()}/feature_info_query"
    )
    assert feature_info_query_service.wait_for_service(10)
    feature_access_mode_service = test_node.create_client(
        FeatureAccessModeGet, f"/{test_node.camera_node_name()}/features/access_mode_get"
    )
    assert feature_access_mode_service.wait_for_service(10)
    feature_int_info_service = test_node.create_client(
        FeatureIntInfoGet, f"/{test_node.camera_node_name()}/features/int_info_get"
    )
    assert feature_int_info_service.wait_for_service(10)
    feature_float_info_service = test_node.create_client(
        FeatureFloatInfoGet, f"/{test_node.camera_node_name()}/features/float_info_get"
    )
    assert feature_float_info_service.wait_for_service(10)
    feature_enum_info_service = test_node.create_client(
        FeatureEnumInfoGet, f"/{test_node.camera_node_name()}/features/enum_info_get"
    )
    assert feature_enum_info_service.wait_for_service(10)
    feature_string_info_service = test_node.create_client(
        FeatureStringInfoGet, f"/{test_node.camera_node_name()}/features/string_info_get"
    )
    assert feature_string_info_service.wait_for_service(10)
    feature_raw_info_service = test_node.create_client(
        FeatureRawInfoGet, f"/{test_node.camera_node_name()}/features/raw_info_get"
    )
    assert feature_raw_info_service.wait_for_service(10)

    feature_info_response = test_node.call_service_sync(
        feature_info_query_service, FeatureInfoQuery.Request(feature_module=module)
    )

    check_error(feature_info_response.error)
    assert len(feature_info_response.feature_info) > 0

    available_features = [
        info
        for info in feature_info_response.feature_info
        if ensure_access_mode(
            feature_access_mode_service, info.name, writeable=False, module=module
        )
    ]

    for feature_info in available_features:

        if feature_info.data_type == FeatureDataType.INT.value:
            int_info_request = FeatureIntInfoGet.Request(
                feature_name=feature_info.name, feature_module=module
            )
            int_info_response = test_node.call_service_sync(
                feature_int_info_service, int_info_request
            )
            check_error(int_info_response.error)
            assert int_info_response.min <= int_info_response.max
        elif feature_info.data_type == FeatureDataType.FLOAT.value:
            float_info_request = FeatureFloatInfoGet.Request(
                feature_name=feature_info.name, feature_module=module
            )
            float_info_response = test_node.call_service_sync(
                feature_float_info_service, float_info_request
            )
            check_error(float_info_response.error)
            assert float_info_response.min <= float_info_response.max
        elif feature_info.data_type == FeatureDataType.ENUM.value:
            enum_info_request = FeatureEnumInfoGet.Request(
                feature_name=feature_info.name, feature_module=module
            )
            enum_info_response = test_node.call_service_sync(
                feature_enum_info_service, enum_info_request
            )
            check_error(enum_info_response.error)
            for option in enum_info_response.available_values:
                assert option in enum_info_response.possible_values
        elif feature_info.data_type == FeatureDataType.STRING.value:
            string_info_request = FeatureStringInfoGet.Request(
                feature_name=feature_info.name, feature_module=module
            )
            string_info_response = test_node.call_service_sync(
                feature_string_info_service, string_info_request
            )
            check_error(string_info_response.error)
            assert string_info_response.max_length > 0
        elif feature_info.data_type == FeatureDataType.RAW.value:
            raw_info_request = FeatureRawInfoGet.Request(
                feature_name=feature_info.name, feature_module=module
            )
            raw_info_response = test_node.call_service_sync(
                feature_raw_info_service, raw_info_request
            )
            check_error(raw_info_response.error)
            assert raw_info_response.max_length > 0


@pytest.mark.launch(fixture=vimbax_camera_node)
@pytest.mark.parametrize("module", MODULE_LIST, ids=MODULE_NAME_LIST)
def test_feature_int_get(test_node: TestNode, launch_context, module: FeatureModule):
    feature_info_query_service = test_node.create_client(
        FeatureInfoQuery, f"/{test_node.camera_node_name()}/feature_info_query"
    )
    assert feature_info_query_service.wait_for_service(10)
    feature_access_mode_service = test_node.create_client(
        FeatureAccessModeGet, f"/{test_node.camera_node_name()}/features/access_mode_get"
    )
    assert feature_access_mode_service.wait_for_service(10)
    feature_int_get_service = test_node.create_client(
        FeatureIntGet, f"/{test_node.camera_node_name()}/features/int_get"
    )
    assert feature_int_get_service.wait_for_service(10)

    feature_info_response = test_node.call_service_sync(
        feature_info_query_service, FeatureInfoQuery.Request(feature_module=module)
    )

    check_error(feature_info_response.error)
    assert len(feature_info_response.feature_info) > 0

    available_features = [
        info
        for info in feature_info_response.feature_info
        if info.data_type == FeatureDataType.INT.value
        and ensure_access_mode(
            feature_access_mode_service, info.name, writeable=False, module=module
        )
    ]

    for feature_info in available_features:
        response = test_node.call_service_sync(
            feature_int_get_service,
            FeatureIntGet.Request(feature_name=feature_info.name, feature_module=module),
        )
        check_error(response.error)


@pytest.mark.launch(fixture=vimbax_camera_node)
@pytest.mark.parametrize("module", MODULE_LIST, ids=MODULE_NAME_LIST)
def test_feature_int_set(test_node: TestNode, launch_context, module: FeatureModule):
    features_ignore = [
            "TestPendingAck",
            "CustomModuleRegData",
            "AutoModeRegionOffsetY",
            "GVSPPacketSize",
            "GVSPMissingSize",
            "GevDeviceForceIPAddress",
            "GevDeviceForceMACAddress",
            "GevDeviceForceGateway",
            "InterfaceHailPace",
            "InterfacePingPace"
    ]

    feature_info_query_service = test_node.create_client(
        FeatureInfoQuery, f"/{test_node.camera_node_name()}/feature_info_query"
    )
    assert feature_info_query_service.wait_for_service(10)
    feature_access_mode_service = test_node.create_client(
        FeatureAccessModeGet, f"/{test_node.camera_node_name()}/features/access_mode_get"
    )
    assert feature_access_mode_service.wait_for_service(10)
    feature_int_get_service = test_node.create_client(
        FeatureIntGet, f"/{test_node.camera_node_name()}/features/int_get"
    )
    assert feature_int_get_service.wait_for_service(10)
    feature_int_set_service = test_node.create_client(
        FeatureIntSet, f"/{test_node.camera_node_name()}/features/int_set"
    )
    assert feature_int_set_service.wait_for_service(10)
    feature_int_info_service = test_node.create_client(
        FeatureIntInfoGet, f"/{test_node.camera_node_name()}/features/int_info_get"
    )
    assert feature_int_info_service.wait_for_service(10)

    feature_info_response = test_node.call_service_sync(
        feature_info_query_service, FeatureInfoQuery.Request(feature_module=module)
    )

    check_error(feature_info_response.error)
    assert len(feature_info_response.feature_info) > 0

    available_features = [
        info
        for info in feature_info_response.feature_info
        if info.data_type == FeatureDataType.INT.value
        and ensure_access_mode(feature_access_mode_service, info.name, module=module)
    ]

    for feature_info in available_features:
        if feature_info.name in features_ignore:
            continue

        int_get_response = test_node.call_service_sync(
            feature_int_get_service,
            FeatureIntGet.Request(feature_name=feature_info.name, feature_module=module),
        )
        check_error(int_get_response.error)

        int_info_response = test_node.call_service_sync(
            feature_int_info_service,
            FeatureIntInfoGet.Request(feature_name=feature_info.name, feature_module=module),
        )
        check_error(int_info_response.error)

        default_value = int_get_response.value

        for value in [int_info_response.min, int_info_response.max, default_value]:
            print(f"{feature_info.name}: Setting value to {value}")
            set_request = FeatureIntSet.Request(
                feature_name=feature_info.name, feature_module=module
            )
            set_request.value = value

            set_response = test_node.call_service_sync(feature_int_set_service, set_request)
            check_error(set_response.error)

            get_set_response = test_node.call_service_sync(
                feature_int_get_service,
                FeatureIntGet.Request(feature_name=feature_info.name, feature_module=module),
            )
            check_error(get_set_response.error)
            assert get_set_response.value == value


@pytest.mark.launch(fixture=vimbax_camera_node)
@pytest.mark.parametrize("module", MODULE_LIST, ids=MODULE_NAME_LIST)
def test_feature_float_get(test_node: TestNode, launch_context, module: FeatureModule):
    feature_info_query_service = test_node.create_client(
        FeatureInfoQuery, f"/{test_node.camera_node_name()}/feature_info_query"
    )
    assert feature_info_query_service.wait_for_service(10)
    feature_access_mode_service = test_node.create_client(
        FeatureAccessModeGet, f"/{test_node.camera_node_name()}/features/access_mode_get"
    )
    assert feature_access_mode_service.wait_for_service(10)
    feature_float_get_service = test_node.create_client(
        FeatureFloatGet, f"/{test_node.camera_node_name()}/features/float_get"
    )
    assert feature_float_get_service.wait_for_service(10)

    feature_info_response = test_node.call_service_sync(
        feature_info_query_service, FeatureInfoQuery.Request(feature_module=module)
    )

    check_error(feature_info_response.error)
    assert len(feature_info_response.feature_info) > 0

    available_features = [
        info
        for info in feature_info_response.feature_info
        if info.data_type == FeatureDataType.FLOAT.value
        and ensure_access_mode(
            feature_access_mode_service, info.name, writeable=False, module=module
        )
    ]

    for feature_info in available_features:
        response = test_node.call_service_sync(
            feature_float_get_service,
            FeatureFloatGet.Request(feature_name=feature_info.name, feature_module=module),
        )
        check_error(response.error)


@pytest.mark.launch(fixture=vimbax_camera_node)
@pytest.mark.parametrize("module", MODULE_LIST, ids=MODULE_NAME_LIST)
def test_feature_float_set(test_node: TestNode, launch_context, module: FeatureModule):
    features_ignore = ["LvWatchdogTimerDuration", "Gain"]

    feature_info_query_service = test_node.create_client(
        FeatureInfoQuery, f"/{test_node.camera_node_name()}/feature_info_query"
    )
    assert feature_info_query_service.wait_for_service(10)
    feature_access_mode_service = test_node.create_client(
        FeatureAccessModeGet, f"/{test_node.camera_node_name()}/features/access_mode_get"
    )
    assert feature_access_mode_service.wait_for_service(10)
    feature_float_get_service = test_node.create_client(
        FeatureFloatGet, f"/{test_node.camera_node_name()}/features/float_get"
    )
    assert feature_float_get_service.wait_for_service(10)
    feature_float_set_service = test_node.create_client(
        FeatureFloatSet, f"/{test_node.camera_node_name()}/features/float_set"
    )
    assert feature_float_set_service.wait_for_service(10)
    feature_float_info_service = test_node.create_client(
        FeatureFloatInfoGet, f"/{test_node.camera_node_name()}/features/float_info_get"
    )
    assert feature_float_info_service.wait_for_service(10)

    feature_info_response = test_node.call_service_sync(
        feature_info_query_service, FeatureInfoQuery.Request(feature_module=module)
    )

    check_error(feature_info_response.error)
    assert len(feature_info_response.feature_info) > 0

    available_features = [
        info
        for info in feature_info_response.feature_info
        if info.data_type == FeatureDataType.FLOAT.value
        and ensure_access_mode(feature_access_mode_service, info.name, module=module)
    ]

    for feature_info in available_features:
        if feature_info.name in features_ignore:
            continue

        float_get_response = test_node.call_service_sync(
            feature_float_get_service,
            FeatureFloatGet.Request(feature_name=feature_info.name, feature_module=module),
        )
        check_error(float_get_response.error)

        float_info_response = test_node.call_service_sync(
            feature_float_info_service,
            FeatureFloatInfoGet.Request(feature_name=feature_info.name, feature_module=module),
        )
        check_error(float_info_response.error)

        default_value = float_get_response.value

        for value in [float_info_response.min, float_info_response.max, default_value]:
            print(f"{feature_info.name}: Setting value to {value}")
            set_request = FeatureFloatSet.Request(
                feature_name=feature_info.name, feature_module=module
            )
            set_request.value = value

            set_response = test_node.call_service_sync(feature_float_set_service, set_request)
            check_error(set_response.error)

            get_set_response = test_node.call_service_sync(
                feature_float_get_service,
                FeatureFloatGet.Request(feature_name=feature_info.name, feature_module=module),
            )
            check_error(get_set_response.error)
            assert get_set_response.value == value


@pytest.mark.launch(fixture=vimbax_camera_node)
@pytest.mark.parametrize("module", MODULE_LIST, ids=MODULE_NAME_LIST)
def test_feature_enum_get(test_node: TestNode, launch_context, module: FeatureModule):
    feature_info_query_service = test_node.create_client(
        FeatureInfoQuery, f"/{test_node.camera_node_name()}/feature_info_query"
    )
    assert feature_info_query_service.wait_for_service(10)
    feature_access_mode_service = test_node.create_client(
        FeatureAccessModeGet, f"/{test_node.camera_node_name()}/features/access_mode_get"
    )
    assert feature_access_mode_service.wait_for_service(10)
    feature_enum_get_service = test_node.create_client(
        FeatureEnumGet, f"/{test_node.camera_node_name()}/features/enum_get"
    )
    assert feature_enum_get_service.wait_for_service(10)

    feature_info_response = test_node.call_service_sync(
        feature_info_query_service, FeatureInfoQuery.Request(feature_module=module)
    )

    check_error(feature_info_response.error)
    assert len(feature_info_response.feature_info) > 0

    available_features = [
        info
        for info in feature_info_response.feature_info
        if info.data_type == FeatureDataType.ENUM.value
        and ensure_access_mode(
            feature_access_mode_service, info.name, writeable=False, module=module
        )
    ]

    for feature_info in available_features:
        response = test_node.call_service_sync(
            feature_enum_get_service,
            FeatureEnumGet.Request(feature_name=feature_info.name, feature_module=module),
        )
        check_error(response.error)


@pytest.mark.launch(fixture=vimbax_camera_node)
@pytest.mark.parametrize("module", MODULE_LIST, ids=MODULE_NAME_LIST)
def test_feature_enum_set(test_node: TestNode, launch_context, module: FeatureModule):
    features_ignore = [
        "TestPattern",
        "ExposureActiveMode",
        "DevicePowerSavingMode",
        "BalanceWhiteAuto",
    ]

    feature_info_query_service = test_node.create_client(
        FeatureInfoQuery, f"/{test_node.camera_node_name()}/feature_info_query"
    )
    assert feature_info_query_service.wait_for_service(10)
    feature_access_mode_service = test_node.create_client(
        FeatureAccessModeGet, f"/{test_node.camera_node_name()}/features/access_mode_get"
    )
    assert feature_access_mode_service.wait_for_service(10)
    feature_enum_get_service = test_node.create_client(
        FeatureEnumGet, f"/{test_node.camera_node_name()}/features/enum_get"
    )
    assert feature_enum_get_service.wait_for_service(10)
    feature_enum_set_service = test_node.create_client(
        FeatureEnumSet, f"/{test_node.camera_node_name()}/features/enum_set"
    )
    assert feature_enum_set_service.wait_for_service(10)
    feature_enum_info_service = test_node.create_client(
        FeatureEnumInfoGet, f"/{test_node.camera_node_name()}/features/enum_info_get"
    )
    assert feature_enum_info_service.wait_for_service(10)

    feature_info_response = test_node.call_service_sync(
        feature_info_query_service, FeatureInfoQuery.Request(feature_module=module)
    )

    check_error(feature_info_response.error)
    assert len(feature_info_response.feature_info) > 0

    available_features = [
        info
        for info in feature_info_response.feature_info
        if info.data_type == FeatureDataType.ENUM.value
        and ensure_access_mode(feature_access_mode_service, info.name, module=module)
    ]

    for feature_info in available_features:
        if feature_info.name in features_ignore:
            continue

        enum_get_response = test_node.call_service_sync(
            feature_enum_get_service,
            FeatureEnumGet.Request(feature_name=feature_info.name, feature_module=module),
        )
        check_error(enum_get_response.error)

        enum_info_response = test_node.call_service_sync(
            feature_enum_info_service,
            FeatureEnumInfoGet.Request(feature_name=feature_info.name, feature_module=module),
        )
        check_error(enum_info_response.error)

        default_value = enum_get_response.value

        for value in enum_info_response.available_values + [default_value]:
            print(f"{feature_info.name}: Setting value to {value}")
            set_request = FeatureEnumSet.Request(
                feature_name=feature_info.name, feature_module=module
            )
            set_request.value = value

            set_response = test_node.call_service_sync(feature_enum_set_service, set_request)
            check_error(set_response.error)

            get_set_response = test_node.call_service_sync(
                feature_enum_get_service,
                FeatureEnumGet.Request(feature_name=feature_info.name, feature_module=module),
            )
            check_error(get_set_response.error)
            assert get_set_response.value == value


@pytest.mark.launch(fixture=vimbax_camera_node)
@pytest.mark.parametrize("module", MODULE_LIST, ids=MODULE_NAME_LIST)
def test_feature_bool_get(test_node: TestNode, launch_context, module: FeatureModule):
    feature_info_query_service = test_node.create_client(
        FeatureInfoQuery, f"/{test_node.camera_node_name()}/feature_info_query"
    )
    assert feature_info_query_service.wait_for_service(10)
    feature_access_mode_service = test_node.create_client(
        FeatureAccessModeGet, f"/{test_node.camera_node_name()}/features/access_mode_get"
    )
    assert feature_access_mode_service.wait_for_service(10)
    feature_bool_get_service = test_node.create_client(
        FeatureBoolGet, f"/{test_node.camera_node_name()}/features/bool_get"
    )
    assert feature_bool_get_service.wait_for_service(10)

    feature_info_response = test_node.call_service_sync(
        feature_info_query_service, FeatureInfoQuery.Request(feature_module=module)
    )

    check_error(feature_info_response.error)
    assert len(feature_info_response.feature_info) > 0

    available_features = [
        info
        for info in feature_info_response.feature_info
        if info.data_type == FeatureDataType.BOOL.value
        and ensure_access_mode(
            feature_access_mode_service, info.name, writeable=False, module=module
        )
    ]

    for feature_info in available_features:
        response = test_node.call_service_sync(
            feature_bool_get_service,
            FeatureBoolGet.Request(feature_name=feature_info.name, feature_module=module),
        )
        check_error(response.error)


@pytest.mark.launch(fixture=vimbax_camera_node)
@pytest.mark.parametrize("module", MODULE_LIST, ids=MODULE_NAME_LIST)
def test_feature_bool_set(test_node: TestNode, launch_context, module: FeatureModule):
    features_ignore = ["TestPattern", "ExposureActiveMode", "DevicePowerSavingMode"]

    feature_info_query_service = test_node.create_client(
        FeatureInfoQuery, f"/{test_node.camera_node_name()}/feature_info_query"
    )
    assert feature_info_query_service.wait_for_service(10)
    feature_access_mode_service = test_node.create_client(
        FeatureAccessModeGet, f"/{test_node.camera_node_name()}/features/access_mode_get"
    )
    assert feature_access_mode_service.wait_for_service(10)
    feature_bool_get_service = test_node.create_client(
        FeatureBoolGet, f"/{test_node.camera_node_name()}/features/bool_get"
    )
    assert feature_bool_get_service.wait_for_service(10)
    feature_bool_set_service = test_node.create_client(
        FeatureBoolSet, f"/{test_node.camera_node_name()}/features/bool_set"
    )
    assert feature_bool_set_service.wait_for_service(10)

    feature_info_response = test_node.call_service_sync(
        feature_info_query_service, FeatureInfoQuery.Request(feature_module=module)
    )

    check_error(feature_info_response.error)
    assert len(feature_info_response.feature_info) > 0

    available_features = [
        info
        for info in feature_info_response.feature_info
        if info.data_type == FeatureDataType.BOOL.value
        and ensure_access_mode(feature_access_mode_service, info.name, module=module)
    ]

    for feature_info in available_features:
        if feature_info.name in features_ignore:
            continue

        default_get_response = test_node.call_service_sync(
            feature_bool_get_service,
            FeatureBoolGet.Request(feature_name=feature_info.name, feature_module=module),
        )
        check_error(default_get_response.error)

        default_value = default_get_response.value

        for value in [True, False, default_value]:
            print(f"{feature_info.name}: Setting value to {value}")
            set_request = FeatureBoolSet.Request(
                feature_name=feature_info.name, feature_module=module
            )
            set_request.value = value

            set_response = test_node.call_service_sync(feature_bool_set_service, set_request)
            check_error(set_response.error)

            get_set_response = test_node.call_service_sync(
                feature_bool_get_service,
                FeatureBoolGet.Request(feature_name=feature_info.name, feature_module=module),
            )
            check_error(get_set_response.error)


@pytest.mark.launch(fixture=vimbax_camera_node)
@pytest.mark.parametrize("module", MODULE_LIST, ids=MODULE_NAME_LIST)
def test_feature_string_get(test_node: TestNode, launch_context, module: FeatureModule):
    feature_info_query_service = test_node.create_client(
        FeatureInfoQuery, f"/{test_node.camera_node_name()}/feature_info_query"
    )
    assert feature_info_query_service.wait_for_service(10)
    feature_access_mode_service = test_node.create_client(
        FeatureAccessModeGet, f"/{test_node.camera_node_name()}/features/access_mode_get"
    )
    assert feature_access_mode_service.wait_for_service(10)
    feature_string_get_service = test_node.create_client(
        FeatureStringGet, f"/{test_node.camera_node_name()}/features/string_get"
    )
    assert feature_string_get_service.wait_for_service(10)

    feature_info_response = test_node.call_service_sync(
        feature_info_query_service, FeatureInfoQuery.Request(feature_module=module)
    )

    check_error(feature_info_response.error)
    assert len(feature_info_response.feature_info) > 0

    available_features = [
        info
        for info in feature_info_response.feature_info
        if info.data_type == FeatureDataType.STRING.value
        and ensure_access_mode(
            feature_access_mode_service, info.name, writeable=False, module=module
        )
    ]

    for feature_info in available_features:
        response = test_node.call_service_sync(
            feature_string_get_service,
            FeatureStringGet.Request(feature_name=feature_info.name, feature_module=module),
        )
        check_error(response.error)


@pytest.mark.launch(fixture=vimbax_camera_node)
@pytest.mark.parametrize("module", MODULE_LIST, ids=MODULE_NAME_LIST)
def test_feature_string_set(test_node: TestNode, launch_context, module: FeatureModule):
    feature_info_query_service = test_node.create_client(
        FeatureInfoQuery, f"/{test_node.camera_node_name()}/feature_info_query"
    )
    assert feature_info_query_service.wait_for_service(10)
    feature_access_mode_service = test_node.create_client(
        FeatureAccessModeGet, f"/{test_node.camera_node_name()}/features/access_mode_get"
    )
    assert feature_access_mode_service.wait_for_service(10)
    feature_string_get_service = test_node.create_client(
        FeatureStringGet, f"/{test_node.camera_node_name()}/features/string_get"
    )
    assert feature_string_get_service.wait_for_service(10)
    feature_string_set_service = test_node.create_client(
        FeatureStringSet, f"/{test_node.camera_node_name()}/features/string_set"
    )
    feature_string_set_service.wait_for_service(10)
    feature_string_info_service = test_node.create_client(
        FeatureStringInfoGet, f"/{test_node.camera_node_name()}/features/string_info_get"
    )
    feature_string_info_service.wait_for_service(10)

    feature_info_response = test_node.call_service_sync(
        feature_info_query_service, FeatureInfoQuery.Request(feature_module=module)
    )

    check_error(feature_info_response.error)
    assert len(feature_info_response.feature_info) > 0

    available_features = [
        info
        for info in feature_info_response.feature_info
        if info.data_type == FeatureDataType.STRING.value
        and ensure_access_mode(feature_access_mode_service, info.name, module=module)
    ]

    for feature_info in available_features:
        default_response = test_node.call_service_sync(
            feature_string_get_service,
            FeatureStringGet.Request(feature_name=feature_info.name, feature_module=module),
        )
        check_error(default_response.error)

        info_response = test_node.call_service_sync(
            feature_string_info_service,
            FeatureStringInfoGet.Request(feature_name=feature_info.name, feature_module=module),
        )
        check_error(info_response.error)

        single_char_str = "".join(random.choices(string.ascii_letters + string.digits, k=1))
        max_length_str = "".join(
            random.choices(string.ascii_letters + string.digits, k=info_response.max_length)
        )

        for value in [single_char_str, max_length_str, default_response.value]:
            print(f"Setting string to value {value}")
            set_request = FeatureStringSet.Request(
                feature_name=feature_info.name, feature_module=module
            )
            set_request.value = value
            set_response = test_node.call_service_sync(feature_string_set_service, set_request)

            check_error(set_response.error)

            get_set_response = test_node.call_service_sync(
                feature_string_get_service,
                FeatureStringGet.Request(feature_name=feature_info.name, feature_module=module),
            )

            check_error(get_set_response.error)
            assert get_set_response.value == value


@pytest.mark.launch(fixture=vimbax_camera_node)
@pytest.mark.parametrize("module", MODULE_LIST, ids=MODULE_NAME_LIST)
def test_feature_raw_get(test_node: TestNode, launch_context, module: FeatureModule):
    feature_info_query_service = test_node.create_client(
        FeatureInfoQuery, f"/{test_node.camera_node_name()}/feature_info_query"
    )
    assert feature_info_query_service.wait_for_service(10)
    feature_access_mode_service = test_node.create_client(
        FeatureAccessModeGet, f"/{test_node.camera_node_name()}/features/access_mode_get"
    )
    assert feature_access_mode_service.wait_for_service(10)
    feature_raw_get_service = test_node.create_client(
        FeatureRawGet, f"/{test_node.camera_node_name()}/features/raw_get"
    )
    assert feature_raw_get_service.wait_for_service(10)

    feature_info_response = test_node.call_service_sync(
        feature_info_query_service, FeatureInfoQuery.Request(feature_module=module)
    )

    check_error(feature_info_response.error)
    assert len(feature_info_response.feature_info) > 0

    available_features = [
        info
        for info in feature_info_response.feature_info
        if info.data_type == FeatureDataType.RAW.value
        and ensure_access_mode(
            feature_access_mode_service, info.name, writeable=False, module=module
        )
    ]

    for feature_info in available_features:
        response = test_node.call_service_sync(
            feature_raw_get_service,
            FeatureRawGet.Request(feature_name=feature_info.name, feature_module=module),
        )
        check_error(response.error)
        assert len(response.buffer) > 0
        assert len(response.buffer) == response.buffer_size
