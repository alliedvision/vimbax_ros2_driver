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

from enum import Enum

from typing import List

from vimbax_camera_msgs.msg import Error
from vimbax_camera_msgs.msg import FeatureInfo

from vimbax_camera_msgs.srv import FeatureAccessModeGet


class FeatureDataType(Enum):
    UNKNOWN = 0
    INT = 1
    FLOAT = 2
    ENUM = 3
    STRING = 4
    BOOL = 5
    COMMAND = 6
    RAW = 7
    NONE = 8


sfnc_namespaces = ["Standard", "Custom"]

features_ignore_map = {
    FeatureDataType.INT.value: [],
    FeatureDataType.FLOAT.value: ["LvWatchdogTimerDuration", "Gain"],
    FeatureDataType.ENUM.value: [],
    FeatureDataType.STRING.value: [],
    FeatureDataType.BOOL.value: [],
    FeatureDataType.COMMAND.value: [],
    FeatureDataType.RAW.value: [],
}


def check_error(error: Error):
    assert error.code == 0, f"Unexpected error {error.code} ({error.text})"


def ensure_access_mode(service, name, readable: bool = True, writeable: bool = True) -> bool:
    response = service.call(FeatureAccessModeGet.Request(feature_name=name))

    check_error(response.error)

    print(response)

    if readable and not response.is_readable:
        return False
    if writeable and not response.is_writeable:
        return False

    return True


def check_feature_info(feature_info: FeatureInfo):
    assert feature_info.name != ""
    assert feature_info.category != ""
    assert feature_info.display_name != ""
    assert feature_info.sfnc_namespace in sfnc_namespaces
    assert feature_info.data_type > 0 and feature_info.data_type < 9


def filter_features(
    features: List[FeatureInfo],
    acces_mode_service,
    type: FeatureDataType,
    readable: bool = True,
    writeable: bool = True,
):
    def check(feature: FeatureInfo):
        return (
            feature.data_type == type.value
            and feature.name not in features_ignore_map[feature.data_type]
            and ensure_access_mode(
                acces_mode_service, feature.name, readable=readable, writeable=writeable
            )
        )

    return [feat for feat in features if check(feat)]
