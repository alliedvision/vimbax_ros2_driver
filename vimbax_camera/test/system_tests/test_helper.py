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


from enum import Enum

from typing import List

from vimbax_camera_msgs.msg import Error
from vimbax_camera_msgs.msg import FeatureInfo
from vimbax_camera_msgs.msg import FeatureModule

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


def ensure_access_mode(
    service, name, readable: bool = True, writeable: bool = True, module: FeatureModule = None
) -> bool:

    if module is None:
        response = service.call(FeatureAccessModeGet.Request(feature_name=name))
    else:
        response = service.call(
            FeatureAccessModeGet.Request(feature_name=name, feature_module=module)
        )

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
