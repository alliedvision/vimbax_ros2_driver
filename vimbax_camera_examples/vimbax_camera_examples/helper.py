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

import rclpy
from rclpy.node import Node
import vimbax_camera_msgs.srv as srv


def single_service_call(node: Node, type, name, request):
    client = node.create_client(type, name)

    if not client.wait_for_service(120.0):
        print("Service got not ready in time")
        exit(1)

    future = client.call_async(request)

    rclpy.spin_until_future_complete(node, future)

    return future.result()


class FeatureTypeInfo:
    def __init__(self, value_type, get_service_type, set_service_type, info_service_type,
                 service_base_path, encode=None) -> None:
        self.value_type = value_type
        self.get_service_type = get_service_type
        self.set_service_type = set_service_type
        self.info_service_type = info_service_type
        self.service_base_path = service_base_path
        self.encode = encode


feature_type_dict = {
    "Int": FeatureTypeInfo(int, srv.FeatureIntGet,
                           srv.FeatureIntSet, srv.FeatureIntInfoGet, "features/int"),
    "Float": FeatureTypeInfo(float, srv.FeatureFloatGet,
                             srv.FeatureFloatSet, srv.FeatureFloatInfoGet, "features/float"),
    "String": FeatureTypeInfo(str, srv.FeatureStringGet,
                              srv.FeatureStringSet, srv.FeatureStringInfoGet, "features/string"),
    "Bool": FeatureTypeInfo(bool, srv.FeatureBoolGet,
                            srv.FeatureBoolSet, None, "features/bool"),
    "Enum": FeatureTypeInfo(str, srv.FeatureEnumGet,
                            srv.FeatureEnumSet, srv.FeatureEnumInfoGet, "features/enum"),
    "Raw": FeatureTypeInfo([int], srv.FeatureRawGet,
                           srv.FeatureRawSet, srv.FeatureRawInfoGet, "features/raw"),
}


def print_feature_info(info):
    if isinstance(info, srv.FeatureIntInfoGet.Response):
        print(f"min: {info.min} max: {info.max} inc: {info.inc}")
    elif isinstance(info, srv.FeatureFloatInfoGet.Response):
        if info.inc_available:
            print(f"min: {info.min} max: {info.max} inc: {info.inc}")
        else:
            print(f"min: {info.min} max: {info.max}")
    elif (isinstance(info, srv.FeatureStringInfoGet.Response)
          or isinstance(info, srv.FeatureRawInfoGet.Response)):
        print(f"max length: {info.max_length}")
    elif isinstance(info, srv.FeatureEnumInfoGet.Response):
        print(f"all: {info.possible_values} available: {info.available_values}")
    else:
        print(f"Unknown feature info type {type(info)}")
