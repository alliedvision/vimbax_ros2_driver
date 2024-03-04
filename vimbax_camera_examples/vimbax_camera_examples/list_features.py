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
import argparse
import vimbax_camera_msgs.srv
from .helper import single_service_call


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("node_name")

    (args, rosargs) = parser.parse_known_args()

    rclpy.init(args=rosargs)

    type_names = ["Unknown", "Int", "Float", "Enum", "String", "Bool", "Command",
                   "Raw", "None", "?"]

    def print_info_data(info):
        for entry in info.feature_info:
            print(f"  name: {entry.name}")
            print(f"   display_name: {entry.display_name}")
            print(f"   category: {entry.category}")
            print(f"   sfnc_namespace: {entry.sfnc_namespace}")
            print(f"   unit: {entry.unit}")
            type_name = type_names[max(min(len(type_names)-1,entry.data_type),0)]
            print(f"   data_type: {entry.data_type} ({type_name})")
            print(f"   polling_time: {entry.polling_time}")
            print(f"   flag_none: {entry.flags.flag_none}")
            print(f"   flag_read: {entry.flags.flag_read}")
            print(f"   flag_write: {entry.flags.flag_write}")
            print(f"   flag_volatile: {entry.flags.flag_volatile}")
            print(f"   flag_modify_write: {entry.flags.flag_modify_write}")

    node = Node("_feature_info_query")

    service_type = vimbax_camera_msgs.srv.FeatureInfoQuery

    request = service_type.Request()
    response = single_service_call(
        node, service_type, f"{args.node_name}/feature_info_query", request)

    if response.error.code == 0:
        print_info_data(response)
    else:
        print(f"Settings {args.operation} failed with {response.error}")