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
from .helper import single_service_call, feature_type_dict


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("node_name")
    parser.add_argument("feature_type", choices=feature_type_dict.keys())
    parser.add_argument("feature_name")
    parser.add_argument("feature_value")

    (args, rosargs) = parser.parse_known_args()

    rclpy.init(args=rosargs)

    node = Node("_feature_set")

    feature_type = feature_type_dict[args.feature_type]

    request = feature_type.set_service_type.Request()
    request.feature_name = args.feature_name
    request.value = feature_type_dict[args.feature_type].value_type(args.feature_value)
    response = single_service_call(node, feature_type.set_service_type,
                                   f"{args.node_name}/{feature_type.service_base_path}_set",
                                   request)

    if response.error == 0:
        print(f"Changed feature {args.feature_name} to {args.feature_value}")
    else:
        print(f"Setting feature {args.feature_name} value failed with {response.error}")
