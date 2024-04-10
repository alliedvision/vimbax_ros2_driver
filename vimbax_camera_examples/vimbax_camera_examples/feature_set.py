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
from .helper import (
    single_service_call,
    feature_type_dict,
    get_module_from_string,
    build_topic_path,
)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("node_namespace")
    parser.add_argument("feature_type", choices=feature_type_dict.keys())
    parser.add_argument("feature_name")
    parser.add_argument("feature_value")
    parser.add_argument(
        "-m",
        "--module",
        choices=["remote_device", "system", "interface", "local_device", "stream"],
        default="remote_device",
        dest="module",
    )

    (args, rosargs) = parser.parse_known_args()

    rclpy.init(args=rosargs)

    node = Node("vimbax_feature_set_example")

    feature_type = feature_type_dict[args.feature_type]

    # Build topic path from namespace and topic name
    topic: str = build_topic_path(args.node_namespace, f"/{feature_type.service_base_path}_set")

    request = feature_type.set_service_type.Request()
    request.feature_name = args.feature_name
    request.value = feature_type_dict[args.feature_type].value_type(args.feature_value)
    request.feature_module = get_module_from_string(args.module)
    response = single_service_call(node, feature_type.set_service_type, topic, request)

    if response.error.code == 0:
        print(f"Changed feature {args.feature_name} to {args.feature_value}")
    else:
        print(f"Setting feature {args.feature_name} value failed with {response.error}")
