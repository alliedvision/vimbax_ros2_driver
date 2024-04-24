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


import rclpy
from rclpy.node import Node
import argparse
import vimbax_camera_msgs.srv
from .helper import single_service_call, get_module_from_string, build_topic_path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("node_namespace")
    parser.add_argument(
        "-m",
        "--module",
        choices=["remote_device", "system", "interface", "local_device", "stream"],
        default="remote_device",
        dest="module",
    )

    (args, rosargs) = parser.parse_known_args()

    rclpy.init(args=rosargs)

    type_names = [
        "Unknown",
        "Int",
        "Float",
        "Enum",
        "String",
        "Bool",
        "Command",
        "Raw",
        "None",
        "?",
    ]

    def print_info_data(info):
        for entry in info.feature_info:
            print(f"  name: {entry.name}")
            print(f"   display_name: {entry.display_name}")
            print(f"   category: {entry.category}")
            print(f"   sfnc_namespace: {entry.sfnc_namespace}")
            print(f"   unit: {entry.unit}")
            type_name = type_names[max(min(len(type_names) - 1, entry.data_type), 0)]
            print(f"   data_type: {entry.data_type} ({type_name})")
            print(f"   polling_time: {entry.polling_time}")
            print(f"   flag_none: {entry.flags.flag_none}")
            print(f"   flag_read: {entry.flags.flag_read}")
            print(f"   flag_write: {entry.flags.flag_write}")
            print(f"   flag_volatile: {entry.flags.flag_volatile}")
            print(f"   flag_modify_write: {entry.flags.flag_modify_write}")

    node = Node("vimbax_feature_info_query_example")

    service_type = vimbax_camera_msgs.srv.FeatureInfoQuery

    # Build topic path from namespace and topic name
    topic: str = build_topic_path(args.node_namespace, "/feature_info_query")

    request = service_type.Request()
    request.feature_module = get_module_from_string(args.module)
    response = single_service_call(node, service_type, topic, request)

    if response.error.code == 0:
        print_info_data(response)
    else:
        print(f"Feature info query failed with {response.error}")
