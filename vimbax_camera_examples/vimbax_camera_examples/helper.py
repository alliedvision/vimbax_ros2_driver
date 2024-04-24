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
import rclpy.client
from rclpy.node import Node
import vimbax_camera_msgs.srv as srv
import vimbax_camera_msgs.msg as msg


def wait_for_service(client: rclpy.client.Client):
    if not client.wait_for_service(5.0):
        print("Waiting for service...", end='', flush=True)
        for i in range(30):
            if client.wait_for_service(5.0):
                print("")
                return True
            else:
                print(f"{(i+1) * 5}...", end='', flush=True)

        print("Timeout!")
        return False
    else:
        return True


def single_service_call(node: Node, type, name, request):
    client = node.create_client(type, name)

    if not wait_for_service(client):
        print("Service got not ready in time")
        exit(1)

    future = client.call_async(request)

    rclpy.spin_until_future_complete(node, future)

    return future.result()


class FeatureTypeInfo:
    def __init__(
        self,
        value_type,
        get_service_type,
        set_service_type,
        info_service_type,
        service_base_path,
        encode=None,
    ) -> None:
        self.value_type = value_type
        self.get_service_type = get_service_type
        self.set_service_type = set_service_type
        self.info_service_type = info_service_type
        self.service_base_path = service_base_path
        self.encode = encode


feature_type_dict = {
    "Int": FeatureTypeInfo(
        int, srv.FeatureIntGet, srv.FeatureIntSet, srv.FeatureIntInfoGet, "features/int"
    ),
    "Float": FeatureTypeInfo(
        float, srv.FeatureFloatGet, srv.FeatureFloatSet, srv.FeatureFloatInfoGet, "features/float"
    ),
    "String": FeatureTypeInfo(
        str,
        srv.FeatureStringGet,
        srv.FeatureStringSet,
        srv.FeatureStringInfoGet,
        "features/string",
    ),
    "Bool": FeatureTypeInfo(bool, srv.FeatureBoolGet, srv.FeatureBoolSet, None, "features/bool"),
    "Enum": FeatureTypeInfo(
        str, srv.FeatureEnumGet, srv.FeatureEnumSet, srv.FeatureEnumInfoGet, "features/enum"
    ),
    "Raw": FeatureTypeInfo(
        [int], srv.FeatureRawGet, srv.FeatureRawSet, srv.FeatureRawInfoGet, "features/raw"
    ),
}


def print_feature_info(info):
    if isinstance(info, srv.FeatureIntInfoGet.Response):
        print(f"min: {info.min} max: {info.max} inc: {info.inc}")
    elif isinstance(info, srv.FeatureFloatInfoGet.Response):
        if info.inc_available:
            print(f"min: {info.min} max: {info.max} inc: {info.inc}")
        else:
            print(f"min: {info.min} max: {info.max}")
    elif isinstance(info, srv.FeatureStringInfoGet.Response) or isinstance(
        info, srv.FeatureRawInfoGet.Response
    ):
        print(f"max length: {info.max_length}")
    elif isinstance(info, srv.FeatureEnumInfoGet.Response):
        print(f"all: {info.possible_values} available: {info.available_values}")
    else:
        print(f"Unknown feature info type {type(info)}")


def get_module_from_string(module_str):
    if module_str == "remote_device":
        return msg.FeatureModule(id=msg.FeatureModule.MODULE_REMOTE_DEVICE)
    elif module_str == "system":
        return msg.FeatureModule(id=msg.FeatureModule.MODULE_SYSTEM)
    elif module_str == "interface":
        return msg.FeatureModule(id=msg.FeatureModule.MODULE_INTERFACE)
    elif module_str == "local_device":
        return msg.FeatureModule(id=msg.FeatureModule.MODULE_LOCAL_DEVICE)
    elif module_str == "stream":
        return msg.FeatureModule(id=msg.FeatureModule.MODULE_STREAM)


def build_topic_path(namespace: str, topic: str):
    """Build topic path and handle root namespace."""
    namespace = namespace.strip("/")
    topic: str = f"/{topic.strip('/')}"
    if len(namespace) != 0:
        topic = f"/{namespace}/{topic.strip('/')}"
    return topic
