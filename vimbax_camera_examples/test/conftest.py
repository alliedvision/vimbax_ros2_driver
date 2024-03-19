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
import rclpy
import random
import string
from vimbax_camera_msgs.msg import Error
from launch_pytest.tools.process import wait_for_exit_sync


@pytest.fixture
def node_test_id():
    return "".join(random.choices(string.ascii_lowercase + string.digits, k=8))


@pytest.fixture
def camera_test_node_name(node_test_id):
    return f"vimbax_camera_pytest_{node_test_id}"


@pytest.fixture(autouse=True)
def init_shutdown_ros():
    rclpy.init()

    yield

    rclpy.shutdown()


def check_error(error: Error):
    assert error.code == 0, f"Unexpected error {error.code} ({error.text})"


def call_service(srv_type, url, req):
    node = rclpy.create_node("_test_feature_get")
    client = node.create_client(srv_type, url)

    assert client.wait_for_service(timeout_sec=10.0), f"Service {url} not reachable"
    fut = client.call_async(req)
    rclpy.spin_until_future_complete(node, future=fut, timeout_sec=10.0)
    assert fut.done()
    res: srv_type.Response = fut.result()
    assert res is not None
    check_error(res.error)
    node.destroy_node()
    return res


def assert_clean_shutdown(launch_context, action):

    assert wait_for_exit_sync(
        launch_context, action, timeout=10.0
    ), "Process did not finish in 10 seconds"

    assert action.return_code is not None, "Process should have exited here!"

    assert action.return_code == 0, "The Process should not return an error!"
