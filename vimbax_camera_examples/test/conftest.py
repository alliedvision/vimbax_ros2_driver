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
