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

import time

from vimbax_camera_msgs.srv import StreamStartStop
from vimbax_camera_msgs.srv import Status

from conftest import vimbax_camera_node, TestNode

from test_helper import check_error


@pytest.mark.launch(fixture=vimbax_camera_node)
def test_stream_start_stop_services(test_node: TestNode, launch_context):
    start_service = test_node.create_client(
        StreamStartStop, f"/{test_node.camera_node_name()}/stream_start"
    )
    assert start_service.wait_for_service(10)
    stop_service = test_node.create_client(
        StreamStartStop, f"/{test_node.camera_node_name()}/stream_stop"
    )
    assert stop_service.wait_for_service(10)


@pytest.mark.launch(fixture=vimbax_camera_node)
def test_stream_auto_stream_start_stop(test_node: TestNode, launch_context):
    status_service = test_node.create_client(Status, f"/{test_node.camera_node_name()}/status")
    assert status_service.wait_for_service(10)

    status = test_node.call_service_sync(status_service, Status.Request())
    assert not status.streaming
    test_node.subscribe_image_raw()

    test_node.clear_queue()
    assert test_node.wait_for_frame(5.0)
    # Camera needs time to detect graph change
    time.sleep(2.0)
    status = test_node.call_service_sync(status_service, Status.Request())
    assert status.streaming
    test_node.unsubscribe_image_raw()
    time.sleep(2.0)
    assert test_node.image_queue.empty()

    status = test_node.call_service_sync(status_service, Status.Request())
    assert not status.streaming


@pytest.mark.launch(fixture=vimbax_camera_node)
def test_streaming(test_node: TestNode, launch_context):
    test_node.subscribe_image_raw()
    assert test_node.wait_for_frame(5.0)
    frame_count = 0
    for i in range(100):
        assert test_node.wait_for_frame(5.0)
        frame_count += 1

    assert frame_count == 100

    test_node.unsubscribe_image_raw()


@pytest.mark.launch(fixture=vimbax_camera_node)
def test_stream_manual_start_stop(test_node: TestNode, launch_context):
    status_service = test_node.create_client(Status, f"/{test_node.camera_node_name()}/status")
    assert status_service.wait_for_service(10)

    start_service = test_node.create_client(
        StreamStartStop, f"/{test_node.camera_node_name()}/stream_start"
    )
    assert start_service.wait_for_service(1)
    stop_service = test_node.create_client(
        StreamStartStop, f"/{test_node.camera_node_name()}/stream_stop"
    )
    assert stop_service.wait_for_service(1)

    status = test_node.call_service_sync(status_service, Status.Request())
    assert not status.streaming
    test_node.subscribe_image_raw()

    assert test_node.wait_for_frame(10.0)
    # Camera needs time to detect graph change
    time.sleep(1.0)
    status = test_node.call_service_sync(status_service, Status.Request())
    assert status.streaming

    stop_result = stop_service.call(StreamStartStop.Request())
    check_error(stop_result.error)

    status = test_node.call_service_sync(status_service, Status.Request())
    assert not status.streaming

    start_result = start_service.call(StreamStartStop.Request())
    check_error(start_result.error)

    assert test_node.wait_for_frame(10.0)

    status = test_node.call_service_sync(status_service, Status.Request())
    assert status.streaming

    test_node.unsubscribe_image_raw()
    time.sleep(2.0)
    assert test_node.image_queue.empty()

    status = test_node.call_service_sync(status_service, Status.Request())
    assert status is not None, "GetStatus service call did not complete!"
    assert not status.streaming
