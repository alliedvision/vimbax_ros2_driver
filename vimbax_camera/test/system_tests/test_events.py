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

from threading import Event

from vimbax_camera_msgs.srv import FeatureEnumInfoGet
from vimbax_camera_msgs.srv import FeatureCommandRun

from vimbax_camera_msgs.msg import EventData

from vimbax_camera_events.event_subscriber import EventSubscriber

from conftest import vimbax_camera_node, TestNode


@pytest.mark.launch(fixture=vimbax_camera_node)
def test_genicam_events(test_node: TestNode, launch_context):
    enum_info_service = test_node.create_client(
        FeatureEnumInfoGet, f"{test_node.camera_node_name()}/features/enum_info_get"
    )
    assert enum_info_service.wait_for_service(10)
    command_run_service = test_node.create_client(
        FeatureCommandRun, f"{test_node.camera_node_name()}/features/command_run"
    )
    assert command_run_service.wait_for_service(10)

    enum_info_request = FeatureEnumInfoGet.Request(feature_name="EventSelector")
    enum_info_response = test_node.call_service_sync(enum_info_service, enum_info_request)
    if "Test" not in enum_info_response.available_values:
        pytest.skip("Test event not supported")

    subscriber = EventSubscriber(EventData, test_node, f"{test_node.camera_node_name()}/events")

    on_event_event = Event()
    on_subscription_ready = Event()

    def on_event(name, data):
        print("Got event")
        on_event_event.set()

    def on_event_subscribed(future):
        future.result()
        on_subscription_ready.set()

    subscriber.subscribe_event("Test", on_event).add_done_callback(on_event_subscribed)

    assert on_subscription_ready.wait(5)

    run_result = test_node.call_service_sync(
        command_run_service, FeatureCommandRun.Request(feature_name="TestEventGenerate")
    )

    assert run_result is not None, "The service call timed out"
    assert run_result.error.code == 0
    assert on_event_event.wait(1)
