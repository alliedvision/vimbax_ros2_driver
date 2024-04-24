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


from rclpy.node import Node

from rclpy.task import Future

from vimbax_camera_msgs.srv import SubscribeEvent, UnsubscribeEvent


class EventSubscribeException(Exception):
    def __init__(self, name, error):
        self.name = name
        self.error = error


class EventSubscription:
    def __init__(self, event_subscriber, evt_name: str):
        self.event_subscriber = event_subscriber
        self.evt_name = evt_name

    def destroy(self):
        self.event_subscriber.destroy_subscription(self)


class EventSubscriber:
    def __init__(self, evt_type, node: Node, topic: str) -> None:
        self._evt_type = evt_type
        self._node = node
        self._base_topic = topic
        self._event_subscribe_client = node.create_client(
            SubscribeEvent,
            f"{self._base_topic}/_event_subscribe"
        )
        self._event_unsubscribe_client = node.create_client(
            UnsubscribeEvent,
            f"{self._base_topic}/_event_unsubscribe"
        )
        self._ros_subscriptions = {}

    def subscribe_event(self, name: str, callback) -> Future:
        request = SubscribeEvent.Request()
        request.name = name

        subscription_future = Future()

        def on_event(data):
            callback(name, data)

        def on_subscribed(response):
            error = response.result().error
            if error.code != 0:
                subscription_future.set_exception(EventSubscribeException(name, error))
            else:
                subscription = self._node.create_subscription(
                    self._evt_type, f"{self._base_topic}/event_{name}", on_event, 10)
                self._ros_subscriptions[name] = subscription
                subscription_future.set_result(EventSubscription(self, name))

        self._event_subscribe_client.wait_for_service()
        self._event_subscribe_client.call_async(request).add_done_callback(on_subscribed)

        return subscription_future

    def destroy_subscription(self, event_subscription: EventSubscription):
        evt_name = event_subscription.evt_name
        self._ros_subscriptions[evt_name].destroy()
        del self._ros_subscriptions[evt_name]
        request = UnsubscribeEvent.Request()
        request.name = evt_name
        self._event_unsubscribe_client.call_async(request)
