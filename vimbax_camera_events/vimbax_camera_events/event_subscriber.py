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

from rclpy.node import Node

from vimbax_camera_msgs.srv import SubscribeEvent, UnsubscribeEvent


class EventSubscribtion:
    def __init__(self, event_subscriber, evt_name: str):
        self.event_subscriber = event_subscriber
        self.evt_name = evt_name

    def destroy(self):
        self.event_subscriber.destroy_subscribtion(self)


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
        self._ros_subscribtions = {}

    def subscribe_event(self, event: str, callback) -> EventSubscribtion:
        request = SubscribeEvent.Request()
        request.name = event

        def on_subscribed(response):
            subscribtion = self._node.create_subscription(
                self._evt_type, f"{self._base_topic}/event_{event}", callback, 10)
            self._ros_subscribtions[event] = subscribtion

        self._event_subscribe_client.wait_for_service()
        self._event_subscribe_client.call_async(request).add_done_callback(on_subscribed)

        return EventSubscribtion(self, event)

    def destroy_subscribtion(self, event_subscribtion: EventSubscribtion):
        evt_name = event_subscribtion.evt_name
        self._ros_subscribtions[evt_name].destroy()
        del self._ros_subscribtions[evt_name]
        request = UnsubscribeEvent.Request()
        request.name = evt_name
        self._event_unsubscribe_client.call_async(request)
