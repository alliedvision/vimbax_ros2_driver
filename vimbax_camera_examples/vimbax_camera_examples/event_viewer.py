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

from vimbax_camera_events.event_subscriber import EventSubscriber, EventSubscribeException
from vimbax_camera_msgs.msg import EventData


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("node_namespace")
    parser.add_argument("events", nargs="+")

    (args, rosargs) = parser.parse_known_args()

    rclpy.init(args=rosargs)

    node = Node("_feature_command_execute")

    event_subscriber = EventSubscriber(EventData, node, f"{args.node_namespace}/events")

    def print_event_data(event):
        for entry in event.entries:
            print(f"  {entry.name}: {entry.value}")

    event_subscriptions = []
    pending_subscriptions = set()

    for event_name in args.events:

        def event_callback(name, data):
            print(f"Got event {name}")
            print_event_data(data)

        def on_subscribed(future):
            pending_subscriptions.remove(future)
            try:
                event_subscriptions.append(future.result())
            except EventSubscribeException as ex:
                print(f"Subscribing to event {ex.name} failed with "
                      f"{ex.error.code} ({ex.error.text})")
                # Stop executor if no subscription is pending or active
                if len(pending_subscriptions) == 0 and len(event_subscriptions) == 0:
                    rclpy.shutdown()

        future = event_subscriber.subscribe_event(event_name, event_callback)

        future.add_done_callback(on_subscribed)

        pending_subscriptions.add(future)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    for event_subscription in event_subscriptions:
        event_subscription.destroy()
