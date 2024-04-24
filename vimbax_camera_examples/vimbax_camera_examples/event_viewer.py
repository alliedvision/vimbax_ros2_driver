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
from .helper import build_topic_path

from vimbax_camera_events.event_subscriber import EventSubscriber, EventSubscribeException
from vimbax_camera_msgs.msg import EventData


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("node_namespace")
    parser.add_argument("events", nargs="+")

    (args, rosargs) = parser.parse_known_args()

    rclpy.init(args=rosargs)

    node = Node("vimbax_feature_command_execute_example")

    # Build topic path from namespace and topic name
    topic: str = build_topic_path(args.node_namespace, '/events')

    event_subscriber = EventSubscriber(EventData, node, topic)

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
