// Copyright 2024 Allied Vision Technologies GmbH. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef VIMBAX_CAMERA_EVENTS__EVENT_PUBLISHER_HPP_
#define VIMBAX_CAMERA_EVENTS__EVENT_PUBLISHER_HPP_

#include <string>
#include <memory>
#include <functional>
#include <map>

#include <rclcpp/rclcpp.hpp>

#include <vimbax_camera_events/event_publisher_base.hpp>


namespace vimbax_camera_events
{

template<typename T>
class EventPublisher : public EventPublisherBase
{
public:
  using SharedPtr = std::shared_ptr<EventPublisher<T>>;

  EventPublisher(
    rclcpp::Node::SharedPtr node,
    const std::string & topic_name,
    OnEventSubscribed on_event_subscribed,
    OnEventUnsubscribed on_event_unsubscribed)
  : EventPublisherBase(node, topic_name, on_event_subscribed, on_event_unsubscribed)
  {
  }


  void publish_event(const std::string & event_name, const T & event)
  {
    auto publisher =
      std::dynamic_pointer_cast<rclcpp::Publisher<T>>(get_event_publisher(event_name));

    if (publisher) {
      publisher->publish(event);
    }
  }

protected:
  rclcpp::PublisherBase::SharedPtr create_event_publisher(
    std::shared_ptr<rclcpp::Node> node,
    const std::string & topic_name,
    const rclcpp::QoS & qos) override
  {
    return node->create_publisher<T>(topic_name, qos);
  }
};

}  // namespace vimbax_camera_events

#endif  // VIMBAX_CAMERA_EVENTS__EVENT_PUBLISHER_HPP_
