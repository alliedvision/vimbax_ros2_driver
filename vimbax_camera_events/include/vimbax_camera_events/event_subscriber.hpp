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

#ifndef VIMBAX_CAMERA_EVENTS__EVENT_SUBSCRIBER_HPP_
#define VIMBAX_CAMERA_EVENTS__EVENT_SUBSCRIBER_HPP_

#include <memory>
#include <functional>
#include <future>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <vimbax_camera_events/event_subscriber_base.hpp>

namespace vimbax_camera_events
{

template<typename T>
class EventSubscriber : public EventSubscriberBase
{
public:
  template<typename ... Args>
  static std::shared_ptr<EventSubscriber<T>> make_shared(Args... args)
  {
    return std::shared_ptr<EventSubscriber<T>>(new EventSubscriber<T>(args ...));
  }

  std::shared_future<std::shared_ptr<EventSubscription<T>>> subscribe_event(
    const std::string & event, std::function<void(const T &)> callback)
  {
    return EventSubscriberBase::subscribe_event(event, callback);
  }

private:
  EventSubscriber(rclcpp::Node::SharedPtr node, const std::string & topic)
  : EventSubscriberBase(node, topic)
  {
  }
};

}  // namespace vimbax_camera_events

#endif  // VIMBAX_CAMERA_EVENTS__EVENT_SUBSCRIBER_HPP_
