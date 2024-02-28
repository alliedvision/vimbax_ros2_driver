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

#include <vimbax_camera_events/event_subscriber_base.hpp>

namespace vimbax_camera_events
{
EventSubscriberBase::EventSubscriberBase(rclcpp::Node::SharedPtr node, const std::string & topic)
: std::enable_shared_from_this<EventSubscriberBase>(), base_topic_{topic}, node_{node}
{
  subscribe_service_client_ = node_->create_client<vimbax_camera_msgs::srv::SubscribeEvent>(
    subscribe_topic_name(base_topic_)
  );

  unsubscribe_service_client_ = node_->create_client<vimbax_camera_msgs::srv::UnsubscribeEvent>(
    unsubscribe_topic_name(base_topic_)
  );

  using namespace std::chrono_literals;
  subscribe_service_client_->wait_for_service(10s);
  unsubscribe_service_client_->wait_for_service(10s);
}
}  // namespace vimbax_camera_events
