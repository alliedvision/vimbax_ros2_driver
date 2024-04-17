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

#ifndef VIMBAX_CAMERA_EVENTS__EVENT_PUBLISHER_BASE_HPP_
#define VIMBAX_CAMERA_EVENTS__EVENT_PUBLISHER_BASE_HPP_

#include <string>
#include <memory>
#include <functional>
#include <map>

#include <rclcpp/rclcpp.hpp>

#include <vimbax_camera_msgs/msg/error.hpp>

#include <vimbax_camera_msgs/srv/subscribe_event.hpp>
#include <vimbax_camera_msgs/srv/unsubscribe_event.hpp>

namespace vimbax_camera_events
{
using vimbax_camera_msgs::msg::Error;

class EventPublisherBase
{
public:
  using OnEventSubscribed = std::function<Error(const std::string &)>;
  using OnEventUnsubscribed = std::function<void (const std::string &)>;

  EventPublisherBase(
    std::shared_ptr<rclcpp::Node> node,
    const std::string & topic_name,
    OnEventSubscribed on_event_subscribed,
    OnEventUnsubscribed on_event_unsubscribed);

protected:
  rclcpp::PublisherBase::SharedPtr get_event_publisher(const std::string & event);

  virtual rclcpp::PublisherBase::SharedPtr create_event_publisher(
    std::shared_ptr<rclcpp::Node> node,
    const std::string & topic_name,
    const rclcpp::QoS & qos
  ) = 0;

private:
  struct SubscribtionDetail
  {
    rclcpp::PublisherBase::SharedPtr publisher_;
    std::atomic_size_t count_;
  };

  void unsubscribe(const std::string & event);

  std::string base_topic_name_;
  rclcpp::Node::SharedPtr node_;
  OnEventUnsubscribed on_event_unsubscribed_;

  rclcpp::Service<vimbax_camera_msgs::srv::SubscribeEvent>::SharedPtr subscription_service_;
  rclcpp::Service<vimbax_camera_msgs::srv::UnsubscribeEvent>::SharedPtr unsubscription_service_;
  std::map<std::string, std::unique_ptr<SubscribtionDetail>> subscribtion_detail_map_;

  rclcpp::TimerBase::SharedPtr event_check_timer_;
};

}  // namespace vimbax_camera_events

#endif  // VIMBAX_CAMERA_EVENTS__EVENT_PUBLISHER_BASE_HPP_
