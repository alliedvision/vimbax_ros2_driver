// Copyright (c) 2024 Allied Vision Technologies GmbH. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Allied Vision Technologies GmbH nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

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
