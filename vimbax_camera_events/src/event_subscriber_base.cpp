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
