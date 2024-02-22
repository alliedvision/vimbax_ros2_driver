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

#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include <vimbax_camera_events/event_subscriber.hpp>

#include <vimbax_camera_msgs/msg/event_data.hpp>


int main(int argc, char * argv[])
{
  auto const args = rclcpp::init_and_remove_ros_arguments(argc, argv);

  if (args.size() < 2) {
    std::cerr << "Usage: " + args[0] + " <event name>" << std::endl;
    return 1;
  }

  auto node = rclcpp::Node::make_shared("_event_viewer");

  auto event_subscriber =
    vimbax_camera_events::EventSubscriber<vimbax_camera_msgs::msg::EventData>::make_shared(
    node,
    "/vimbax_camera_test/events"
    );


  auto event_subscibtion = event_subscriber->subscribe_event(
    args[1], [&](auto event_data) {
      RCLCPP_INFO(node->get_logger(), "Got event meta data:");
      for (auto const & entry : event_data.entries) {
        RCLCPP_INFO(node->get_logger(), "%s: %s", entry.name.c_str(), entry.value.c_str());
      }
    });


  rclcpp::spin(node);

  return 0;
}
