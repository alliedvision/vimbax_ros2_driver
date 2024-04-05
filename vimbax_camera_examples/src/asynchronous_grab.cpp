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
#include <queue>

#include <rclcpp/rclcpp.hpp>

#include <image_transport/image_transport.hpp>


int main(int argc, char * argv[])
{
  auto const args = rclcpp::init_and_remove_ros_arguments(argc, argv);

  if (args.size() < 2) {
    std::cout << "Usage asynchronous_grab_cpp <vimbax camera node name>" << std::endl;
    return 1;
  }

  auto node = rclcpp::Node::make_shared("_asynchronous_grab_cpp");

  int64_t frames_missing = 0;
  int64_t frame_complete = 0;
  std::chrono::nanoseconds last_timestamp{0};

  auto const topic_name = "/" + args[1] + "/image_raw";

  auto subscription_options = rclcpp::SubscriptionOptions();
  subscription_options.event_callbacks.message_lost_callback =
    [&frames_missing](rclcpp::QOSMessageLostInfo & info) {
      std::cout << info.total_count_change << " missing frames detected!!!" << std::endl;
      frames_missing += info.total_count_change;
    };

  auto subscription = image_transport::create_subscription(
    node.get(), topic_name,
    [&](sensor_msgs::msg::Image::ConstSharedPtr imgmsg) {
      auto const timestamp = std::chrono::nanoseconds{imgmsg->header.stamp.nanosec} +
      std::chrono::seconds{imgmsg->header.stamp.sec};

      auto const diff =
      std::chrono::duration_cast<std::chrono::microseconds>(timestamp - last_timestamp).count();

      auto const mfps = 1000000000 / diff;

      std::cout << "Got frame diff " << diff << " mfps: " << mfps << std::endl;

      last_timestamp = timestamp;
      frame_complete++;
    }, "raw", rmw_qos_profile_default, subscription_options);

  rclcpp::spin(node);

  std::cout << "Missed " << frames_missing << "/" << frame_complete + frames_missing << " frames"
            << std::endl;
}
