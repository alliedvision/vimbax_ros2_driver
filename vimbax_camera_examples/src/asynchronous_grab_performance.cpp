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
#include <rclcpp_components/register_node_macro.hpp>

#include <image_transport/image_transport.hpp>


namespace vimbax_camera_examples
{
class AsynchronousGrabPerformance : public rclcpp::Node
{
public:
  explicit AsynchronousGrabPerformance(const rclcpp::NodeOptions & options)
  : rclcpp::Node("_asynchronous_grab_performance", options)
  {
    auto subscription_options = rclcpp::SubscriptionOptions();
    subscription_options.event_callbacks.message_lost_callback =
      [this](rclcpp::QOSMessageLostInfo & info) {
        std::cout << info.total_count_change << " missing frames detected!!!" << std::endl;
        frames_missing += info.total_count_change;
      };

    subscription = image_transport::create_subscription(
      this, "/image",
      [this](sensor_msgs::msg::Image::ConstSharedPtr imgmsg) {
        auto const timestamp = std::chrono::nanoseconds{imgmsg->header.stamp.nanosec} +
        std::chrono::seconds{imgmsg->header.stamp.sec};

        auto const diff =
        std::chrono::duration_cast<std::chrono::microseconds>(timestamp - last_timestamp).count();

        auto const mfps = 1000000000 / diff;

        std::cout << "Got frame diff " << diff << " mfps: " << mfps << std::endl;

        last_timestamp = timestamp;
        frame_complete++;
      }, "raw", rmw_qos_profile_default, subscription_options);
  }

  ~AsynchronousGrabPerformance()
  {
    std::cout << "Missed " << frames_missing << "/" << frame_complete << " frames" << std::endl;
  }

private:
  image_transport::Subscriber subscription;
  int64_t last_frame_id{-1};
  int64_t frames_missing{0};
  int64_t frame_complete{0};
  std::chrono::nanoseconds last_timestamp{0};
  std::deque<uint64_t> diffs{};
};

}  // namespace vimbax_camera_examples

RCLCPP_COMPONENTS_REGISTER_NODE(vimbax_camera_examples::AsynchronousGrabPerformance)
