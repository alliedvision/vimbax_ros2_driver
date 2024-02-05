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

#ifndef VIMBAX_CAMERA__VIMBAX_CAMERA_NODE_HPP_
#define VIMBAX_CAMERA__VIMBAX_CAMERA_NODE_HPP_

#include <string>
#include <memory>
#include <thread>
#include <mutex>
#include <memory_resource>
#include <atomic>

#include <rclcpp/rclcpp.hpp>

#include <image_transport/image_transport.hpp>

#include <vimbax_camera_msgs/srv/feature_int_get.hpp>
#include <vimbax_camera_msgs/srv/feature_int_set.hpp>
#include <vimbax_camera_msgs/srv/feature_float_get.hpp>
#include <vimbax_camera_msgs/srv/feature_float_set.hpp>
#include <vimbax_camera_msgs/srv/feature_string_get.hpp>
#include <vimbax_camera_msgs/srv/feature_string_set.hpp>

#include <vimbax_camera/loader/vmbc_api.hpp>
#include <vimbax_camera/vimbax_camera.hpp>


namespace vimbax_camera
{
class VimbaXCameraNode
{
public:
  using NodeBaseInterface = rclcpp::node_interfaces::NodeBaseInterface;

  explicit VimbaXCameraNode(const rclcpp::NodeOptions & options);
  ~VimbaXCameraNode();

  NodeBaseInterface::SharedPtr get_node_base_interface() const;

private:
  using OnSetParametersCallbackHandle = rclcpp::Node::OnSetParametersCallbackHandle;

  static std::string get_node_name();

  bool initialize_parameters();
  bool initialize_api();
  bool initialize_publisher();
  bool initialize_camera();
  bool initialize_services();
  bool initialize_graph_notify();

  void start_streaming();
  void stop_streaming();

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<VmbCAPI> api_;
  std::shared_ptr<VimbaXCamera> camera_;

  image_transport::Publisher image_publisher_;

  OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

  rclcpp::Service<vimbax_camera_msgs::srv::FeatureIntGet>::SharedPtr feature_int_get_service_;
  rclcpp::Service<vimbax_camera_msgs::srv::FeatureIntSet>::SharedPtr feature_int_set_service_;
  rclcpp::Service<vimbax_camera_msgs::srv::FeatureFloatGet>::SharedPtr feature_float_get_service_;
  rclcpp::Service<vimbax_camera_msgs::srv::FeatureFloatSet>::SharedPtr feature_float_set_service_;
  rclcpp::Service<vimbax_camera_msgs::srv::FeatureStringGet>::SharedPtr feature_string_get_service_;
  rclcpp::Service<vimbax_camera_msgs::srv::FeatureStringSet>::SharedPtr feature_string_set_service_;

  std::unique_ptr<std::thread> graph_notify_thread_;
  std::atomic_bool stop_threads_{false};
};

}  // namespace vimbax_camera

#endif  // VIMBAX_CAMERA__VIMBAX_CAMERA_NODE_HPP_
