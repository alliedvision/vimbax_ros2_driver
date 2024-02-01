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

#include <vimbax_camera_msgs/srv/settings_load_save.hpp>

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

  const std::string parameter_camera_id = "camera_id";
  const std::string parameter_settings_file = "settings_file";
  const std::string parameter_buffer_count = "buffer_count";

  static std::string get_node_name();

  bool initialize_parameters();
  bool initialize_api();
  bool initialize_publisher();
  bool initialize_camera();
  bool initialize_graph_notify();
  bool initialize_services();

  void start_streaming();
  void stop_streaming();

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<VmbCAPI> api_;
  std::shared_ptr<VimbaXCamera> camera_;

  // Publishers
  image_transport::Publisher image_publisher_;

  // Services
  rclcpp::Service<vimbax_camera_msgs::srv::SettingsLoadSave>::SharedPtr settings_save_service_;
  rclcpp::Service<vimbax_camera_msgs::srv::SettingsLoadSave>::SharedPtr settings_load_service_;

  OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

  std::unique_ptr<std::thread> graph_notify_thread_;
  std::atomic_bool stop_threads_{false};
};

}  // namespace vimbax_camera

#endif  // VIMBAX_CAMERA__VIMBAX_CAMERA_NODE_HPP_
