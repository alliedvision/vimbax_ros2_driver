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
#include <vimbax_camera_msgs/srv/feature_int_info_get.hpp>
#include <vimbax_camera_msgs/srv/feature_float_get.hpp>
#include <vimbax_camera_msgs/srv/feature_float_set.hpp>
#include <vimbax_camera_msgs/srv/feature_float_info_get.hpp>
#include <vimbax_camera_msgs/srv/feature_string_get.hpp>
#include <vimbax_camera_msgs/srv/feature_string_set.hpp>
#include <vimbax_camera_msgs/srv/feature_string_info_get.hpp>
#include <vimbax_camera_msgs/srv/feature_bool_get.hpp>
#include <vimbax_camera_msgs/srv/feature_bool_set.hpp>
#include <vimbax_camera_msgs/srv/feature_command_is_done.hpp>
#include <vimbax_camera_msgs/srv/feature_command_run.hpp>
#include <vimbax_camera_msgs/srv/feature_enum_get.hpp>
#include <vimbax_camera_msgs/srv/feature_enum_set.hpp>
#include <vimbax_camera_msgs/srv/feature_enum_info_get.hpp>
#include <vimbax_camera_msgs/srv/feature_enum_as_int_get.hpp>
#include <vimbax_camera_msgs/srv/feature_enum_as_string_get.hpp>
#include <vimbax_camera_msgs/srv/feature_raw_get.hpp>
#include <vimbax_camera_msgs/srv/feature_raw_set.hpp>
#include <vimbax_camera_msgs/srv/feature_raw_info_get.hpp>
#include <vimbax_camera_msgs/srv/feature_access_mode_get.hpp>
#include <vimbax_camera_msgs/srv/settings_load_save.hpp>

#include <vimbax_camera/loader/vmbc_api.hpp>
#include <vimbax_camera/vimbax_camera.hpp>


namespace vimbax_camera
{
class VimbaXCameraNode
{
public:
  using NodeBaseInterface = rclcpp::node_interfaces::NodeBaseInterface;
  ~VimbaXCameraNode();

  static std::shared_ptr<VimbaXCameraNode> make_shared(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  NodeBaseInterface::SharedPtr get_node_base_interface() const;

private:
  VimbaXCameraNode() = default;

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
  rclcpp::Service<vimbax_camera_msgs::srv::FeatureIntGet>::SharedPtr feature_int_get_service_;
  rclcpp::Service<vimbax_camera_msgs::srv::FeatureIntSet>::SharedPtr feature_int_set_service_;
  rclcpp::Service<vimbax_camera_msgs::srv::FeatureIntInfoGet>::SharedPtr feature_int_info_get_service_;
  rclcpp::Service<vimbax_camera_msgs::srv::FeatureFloatGet>::SharedPtr feature_float_get_service_;
  rclcpp::Service<vimbax_camera_msgs::srv::FeatureFloatSet>::SharedPtr feature_float_set_service_;
  rclcpp::Service<vimbax_camera_msgs::srv::FeatureFloatInfoGet>::SharedPtr feature_float_info_get_service_;
  rclcpp::Service<vimbax_camera_msgs::srv::FeatureStringGet>::SharedPtr feature_string_get_service_;
  rclcpp::Service<vimbax_camera_msgs::srv::FeatureStringSet>::SharedPtr feature_string_set_service_;
  rclcpp::Service<vimbax_camera_msgs::srv::FeatureStringInfoGet>::SharedPtr feature_string_info_get_service_;
  rclcpp::Service<vimbax_camera_msgs::srv::FeatureBoolGet>::SharedPtr feature_bool_get_service_;
  rclcpp::Service<vimbax_camera_msgs::srv::FeatureBoolSet>::SharedPtr feature_bool_set_service_;
  rclcpp::Service<vimbax_camera_msgs::srv::FeatureCommandIsDone>::SharedPtr feature_command_is_done_service_;
  rclcpp::Service<vimbax_camera_msgs::srv::FeatureCommandRun>::SharedPtr feature_command_run_service_;
  rclcpp::Service<vimbax_camera_msgs::srv::FeatureEnumGet>::SharedPtr feature_enum_get_service_;
  rclcpp::Service<vimbax_camera_msgs::srv::FeatureEnumSet>::SharedPtr feature_enum_set_service_;
  rclcpp::Service<vimbax_camera_msgs::srv::FeatureEnumInfoGet>::SharedPtr feature_enum_info_get_service_;
  rclcpp::Service<vimbax_camera_msgs::srv::FeatureEnumAsIntGet>::SharedPtr feature_enum_as_int_get_service_;
  rclcpp::Service<vimbax_camera_msgs::srv::FeatureEnumAsStringGet>::SharedPtr feature_enum_as_string_get_service_;
  rclcpp::Service<vimbax_camera_msgs::srv::FeatureRawGet>::SharedPtr feature_raw_get_service_;
  rclcpp::Service<vimbax_camera_msgs::srv::FeatureRawSet>::SharedPtr feature_raw_set_service_;
  rclcpp::Service<vimbax_camera_msgs::srv::FeatureRawInfoGet>::SharedPtr feature_raw_info_get_service_;
  rclcpp::Service<vimbax_camera_msgs::srv::FeatureAccessModeGet>::SharedPtr feature_access_mode_get_service_;
  rclcpp::Service<vimbax_camera_msgs::srv::SettingsLoadSave>::SharedPtr settings_save_service_;
  rclcpp::Service<vimbax_camera_msgs::srv::SettingsLoadSave>::SharedPtr settings_load_service_;

  OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

  std::unique_ptr<std::thread> graph_notify_thread_;
  std::atomic_bool stop_threads_{false};
};

}  // namespace vimbax_camera

#endif  // VIMBAX_CAMERA__VIMBAX_CAMERA_NODE_HPP_
