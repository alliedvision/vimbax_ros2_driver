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

#ifdef __unix__
#include <unistd.h>
#endif

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <vimbax_camera/vimbax_camera_helper.hpp>

#include <vimbax_camera/vimbax_camera_node.hpp>

namespace vimbax_camera
{

using helper::get_logger;

VimbaXCameraNode::VimbaXCameraNode(const rclcpp::NodeOptions & options)
{
  node_ = helper::create_node(get_node_name(), options);

  if (!initialize_parameters()) {
    return;
  }

  if (!initialize_api()) {
    return;
  }

  if (!initialize_camera()) {
    return;
  }

  if (!initialize_publisher()) {
    return;
  }

  if (!initialize_graph_notify()) {
    return;
  }
}

VimbaXCameraNode::~VimbaXCameraNode()
{
  stop_threads_.store(true, std::memory_order::memory_order_relaxed);

  graph_notify_thread_->join();

  camera_.reset();
}

bool VimbaXCameraNode::initialize_parameters()
{
  auto cameraIdParamDesc = rcl_interfaces::msg::ParameterDescriptor{};
  cameraIdParamDesc.description = "Id of camera to open";
  cameraIdParamDesc.read_only = true;
  node_->declare_parameter("camera_id", "", cameraIdParamDesc);

  return true;
}

bool VimbaXCameraNode::initialize_api()
{
  RCLCPP_INFO(get_logger(), "Starting VimbaX camera node ...");
  RCLCPP_INFO(get_logger(), "Loading VimbaX api ...");
  api_ = VmbCAPI::get_instance();
  if (!api_) {
    RCLCPP_FATAL(get_logger(), "VmbC loading failed!");
    rclcpp::shutdown();
    return false;
  }

  VmbVersionInfo_t versionInfo{};
  if (api_->VersionQuery(&versionInfo, sizeof(versionInfo)) != VmbErrorSuccess) {
    RCLCPP_WARN(get_logger(), "Reading VmbC version failed!");
  }

  RCLCPP_INFO(
    get_logger(), "Successfully loaded VmbC API version %d.%d.%d",
    versionInfo.major, versionInfo.minor, versionInfo.patch);


  return true;
}

bool VimbaXCameraNode::initialize_publisher()
{
  image_publisher_ = image_transport::create_publisher(node_.get(), "~/image_raw");

  if (!image_publisher_) {
    return false;
  }

  return true;
}

bool VimbaXCameraNode::initialize_camera()
{
  camera_ = VimbaXCamera::open(api_, node_->get_parameter("camera_id").as_string());

  if (!camera_) {
    RCLCPP_FATAL(get_logger(), "Failed to open camera");
    rclcpp::shutdown();
    return false;
  }

  return true;
}

bool VimbaXCameraNode::initialize_graph_notify()
{
  graph_notify_thread_ = std::make_unique<std::thread>(
    [&] {
      while (!stop_threads_.load(std::memory_order::memory_order_relaxed)) {
        auto event = node_->get_graph_event();
        node_->wait_for_graph_change(event, std::chrono::milliseconds(5));

        if (event->check_and_clear()) {
          if (image_publisher_.getNumSubscribers() > 0 && !camera_->is_streaming()) {
            start_streaming();
          } else if (image_publisher_.getNumSubscribers() == 0 && camera_->is_streaming()) {
            stop_streaming();
          }
        }
      }
    });

  if (!graph_notify_thread_) {
    return false;
  }

  return true;
}

void VimbaXCameraNode::start_streaming()
{
  camera_->start_streaming(
    7, [this](std::shared_ptr<VimbaXCamera::Frame> frame) {
      static int64_t lastFrameId = -1;
      auto const diff = frame->get_frame_id() - lastFrameId;

      if (diff > 1) {
        RCLCPP_WARN(get_logger(), "%ld frames missing", diff - 1);
      }

      lastFrameId = frame->get_frame_id();

      image_publisher_.publish(*frame);

      frame->queue();
    });

  RCLCPP_INFO(get_logger(), "Stream started");
}

void VimbaXCameraNode::stop_streaming()
{
  camera_->stop_streaming();

  RCLCPP_INFO(get_logger(), "Stream stopped");
}

std::string VimbaXCameraNode::get_node_name()
{
  auto const pidString = [] {
#ifdef __unix__
      return std::to_string(getpid());
#endif
    }();

  return "vimbax_camera_" + pidString;
}

VimbaXCameraNode::NodeBaseInterface::SharedPtr VimbaXCameraNode::get_node_base_interface() const
{
  return node_->get_node_base_interface();
}

}  // namespace vimbax_camera

RCLCPP_COMPONENTS_REGISTER_NODE(vimbax_camera::VimbaXCameraNode)
