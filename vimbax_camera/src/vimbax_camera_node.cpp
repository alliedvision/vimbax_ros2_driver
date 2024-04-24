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

#ifdef __unix__
#include <unistd.h>
#endif

#define CHK_SVC(a) {if (!a) { \
      return false; \
    }};

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <vimbax_camera/vimbax_camera_helper.hpp>

#include <vimbax_camera/vimbax_camera_node.hpp>
#include <vimbax_camera/vimbax_camera.hpp>

namespace vimbax_camera
{

using helper::get_logger;
using helper::vmb_error_to_string;

constexpr std::optional<VimbaXCamera::Module> map_module(
  const vimbax_camera_msgs::msg::FeatureModule & mod)
{
  switch (mod.id) {
    case vimbax_camera_msgs::msg::FeatureModule::MODULE_REMOTE_DEVICE:
      return VimbaXCamera::Module::RemoteDevice;
    case vimbax_camera_msgs::msg::FeatureModule::MODULE_SYSTEM:
      return VimbaXCamera::Module::System;
    case vimbax_camera_msgs::msg::FeatureModule::MODULE_INTERFACE:
      return VimbaXCamera::Module::Interface;
    case vimbax_camera_msgs::msg::FeatureModule::MODULE_LOCAL_DEVICE:
      return VimbaXCamera::Module::LocalDevice;
    case vimbax_camera_msgs::msg::FeatureModule::MODULE_STREAM:
      return VimbaXCamera::Module::Stream;
    default:
      return std::nullopt;
  }
}

VimbaXCameraNode::VimbaXCameraNode(const rclcpp::NodeOptions & options)
{
  if (!initialize(options)) {
    rclcpp::shutdown();
  }
}

bool VimbaXCameraNode::initialize(const rclcpp::NodeOptions & options)
{
  node_ = helper::create_node(get_node_name(), options);

  if (!node_) {
    return false;
  }

  if (!initialize_parameters()) {
    return false;
  }

  if (!initialize_api()) {
    return false;
  }

  if (!initialize_camera_observer()) {
    return false;
  }

  if (!initialize_camera()) {
    return false;
  }

  if (!initialize_callback_groups()) {
    return false;
  }

  if (!initialize_publisher()) {
    return false;
  }

  if (!initialize_feature_services()) {
    return false;
  }

  if (!initialize_settings_services()) {
    return false;
  }

  if (!initialize_status_services()) {
    return false;
  }

  if (!initialize_stream_services()) {
    return false;
  }

  if (!initialize_events()) {
    return false;
  }

  if (!initialize_graph_notify()) {
    return false;
  }


  RCLCPP_INFO(get_logger(), "Initialization done!");
  return true;
}

VimbaXCameraNode::~VimbaXCameraNode()
{
  stop_threads_.store(true, std::memory_order::memory_order_relaxed);

  if (api_) {
    deinitialize_camera_observer();
  }

  if (graph_notify_thread_) {
    graph_notify_thread_->join();
  }

  std::unique_lock lock(camera_mutex_);

  if (camera_ && camera_->is_streaming()) {
    camera_->stop_streaming();
  }

  last_camera_id_.clear();
  camera_.reset();
}

bool VimbaXCameraNode::initialize_events()
{
  feature_invalidation_event_publisher_ =
    std::make_shared<vimbax_camera_events::EventPublisher<std_msgs::msg::Empty>>(
    node_, "feature_invalidation",
    [this](const std::string & name) -> vimbax_camera_msgs::msg::Error
    {
      std::shared_lock lock(camera_mutex_);
      if (is_available_) {
        auto const res = camera_->feature_invalidation_register(
          name, [this](auto name) {
            feature_invalidation_event_publisher_->publish_event(
              name, std_msgs::msg::Empty{});
          });

        if (!res) {
          return res.error().to_error_msg();
        }
      } else {
        return vimbax_camera_msgs::msg::Error{}
        .set__code(VmbErrorNotFound).set__text("VmbErrorNotFound");
      }

      return vimbax_camera_msgs::msg::Error{};
    },
    [this](const std::string & name) -> void {
      camera_->feature_invalidation_unregister(name);
    });

  if (!feature_invalidation_event_publisher_) {
    return false;
  }

  event_event_publisher_ =
    std::make_shared<vimbax_camera_events::EventPublisher<vimbax_camera_msgs::msg::EventData>>(
    node_, "events", [this](const std::string & name) -> vimbax_camera_msgs::msg::Error
    {
      std::shared_lock lock(camera_mutex_);
      if (is_available_) {
        auto const event_feature_name = "Event" + name;

        auto const sel_res = camera_->feature_enum_set(SFNCFeatures::EventSelector.data(), name);

        if (!sel_res) {
          return sel_res.error().to_error_msg();
        }

        auto const on_res = camera_->feature_enum_set(SFNCFeatures::EventNotification.data(), "On");

        if (!on_res) {
          return on_res.error().to_error_msg();
        }

        auto const res = camera_->feature_invalidation_register(
          event_feature_name,
          [this, name](auto)
          {
            auto const res = camera_->get_event_meta_data(name);

            vimbax_camera_msgs::msg::EventData data{};

            if (res) {
              std::transform(
                res->cbegin(), res->cend(), std::back_inserter(data.entries),
                [](auto pair) {
                  return vimbax_camera_msgs::msg::EventDataEntry{}
                  .set__name(pair.first).set__value(pair.second);
                });
            }

            event_event_publisher_->publish_event(name, data);
          });

        if (!res) {
          return res.error().to_error_msg();
        }
      } else {
        return vimbax_camera_msgs::msg::Error{}
        .set__code(VmbErrorNotFound).set__text("VmbErrorNotFound");
      }

      return vimbax_camera_msgs::msg::Error{};
    },
    [this](const std::string & name) -> void {
      std::shared_lock lock(camera_mutex_);
      if (is_available_) {
        auto const event_feature_name = "Event" + name;

        camera_->feature_invalidation_unregister(event_feature_name);

        auto const sel_res = camera_->feature_enum_set(SFNCFeatures::EventSelector.data(), name);

        if (!sel_res) {
          return;
        }

        auto const off_res = camera_->feature_enum_set(
          SFNCFeatures::EventNotification.data(), "Off");

        if (!off_res) {
          return;
        }
      } else {
        return;
      }

      return;
    });

  if (!event_event_publisher_) {
    return false;
  }

  return true;
}

bool VimbaXCameraNode::initialize_parameters()
{
  auto const cameraIdParamDesc = rcl_interfaces::msg::ParameterDescriptor{}
  .set__description("Id of camera to open").set__read_only(true);
  node_->declare_parameter(parameter_camera_id, "", cameraIdParamDesc);

  auto const settingsFileParamDesc = rcl_interfaces::msg::ParameterDescriptor{}
  .set__description("Settings file to load at startup").set__read_only(true);
  node_->declare_parameter(parameter_settings_file, "", settingsFileParamDesc);

  auto const bufferCountRange = rcl_interfaces::msg::IntegerRange{}
  .set__from_value(3).set__step(1).set__to_value(1000);
  auto const bufferCountParamDesc = rcl_interfaces::msg::ParameterDescriptor{}
  .set__description("Number of buffers used for streaming").set__integer_range({bufferCountRange});
  node_->declare_parameter(parameter_buffer_count, 7, bufferCountParamDesc);

  auto const autostartStreamParamDesc = rcl_interfaces::msg::ParameterDescriptor{}
  .set__description(
    "Auto start/stop stream while subscribing/unsubscribing to image publisher").set__read_only(
    false);
  node_->declare_parameter(parameter_autostream, 1, autostartStreamParamDesc);

  auto const command_feature_timeout_param_desc = rcl_interfaces::msg::ParameterDescriptor{}
  .set__description("Timeout for command features");
  node_->declare_parameter(
    parameter_command_feature_timeout, 0, command_feature_timeout_param_desc);

  auto const use_ros_time_param_desc = rcl_interfaces::msg::ParameterDescriptor{}
  .set__description("Use ROS time instead of camera timestamp in image message header");
  node_->declare_parameter(parameter_use_ros_time, false, use_ros_time_param_desc);

  parameter_callback_handle_ = node_->add_on_set_parameters_callback(
    [this](
      const std::vector<rclcpp::Parameter> & params) -> rcl_interfaces::msg::SetParametersResult {
      for (auto const & param : params) {
        if (param.get_name() == parameter_buffer_count) {
          if (camera_->is_streaming()) {
            return rcl_interfaces::msg::SetParametersResult{}
            .set__successful(false)
            .set__reason("Buffer count change not supported while streaming");
          }
        }
      }

      return rcl_interfaces::msg::SetParametersResult{}.set__successful(true);
    });

  return true;
}

bool VimbaXCameraNode::initialize_api()
{
  RCLCPP_INFO(get_logger(), "Initializing VimbaX API ...");
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
  RCLCPP_INFO(get_logger(), "Initializing publisher ...");

  auto qos = rmw_qos_profile_default;
  qos.depth = 10;

  camera_publisher_ = image_transport::create_camera_publisher(node_.get(), "image_raw", qos);

  if (!camera_publisher_) {
    return false;
  }

  return true;
}

bool VimbaXCameraNode::initialize_camera(bool reconnect /*= false*/)
{
  RCLCPP_INFO(get_logger(), "Initializing camera ...");

  std::unique_lock lock(camera_mutex_);
  camera_ = VimbaXCamera::open(
    api_, last_camera_id_.empty() ?
    node_->get_parameter(parameter_camera_id).as_string() : last_camera_id_);

  if (!camera_) {
    if (reconnect) {
      RCLCPP_WARN(get_logger(), "Failed to reopen camera");
    } else {
      RCLCPP_FATAL(get_logger(), "Failed to open camera");
      rclcpp::shutdown();
    }
    return false;
  }

  auto const result = camera_->query_camera_info();
  if (result) {
    last_camera_id_ = (*result).cameraIdString;
  }

  auto const settingsFile = node_->get_parameter(parameter_settings_file).as_string();

  if (!settingsFile.empty()) {
    auto const loadResult = camera_->settings_load(settingsFile);
    if (!loadResult) {
      RCLCPP_ERROR(
        get_logger(), "Loading settings from file %s failed with error %d (%s)",
        settingsFile.c_str(), loadResult.error().code,
        (vmb_error_to_string(loadResult.error().code)).data());
    }
  }

  auto const info_res = camera_->camera_info_get();

  if (!info_res) {
    return false;
  }

  if (!reconnect) {
    auto const camera_frame_id_param_desc = rcl_interfaces::msg::ParameterDescriptor{}
    .set__description("Frame id of published images").set__read_only(true);
    node_->declare_parameter(
      parameter_frame_id, "vimbax_camera_" + info_res->device_id, camera_frame_id_param_desc);


    auto const camera_info_param_desc = rcl_interfaces::msg::ParameterDescriptor{}
    .set__description("Url of camera info file").set__read_only(true);

    node_->declare_parameter(parameter_camera_info_url, "", camera_info_param_desc);
  }

  camera_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(
    node_.get(), info_res->device_id,
    node_->get_parameter(parameter_camera_info_url).as_string());

  is_available_ = true;

  return true;
}

bool VimbaXCameraNode::initialize_camera_observer()
{
  RCLCPP_INFO(get_logger(), "Initializing camera observer...");

  auto err = api_->FeatureEnumSet(
    gVmbHandle, SFNCFeatures::EventSelector.data(), "CameraDiscovery");

  if (err != VmbErrorSuccess) {
    RCLCPP_ERROR(
      get_logger(), "%s failed with error %d (%s)", __FUNCTION__, err,
      (vmb_error_to_string(err)).data());
    return false;
  }

  err = api_->FeatureEnumSet(
    gVmbHandle, SFNCFeatures::EventNotification.data(), "On");

  if (err != VmbErrorSuccess) {
    RCLCPP_ERROR(
      get_logger(), "%s failed with error %d (%s)", __FUNCTION__, err,
      (vmb_error_to_string(err)).data());
    return false;
  }

  err = api_->FeatureInvalidationRegister(
    gVmbHandle, SFNCFeatures::EventCameraDiscovery.data(),
    this->camera_discovery_callback, reinterpret_cast<void *>(this));

  if (err != VmbErrorSuccess) {
    RCLCPP_ERROR(
      get_logger(), "%s failed with error %d (%s)", __FUNCTION__, err,
      (vmb_error_to_string(err)).data());
    return false;
  }

  return true;
}

bool VimbaXCameraNode::deinitialize_camera_observer()
{
  auto err = api_->FeatureEnumSet(
    gVmbHandle, SFNCFeatures::EventSelector.data(), "CameraDiscovery");

  if (err != VmbErrorSuccess) {
    RCLCPP_ERROR(
      get_logger(), "%s failed with error %d (%s)", __FUNCTION__, err,
      (vmb_error_to_string(err)).data());
    return false;
  }

  err = api_->FeatureEnumSet(gVmbHandle, SFNCFeatures::EventNotification.data(), "Off");

  if (err != VmbErrorSuccess) {
    RCLCPP_ERROR(
      get_logger(), "%s failed with error %d (%s)", __FUNCTION__, err,
      (vmb_error_to_string(err)).data());
    return false;
  }

  err = api_->FeatureInvalidationUnregister(
    gVmbHandle, SFNCFeatures::EventCameraDiscovery.data(),
    this->camera_discovery_callback);

  if (err != VmbErrorSuccess) {
    RCLCPP_ERROR(
      get_logger(), "%s failed with error %d (%s)", __FUNCTION__, err,
      (vmb_error_to_string(err)).data());
    return false;
  }

  return true;
}

void VimbaXCameraNode::camera_discovery_callback(
  const VmbHandle_t handle, const char * name, void * context)
{
  if (context) {
    (reinterpret_cast<VimbaXCameraNode *>(context))->on_camera_discovery_callback(handle, name);
  }
}

void VimbaXCameraNode::on_camera_discovery_callback(const VmbHandle_t handle, const char *)
{
  std::string camera_id;
  VmbUint32_t count = 0;

  auto err = api_->FeatureStringGet(
    handle, SFNCFeatures::EventCameraDiscoveryCameraID.data(), nullptr, count, &count);

  if (err == VmbErrorSuccess && count > 0) {
    std::string str_id;
    str_id.resize(count);
    err = api_->FeatureStringGet(
      handle, SFNCFeatures::EventCameraDiscoveryCameraID.data(), &str_id[0], count, &count);
    if (err == VmbErrorSuccess) {
      camera_id = &*str_id.begin();
    }
  }

  std::shared_lock camera_lock(camera_mutex_);
  auto const camera_connected = (camera_ != nullptr);
  auto const last_camera_id = last_camera_id_;
  camera_lock.unlock();

  // When last_camera_id is empty no camera was ever connected so return here.
  if (last_camera_id.empty()) {
    return;
  }

  if (err == VmbErrorSuccess) {
    const char * reason = nullptr;
    static bool stream_restart_required = false;

    err = api_->FeatureEnumGet(handle, SFNCFeatures::EventCameraDiscoveryType.data(), &reason);
    if (err == VmbErrorSuccess) {
      if (std::strcmp(reason, "Missing") == 0) {
        if ((camera_id.find(last_camera_id) != std::string::npos) && camera_connected) {
          RCLCPP_ERROR(
            get_logger(), "%s: Camera '%s' disconnected. Waiting for reconnection...",
            __FUNCTION__, last_camera_id.c_str());

          stream_restart_required = camera_->is_streaming();
          std::unique_lock lock{camera_mutex_};
          is_available_ = false;
          camera_.reset();
        }
      } else if (std::strcmp(reason, "Detected") == 0) {
        if (camera_id.find(last_camera_id) != std::string::npos && !camera_connected) {
          RCLCPP_INFO(
            get_logger(), "%s: Camera '%s' reconnected.", __FUNCTION__, last_camera_id.c_str());

          std::thread(
            [this] {
              if (initialize_camera(true)) {
                // Notify graph context that a stream restart is required
                stream_restart_required_ = stream_restart_required;
                stream_restart_required = false;
              }
            }).detach();
        }
      }
    }
  } else {
    RCLCPP_ERROR(
      get_logger(), "%s: Error while accessing EventCameraDiscoveryCameraID: %d (%s)", __FUNCTION__,
      err, (vmb_error_to_string(err)).data());
  }
}

bool VimbaXCameraNode::initialize_graph_notify()
{
  RCLCPP_INFO(get_logger(), "Initializing graph notify ...");
  graph_notify_thread_ = std::make_unique<std::thread>(
    [this] {
      size_t last_num_subscribers = 0;
      while (!stop_threads_.load(std::memory_order::memory_order_relaxed)) {
        auto event = node_->get_graph_event();
        node_->wait_for_graph_change(event, std::chrono::milliseconds(50));
        auto current_num_subscribers = camera_publisher_.getNumSubscribers();

        if (stream_restart_required_) {
          start_streaming();
          stream_restart_required_ = false;
        }

        event->check_and_clear();

        if (is_available_) {
          auto const subscriber_change =
          int64_t(current_num_subscribers) - int64_t(last_num_subscribers);

          if (subscriber_change > 0) {
            if (node_->get_parameter(parameter_autostream).as_int() == 1 &&
            !is_streaming() &&
            (!stream_stopped_by_service_ || current_num_subscribers > last_num_subscribers))
            {
              start_streaming();
              stream_stopped_by_service_ = false;
            }
          } else if (subscriber_change < 0 && current_num_subscribers == 0) {
            if (node_->get_parameter(parameter_autostream).as_int() == 1 &&
            is_streaming())
            {
              stop_streaming();
            }
            stream_stopped_by_service_ = false;
          }

          last_num_subscribers = current_num_subscribers;
        }
      }
    });

  if (!graph_notify_thread_) {
    return false;
  }

  return true;
}

bool VimbaXCameraNode::initialize_callback_groups()
{
  feature_callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  if (!feature_callback_group_) {
    return false;
  }

  settings_load_save_callback_group_ =
    node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  if (!settings_load_save_callback_group_) {
    return false;
  }

  status_callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  if (!status_callback_group_) {
    return false;
  }

  stream_start_stop_callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  if (!stream_start_stop_callback_group_) {
    return false;
  }

  return true;
}

bool VimbaXCameraNode::initialize_feature_services()
{
  RCLCPP_INFO(get_logger(), "Initializing feature services ...");

  if (!initialize_int_feature_services()) {
    return false;
  }

  if (!initialize_float_feature_services()) {
    return false;
  }

  if (!initialize_bool_feature_services()) {
    return false;
  }

  if (!initialize_command_feature_services()) {
    return false;
  }

  if (!initialize_enum_feature_services()) {
    return false;
  }

  if (!initialize_string_feature_services()) {
    return false;
  }

  if (!initialize_raw_feature_services()) {
    return false;
  }

  if (!initialize_generic_feature_services()) {
    return false;
  }

  return true;
}


bool VimbaXCameraNode::initialize_int_feature_services()
{
  feature_int_get_service_ =
    node_->create_service<vimbax_camera_msgs::srv::FeatureIntGet>(
    "features/int_get", [this](
      const vimbax_camera_msgs::srv::FeatureIntGet::Request::ConstSharedPtr request,
      const vimbax_camera_msgs::srv::FeatureIntGet::Response::SharedPtr response)
    {
      std::shared_lock lock(camera_mutex_);
      if (is_available_) {
        auto const feature_module = map_module(request->feature_module);
        if (feature_module) {
          auto const result = camera_->feature_int_get(request->feature_name, *feature_module);
          if (!result) {
            response->set__error(result.error().to_error_msg());
          } else {
            response->value = *result;
          }
        } else {
          response->set__error(error{VmbErrorBadParameter}.to_error_msg());
        }
      } else {
        response->set__error(error{VmbErrorNotFound}.to_error_msg());
      }
    }, rmw_qos_profile_services_default, feature_callback_group_);

  CHK_SVC(feature_int_get_service_);

  feature_int_set_service_ =
    node_->create_service<vimbax_camera_msgs::srv::FeatureIntSet>(
    "features/int_set", [this](
      const vimbax_camera_msgs::srv::FeatureIntSet::Request::ConstSharedPtr request,
      const vimbax_camera_msgs::srv::FeatureIntSet::Response::SharedPtr response)
    {
      std::shared_lock lock(camera_mutex_);
      if (is_available_) {
        auto const feature_module = map_module(request->feature_module);
        if (feature_module) {
          auto const result = camera_->feature_int_set(
            request->feature_name, request->value, *feature_module);
          if (!result) {
            response->set__error(result.error().to_error_msg());
          }
        } else {
          response->set__error(error{VmbErrorBadParameter}.to_error_msg());
        }
      } else {
        response->set__error(error{VmbErrorNotFound}.to_error_msg());
      }
    }, rmw_qos_profile_services_default, feature_callback_group_);

  CHK_SVC(feature_int_set_service_);

  feature_int_info_get_service_ =
    node_->create_service<vimbax_camera_msgs::srv::FeatureIntInfoGet>(
    "features/int_info_get", [this](
      const vimbax_camera_msgs::srv::FeatureIntInfoGet::Request::ConstSharedPtr request,
      const vimbax_camera_msgs::srv::FeatureIntInfoGet::Response::SharedPtr response)
    {
      std::shared_lock lock(camera_mutex_);
      if (is_available_) {
        auto const feature_module = map_module(request->feature_module);
        if (feature_module) {
          auto const result = camera_->feature_int_info_get(request->feature_name, *feature_module);
          if (!result) {
            response->set__error(result.error().to_error_msg());
          } else {
            response->min = (*result)[0];
            response->max = (*result)[1];
            response->inc = (*result)[2];
          }
        } else {
          response->set__error(error{VmbErrorBadParameter}.to_error_msg());
        }
      } else {
        response->set__error(error{VmbErrorNotFound}.to_error_msg());
      }
    }, rmw_qos_profile_services_default, feature_callback_group_);

  CHK_SVC(feature_int_info_get_service_);

  return true;
}

bool VimbaXCameraNode::initialize_float_feature_services()
{
  feature_float_get_service_ =
    node_->create_service<vimbax_camera_msgs::srv::FeatureFloatGet>(
    "features/float_get", [this](
      const vimbax_camera_msgs::srv::FeatureFloatGet::Request::ConstSharedPtr request,
      const vimbax_camera_msgs::srv::FeatureFloatGet::Response::SharedPtr response)
    {
      std::shared_lock lock(camera_mutex_);
      if (is_available_) {
        auto const feature_module = map_module(request->feature_module);
        if (feature_module) {
          auto const result = camera_->feature_float_get(request->feature_name, *feature_module);

          if (!result) {
            response->set__error(result.error().to_error_msg());
          } else {
            response->value = *result;
          }
        } else {
          response->set__error(error{VmbErrorBadParameter}.to_error_msg());
        }
      } else {
        response->set__error(error{VmbErrorNotFound}.to_error_msg());
      }
    }, rmw_qos_profile_services_default, feature_callback_group_);

  CHK_SVC(feature_float_get_service_);

  feature_float_set_service_ =
    node_->create_service<vimbax_camera_msgs::srv::FeatureFloatSet>(
    "features/float_set", [this](
      const vimbax_camera_msgs::srv::FeatureFloatSet::Request::ConstSharedPtr request,
      const vimbax_camera_msgs::srv::FeatureFloatSet::Response::SharedPtr response)
    {
      std::shared_lock lock(camera_mutex_);
      if (is_available_) {
        auto const feature_module = map_module(request->feature_module);
        if (feature_module) {
          auto const result = camera_->feature_float_set(
            request->feature_name, request->value, *feature_module);
          if (!result) {
            response->set__error(result.error().to_error_msg());
          }
        } else {
          response->set__error(error{VmbErrorBadParameter}.to_error_msg());
        }
      } else {
        response->set__error(error{VmbErrorNotFound}.to_error_msg());
      }
    }, rmw_qos_profile_services_default, feature_callback_group_);

  CHK_SVC(feature_float_set_service_);

  feature_float_info_get_service_ =
    node_->create_service<vimbax_camera_msgs::srv::FeatureFloatInfoGet>(
    "features/float_info_get", [this](
      const vimbax_camera_msgs::srv::FeatureFloatInfoGet::Request::ConstSharedPtr request,
      const vimbax_camera_msgs::srv::FeatureFloatInfoGet::Response::SharedPtr response)
    {
      std::shared_lock lock(camera_mutex_);
      if (is_available_) {
        auto const feature_module = map_module(request->feature_module);
        if (feature_module) {
          auto const result = camera_->feature_float_info_get(
            request->feature_name, *feature_module);
          if (!result) {
            response->set__error(result.error().to_error_msg());
          } else {
            response->min = (*result).min;
            response->max = (*result).max;
            response->inc = (*result).inc;
            response->inc_available = (*result).inc_available;
          }
        } else {
          response->set__error(error{VmbErrorBadParameter}.to_error_msg());
        }
      } else {
        response->set__error(error{VmbErrorNotFound}.to_error_msg());
      }
    }, rmw_qos_profile_services_default, feature_callback_group_);

  CHK_SVC(feature_float_info_get_service_);

  return true;
}

bool VimbaXCameraNode::initialize_string_feature_services()
{
  feature_string_get_service_ =
    node_->create_service<vimbax_camera_msgs::srv::FeatureStringGet>(
    "features/string_get", [this](
      const vimbax_camera_msgs::srv::FeatureStringGet::Request::ConstSharedPtr request,
      const vimbax_camera_msgs::srv::FeatureStringGet::Response::SharedPtr response)
    {
      std::shared_lock lock(camera_mutex_);
      if (is_available_) {
        auto const feature_module = map_module(request->feature_module);
        if (feature_module) {
          auto const result = camera_->feature_string_get(request->feature_name, *feature_module);
          if (!result) {
            response->set__error(result.error().to_error_msg());
          } else {
            response->value = *result;
          }
        } else {
          response->set__error(error{VmbErrorBadParameter}.to_error_msg());
        }
      } else {
        response->set__error(error{VmbErrorNotFound}.to_error_msg());
      }
    }, rmw_qos_profile_services_default, feature_callback_group_);

  CHK_SVC(feature_string_get_service_);

  feature_string_set_service_ =
    node_->create_service<vimbax_camera_msgs::srv::FeatureStringSet>(
    "features/string_set", [this](
      const vimbax_camera_msgs::srv::FeatureStringSet::Request::ConstSharedPtr request,
      const vimbax_camera_msgs::srv::FeatureStringSet::Response::SharedPtr response)
    {
      std::shared_lock lock(camera_mutex_);
      if (is_available_) {
        auto const feature_module = map_module(request->feature_module);
        if (feature_module) {
          auto const result = camera_->feature_string_set(
            request->feature_name, request->value, *feature_module);
          if (!result) {
            response->set__error(result.error().to_error_msg());
          }
        } else {
          response->set__error(error{VmbErrorBadParameter}.to_error_msg());
        }
      } else {
        response->set__error(error{VmbErrorNotFound}.to_error_msg());
      }
    }, rmw_qos_profile_services_default, feature_callback_group_);

  CHK_SVC(feature_string_set_service_);

  feature_string_info_get_service_ =
    node_->create_service<vimbax_camera_msgs::srv::FeatureStringInfoGet>(
    "features/string_info_get", [this](
      const vimbax_camera_msgs::srv::FeatureStringInfoGet::Request::ConstSharedPtr request,
      const vimbax_camera_msgs::srv::FeatureStringInfoGet::Response::SharedPtr response)
    {
      std::shared_lock lock(camera_mutex_);
      if (is_available_) {
        auto const feature_module = map_module(request->feature_module);
        if (feature_module) {
          auto const result = camera_->feature_string_info_get(
            request->feature_name, *feature_module);
          if (!result) {
            response->set__error(result.error().to_error_msg());
          } else {
            response->max_length = *result;
          }
        } else {
          response->set__error(error{VmbErrorBadParameter}.to_error_msg());
        }
      } else {
        response->set__error(error{VmbErrorNotFound}.to_error_msg());
      }
    }, rmw_qos_profile_services_default, feature_callback_group_);

  CHK_SVC(feature_string_info_get_service_);

  return true;
}

bool VimbaXCameraNode::initialize_bool_feature_services()
{
  feature_bool_get_service_ =
    node_->create_service<vimbax_camera_msgs::srv::FeatureBoolGet>(
    "features/bool_get", [this](
      const vimbax_camera_msgs::srv::FeatureBoolGet::Request::ConstSharedPtr request,
      const vimbax_camera_msgs::srv::FeatureBoolGet::Response::SharedPtr response)
    {
      std::shared_lock lock(camera_mutex_);
      if (is_available_) {
        auto const feature_module = map_module(request->feature_module);
        if (feature_module) {
          auto const result = camera_->feature_bool_get(request->feature_name, *feature_module);
          if (!result) {
            response->set__error(result.error().to_error_msg());
          } else {
            response->value = *result;
          }
        } else {
          response->set__error(error{VmbErrorBadParameter}.to_error_msg());
        }
      } else {
        response->set__error(error{VmbErrorNotFound}.to_error_msg());
      }
    }, rmw_qos_profile_services_default, feature_callback_group_);

  CHK_SVC(feature_bool_get_service_);

  feature_bool_set_service_ =
    node_->create_service<vimbax_camera_msgs::srv::FeatureBoolSet>(
    "features/bool_set", [this](
      const vimbax_camera_msgs::srv::FeatureBoolSet::Request::ConstSharedPtr request,
      const vimbax_camera_msgs::srv::FeatureBoolSet::Response::SharedPtr response)
    {
      std::shared_lock lock(camera_mutex_);
      if (is_available_) {
        auto const feature_module = map_module(request->feature_module);
        if (feature_module) {
          auto const result = camera_->feature_bool_set(
            request->feature_name, request->value, *feature_module);
          if (!result) {
            response->set__error(result.error().to_error_msg());
          }
        } else {
          response->set__error(error{VmbErrorBadParameter}.to_error_msg());
        }
      } else {
        response->set__error(error{VmbErrorNotFound}.to_error_msg());
      }
    }, rmw_qos_profile_services_default, feature_callback_group_);

  CHK_SVC(feature_bool_set_service_);

  return true;
}

bool VimbaXCameraNode::initialize_command_feature_services()
{
  feature_command_is_done_service_ =
    node_->create_service<vimbax_camera_msgs::srv::FeatureCommandIsDone>(
    "features/command_is_done", [this](
      const vimbax_camera_msgs::srv::FeatureCommandIsDone::Request::ConstSharedPtr request,
      const vimbax_camera_msgs::srv::FeatureCommandIsDone::Response::SharedPtr response)
    {
      std::shared_lock lock(camera_mutex_);
      if (is_available_) {
        auto const feature_module = map_module(request->feature_module);
        if (feature_module) {
          auto const result = camera_->feature_command_is_done(
            request->feature_name, *feature_module);
          if (!result) {
            response->set__error(result.error().to_error_msg());
          } else {
            response->is_done = *result;
          }
        } else {
          response->set__error(error{VmbErrorBadParameter}.to_error_msg());
        }
      } else {
        response->set__error(error{VmbErrorNotFound}.to_error_msg());
      }
    }, rmw_qos_profile_services_default, feature_callback_group_);

  CHK_SVC(feature_command_is_done_service_);

  feature_command_run_service_ =
    node_->create_service<vimbax_camera_msgs::srv::FeatureCommandRun>(
    "features/command_run", [this](
      const vimbax_camera_msgs::srv::FeatureCommandRun::Request::ConstSharedPtr request,
      const vimbax_camera_msgs::srv::FeatureCommandRun::Response::SharedPtr response)
    {
      std::shared_lock lock(camera_mutex_);

      auto const timeout = node_->get_parameter(parameter_command_feature_timeout).as_int();

      if (is_available_) {
        auto const feature_module = map_module(request->feature_module);
        if (feature_module) {
          auto const result = camera_->feature_command_run(
            request->feature_name,
            timeout > 0 ? std::optional{std::chrono::milliseconds{timeout}} : std::nullopt,
            *feature_module);
          if (!result) {
            response->set__error(result.error().to_error_msg());
          }
        } else {
          response->set__error(error{VmbErrorBadParameter}.to_error_msg());
        }
      } else {
        response->set__error(error{VmbErrorNotFound}.to_error_msg());
      }
    }, rmw_qos_profile_services_default, feature_callback_group_);

  CHK_SVC(feature_command_run_service_);

  return true;
}

bool VimbaXCameraNode::initialize_enum_feature_services()
{
  feature_enum_get_service_ =
    node_->create_service<vimbax_camera_msgs::srv::FeatureEnumGet>(
    "features/enum_get", [this](
      const vimbax_camera_msgs::srv::FeatureEnumGet::Request::ConstSharedPtr request,
      const vimbax_camera_msgs::srv::FeatureEnumGet::Response::SharedPtr response)
    {
      std::shared_lock lock(camera_mutex_);
      if (is_available_) {
        auto const feature_module = map_module(request->feature_module);
        if (feature_module) {
          auto const result = camera_->feature_enum_get(request->feature_name, *feature_module);
          if (!result) {
            response->set__error(result.error().to_error_msg());
          } else {
            response->value = *result;
          }
        } else {
          response->set__error(error{VmbErrorBadParameter}.to_error_msg());
        }
      } else {
        response->set__error(error{VmbErrorNotFound}.to_error_msg());
      }
    }, rmw_qos_profile_services_default, feature_callback_group_);

  CHK_SVC(feature_enum_get_service_);

  feature_enum_set_service_ =
    node_->create_service<vimbax_camera_msgs::srv::FeatureEnumSet>(
    "features/enum_set", [this](
      const vimbax_camera_msgs::srv::FeatureEnumSet::Request::ConstSharedPtr request,
      const vimbax_camera_msgs::srv::FeatureEnumSet::Response::SharedPtr response)
    {
      std::shared_lock lock(camera_mutex_);
      if (is_available_) {
        auto const feature_module = map_module(request->feature_module);
        if (feature_module) {
          auto const result = camera_->feature_enum_set(
            request->feature_name, request->value, *feature_module);
          if (!result) {
            response->set__error(result.error().to_error_msg());
          }
        } else {
          response->set__error(error{VmbErrorBadParameter}.to_error_msg());
        }
      } else {
        response->set__error(error{VmbErrorNotFound}.to_error_msg());
      }
    }, rmw_qos_profile_services_default, feature_callback_group_);

  CHK_SVC(feature_enum_set_service_);

  feature_enum_info_get_service_ =
    node_->create_service<vimbax_camera_msgs::srv::FeatureEnumInfoGet>(
    "features/enum_info_get", [this](
      const vimbax_camera_msgs::srv::FeatureEnumInfoGet::Request::ConstSharedPtr request,
      const vimbax_camera_msgs::srv::FeatureEnumInfoGet::Response::SharedPtr response)
    {
      std::shared_lock lock(camera_mutex_);
      if (is_available_) {
        auto const feature_module = map_module(request->feature_module);
        if (feature_module) {
          auto const result = camera_->feature_enum_info_get(
            request->feature_name, *feature_module);

          if (!result) {
            response->set__error(result.error().to_error_msg());
          } else {
            response->possible_values = (*result)[0];
            response->available_values = (*result)[1];
          }
        } else {
          response->set__error(error{VmbErrorBadParameter}.to_error_msg());
        }
      } else {
        response->set__error(error{VmbErrorNotFound}.to_error_msg());
      }
    }, rmw_qos_profile_services_default, feature_callback_group_);

  CHK_SVC(feature_enum_info_get_service_);

  feature_enum_as_int_get_service_ =
    node_->create_service<vimbax_camera_msgs::srv::FeatureEnumAsIntGet>(
    "features/enum_as_int_get", [this](
      const vimbax_camera_msgs::srv::FeatureEnumAsIntGet::Request::ConstSharedPtr request,
      const vimbax_camera_msgs::srv::FeatureEnumAsIntGet::Response::SharedPtr response)
    {
      std::shared_lock lock(camera_mutex_);
      if (is_available_) {
        auto const feature_module = map_module(request->feature_module);
        if (feature_module) {
          auto const result =
          camera_->feature_enum_as_int_get(request->feature_name, request->option, *feature_module);
          if (!result) {
            response->set__error(result.error().to_error_msg());
          } else {
            response->value = *result;
          }
        } else {
          response->set__error(error{VmbErrorBadParameter}.to_error_msg());
        }
      } else {
        response->set__error(error{VmbErrorNotFound}.to_error_msg());
      }
    }, rmw_qos_profile_services_default, feature_callback_group_);

  CHK_SVC(feature_enum_as_int_get_service_);

  feature_enum_as_string_get_service_ =
    node_->create_service<vimbax_camera_msgs::srv::FeatureEnumAsStringGet>(
    "features/enum_as_string_get", [this](
      const vimbax_camera_msgs::srv::FeatureEnumAsStringGet::Request::ConstSharedPtr request,
      const vimbax_camera_msgs::srv::FeatureEnumAsStringGet::Response::SharedPtr response)
    {
      std::shared_lock lock(camera_mutex_);
      if (is_available_) {
        auto const feature_module = map_module(request->feature_module);
        if (feature_module) {
          auto const result = camera_->feature_enum_as_string_get(
            request->feature_name, request->value, *feature_module);

          if (!result) {
            response->set__error(result.error().to_error_msg());
          } else {
            response->option = *result;
          }
        } else {
          response->set__error(error{VmbErrorBadParameter}.to_error_msg());
        }
      } else {
        response->set__error(error{VmbErrorNotFound}.to_error_msg());
      }
    }, rmw_qos_profile_services_default, feature_callback_group_);

  CHK_SVC(feature_enum_as_string_get_service_);

  return true;
}

bool VimbaXCameraNode::initialize_raw_feature_services()
{
  feature_raw_get_service_ =
    node_->create_service<vimbax_camera_msgs::srv::FeatureRawGet>(
    "features/raw_get", [this](
      const vimbax_camera_msgs::srv::FeatureRawGet::Request::ConstSharedPtr request,
      const vimbax_camera_msgs::srv::FeatureRawGet::Response::SharedPtr response)
    {
      std::shared_lock lock(camera_mutex_);
      if (is_available_) {
        auto const feature_module = map_module(request->feature_module);
        if (feature_module) {
          auto const result = camera_->feature_raw_get(request->feature_name, *feature_module);

          if (!result) {
            response->set__error(result.error().to_error_msg());
          } else {
            response->buffer = *result;
            response->buffer_size = (*result).size();
          }
        } else {
          response->set__error(error{VmbErrorBadParameter}.to_error_msg());
        }
      } else {
        response->set__error(error{VmbErrorNotFound}.to_error_msg());
      }
    }, rmw_qos_profile_services_default, feature_callback_group_);

  CHK_SVC(feature_raw_get_service_);

  feature_raw_set_service_ =
    node_->create_service<vimbax_camera_msgs::srv::FeatureRawSet>(
    "features/raw_set", [this](
      const vimbax_camera_msgs::srv::FeatureRawSet::Request::ConstSharedPtr request,
      const vimbax_camera_msgs::srv::FeatureRawSet::Response::SharedPtr response)
    {
      std::shared_lock lock(camera_mutex_);
      if (is_available_) {
        auto const feature_module = map_module(request->feature_module);
        if (feature_module) {
          auto const result = camera_->feature_raw_set(
            request->feature_name, request->buffer, *feature_module);

          if (!result) {
            response->set__error(result.error().to_error_msg());
          }
        } else {
          response->set__error(error{VmbErrorBadParameter}.to_error_msg());
        }
      } else {
        response->set__error(error{VmbErrorNotFound}.to_error_msg());
      }
    }, rmw_qos_profile_services_default, feature_callback_group_);

  CHK_SVC(feature_raw_set_service_);

  feature_raw_info_get_service_ =
    node_->create_service<vimbax_camera_msgs::srv::FeatureRawInfoGet>(
    "features/raw_info_get", [this](
      const vimbax_camera_msgs::srv::FeatureRawInfoGet::Request::ConstSharedPtr request,
      const vimbax_camera_msgs::srv::FeatureRawInfoGet::Response::SharedPtr response)
    {
      std::shared_lock lock(camera_mutex_);
      if (is_available_) {
        auto const feature_module = map_module(request->feature_module);
        if (feature_module) {
          auto const result = camera_->feature_raw_info_get(request->feature_name, *feature_module);
          if (!result) {
            response->set__error(result.error().to_error_msg());
          } else {
            response->max_length = *result;
          }
        } else {
          response->set__error(error{VmbErrorBadParameter}.to_error_msg());
        }
      } else {
        response->set__error(error{VmbErrorNotFound}.to_error_msg());
      }
    }, rmw_qos_profile_services_default, feature_callback_group_);

  CHK_SVC(feature_raw_info_get_service_);

  return true;
}

bool VimbaXCameraNode::initialize_generic_feature_services()
{
  feature_access_mode_get_service_ =
    node_->create_service<vimbax_camera_msgs::srv::FeatureAccessModeGet>(
    "features/access_mode_get", [this](
      const vimbax_camera_msgs::srv::FeatureAccessModeGet::Request::ConstSharedPtr request,
      const vimbax_camera_msgs::srv::FeatureAccessModeGet::Response::SharedPtr response)
    {
      std::shared_lock lock(camera_mutex_);
      if (is_available_) {
        auto const feature_module = map_module(request->feature_module);
        if (feature_module) {
          auto const result = camera_->feature_access_mode_get(
            request->feature_name, *feature_module);

          if (!result) {
            response->set__error(result.error().to_error_msg());
          } else {
            response->is_readable = (*result)[0];
            response->is_writeable = (*result)[1];
          }
        } else {
          response->set__error(error{VmbErrorBadParameter}.to_error_msg());
        }
      } else {
        response->set__error(error{VmbErrorNotFound}.to_error_msg());
      }
    }, rmw_qos_profile_services_default, feature_callback_group_);

  CHK_SVC(feature_access_mode_get_service_);

  feature_info_query_service_ =
    node_->create_service<vimbax_camera_msgs::srv::FeatureInfoQuery>(
    "feature_info_query", [this](
      const vimbax_camera_msgs::srv::FeatureInfoQuery::Request::ConstSharedPtr request,
      const vimbax_camera_msgs::srv::FeatureInfoQuery::Response::SharedPtr response)
    {
      std::shared_lock lock(camera_mutex_);
      if (is_available_) {
        auto const feature_module = map_module(request->feature_module);
        if (feature_module) {
          std::vector<std::string> feature_names;

          // If our list is empty we want to query infos for all features
          if (request->feature_names.size() == 0) {
            auto const result = camera_->features_list_get(*feature_module);
            if (!result) {
              response->set__error(result.error().to_error_msg());
              return;
            } else {
              feature_names = *result;
            }
          } else {
            feature_names = request->feature_names;
          }

          auto const result = camera_->feature_info_query_list(feature_names, *feature_module);
          if (!result) {
            response->set__error(result.error().to_error_msg());
            return;
          } else {
            std::transform(
              result->begin(), result->end(),
              std::back_inserter(response->feature_info),
              [](const feature_info & info) {
                auto const flags = vimbax_camera_msgs::msg::FeatureFlags{}
                .set__flag_none(info.flags.flag_none)
                .set__flag_read(info.flags.flag_read)
                .set__flag_write(info.flags.flag_write)
                .set__flag_volatile(info.flags.flag_volatile)
                .set__flag_modify_write(info.flags.flag_modify_write);

                return vimbax_camera_msgs::msg::FeatureInfo{}
                .set__name(info.name)
                .set__category(info.category)
                .set__display_name(info.display_name)
                .set__sfnc_namespace(info.sfnc_namespace)
                .set__unit(info.unit)
                .set__data_type(info.data_type)
                .set__flags(flags)
                .set__polling_time(info.polling_time);
              });
          }
        } else {
          response->set__error(error{VmbErrorBadParameter}.to_error_msg());
        }
      } else {
        response->set__error(error{VmbErrorNotFound}.to_error_msg());
      }
    }, rmw_qos_profile_services_default, feature_callback_group_);

  CHK_SVC(feature_info_query_service_);

  features_list_get_service_ =
    node_->create_service<vimbax_camera_msgs::srv::FeaturesListGet>(
    "features/list_get", [this](
      const vimbax_camera_msgs::srv::FeaturesListGet::Request::ConstSharedPtr request,
      const vimbax_camera_msgs::srv::FeaturesListGet::Response::SharedPtr response)
    {
      std::shared_lock lock(camera_mutex_);
      if (is_available_) {
        auto const feature_module = map_module(request->feature_module);
        if (feature_module) {
          auto const result = camera_->features_list_get(*feature_module);
          if (!result) {
            response->set__error(result.error().to_error_msg());
          } else {
            response->feature_list = *result;
          }
        } else {
          response->set__error(error{VmbErrorBadParameter}.to_error_msg());
        }
      } else {
        response->set__error(error{VmbErrorNotFound}.to_error_msg());
      }
    }, rmw_qos_profile_services_default, feature_callback_group_);

  CHK_SVC(features_list_get_service_);

  return true;
}

bool VimbaXCameraNode::initialize_settings_services()
{
  RCLCPP_INFO(get_logger(), "Initializing settings services ...");

  settings_save_service_ =
    node_->create_service<vimbax_camera_msgs::srv::SettingsLoadSave>(
    "settings/save", [this](
      const vimbax_camera_msgs::srv::SettingsLoadSave::Request::ConstSharedPtr request,
      const vimbax_camera_msgs::srv::SettingsLoadSave::Response::SharedPtr response)
    {
      std::shared_lock lock(camera_mutex_);
      if (is_available_) {
        auto const result = camera_->settings_save(request->filename);
        if (!result) {
          response->set__error(result.error().to_error_msg());
        }
      } else {
        response->set__error(error{VmbErrorNotFound}.to_error_msg());
      }
    }, rmw_qos_profile_services_default, settings_load_save_callback_group_);

  CHK_SVC(settings_save_service_);

  settings_load_service_ =
    node_->create_service<vimbax_camera_msgs::srv::SettingsLoadSave>(
    "settings/load", [this](
      const vimbax_camera_msgs::srv::SettingsLoadSave::Request::ConstSharedPtr request,
      const vimbax_camera_msgs::srv::SettingsLoadSave::Response::SharedPtr response)
    {
      std::shared_lock lock(camera_mutex_);
      if (is_available_) {
        auto const result = camera_->settings_load(request->filename);
        if (!result) {
          response->set__error(result.error().to_error_msg());
        }
      } else {
        response->set__error(error{VmbErrorNotFound}.to_error_msg());
      }
    }, rmw_qos_profile_services_default, settings_load_save_callback_group_);

  CHK_SVC(settings_load_service_);

  return true;
}

bool VimbaXCameraNode::initialize_status_services()
{
  RCLCPP_INFO(get_logger(), "Initializing status services ...");

  status_service_ = node_->create_service<vimbax_camera_msgs::srv::Status>(
    "status", [this](
      const vimbax_camera_msgs::srv::Status::Request::ConstSharedPtr,
      const vimbax_camera_msgs::srv::Status::Response::SharedPtr response)
    {
      std::shared_lock lock(camera_mutex_);
      if (is_available_) {
        auto const info = camera_->camera_info_get();
        if (!info) {
          response->set__error(info.error().to_error_msg());
        } else {
          std::vector<vimbax_camera_msgs::msg::TriggerInfo> trigger_info{};

          std::transform(
            info->trigger_info.begin(), info->trigger_info.end(), std::back_inserter(trigger_info),
            [](auto const trigger_info) {
              return vimbax_camera_msgs::msg::TriggerInfo{}
              .set__selector(trigger_info.selector)
              .set__mode(trigger_info.mode)
              .set__source(trigger_info.source);
            });

          response->set__display_name(info->display_name)
          .set__model_name(info->model_name)
          .set__device_firmware_version(info->firmware_version)
          .set__device_id(info->device_id)
          .set__device_user_id(info->device_user_id)
          .set__device_serial_number(info->device_serial_number)
          .set__interface_id(info->interface_id)
          .set__transport_layer_id(info->transport_layer_id)
          .set__streaming(info->streaming)
          .set__width(info->width)
          .set__height(info->height)
          .set__frame_rate(info->frame_rate)
          .set__pixel_format(info->pixel_format)
          .set__trigger_info(trigger_info);

          if (info->ip_address) {
            response->set__ip_address(*info->ip_address);
          }

          if (info->mac_address) {
            response->set__mac_address(*info->mac_address);
          }
        }
      } else {
        response->set__error(error{VmbErrorNotFound}.to_error_msg());
      }
    }, rmw_qos_profile_services_default, status_callback_group_);

  CHK_SVC(status_service_);

  connection_status_service_ = node_->create_service<vimbax_camera_msgs::srv::ConnectionStatus>(
    "connected", [this](
      const vimbax_camera_msgs::srv::ConnectionStatus::Request::ConstSharedPtr,
      const vimbax_camera_msgs::srv::ConnectionStatus::Response::SharedPtr response)
    {
      std::shared_lock lock(camera_mutex_);
      response->set__connected(camera_ != nullptr);
    }, rmw_qos_profile_services_default, status_callback_group_);

  CHK_SVC(connection_status_service_);

  return true;
}

bool VimbaXCameraNode::initialize_stream_services()
{
  RCLCPP_INFO(get_logger(), "Initializing stream services ...");

  stream_start_service_ =
    node_->create_service<vimbax_camera_msgs::srv::StreamStartStop>(
    "stream_start", [this](
      const vimbax_camera_msgs::srv::StreamStartStop::Request::ConstSharedPtr,
      const vimbax_camera_msgs::srv::StreamStartStop::Response::SharedPtr response)
    {
      std::shared_lock lock(camera_mutex_);
      if (is_available_) {
        auto const result = start_streaming();
        if (!result) {
          response->set__error(result.error().to_error_msg());
        }
        stream_stopped_by_service_ = false;
      } else {
        response->set__error(error{VmbErrorNotFound}.to_error_msg());
      }
    }, rmw_qos_profile_services_default, stream_start_stop_callback_group_);

  CHK_SVC(stream_start_service_);

  stream_stop_service_ =
    node_->create_service<vimbax_camera_msgs::srv::StreamStartStop>(
    "stream_stop", [this](
      const vimbax_camera_msgs::srv::StreamStartStop::Request::ConstSharedPtr,
      const vimbax_camera_msgs::srv::StreamStartStop::Response::SharedPtr response)
    {
      std::shared_lock lock(camera_mutex_);
      if (is_available_) {
        auto const result = stop_streaming();
        if (!result) {
          response->set__error(result.error().to_error_msg());
        }
        stream_stopped_by_service_ = true;
      } else {
        response->set__error(error{VmbErrorNotFound}.to_error_msg());
      }
    }, rmw_qos_profile_services_default, stream_start_stop_callback_group_);

  CHK_SVC(stream_stop_service_);

  return true;
}

result<void> VimbaXCameraNode::start_streaming()
{
  if (!is_available_) {
    return error{VmbErrorNotFound};
  }

  auto const buffer_count = node_->get_parameter(parameter_buffer_count).as_int();

  std::unique_lock stream_state_lock(stream_state_mutex_, std::defer_lock);
  std::shared_lock camera_lock(camera_mutex_, std::defer_lock);
  std::lock(stream_state_lock, camera_lock);

  auto result = camera_->start_streaming(
    buffer_count,
    [this](std::shared_ptr<VimbaXCamera::Frame> frame) {
      if (last_frame_id_) {
        auto const diff = frame->get_frame_id() - *last_frame_id_;

        if (diff > 1) {
          RCLCPP_WARN(get_logger(), "%ld frames missing", diff - 1);
        }
      }
      last_frame_id_ = frame->get_frame_id();

      frame->header.set__frame_id(node_->get_parameter(parameter_frame_id).as_string());


      if (node_->get_parameter(parameter_use_ros_time).as_bool()) {
        frame->header.stamp = node_->now();
      } else {
        std::chrono::nanoseconds vmbTimeStamp{frame->get_timestamp_ns()};
        auto const seconds = std::chrono::floor<std::chrono::seconds>(vmbTimeStamp);
        auto const nanoseconds =
        std::chrono::duration_cast<std::chrono::nanoseconds>(vmbTimeStamp - seconds);

        frame->header.stamp.sec = int32_t(seconds.count());
        frame->header.stamp.nanosec = nanoseconds.count();
      }

      auto const camera_info = [&] {
        auto const loaded_info = camera_info_manager_->getCameraInfo();

        if (loaded_info.width != frame->width || loaded_info.height != frame->height) {
          return sensor_msgs::msg::CameraInfo{}.set__width(frame->width).set__height(frame->height);
        }

        return loaded_info;
      }().set__header(frame->header);


      camera_publisher_.publish(*frame, camera_info);

      auto const queue_error = frame->queue();
      if (queue_error != VmbErrorSuccess) {
        RCLCPP_ERROR(
          get_logger(), "Frame requeue failed with %d (%s)", queue_error,
          (vmb_error_to_string(queue_error)).data());
      }
    });

  if (result) {
    RCLCPP_INFO(get_logger(), "Stream started using %ld buffers", buffer_count);
  }
  return result;
}

result<void> VimbaXCameraNode::stop_streaming()
{
  if (!is_available_) {
    return error{VmbErrorNotFound};
  }

  std::unique_lock stream_state_lock(stream_state_mutex_, std::defer_lock);
  std::shared_lock camera_lock(camera_mutex_, std::defer_lock);
  std::lock(stream_state_lock, camera_lock);

  auto error = camera_->stop_streaming();

  last_frame_id_ = std::nullopt;

  RCLCPP_INFO(get_logger(), "Stream stopped");
  return error;
}

bool VimbaXCameraNode::is_streaming()
{
  std::shared_lock lock(camera_mutex_);
  return (!camera_) ? false : camera_->is_streaming();
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
