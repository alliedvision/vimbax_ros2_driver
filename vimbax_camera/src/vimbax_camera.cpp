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

#include <optional>
#include <filesystem>
#include <regex>

#include <rclcpp/rclcpp.hpp>

#include <vimbax_camera/vimbax_camera.hpp>

namespace fs = std::filesystem;

namespace vimbax_camera
{
using helper::get_logger;
using helper::vmb_error_to_string;

std::optional<uint32_t> decode_ip_addr(const std::string & ip_addr_str)
{
  const std::regex ip_addr_regex{"([0-9]{1,3}).([0-9]{1,3}).([0-9]{1,3}).([0-9]{1,3})"};
  std::smatch match;
  if (!std::regex_match(ip_addr_str, match, ip_addr_regex)) {
    return {};
  }

  uint32_t ip_addr = uint32_t(std::stoi(match[1]) & 0xFF) << 24 |
    uint32_t(std::stoi(match[2]) & 0xFF) << 16 |
    uint32_t(std::stoi(match[3]) & 0xFF) << 8 |
    uint32_t(std::stoi(match[4]) & 0xFF);

  return ip_addr;
}


std::optional<uint64_t> decode_mac_addr(const std::string & mac_addr_str)
{
  // Matching formats: aa:bb:cc:dd:ee:ff, aa-bb-cc-dd-ee-ff, aabbccddeeff
  const std::regex mac_addr_regex{
    "([0-9a-zA-Z]{2})[:-]?([0-9a-zA-Z]{2})[:-]?([0-9a-zA-Z]{2})[:-]?"
    "([0-9a-zA-Z]{2})[:-]?([0-9a-zA-Z]{2})[:-]?([0-9a-zA-Z]{2})"};
  std::smatch match;
  if (!std::regex_match(mac_addr_str, match, mac_addr_regex)) {
    return {};
  }

  uint64_t mac_addr = uint64_t(std::stoi(match[1], 0, 16) & 0xFF) << 40 |
    uint64_t(std::stoi(match[2], 0, 16) & 0xFF) << 32 |
    uint64_t(std::stoi(match[3], 0, 16) & 0xFF) << 24 |
    uint64_t(std::stoi(match[4], 0, 16) & 0xFF) << 16 |
    uint64_t(std::stoi(match[5], 0, 16) & 0xFF) << 8 |
    uint64_t(std::stoi(match[6], 0, 16) & 0xFF);

  return mac_addr;
}

std::optional<std::string> get_camera_id_from_addr(
  std::shared_ptr<VmbCAPI> api,
  const std::string & addr)
{
  auto const check_addr = [&]() -> std::optional<std::function<bool(VmbHandle_t)>> {
      auto const ip_addr = decode_ip_addr(addr);
      if (ip_addr) {
        RCLCPP_INFO(get_logger(), "Opening camera by ip address %s", addr.c_str());
        return [ip_addr, api](VmbHandle_t handle) -> bool {
                 auto const device_ip_addr = api->feature_int_get(handle, "GevDeviceIPAddress");
                 if (!device_ip_addr) {
                   return false;
                 }

                 RCLCPP_DEBUG(
                   get_logger(),
                   "Check ip address requested: %u device: %ld", *ip_addr, *device_ip_addr);

                 return *ip_addr == *device_ip_addr;
               };
      }

      auto const mac_addr = decode_mac_addr(addr);
      if (mac_addr) {
        RCLCPP_INFO(get_logger(), "Opening camera by mac address %s", addr.c_str());
        return [mac_addr, api](VmbHandle_t handle) -> bool {
                 auto const device_mac_addr = api->feature_int_get(handle, "GevDeviceMACAddress");
                 if (!device_mac_addr) {
                   return false;
                 }

                 RCLCPP_DEBUG(
                   get_logger(),
                   "Check mac address requested: %lu device: %ld", *mac_addr, *device_mac_addr);

                 return *mac_addr == uint64_t(*device_mac_addr);
               };
      }

      return {};
    }();

  if (!check_addr) {
    return {};
  }

  auto const interface_list = api->interface_list_get();
  if (!interface_list) {
    return {};
  }

  for (auto const & interface : *interface_list) {
    RCLCPP_DEBUG(
      get_logger(), "Found interface %s type %d", interface.interfaceName, interface.interfaceType);

    if (interface.interfaceType == VmbTransportLayerTypeGEV) {
      auto const selector_info =
        api->feature_int_info_get(interface.interfaceHandle, "DeviceSelector");
      if (!selector_info) {
        continue;
      }

      for (VmbInt64_t i = (*selector_info)[0]; i <= (*selector_info)[1]; i++) {
        if (api->feature_int_set(interface.interfaceHandle, "DeviceSelector", i)) {
          if ((*check_addr)(interface.interfaceHandle)) {
            auto const device_id = api->feature_string_get(interface.interfaceHandle, "DeviceID");

            if (!device_id) {
              continue;
            }

            return *device_id;
          }
        }
      }
    }
  }

  return {};
}


std::shared_ptr<VimbaXCamera> VimbaXCamera::open(
  std::shared_ptr<VmbCAPI> api,
  const std::string & name)
{
  auto check_access = [](const VmbCameraInfo_t & info) {
      return (info.permittedAccess & VmbAccessModeType::VmbAccessModeExclusive) != 0;
    };

  auto open_camera =
    [&](const std::string & idStr) -> std::optional<VmbHandle_t> {
      VmbHandle_t camera_handle;
      auto const open_error =
        api->CameraOpen(idStr.c_str(), VmbAccessModeType::VmbAccessModeExclusive, &camera_handle);

      if (open_error != VmbErrorSuccess) {
        RCLCPP_ERROR(
          get_logger(), "Failed to open camera %s. Error %d (%s)", idStr.c_str(), open_error,
          (vmb_error_to_string(open_error)).data());
        return std::nullopt;
      }

      return camera_handle;
    };

  auto const available_cameras =
    [&]() -> std::vector<VmbCameraInfo_t> {
      uint32_t available_cameras_count{0};
      auto const count_error = api->CamerasList(nullptr, 0, &available_cameras_count, 0);

      if (count_error != VmbErrorSuccess) {
        RCLCPP_ERROR(
          get_logger(), "Reading camera list size failed with error %d (%s)", count_error,
          (vmb_error_to_string(count_error)).data());
        return {};
      }

      std::vector<VmbCameraInfo_t> camera_list{};
      camera_list.resize(available_cameras_count);
      uint32_t cameras_found = 0;

      auto const error = api->CamerasList(
        camera_list.data(), available_cameras_count, &cameras_found, sizeof(VmbCameraInfo_t));

      if (error != VmbErrorSuccess) {
        RCLCPP_ERROR(
          get_logger(), "List first camera failed with error %d (%s)", error,
          (vmb_error_to_string(error)).data());
        return {};
      }

      return camera_list;
    }();

  if (name.empty()) {
    RCLCPP_INFO(get_logger(), "No camera requested opening first available");

    if (available_cameras.empty()) {
      RCLCPP_ERROR(get_logger(), "List of available cameras is empty");
      return nullptr;
    }

    for (auto const & info : available_cameras) {
      if (check_access(info)) {
        RCLCPP_INFO(get_logger(), "Try opening camera with extended id %s", info.cameraIdExtended);

        auto const opt_handle = open_camera(info.cameraIdExtended);

        if (opt_handle) {
          return std::unique_ptr<VimbaXCamera>(new VimbaXCamera{api, *opt_handle});
        }
      }
    }

    RCLCPP_ERROR(get_logger(), "No camera available!");

    return nullptr;
  } else {
    auto const check_camera_info =
      [&name](const VmbCameraInfo_t & info) -> bool {
        return info.cameraIdString == name ||
               info.cameraIdExtended == name ||
               info.serialString == name;
      };
    // Try open by serial number
    if (!available_cameras.empty()) {
      for (auto const & info : available_cameras) {
        if (check_camera_info(info) && (info.permittedAccess & VmbAccessModeExclusive) != 0) {
          auto const opt_handle = open_camera(info.cameraIdExtended);

          if (opt_handle) {
            return std::unique_ptr<VimbaXCamera>(new VimbaXCamera{api, *opt_handle});
          }
        }
      }
    }

    RCLCPP_DEBUG(get_logger(), "Trying to open by ip/mac address");

    auto const opt_id_by_addr = get_camera_id_from_addr(api, name);
    if (opt_id_by_addr) {
      auto const opt_handle = open_camera(*opt_id_by_addr);

      if (opt_handle) {
        return std::unique_ptr<VimbaXCamera>(new VimbaXCamera{api, *opt_handle});
      }
    }

    RCLCPP_DEBUG(get_logger(), "No matching camera found, falling back to VmbCameraOpen");

    auto const opt_handle = open_camera(name);

    if (opt_handle) {
      return std::unique_ptr<VimbaXCamera>(new VimbaXCamera{api, *opt_handle});
    }

    RCLCPP_ERROR(get_logger(), "Failed to open given camera %s", name.c_str());
  }

  return nullptr;
}

VimbaXCamera::VimbaXCamera(std::shared_ptr<VmbCAPI> api, VmbHandle_t camera_handle)
: api_{std::move(api)}, camera_handle_{camera_handle}
{
  auto const err =
    api_->CameraInfoQueryByHandle(camera_handle_, &camera_info_, sizeof(camera_info_));

  if (err != VmbErrorSuccess) {
    RCLCPP_ERROR(get_logger(), "Failed to query camera info!");
  }

  RCLCPP_INFO(
    get_logger(), "Opened camera info model name: %s, camera name: %s, serial: %s",
    camera_info_.modelName, camera_info_.cameraName, camera_info_.serialString);

  RCLCPP_INFO(get_logger(), "Camera extended id %s", camera_info_.cameraIdExtended);

  initialize_feature_map(Module::System);
  initialize_feature_map(Module::Interface);
  initialize_feature_map(Module::LocalDevice);
  initialize_feature_map(Module::RemoteDevice);
  initialize_feature_map(Module::Stream);


  if (has_feature(SFNCFeatures::DeviceTimestampFrequency, Module::LocalDevice)) {
    auto const handle = get_module_handle(Module::LocalDevice);
    auto const timestamp_frequency =
      feature_int_get(SFNCFeatures::DeviceTimestampFrequency, handle);

    if (timestamp_frequency) {
      timestamp_frequency_ = *timestamp_frequency;
    }
  }

  if (has_feature(SFNCFeatures::GVSPAdjustPacketSize, Module::Stream)) {
    auto const result =
      feature_command_run(SFNCFeatures::GVSPAdjustPacketSize, get_module_handle(Module::Stream));
    if (!result) {
      RCLCPP_INFO(
        get_logger(),
        "Packet size adjustment failed with %s",
        vmb_error_to_string(result.error().code).data());
    }
  }
}

VimbaXCamera::~VimbaXCamera()
{
  if (is_alive()) {
    stop_streaming();
  } else if (frame_processing_thread_) {
    frame_processing_enable_.store(false);
    frame_ready_cv_.notify_all();
    frame_processing_thread_->join();
  }

  if (api_ && camera_handle_) {
    api_->CameraClose(camera_handle_);
    camera_handle_ = nullptr;
  }
}

void VimbaXCamera::initialize_feature_map(Module module)
{
  auto const handle = get_module_handle(module);
  VmbUint32_t feature_list_size{};

  api_->FeaturesList(handle, nullptr, 0, &feature_list_size, 0);

  std::vector<VmbFeatureInfo_t> feature_list{};
  feature_list.resize(feature_list_size);

  api_->FeaturesList(
    handle, feature_list.data(), feature_list.size(),
    &feature_list_size, sizeof(VmbFeatureInfo_t));

  for (auto const & info : feature_list) {
    feature_info_map_[std::size_t(module)].emplace(info.name, info);
    feature_category_map_[std::size_t(module)].emplace(info.category, info.name);
  }
}

constexpr VmbHandle_t VimbaXCamera::get_module_handle(Module module) const
{
  switch (module) {
    case Module::System:
      return camera_info_.transportLayerHandle;
    case Module::Interface:
      return camera_info_.interfaceHandle;
    case Module::LocalDevice:
      return camera_info_.localDeviceHandle;
    case Module::RemoteDevice:
      return camera_handle_;
    case Module::Stream:
      return camera_info_.streamHandles[0];
    default:
      break;
  }

  return nullptr;
}

VmbFeatureInfo VimbaXCamera::get_feature_info(const std::string & name, Module module) const
{
  return feature_info_map_[std::size_t(module)].at(name);
}

bool VimbaXCamera::is_alive() const
{
  VmbCameraInfo camera_info{};

  auto const err = api_->CameraInfoQueryByHandle(camera_handle_, &camera_info, sizeof(camera_info));

  return (err == VmbErrorNotFound) ? false : true;
}

bool VimbaXCamera::has_feature(const std::string_view & name, Module module) const
{
  if (feature_info_map_[std::size_t(module)].count(name.data())) {
    return true;
  }

  return false;
}

result<void> VimbaXCamera::start_streaming(
  int buffer_count,
  std::function<void(std::shared_ptr<Frame>)> on_frame,
  bool start_acquisition)
{
  auto expected_state = StreamState::kStopped;

  if (stream_state_.compare_exchange_strong(expected_state, StreamState::kStarting)) {
    frames_.clear();
    frames_.resize(buffer_count);

    uint32_t payload_size{};

    auto const payload_size_error = api_->PayloadSizeGet(camera_handle_, &payload_size);
    if (payload_size_error != VmbErrorSuccess) {
      return error{payload_size_error};
    }

    auto const pixel_format = get_pixel_format();

    if (!is_valid_pixel_format(*pixel_format)) {
      RCLCPP_ERROR(get_logger(), "Unsupported pixel format");
      return error{VmbErrorNotSupported};
    }

    auto alignment{1};

    if (has_feature(SFNCFeatures::StreamBufferAlignment)) {
      auto const alignment_res =
        feature_int_get(SFNCFeatures::StreamBufferAlignment, camera_info_.streamHandles[0]);

      alignment = alignment_res ? *alignment_res : 1;
    }

    RCLCPP_INFO(get_logger(), "Buffer alignment: %d", alignment);

    for (auto & frame : frames_) {
      auto new_frame = Frame::create(shared_from_this(), payload_size, alignment);

      if (!new_frame) {
        RCLCPP_ERROR(get_logger(), "Failed to create frame");
        return new_frame.error();
      }

      frame = *new_frame;

      frame->set_callback(on_frame);
    }

    frame_processing_enable_ = true;
    frame_processing_thread_ = std::make_shared<std::thread>(
      [this] {
        while (frame_processing_enable_) {
          auto const frame_opt = [&]() -> std::optional<std::shared_ptr<Frame>> {
            std::unique_lock lock{frame_ready_queue_mutex_};
            frame_ready_cv_.wait(
              lock, [&]  {
                return !frame_ready_queue_.empty() || !frame_processing_enable_;
              });

            if (!frame_ready_queue_.empty()) {
              auto const ret = frame_ready_queue_.front();
              frame_ready_queue_.pop();
              return ret;
            } else {
              return std::nullopt;
            }
          }();

          if (frame_opt) {
            (*frame_opt)->on_frame_ready();
          }
        }
      });

    auto const capture_start_error = api_->CaptureStart(camera_handle_);
    if (capture_start_error != VmbErrorSuccess) {
      RCLCPP_ERROR(
        get_logger(), "Capture start failed with error %d (%s)", capture_start_error,
        (vmb_error_to_string(capture_start_error)).data());
      return error{capture_start_error};
    }

    for (auto const & frame : frames_) {
      auto const queue_error = frame->queue();
      if (queue_error != VmbErrorSuccess) {
        RCLCPP_ERROR(
          get_logger(), "Queue frame failed with error %d (%s)", queue_error,
          (vmb_error_to_string(queue_error)).data());
        return error{queue_error};
      }
    }

    if (start_acquisition) {
      auto const acquisition_start_error = feature_command_run(SFNCFeatures::AcquisitionStart);
      if (!acquisition_start_error) {
        RCLCPP_ERROR(
          get_logger(), "Acquisition start failed with error %d (%s)",
          acquisition_start_error.error().code,
          (vmb_error_to_string(acquisition_start_error.error().code)).data());
        return acquisition_start_error.error();
      }
    }

    stream_state_.store(StreamState::kActive);

    return {};
  } else if (expected_state == StreamState::kActive) {
    return {};
  } else {
    return error{VmbErrorInvalidCall};
  }
}

result<void> VimbaXCamera::stop_streaming()
{
  auto expected_state = StreamState::kActive;

  if (stream_state_.compare_exchange_strong(expected_state, StreamState::kStopping)) {
    if (is_alive()) {
      auto const acquisition_stop_error = feature_command_run(SFNCFeatures::AcquisitionStop);
      if (!acquisition_stop_error) {
        RCLCPP_ERROR(
          get_logger(), "Acquisition stop failed with error %d (%s)",
          acquisition_stop_error.error().code,
          (vmb_error_to_string(acquisition_stop_error.error().code)).data());
        return acquisition_stop_error.error();
      }
    }

    auto const capture_stop_error = api_->CaptureEnd(camera_handle_);
    if (capture_stop_error != VmbErrorSuccess) {
      RCLCPP_ERROR(
        get_logger(), "Capture stop failed with error %d (%s)", capture_stop_error,
        (vmb_error_to_string(capture_stop_error)).data());
      return error{capture_stop_error};
    }

    // Stop frame processing before revoking the frames to avoid requeue errors
    if (frame_processing_thread_) {
      frame_processing_enable_ = false;
      {
        std::lock_guard guard{frame_ready_queue_mutex_};
        while (!frame_ready_queue_.empty()) {
          frame_ready_queue_.pop();
        }
      }
      frame_ready_cv_.notify_all();
      frame_processing_thread_->join();
      frame_processing_thread_.reset();
    }

    auto const flush_error = api_->CaptureQueueFlush(camera_handle_);
    if (flush_error != VmbErrorSuccess) {
      RCLCPP_ERROR(
        get_logger(), "Flush capture queue failed with error %d (%s)", flush_error,
        (vmb_error_to_string(flush_error)).data());
      return error{flush_error};
    }

    auto const revoke_error = api_->FrameRevokeAll(camera_handle_);
    if (revoke_error != VmbErrorSuccess) {
      RCLCPP_ERROR(
        get_logger(), "Revoking frames failed with error %d (%s)", revoke_error,
        (vmb_error_to_string(revoke_error)).data());
      return error{revoke_error};
    }

    frames_.clear();

    stream_state_.store(StreamState::kStopped);

    return {};
  } else if (expected_state == StreamState::kStopped) {
    return {};
  } else {
    return error{VmbErrorInvalidCall};
  }
}

result<VmbCameraInfo> VimbaXCamera::query_camera_info() const
{
  RCLCPP_DEBUG(get_logger(), "%s", __FUNCTION__);

  VmbCameraInfo camera_info{};

  auto const err = api_->CameraInfoQueryByHandle(camera_handle_, &camera_info, sizeof(camera_info));

  if (err != VmbErrorSuccess) {
    RCLCPP_ERROR(get_logger(), "Failed to query camera info!");
    return error{err};
  }

  return camera_info;
}

result<std::vector<std::string>> VimbaXCamera::features_list_get(Module module) const
{
  RCLCPP_DEBUG(get_logger(), "%s", __FUNCTION__);

  std::vector<std::string> feature_list{};

  auto const & feature_map = feature_info_map_[std::size_t(module)];

  std::transform(
    feature_map.begin(), feature_map.end(), std::back_insert_iterator(feature_list),
    [](auto feature_info) {
      return feature_info.first;
    });

  return feature_list;
}

result<bool> VimbaXCamera::feature_command_is_done(
  const std::string_view & name,
  Module module) const
{
  RCLCPP_DEBUG(get_logger(), "%s(%s)", __FUNCTION__, name.data());

  auto const handle = get_module_handle(module);

  bool value{};
  auto const err =
    api_->FeatureCommandIsDone(handle, name.data(), reinterpret_cast<bool *>(&value));

  if (err != VmbErrorSuccess) {
    RCLCPP_ERROR(
      get_logger(), "%s failed with error %d (%s)", __FUNCTION__, err,
      vmb_error_to_string(err).data());
    return error{err};
  }

  return value;
}

result<void> VimbaXCamera::feature_command_run(
  const std::string_view & name,
  const std::optional<std::chrono::milliseconds> & timeout,
  const Module module) const
{
  return feature_command_run(name, get_module_handle(module), timeout);
}

result<void> VimbaXCamera::feature_command_run(
  const std::string_view & name, VmbHandle_t handle,
  const std::optional<std::chrono::milliseconds> & timeout) const
{
  using namespace std::chrono_literals;
  auto const default_timeout = 1s;

  auto const run_error = api_->FeatureCommandRun(handle, name.data());

  if (run_error != VmbErrorSuccess) {
    return error{run_error};
  }

  auto const poll_start_tp = std::chrono::steady_clock::now();

  auto is_timed_out = [&]() -> bool {
      auto const now_tp = std::chrono::steady_clock::now();
      auto const diff = now_tp - poll_start_tp;

      return diff >= timeout.value_or(default_timeout);
    };

  bool done{false};

  do {
    auto const done_error = api_->FeatureCommandIsDone(handle, name.data(), &done);
    if (done_error != VmbErrorSuccess) {
      return error{done_error};
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    if (is_timed_out()) {
      RCLCPP_ERROR(get_logger(), "Waiting for command %s done timed out!", name.data());
      return error{VmbErrorTimeout};
    }
  } while(!done);

  return {};
}

result<int64_t> VimbaXCamera::feature_int_get(
  const std::string_view & name,
  const Module module) const
{
  return feature_int_get(name, get_module_handle(module));
}

result<int64_t> VimbaXCamera::feature_int_get(
  const std::string_view & name,
  VmbHandle_t handle) const
{
  return api_->feature_int_get(handle, name);
}

result<void> VimbaXCamera::feature_int_set(
  const std::string_view & name,
  const int64_t value,
  const Module module) const
{
  auto const handle = get_module_handle(module);

  return api_->feature_int_set(handle, name, value);
}

result<std::array<int64_t, 3>>
VimbaXCamera::feature_int_info_get(
  const std::string_view & name,
  const Module module) const
{
  RCLCPP_DEBUG(get_logger(), "%s('%s')", __FUNCTION__, name.data());

  auto const handle = get_module_handle(module);
  auto const result = api_->feature_int_info_get(handle, name);

  if (!result) {
    RCLCPP_ERROR(
      get_logger(), "%s failed with error %d (%s)", __FUNCTION__, result.error().code,
      vmb_error_to_string(result.error().code).data());
  }

  return result;
}

result<_Float64> VimbaXCamera::feature_float_get(
  const std::string_view & name,
  const Module module) const
{
  return feature_float_get(name, get_module_handle(module));
}

result<_Float64> VimbaXCamera::feature_float_get(
  const std::string_view & name, VmbHandle_t handle) const
{
  RCLCPP_DEBUG(get_logger(), "%s('%s')", __FUNCTION__, name.data());

  _Float64 value{};
  auto const err =
    api_->FeatureFloatGet(handle, name.data(), reinterpret_cast<_Float64 *>(&value));

  if (err != VmbErrorSuccess) {
    RCLCPP_ERROR(
      get_logger(), "%s failed with error %d (%s)", __FUNCTION__, err,
      vmb_error_to_string(err).data());
    return error{err};
  }

  return value;
}

result<void>
VimbaXCamera::feature_float_set(
  const std::string_view & name,
  const _Float64 value,
  const Module module) const
{
  RCLCPP_DEBUG(get_logger(), "%s('%s', %lf)", __FUNCTION__, name.data(), value);

  auto const handle = get_module_handle(module);

  auto const err =
    api_->FeatureFloatSet(handle, name.data(), value);

  if (err != VmbErrorSuccess) {
    RCLCPP_ERROR(
      get_logger(), "%s failed with error %d (%s)", __FUNCTION__, err,
      vmb_error_to_string(err).data());
    return error{err};
  }

  return {};
}

result<feature_float_info> VimbaXCamera::feature_float_info_get(
  const std::string_view & name,
  const Module module) const
{
  RCLCPP_DEBUG(get_logger(), "%s('%s')", __FUNCTION__, name.data());

  auto const handle = get_module_handle(module);

  feature_float_info info{};

  auto err =
    api_->FeatureFloatRangeQuery(
    handle,
    name.data(),
    reinterpret_cast<double *>(&info.min),
    reinterpret_cast<double *>(&info.max));

  if (err != VmbErrorSuccess) {
    RCLCPP_ERROR(
      get_logger(), "%s failed with error %d (%s)", __FUNCTION__, err,
      vmb_error_to_string(err).data());
    return error{err};
  }

  err =
    api_->FeatureFloatIncrementQuery(
    handle,
    name.data(),
    &info.inc_available,
    reinterpret_cast<double *>(&info.inc));

  if (err != VmbErrorSuccess) {
    RCLCPP_ERROR(
      get_logger(), "%s failed with error %d (%s)", __FUNCTION__, err,
      vmb_error_to_string(err).data());
    return error{err};
  }

  return info;
}

result<std::string> VimbaXCamera::feature_string_get(
  const std::string_view & name,
  const Module module) const
{
  return feature_string_get(name, get_module_handle(module));
}


result<std::string> VimbaXCamera::feature_string_get(
  const std::string_view & name,
  VmbHandle_t handle) const
{
  return api_->feature_string_get(handle, name);
}


result<void>
VimbaXCamera::feature_string_set(
  const std::string_view & name,
  const std::string_view value,
  const Module module) const
{
  RCLCPP_DEBUG(get_logger(), "%s('%s', '%s')", __FUNCTION__, name.data(), value.data());

  auto const handle = get_module_handle(module);

  auto const err =
    api_->FeatureStringSet(handle, name.data(), value.data());

  if (err != VmbErrorSuccess) {
    RCLCPP_ERROR(
      get_logger(), "%s failed with error %d (%s)", __FUNCTION__, err,
      vmb_error_to_string(err).data());
    return error{err};
  }

  return {};
}

result<uint32_t> VimbaXCamera::feature_string_info_get(
  const std::string_view & name,
  const Module module) const
{
  RCLCPP_DEBUG(get_logger(), "%s('%s')", __FUNCTION__, name.data());

  auto const handle = get_module_handle(module);

  uint32_t value{};

  auto err =
    api_->FeatureStringMaxlengthQuery(
    handle,
    name.data(),
    reinterpret_cast<VmbUint32_t *>(&value));

  if (err != VmbErrorSuccess) {
    RCLCPP_ERROR(
      get_logger(), "%s failed with error %d (%s)", __FUNCTION__, err,
      vmb_error_to_string(err).data());
    return error{err};
  }

  return value;
}

result<bool> VimbaXCamera::feature_bool_get(
  const std::string_view & name,
  const Module module) const
{
  RCLCPP_DEBUG(get_logger(), "%s('%s')", __FUNCTION__, name.data());

  auto const handle = get_module_handle(module);

  bool value{};
  auto const err =
    api_->FeatureBoolGet(handle, name.data(), reinterpret_cast<bool *>(&value));

  if (err != VmbErrorSuccess) {
    RCLCPP_ERROR(
      get_logger(), "%s failed with error %d (%s)", __FUNCTION__, err,
      vmb_error_to_string(err).data());
    return error{err};
  }

  return value;
}

result<void> VimbaXCamera::feature_bool_set(
  const std::string_view & name,
  const bool value,
  const Module module) const
{
  RCLCPP_DEBUG(get_logger(), "%s('%s', %d)", __FUNCTION__, name.data(), value);

  auto const handle = get_module_handle(module);

  auto const err =
    api_->FeatureBoolSet(handle, name.data(), value);

  if (err != VmbErrorSuccess) {
    RCLCPP_ERROR(
      get_logger(), "%s failed with error %d (%s)", __FUNCTION__, err,
      vmb_error_to_string(err).data());
    return error{err};
  }

  return {};
}

result<std::string> VimbaXCamera::feature_enum_get(
  const std::string_view & name,
  const Module module) const
{
  return feature_enum_get(name, get_module_handle(module));
}

result<std::string> VimbaXCamera::feature_enum_get(
  const std::string_view & name,
  VmbHandle_t handle) const
{
  RCLCPP_DEBUG(get_logger(), "%s('%s')", __FUNCTION__, name.data());

  const char * value{nullptr};
  auto const err = api_->FeatureEnumGet(handle, name.data(), &value);

  if (err != VmbErrorSuccess) {
    RCLCPP_ERROR(
      get_logger(), "%s failed with error %d (%s)", __FUNCTION__, err,
      vmb_error_to_string(err).data());
    return error{err};
  }

  return std::string{value};
}

result<void>
VimbaXCamera::feature_enum_set(
  const std::string_view & name,
  const std::string_view & value,
  const Module module) const
{
  RCLCPP_DEBUG(get_logger(), "%s('%s', '%s')", __FUNCTION__, name.data(), value.data());

  auto const handle = get_module_handle(module);

  auto const err =
    api_->FeatureEnumSet(handle, name.data(), value.data());

  if (err != VmbErrorSuccess) {
    RCLCPP_ERROR(
      get_logger(), "%s failed with error %d (%s)", __FUNCTION__, err,
      vmb_error_to_string(err).data());
    return error{err};
  }

  return {};
}

result<std::array<std::vector<std::string>, 2>>
VimbaXCamera::feature_enum_info_get(
  const std::string_view & name,
  const Module module) const
{
  RCLCPP_DEBUG(get_logger(), "%s('%s')", __FUNCTION__, name.data());

  auto const handle = get_module_handle(module);

  uint32_t num_found{};
  bool available{false};
  std::array<std::vector<std::string>, 2> values;  // 0: possibleValues, 1: availableValues

  auto err =
    api_->FeatureEnumRangeQuery(handle, name.data(), nullptr, 0, &num_found);

  if (err != VmbErrorSuccess) {
    RCLCPP_ERROR(
      get_logger(), "%s failed with error %d (%s)", __FUNCTION__, err,
      vmb_error_to_string(err).data());
    return error{err};
  }

  std::vector<const char *> enum_entries(num_found);

  err =
    api_->FeatureEnumRangeQuery(handle, name.data(), &enum_entries[0], num_found, &num_found);

  if (err != VmbErrorSuccess) {
    RCLCPP_ERROR(
      get_logger(), "%s failed with error %d (%s)", __FUNCTION__, err,
      vmb_error_to_string(err).data());
    return error{err};
  }

  for (auto const & entry : enum_entries) {
    values[0].push_back(std::string(entry));

    err =
      api_->FeatureEnumIsAvailable(handle, name.data(), entry, &available);

    if (err == VmbErrorSuccess && available) {
      values[1].push_back(std::string(entry));
    }
  }

  return values;
}

result<int64_t> VimbaXCamera::feature_enum_as_int_get(
  const std::string_view & name,
  const std::string_view & option,
  const Module module) const
{
  RCLCPP_DEBUG(get_logger(), "%s('%s', '%s')", __FUNCTION__, name.data(), option.data());

  auto const handle = get_module_handle(module);

  int64_t value{-1};

  auto const err = api_->FeatureEnumAsInt(
    handle,
    name.data(),
    option.data(),
    reinterpret_cast<VmbInt64_t *>(&value));

  if (err != VmbErrorSuccess) {
    RCLCPP_ERROR(
      get_logger(), "%s failed to convert enum '%s' option '%s' to int with error %d (%s)",
      __FUNCTION__, name.data(), option.data(), err,
      vmb_error_to_string(err).data());

    return error{err};
  }

  return value;
}

result<std::string>
VimbaXCamera::feature_enum_as_string_get(
  const std::string_view & name,
  const int64_t value,
  const Module module) const
{
  RCLCPP_DEBUG(get_logger(), "%s('%s', %ld)", __FUNCTION__, name.data(), value);

  auto const handle = get_module_handle(module);

  std::string option;
  const char * string_value;

  auto const err = api_->FeatureEnumAsString(
    handle,
    name.data(),
    value,
    reinterpret_cast<const char **>(&string_value));

  if (err != VmbErrorSuccess) {
    RCLCPP_ERROR(
      get_logger(), "Failed to convert enum '%s' option '%s' to int with error %d (%s)",
      name.data(), option.data(), err, vmb_error_to_string(err).data());

    return error{err};
  } else {
    option.assign(string_value);
  }

  return option;
}

result<std::vector<unsigned char>>
VimbaXCamera::feature_raw_get(
  const std::string_view & name,
  const Module module) const
{
  RCLCPP_DEBUG(get_logger(), "%s('%s')", __FUNCTION__, name.data());

  auto const handle = get_module_handle(module);

  uint32_t length{};
  uint32_t size_filled{};

  auto err =
    api_->FeatureRawLengthQuery(handle, name.data(), &length);

  if (err != VmbErrorSuccess) {
    RCLCPP_ERROR(
      get_logger(), "%s failed with error %d (%s)", __FUNCTION__, err,
      vmb_error_to_string(err).data());
    return error{err};
  }

  std::vector<unsigned char> buffer(length);
  err =
    api_->FeatureRawGet(
    handle,
    name.data(),
    reinterpret_cast<char *>(&buffer[0]),
    length,
    &size_filled);

  if (err != VmbErrorSuccess) {
    RCLCPP_ERROR(
      get_logger(), "%s failed with error %d (%s)", __FUNCTION__, err,
      vmb_error_to_string(err).data());
    return error{err};
  }

  return buffer;
}

result<void> VimbaXCamera::feature_raw_set(
  const std::string_view & name,
  const std::vector<uint8_t> buffer,
  const Module module) const
{
  RCLCPP_DEBUG(
    get_logger(), "%s('%s', buffer.size()=%ld)", __FUNCTION__, name.data(), buffer.size());

  auto const handle = get_module_handle(module);

  auto const err =
    api_->FeatureRawSet(
    handle,
    name.data(),
    reinterpret_cast<const char *>(buffer.data()),
    static_cast<uint32_t>(buffer.size()));

  if (err != VmbErrorSuccess) {
    RCLCPP_ERROR(
      get_logger(), "%s failed with error %d (%s)", __FUNCTION__, err,
      vmb_error_to_string(err).data());
    return error{err};
  }

  return {};
}

result<uint32_t> VimbaXCamera::feature_raw_info_get(
  const std::string_view & name,
  const Module module) const
{
  RCLCPP_DEBUG(get_logger(), "%s('%s')", __FUNCTION__, name.data());

  auto const handle = get_module_handle(module);

  uint32_t value{};

  auto err =
    api_->FeatureRawLengthQuery(handle, name.data(), &value);

  if (err != VmbErrorSuccess) {
    RCLCPP_ERROR(
      get_logger(), "%s failed with error %d (%s)", __FUNCTION__, err,
      vmb_error_to_string(err).data());
    return error{err};
  }

  return value;
}

result<std::array<bool, 2>>
VimbaXCamera::feature_access_mode_get(
  const std::string_view & name,
  const Module module) const
{
  RCLCPP_DEBUG(get_logger(), "%s('%s')", __FUNCTION__, name.data());

  std::array<bool, 2> value;

  auto const handle = get_module_handle(module);

  auto const err =
    api_->FeatureAccessQuery(handle, name.data(), &value[0], &value[1]);

  if (err != VmbErrorSuccess) {
    RCLCPP_ERROR(
      get_logger(), "%s failed with error %d (%s)", __FUNCTION__, err,
      vmb_error_to_string(err).data());
    return error{err};
  }

  return value;
}

result<std::vector<feature_info>>
VimbaXCamera::feature_info_query_list(
  const std::vector<std::string> & names,
  const Module module) const
{
  RCLCPP_DEBUG(get_logger(), "%s", __FUNCTION__);

  auto const handle = get_module_handle(module);

  std::vector<feature_info> infos;

  for (auto name : names) {
    VmbFeatureInfo vmb_info{};
    feature_info info{};

    auto const err =
      api_->FeatureInfoQuery(handle, name.data(), &vmb_info, sizeof(vmb_info));

    if (err != VmbErrorSuccess) {
      RCLCPP_ERROR(
        get_logger(), "Reading feature info for '%s' failed with %d (%s)", name.data(),
        err, vmb_error_to_string(err).data());
      return error{err};
    }
    info.name = std::string(vmb_info.name ? vmb_info.name : "");
    info.category = std::string(vmb_info.category ? vmb_info.category : "");
    info.display_name = std::string(vmb_info.displayName ? vmb_info.displayName : "");
    info.sfnc_namespace = std::string(vmb_info.sfncNamespace ? vmb_info.sfncNamespace : "");
    info.unit = std::string(vmb_info.unit ? vmb_info.unit : "");
    info.data_type = static_cast<uint32_t>(vmb_info.featureDataType);
    info.flags.flag_none = vmb_info.featureDataType == 0;
    info.flags.flag_read =
      (vmb_info.featureDataType & VmbFeatureFlagsRead) == VmbFeatureFlagsRead;
    info.flags.flag_write = (
      vmb_info.featureDataType & VmbFeatureFlagsWrite) == VmbFeatureFlagsWrite;
    info.flags.flag_volatile =
      (vmb_info.featureDataType & VmbFeatureFlagsVolatile) == VmbFeatureFlagsVolatile;
    info.flags.flag_modify_write =
      (vmb_info.featureDataType & VmbFeatureFlagsModifyWrite) == VmbFeatureFlagsModifyWrite;
    info.polling_time = vmb_info.pollingTime;

    infos.push_back(info);
  }

  return infos;
}

result<VmbPixelFormatType> VimbaXCamera::get_pixel_format() const
{
  auto const feature_info_res = feature_info_query(SFNCFeatures::PixelFormat);

  if (!feature_info_res) {
    return feature_info_res.error();
  } else if (feature_info_res->sfncNamespace == nullptr) {
    return error{VmbErrorInvalidAddress};
  } else if (std::string_view{feature_info_res->sfncNamespace} != "Standard") {
    return error{VmbErrorNotAvailable};
  }

  auto const current_format_str = feature_enum_get(SFNCFeatures::PixelFormat);

  if (!current_format_str) {
    return current_format_str.error();
  }

  auto const current_format =
    feature_enum_as_int_get(SFNCFeatures::PixelFormat, *current_format_str);

  if (!current_format) {
    return current_format.error();
  }

  return static_cast<VmbPixelFormatType>(*current_format);
}

result<VmbFeatureInfo> VimbaXCamera::feature_info_query(
  const std::string_view & name,
  const Module module) const
{
  RCLCPP_DEBUG(get_logger(), "%s", __FUNCTION__);
  VmbFeatureInfo feature_info{};

  auto const handle = get_module_handle(module);

  auto const err =
    api_->FeatureInfoQuery(handle, name.data(), &feature_info, sizeof(feature_info));

  if (err != VmbErrorSuccess) {
    RCLCPP_ERROR(
      get_logger(), "Reading feature info for '%s' failed with error %d (%s)", name.data(), err,
      (vmb_error_to_string(err)).data());
    return error{err};
  }

  return feature_info;
}

result<void> VimbaXCamera::settings_load(const std::string_view & file_name)
{
  fs::path settings_file_path{file_name};

  if (!fs::exists(settings_file_path)) {
    return error{VmbErrorNotFound};
  }

  auto const persist_settings = get_default_feature_persist_settings();

  auto const err = api_->SettingsLoad(
    camera_handle_,
    settings_file_path.c_str(),
    &persist_settings,
    sizeof(persist_settings));

  if (err != VmbErrorSuccess) {
    return error{err};
  }

  return {};
}

result<void> VimbaXCamera::settings_save(const std::string_view & file_name)
{
  fs::path settings_file_path{file_name};

  if (settings_file_path.extension() != ".xml") {
    return error{VmbErrorInvalidValue};
  }

  if (!fs::exists(settings_file_path.parent_path())) {
    return error{VmbErrorNotFound};
  }

  auto const persist_settings = get_default_feature_persist_settings();

  auto const err = api_->SettingsSave(
    camera_handle_,
    settings_file_path.c_str(),
    &persist_settings,
    sizeof(persist_settings));

  if (err != VmbErrorSuccess) {
    return error{err};
  }

  return {};
}

VmbFeaturePersistSettings VimbaXCamera::get_default_feature_persist_settings() const
{
  return {
    VmbFeaturePersistType::VmbFeaturePersistNoLUT,
    VmbModulePersistFlagsType::VmbModulePersistFlagsRemoteDevice |
    VmbModulePersistFlagsType::VmbModulePersistFlagsLocalDevice |
    VmbModulePersistFlagsType::VmbModulePersistFlagsStreams,
    10,
    VmbLogLevel::VmbLogLevelWarn
  };
}

result<VimbaXCamera::Info> VimbaXCamera::camera_info_get() const
{
  Info info{};

  info.display_name = camera_info_.cameraName;
  info.model_name = camera_info_.modelName;

  auto const firmware_version = feature_string_get(SFNCFeatures::DeviceFirmwareVersion);
  if (!firmware_version) {
    return firmware_version.error();
  }
  info.firmware_version = *firmware_version;

  info.device_id = camera_info_.cameraIdString;

  auto const device_user_id = feature_string_get(SFNCFeatures::DeviceUserId);
  if (!device_user_id) {
    return device_user_id.error();
  }
  info.device_user_id = *device_user_id;

  info.device_serial_number = camera_info_.serialString;


  auto const interface_id =
    feature_string_get(SFNCFeatures::InterfaceId, camera_info_.interfaceHandle);
  if (!interface_id) {
    return interface_id.error();
  }
  info.interface_id = *interface_id;

  auto const transport_layer_id =
    feature_string_get(SFNCFeatures::TransportLayerId, camera_info_.transportLayerHandle);
  if (!transport_layer_id) {
    return transport_layer_id.error();
  }
  info.transport_layer_id = *transport_layer_id;

  info.streaming = is_streaming();

  auto const width = feature_int_get(SFNCFeatures::Width);
  if (!width) {
    return width.error();
  }
  info.width = *width;

  auto const height = feature_int_get(SFNCFeatures::Height);
  if (!height) {
    return height.error();
  } else {
    info.height = *height;
  }

  auto const frame_rate = feature_float_get(SFNCFeatures::AcquisitionFrameRate);
  if (!frame_rate) {
    if (frame_rate.error().code != VmbErrorNotAvailable) {
      return frame_rate.error();
    }
  } else {
    info.frame_rate = *frame_rate;
  }

  auto const pixel_format = feature_enum_get(SFNCFeatures::PixelFormat);
  if (!pixel_format) {
    return pixel_format.error();
  } else {
    info.pixel_format = *pixel_format;
  }

  auto const trigger_selector_info = feature_enum_info_get(SFNCFeatures::TriggerSelector);

  if (!trigger_selector_info) {
    return trigger_selector_info.error();
  }

  for (auto const & selector : (*trigger_selector_info)[1]) {
    if (feature_enum_set(SFNCFeatures::TriggerSelector, selector)) {
      auto const get_opt = [&](auto const & opt) -> std::string {
          if (opt) {
            return *opt;
          }

          return "N/A";
        };

      auto const trigger_mode = feature_enum_get(SFNCFeatures::TriggerMode);
      auto const trigger_source = feature_enum_get(SFNCFeatures::TriggerSource);

      info.trigger_info.emplace_back(selector, get_opt(trigger_mode), get_opt(trigger_source));
    }
  }


  if (has_feature(SFNCFeatures::GevDeviceIPAddress)) {
    auto const ip_address =
      feature_int_get(SFNCFeatures::GevDeviceIPAddress, camera_info_.localDeviceHandle);
    if (ip_address) {
      auto const u8ptr = reinterpret_cast<const uint8_t *>(&(*ip_address));
      info.ip_address =
        std::to_string(u8ptr[3]) + "." +
        std::to_string(u8ptr[2]) + "." +
        std::to_string(u8ptr[1]) + "." +
        std::to_string(u8ptr[0]);
    }
  }

  if (has_feature(SFNCFeatures::GevDeviceMACAddress)) {
    auto const mac_address =
      feature_int_get(SFNCFeatures::GevDeviceMACAddress, camera_info_.localDeviceHandle);
    if (mac_address) {
      auto const u8ptr = reinterpret_cast<const uint8_t *>(&(*mac_address));

      std::stringstream mac_address_stream{};
      mac_address_stream << std::hex << std::setfill('0') << std::setw(2) << int(u8ptr[5]);
      for (int i = 4; i >= 0; i--) {
        mac_address_stream << ":" << std::hex << std::setfill('0') << std::setw(2) << int(u8ptr[i]);
      }

      info.mac_address = mac_address_stream.str();
    }
  }

  return info;
}

bool VimbaXCamera::is_streaming() const
{
  auto const current_state = stream_state_.load();

  return current_state == StreamState::kActive || current_state == StreamState::kStarting;
}

void VimbaXCamera::on_feature_invalidation(VmbHandle_t, const char * name, void * context)
{
  auto _this = reinterpret_cast<VimbaXCamera *>(context);
  std::lock_guard guard{_this->invalidation_callbacks_mutex_};
  auto const it = _this->invalidation_callbacks_.find(name);
  if (it != _this->invalidation_callbacks_.end()) {
    it->second(name);
  }
}

result<void> VimbaXCamera::feature_invalidation_register(
  const std::string_view & name,
  std::function<void(const std::string &)> callback)
{
  std::unique_lock lock{invalidation_callbacks_mutex_};
  invalidation_callbacks_.emplace(name, callback);
  lock.unlock();

  auto const err = api_->FeatureInvalidationRegister(
    camera_handle_, name.data(),
    on_feature_invalidation, this);

  if (err != VmbErrorSuccess) {
    return error{err};
  }

  return {};
}

result<void> VimbaXCamera::feature_invalidation_unregister(const std::string_view & name)
{
  std::unique_lock lock{invalidation_callbacks_mutex_};
  invalidation_callbacks_.extract(std::string{name});
  lock.unlock();

  auto const err =
    api_->FeatureInvalidationUnregister(camera_handle_, name.data(), on_feature_invalidation);

  if (err != VmbErrorSuccess) {
    return error{err};
  }

  return {};
}

result<VimbaXCamera::EventMetaDataList>
VimbaXCamera::get_event_meta_data(const std::string_view & name)
{
  auto const category_path = "/EventControl/EventsData/Event" + std::string{name} + "Data";

  auto const & [start, end] =
    feature_category_map_[std::size_t(Module::RemoteDevice)].equal_range(category_path);

  EventMetaDataList meta_data_list{};

  for (auto it = start; it != end; it++) {
    auto const info = get_feature_info(it->second);
    switch (info.featureDataType) {
      case VmbFeatureDataInt:
        {
          auto const value_res = feature_int_get(it->second);
          if (value_res) {
            meta_data_list.emplace_back(it->second, std::to_string(*value_res));
          }
        }
        break;
      case VmbFeatureDataBool:
        {
          auto const value_res = feature_bool_get(it->second);
          if (value_res) {
            meta_data_list.emplace_back(it->second, std::to_string(*value_res));
          }
        }
        break;
      case VmbFeatureDataFloat:
        {
          auto const value_res = feature_float_get(it->second);
          if (value_res) {
            meta_data_list.emplace_back(it->second, std::to_string(*value_res));
          }
        }
        break;
      case VmbFeatureDataString:
        {
          auto const value_res = feature_string_get(it->second);
          if (value_res) {
            meta_data_list.emplace_back(it->second, *value_res);
          }
        }
        break;
      case VmbFeatureDataEnum:
        {
          auto const value_res = feature_enum_get(it->second);
          if (value_res) {
            meta_data_list.emplace_back(it->second, *value_res);
          }
        }
        break;
      default:
        break;
    }
  }

  return meta_data_list;
}

bool VimbaXCamera::is_valid_pixel_format(VmbPixelFormatType pixel_format)
{
  switch (pixel_format) {
    case VmbPixelFormatMono8:
    case VmbPixelFormatMono12:
    case VmbPixelFormatMono16:
    case VmbPixelFormatBayerGR8:
    case VmbPixelFormatBayerRG8:
    case VmbPixelFormatBayerGB8:
    case VmbPixelFormatBayerBG8:
    case VmbPixelFormatBayerGR10:
    case VmbPixelFormatBayerRG10:
    case VmbPixelFormatBayerGB10:
    case VmbPixelFormatBayerBG10:
    case VmbPixelFormatBayerGR12:
    case VmbPixelFormatBayerRG12:
    case VmbPixelFormatBayerGB12:
    case VmbPixelFormatBayerBG12:
    case VmbPixelFormatBayerGR16:
    case VmbPixelFormatBayerRG16:
    case VmbPixelFormatBayerGB16:
    case VmbPixelFormatBayerBG16:
    case VmbPixelFormatRgb8:
    case VmbPixelFormatBgr8:
    case VmbPixelFormatYuv422_8:
    case VmbPixelFormatYCbCr422_8:
    case VmbPixelFormatYCbCr422_8_CbYCrY:
      return true;
    default:
      return false;
  }

  return false;
}

result<std::shared_ptr<VimbaXCamera::Frame>> VimbaXCamera::Frame::create(
  std::shared_ptr<VimbaXCamera> camera,
  size_t size,
  size_t alignment)
{
  auto const pixel_format = camera->get_pixel_format();

  if (!pixel_format) {
    return pixel_format.error();
  }

  uint32_t const bpp = (*pixel_format >> 16) & 0xFF;

  auto const width = camera->feature_int_get(SFNCFeatures::Width);
  auto const height = camera->feature_int_get(SFNCFeatures::Height);

  if (!width) {
    return width.error();
  } else if (!height) {
    return height.error();
  }

  auto const line = *width * bpp / 8;
  size_t const real_size = *height * line;

  size_t const aligned_size = [&] {
      if (alignment > 1) {
        auto const mask = (alignment - 1);

        auto const alignment = mask + 1;
        auto const offset = (size & mask);
        auto const offset_to_next = (alignment - offset) & mask;

        return size + offset_to_next;
      }

      return size;
    }();

  auto const alloc_mode =
    (real_size == aligned_size) ? AllocationMode::kByImage : AllocationMode::kByTl;

  std::shared_ptr<VimbaXCamera::Frame> frame(new VimbaXCamera::Frame{camera, alloc_mode});

  if (alloc_mode == AllocationMode::kByTl) {
    frame->data.resize(real_size);

    frame->vmb_frame_.buffer = nullptr;
    frame->vmb_frame_.bufferSize = size;
  } else {
    frame->data.resize(size);

    frame->vmb_frame_.buffer = frame->data.data();
    frame->vmb_frame_.bufferSize = frame->data.size();
  }

  frame->step = line;

  auto announce_error =
    camera->api_->FrameAnnounce(camera->camera_handle_, &frame->vmb_frame_, sizeof(vmb_frame_));

  if (announce_error != VmbErrorSuccess) {
    return error{announce_error};
  }

  return frame;
}

void VimbaXCamera::Frame::vmb_frame_callback(
  const VmbHandle_t, const VmbHandle_t, VmbFrame_t * frame)
{
  auto * ptr = reinterpret_cast<VimbaXCamera::Frame *>(frame->context[0]);
  auto shared_frame = ptr->shared_from_this();

  if (frame->receiveStatus == VmbFrameStatusType::VmbFrameStatusComplete) {
    auto shared_camera = shared_frame->camera_.lock();
    if (shared_camera) {
      {
        std::lock_guard guard{shared_camera->frame_ready_queue_mutex_};
        shared_camera->frame_ready_queue_.push(shared_frame);
      }
      shared_camera->frame_ready_cv_.notify_one();
    }
  } else {
    RCLCPP_WARN(get_logger(), "Frame with status %d received", frame->receiveStatus);
    shared_frame->queue();
  }
}

void VimbaXCamera::Frame::on_frame_ready()
{
  encoding = get_image_encoding();
  width = vmb_frame_.width;
  height = vmb_frame_.height;
  is_bigendian = false;


  transform();

  if (callback_) {
    callback_(shared_from_this());
  }
}

uint64_t VimbaXCamera::Frame::timestamp_to_ns(uint64_t timestamp) const
{
  if (!camera_.expired()) {
    auto camera = camera_.lock();


    if (camera->timestamp_frequency_) {
      RCLCPP_DEBUG(get_logger(), "Using timestamp frequency %ld", *camera->timestamp_frequency_);

      if (*camera->timestamp_frequency_ > std::nano::den) {
        return timestamp / ((*camera->timestamp_frequency_) / std::nano::den);
      } else {
        return timestamp * (std::nano::den / (*camera->timestamp_frequency_));
      }
    }
  }

  return timestamp;
}


void VimbaXCamera::Frame::transform()
{
  switch (VmbPixelFormatType(vmb_frame_.pixelFormat)) {
    case VmbPixelFormatMono10:
    case VmbPixelFormatBayerBG10:
    case VmbPixelFormatBayerGB10:
    case VmbPixelFormatBayerGR10:
    case VmbPixelFormatBayerRG10:
      helper::left_shift16(data.data(), vmb_frame_.imageData, data.size(), 6);
      break;
    case VmbPixelFormatMono12:
    case VmbPixelFormatBayerBG12:
    case VmbPixelFormatBayerGB12:
    case VmbPixelFormatBayerGR12:
    case VmbPixelFormatBayerRG12:
      helper::left_shift16(data.data(), vmb_frame_.imageData, data.size(), 4);
      break;
    case VmbPixelFormatMono14:
      helper::left_shift16(data.data(), vmb_frame_.imageData, data.size(), 2);
      break;
    default:
      if (allocation_mode_ == AllocationMode::kByTl) {
        memcpy(data.data(), vmb_frame_.imageData, data.size());
      }
      break;
  }
}

VimbaXCamera::Frame::Frame(std::shared_ptr<VimbaXCamera> camera, AllocationMode allocation_mode)
: camera_{camera}, allocation_mode_{allocation_mode}
{
  vmb_frame_.context[0] = this;
}

VimbaXCamera::Frame::~Frame()
{
}


void VimbaXCamera::Frame::set_callback(std::function<void(std::shared_ptr<Frame> frame)> callback)
{
  callback_ = std::move(callback);
}

int32_t VimbaXCamera::Frame::queue() const
{
  if (!camera_.expired()) {
    auto camera = camera_.lock();
    return camera->api_->CaptureFrameQueue(camera->camera_handle_, &vmb_frame_, vmb_frame_callback);
  }

  return VmbErrorUnknown;
}

std::string VimbaXCamera::Frame::get_image_encoding() const
{
  switch (VmbPixelFormatType(vmb_frame_.pixelFormat)) {
    case VmbPixelFormatMono8:
      return sensor_msgs::image_encodings::MONO8;
    case VmbPixelFormatMono10:
    case VmbPixelFormatMono12:
    case VmbPixelFormatMono14:
    case VmbPixelFormatMono16:
      return sensor_msgs::image_encodings::MONO16;
    case VmbPixelFormatBayerGR8:
      return sensor_msgs::image_encodings::BAYER_GRBG8;
    case VmbPixelFormatBayerRG8:
      return sensor_msgs::image_encodings::BAYER_RGGB8;
    case VmbPixelFormatBayerGB8:
      return sensor_msgs::image_encodings::BAYER_GBRG8;
    case VmbPixelFormatBayerBG8:
      return sensor_msgs::image_encodings::BAYER_BGGR8;
    case VmbPixelFormatBayerGR10:
      return sensor_msgs::image_encodings::BAYER_GRBG16;
    case VmbPixelFormatBayerRG10:
      return sensor_msgs::image_encodings::BAYER_RGGB16;
    case VmbPixelFormatBayerGB10:
      return sensor_msgs::image_encodings::BAYER_GBRG16;
    case VmbPixelFormatBayerBG10:
      return sensor_msgs::image_encodings::BAYER_BGGR16;
    case VmbPixelFormatBayerGR12:
      return sensor_msgs::image_encodings::BAYER_GRBG16;
    case VmbPixelFormatBayerRG12:
      return sensor_msgs::image_encodings::BAYER_RGGB16;
    case VmbPixelFormatBayerGB12:
      return sensor_msgs::image_encodings::BAYER_GBRG16;
    case VmbPixelFormatBayerBG12:
      return sensor_msgs::image_encodings::BAYER_BGGR16;
    case VmbPixelFormatBayerGR16:
      return sensor_msgs::image_encodings::BAYER_GRBG16;
    case VmbPixelFormatBayerRG16:
      return sensor_msgs::image_encodings::BAYER_RGGB16;
    case VmbPixelFormatBayerGB16:
      return sensor_msgs::image_encodings::BAYER_GBRG16;
    case VmbPixelFormatBayerBG16:
      return sensor_msgs::image_encodings::BAYER_BGGR16;
    case VmbPixelFormatRgb8:
      return sensor_msgs::image_encodings::RGB8;
    case VmbPixelFormatBgr8:
      return sensor_msgs::image_encodings::BGR8;
    case VmbPixelFormatRgb16:
      return sensor_msgs::image_encodings::RGB16;
    case VmbPixelFormatBgr16:
      return sensor_msgs::image_encodings::BGR16;
    case VmbPixelFormatArgb8:
      return sensor_msgs::image_encodings::RGBA8;
    case VmbPixelFormatBgra8:
      return sensor_msgs::image_encodings::BGRA8;
    case VmbPixelFormatRgba16:
      return sensor_msgs::image_encodings::RGBA16;
    case VmbPixelFormatBgra16:
      return sensor_msgs::image_encodings::BGRA16;
    case VmbPixelFormatYuv422:
      return sensor_msgs::image_encodings::YUV422;
    case VmbPixelFormatYuv422_8:
    case VmbPixelFormatYCbCr422_8:
    case VmbPixelFormatYCbCr601_422_8:
    case VmbPixelFormatYCbCr709_422_8:
      return sensor_msgs::image_encodings::YUV422_YUY2;
    case VmbPixelFormatYCbCr422_8_CbYCrY:
      return sensor_msgs::image_encodings::YUV422;
    case VmbPixelFormatYCbCr601_422_8_CbYCrY:
    case VmbPixelFormatYCbCr709_422_8_CbYCrY:
      return sensor_msgs::image_encodings::YUV422_YUY2;
    default:
      return sensor_msgs::image_encodings::TYPE_8UC1;
  }
}

int64_t VimbaXCamera::Frame::get_frame_id() const
{
  return vmb_frame_.frameID;
}

uint64_t VimbaXCamera::Frame::get_timestamp_ns() const
{
  return timestamp_to_ns(vmb_frame_.timestamp);
}

}  // namespace vimbax_camera
