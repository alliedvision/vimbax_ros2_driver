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

#include <optional>
#include <filesystem>

#include <rclcpp/rclcpp.hpp>

#include <vimbax_camera/vimbax_camera.hpp>

namespace fs = std::filesystem;

namespace vimbax_camera
{
using helper::get_logger;

std::shared_ptr<VimbaXCamera> VimbaXCamera::open(
  std::shared_ptr<VmbCAPI> api,
  const std::string & name)
{
  auto checkAccess = [](const VmbCameraInfo_t & info) {
      return (info.permittedAccess & VmbAccessModeType::VmbAccessModeExclusive) != 0;
    };

  auto openCamera =
    [&](const std::string & idStr) -> std::optional<VmbHandle_t> {
      VmbHandle_t cameraHandle;
      auto const openError =
        api->CameraOpen(idStr.c_str(), VmbAccessModeType::VmbAccessModeExclusive, &cameraHandle);

      if (openError != VmbErrorSuccess) {
        RCLCPP_ERROR(get_logger(), "Failed to open camera %s with %d", idStr.c_str(), openError);
        return std::nullopt;
      }

      return cameraHandle;
    };

  auto const availableCameras =
    [&]() -> std::vector<VmbCameraInfo_t> {
      uint32_t availableCamerasCount{0};
      auto const countError = api->CamerasList(nullptr, 0, &availableCamerasCount, 0);

      if (countError != VmbErrorSuccess) {
        RCLCPP_ERROR(get_logger(), "Reading camera list size failed with %d", countError);
        return {};
      }

      std::vector<VmbCameraInfo_t> cameraList{};
      cameraList.resize(availableCamerasCount);
      uint32_t camerasFound = 0;

      auto const error = api->CamerasList(
        cameraList.data(), availableCamerasCount, &camerasFound, sizeof(VmbCameraInfo_t));

      if (error != VmbErrorSuccess) {
        RCLCPP_ERROR(get_logger(), "List first camera failed with %d", error);
        return {};
      }

      return cameraList;
    }();

  if (name.empty()) {
    RCLCPP_INFO(get_logger(), "No camera requested opening first available");

    if (availableCameras.empty()) {
      RCLCPP_ERROR(get_logger(), "List cameras returned 0");
      return nullptr;
    }

    for (auto const & info : availableCameras) {
      if (checkAccess(info)) {
        RCLCPP_INFO(get_logger(), "Try opening camera with extended id %s", info.cameraIdExtended);

        auto const optHandle = openCamera(info.cameraIdExtended);

        if (optHandle) {
          return std::unique_ptr<VimbaXCamera>(new VimbaXCamera{api, *optHandle});
        }
      }
    }

    RCLCPP_ERROR(get_logger(), "No camera available!");

    return nullptr;
  } else {
    auto const checkCameraInfo =
      [&name](const VmbCameraInfo_t & info) -> bool {
        return info.cameraIdString == name ||
               info.cameraIdExtended == name ||
               info.serialString == name;
      };
    // Try open by serial number
    if (!availableCameras.empty()) {
      for (auto const & info : availableCameras) {
        if (checkCameraInfo(info) && (info.permittedAccess & VmbAccessModeExclusive) != 0) {
          auto const optHandleSerial = openCamera(info.cameraIdExtended);

          if (optHandleSerial) {
            return std::unique_ptr<VimbaXCamera>(new VimbaXCamera{api, *optHandleSerial});
          }
        }
      }
    }

    RCLCPP_WARN(get_logger(), "No matching camera found, falling back to VmbCameraOpen");

    auto const optHandle = openCamera(name);

    if (optHandle) {
      return std::unique_ptr<VimbaXCamera>(new VimbaXCamera{api, *optHandle});
    }

    RCLCPP_ERROR(get_logger(), "Failed to open given camera %s", name.c_str());
  }

  return nullptr;
}

VimbaXCamera::VimbaXCamera(std::shared_ptr<VmbCAPI> api, VmbHandle_t cameraHandle)
: api_{std::move(api)}, camera_handle_{cameraHandle}
{
  auto const err =
    api_->CameraInfoQueryByHandle(camera_handle_, &camera_info_, sizeof(camera_info_));

  if (err != VmbErrorSuccess) {
    RCLCPP_ERROR(get_logger(), "Failed to query camera info!");
  }

  RCLCPP_INFO(
    get_logger(), "Opened camera info model name: %s, camera name: %s, serial: %s",
    camera_info_.modelName, camera_info_.cameraName, camera_info_.serialString);

  auto const timestamp_frequency =
    feature_int_get("DeviceTimestampFrequency", camera_info_.localDeviceHandle);

  if (timestamp_frequency) {
    timestamp_frequency_ = *timestamp_frequency;
  }

  feature_command_run("GVSPAdjustPacketSize", camera_info_.streamHandles[0]);
}

VimbaXCamera::~VimbaXCamera()
{
  if (is_alive())
  {
    stop_streaming();
  }

  if (api_ && camera_handle_) {
    api_->CameraClose(camera_handle_);
    camera_handle_ = nullptr;
  }
}

bool VimbaXCamera::is_alive()
{
  VmbCameraInfo cameraInfo{};

  auto const err = api_->CameraInfoQueryByHandle(camera_handle_, &cameraInfo, sizeof(cameraInfo));

  return (err == VmbErrorNotFound) ? false : true;
}

result<void> VimbaXCamera::start_streaming(
  int bufferCount,
  std::function<void(std::shared_ptr<Frame>)> onFrame,
  bool startAcquisition)
{
  if (!streaming_) {
    frames_.clear();
    frames_.resize(bufferCount);

    uint32_t payloadSize{};

    auto const payloadSizeError = api_->PayloadSizeGet(camera_handle_, &payloadSize);
    if (payloadSizeError != VmbErrorSuccess) {
      return error{payloadSizeError};
    }

    auto const alignment_res =
      feature_int_get(SFNCFeatures::StreamBufferAlignment, camera_info_.streamHandles[0]);

    auto const alignment = alignment_res ? *alignment_res : 1;

    for (auto & frame : frames_) {
      auto newFrame = Frame::create(shared_from_this(), payloadSize, alignment);

      if (!newFrame) {
        RCLCPP_ERROR(get_logger(), "Failed to create frame");
        return newFrame.error();
      }

      frame = *newFrame;

      frame->set_callback(onFrame);
    }

    auto const capStartError = api_->CaptureStart(camera_handle_);
    if (capStartError != VmbErrorSuccess) {
      RCLCPP_ERROR(get_logger(), "Capture start failed with %d", capStartError);
      return error{capStartError};
    }

    for (auto const & frame : frames_) {
      auto const queueError = frame->queue();
      if (queueError != VmbErrorSuccess) {
        RCLCPP_ERROR(get_logger(), "Queue frame failed with %d", queueError);
        return error{queueError};
      }
    }

    if (startAcquisition) {
      auto const acqStartError = feature_command_run(SFNCFeatures::AcquisitionStart);
      if (!acqStartError) {
        RCLCPP_ERROR(get_logger(), "Acquisition start failed with %d", acqStartError.error().code);
        return acqStartError.error();
      }
    }

    streaming_ = true;
  }

  return {};
}

result<void> VimbaXCamera::stop_streaming()
{
  if (!streaming_) {
    return {};
  }

  if (is_alive())
  {
    auto const acqStopError = feature_command_run(SFNCFeatures::AcquisitionStop);
    if (!acqStopError) {
      RCLCPP_ERROR(get_logger(), "Acquisition stop failed with %d", acqStopError.error().code);
      return acqStopError.error();
    }
  }

  auto const capStopError = api_->CaptureEnd(camera_handle_);
  if (capStopError != VmbErrorSuccess) {
    RCLCPP_ERROR(get_logger(), "Capture stop failed with %d", capStopError);
    return error{capStopError};
  }

  auto const flushError = api_->CaptureQueueFlush(camera_handle_);
  if (flushError != VmbErrorSuccess) {
    RCLCPP_ERROR(get_logger(), "Flush capture queue failed with %d", flushError);
    return error{flushError};
  }

  auto const revokeError = api_->FrameRevokeAll(camera_handle_);
  if (revokeError != VmbErrorSuccess) {
    RCLCPP_ERROR(get_logger(), "Revoking frames failed with %d", revokeError);
    return error{revokeError};
  }

  frames_.clear();

  streaming_ = false;

  return {};
}

result<VmbCameraInfo> VimbaXCamera::query_camera_info() const
{
  VmbCameraInfo cameraInfo{};

  auto const err = api_->CameraInfoQueryByHandle(camera_handle_, &cameraInfo, sizeof(cameraInfo));

  if (err != VmbErrorSuccess) {
    RCLCPP_ERROR(get_logger(), "Failed to query camera info!");
    return error{err};
  }

  return cameraInfo;
}

result<std::vector<std::string>> VimbaXCamera::features_list_get(void) const
{
  std::vector<std::string> feature_list;
  uint32_t feature_count{};
  auto err =
    api_->FeaturesList(camera_handle_, nullptr, 0, &feature_count, sizeof(VmbFeatureInfo_t));

  if (err != VmbErrorSuccess) {
    RCLCPP_ERROR(get_logger(), "%s failed with error %d", __FUNCTION__, err);
    return error{err};
  }

  VmbFeatureInfo_t * features =
    static_cast<VmbFeatureInfo_t *>(malloc(feature_count * sizeof(VmbFeatureInfo_t)));

  err = api_->FeaturesList(
    camera_handle_, features, feature_count, &feature_count, sizeof(VmbFeatureInfo_t));

  if (err != VmbErrorSuccess) {
    RCLCPP_ERROR(get_logger(), "%s failed with error %d", __FUNCTION__, err);
    return error{err};
  }

  for (auto count = 0; count < feature_count; count++) {
    feature_list.push_back(std::string(features[count].name));
  }

  free(features);
  features = nullptr;

  return feature_list;
}

result<bool> VimbaXCamera::feature_command_is_done(const std::string_view & name) const
{
  RCLCPP_INFO(get_logger(), "%s(%s)", __FUNCTION__, name.data());

  bool value{};
  auto const err =
    api_->FeatureCommandIsDone(camera_handle_, name.data(), reinterpret_cast<bool *>(&value));

  if (err != VmbErrorSuccess) {
    RCLCPP_ERROR(get_logger(), "%s failed with error %d", __FUNCTION__, err);
    return error{err};
  }

  return value;
}

result<void> VimbaXCamera::feature_command_run(const std::string_view & name) const
{
  return feature_command_run(name, camera_handle_);
}

result<void> VimbaXCamera::feature_command_run(
  const std::string_view & name, VmbHandle_t handle) const
{
  auto const run_error = api_->FeatureCommandRun(handle, name.data());

  if (run_error != VmbErrorSuccess) {
    return error{run_error};
  }

  bool done{false};
  api_->FeatureCommandIsDone(handle, name.data(), &done);

  while (!done) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    api_->FeatureCommandIsDone(handle, name.data(), &done);
  }

  return {};
}

result<int64_t> VimbaXCamera::feature_int_get(const std::string_view & name) const
{
  return feature_int_get(name, camera_handle_);
}

result<int64_t> VimbaXCamera::feature_int_get(
  const std::string_view & name,
  VmbHandle_t handle) const
{
  RCLCPP_INFO(get_logger(), "%s('%s')", __FUNCTION__, name.data());
  int64_t value{};
  auto const err =
    api_->FeatureIntGet(handle, name.data(), reinterpret_cast<VmbInt64_t *>(&value));

  if (err != VmbErrorSuccess) {
    RCLCPP_ERROR(get_logger(), "%s %s failed with error %d", __FUNCTION__, name.data(), err);
    return error{err};
  }

  return value;
}

result<void> VimbaXCamera::feature_int_set(const std::string_view & name, const int64_t value) const
{
  RCLCPP_INFO(get_logger(), "%s('%s', %ld)", __FUNCTION__, name.data(), value);
  auto const err =
    api_->FeatureIntSet(camera_handle_, name.data(), value);

  if (err != VmbErrorSuccess) {
    RCLCPP_ERROR(get_logger(), "%s failed with error %d", __FUNCTION__, err);
    return error{err};
  }

  return {};
}

result<std::array<int64_t, 3>>
VimbaXCamera::feature_int_info_get(const std::string_view & name) const
{
  RCLCPP_INFO(get_logger(), "%s('%s')", __FUNCTION__, name.data());
  std::array<int64_t, 3> value;

  auto err =
    api_->FeatureIntRangeQuery(
    camera_handle_,
    name.data(),
    reinterpret_cast<VmbInt64_t *>(&value[0]),
    reinterpret_cast<VmbInt64_t *>(&value[1]));

  if (err != VmbErrorSuccess) {
    RCLCPP_ERROR(get_logger(), "%s failed with error %d", __FUNCTION__, err);
    return error{err};
  }

  err =
    api_->FeatureIntIncrementQuery(
    camera_handle_,
    name.data(),
    reinterpret_cast<VmbInt64_t *>(&value[2]));

  if (err != VmbErrorSuccess) {
    RCLCPP_ERROR(get_logger(), "%s failed with error %d", __FUNCTION__, err);
    return error{err};
  }

  return value;
}

result<_Float64> VimbaXCamera::feature_float_get(const std::string_view & name) const
{
  return feature_float_get(name, camera_handle_);
}

result<_Float64> VimbaXCamera::feature_float_get(
  const std::string_view & name, VmbHandle_t handle) const
{
  RCLCPP_INFO(get_logger(), "%s('%s')", __FUNCTION__, name.data());

  _Float64 value{};
  auto const err =
    api_->FeatureFloatGet(handle, name.data(), reinterpret_cast<_Float64 *>(&value));

  if (err != VmbErrorSuccess) {
    RCLCPP_ERROR(get_logger(), "%s failed with error %d", __FUNCTION__, err);
    return error{err};
  }

  return value;
}

result<void>
VimbaXCamera::feature_float_set(const std::string_view & name, const _Float64 value) const
{
  RCLCPP_INFO(get_logger(), "%s('%s', %lf)", __FUNCTION__, name.data(), value);
  auto const err =
    api_->FeatureFloatSet(camera_handle_, name.data(), value);

  if (err != VmbErrorSuccess) {
    RCLCPP_ERROR(get_logger(), "%s failed with error %d", __FUNCTION__, err);
    return error{err};
  }

  return {};
}

result<feature_float_info> VimbaXCamera::feature_float_info_get(const std::string_view & name) const
{
  RCLCPP_INFO(get_logger(), "%s('%s')", __FUNCTION__, name.data());
  feature_float_info info{};

  auto err =
    api_->FeatureFloatRangeQuery(
    camera_handle_,
    name.data(),
    reinterpret_cast<double *>(&info.min),
    reinterpret_cast<double *>(&info.max));

  if (err != VmbErrorSuccess) {
    RCLCPP_ERROR(get_logger(), "%s failed with error %d", __FUNCTION__, err);
    return error{err};
  }

  err =
    api_->FeatureFloatIncrementQuery(
    camera_handle_,
    name.data(),
    &info.inc_available,
    reinterpret_cast<double *>(&info.inc));

  if (err != VmbErrorSuccess) {
    RCLCPP_ERROR(get_logger(), "%s failed with error %d", __FUNCTION__, err);
    return error{err};
  }

  return info;
}

result<std::string> VimbaXCamera::feature_string_get(const std::string_view & name) const
{
  return feature_string_get(name, camera_handle_);
}

result<std::string> VimbaXCamera::feature_string_get(
  const std::string_view & name, VmbHandle_t handle) const
{
  RCLCPP_INFO(get_logger(), "%s('%s')", __FUNCTION__, name.data());

  uint32_t size_filled{};
  std::string value;

  auto err = api_->FeatureStringGet(handle, name.data(), nullptr, 0, &size_filled);

  if (err != VmbErrorSuccess) {
    RCLCPP_ERROR(get_logger(), "%s failed with error %d", __FUNCTION__, err);
    return error{err};
  } else {
    char * buf = static_cast<char *>(malloc(size_filled));

    err = api_->FeatureStringGet(handle, name.data(), buf, size_filled, &size_filled);

    if (err == VmbErrorSuccess) {
      value.assign(buf, size_filled);
    }

    free(buf);
    buf = nullptr;

    if (err != VmbErrorSuccess) {
      RCLCPP_ERROR(get_logger(), "%s failed with error %d", __FUNCTION__, err);
      return error{err};
    }
  }

  return value;
}

result<void>
VimbaXCamera::feature_string_set(const std::string_view & name, const std::string_view value) const
{
  RCLCPP_INFO(get_logger(), "%s('%s', '%s')", __FUNCTION__, name.data(), value.data());
  auto const err =
    api_->FeatureStringSet(camera_handle_, name.data(), value.data());

  if (err != VmbErrorSuccess) {
    RCLCPP_ERROR(get_logger(), "%s failed with error %d", __FUNCTION__, err);
    return error{err};
  }

  return {};
}

result<uint32_t> VimbaXCamera::feature_string_info_get(const std::string_view & name) const
{
  RCLCPP_INFO(get_logger(), "%s('%s')", __FUNCTION__, name.data());
  uint32_t value{};

  auto err =
    api_->FeatureStringMaxlengthQuery(
    camera_handle_,
    name.data(),
    reinterpret_cast<VmbUint32_t *>(&value));

  if (err != VmbErrorSuccess) {
    RCLCPP_ERROR(get_logger(), "%s failed with error %d", __FUNCTION__, err);
    return error{err};
  }

  return value;
}

result<bool> VimbaXCamera::feature_bool_get(const std::string_view & name) const
{
  RCLCPP_INFO(get_logger(), "%s('%s')", __FUNCTION__, name.data());

  bool value{};
  auto const err =
    api_->FeatureBoolGet(camera_handle_, name.data(), reinterpret_cast<bool *>(&value));

  if (err != VmbErrorSuccess) {
    RCLCPP_ERROR(get_logger(), "%s failed with error %d", __FUNCTION__, err);
    return error{err};
  }

  return value;
}

result<void> VimbaXCamera::feature_bool_set(const std::string_view & name, const bool value) const
{
  RCLCPP_INFO(get_logger(), "%s('%s', %d)", __FUNCTION__, name.data(), value);
  auto const err =
    api_->FeatureBoolSet(camera_handle_, name.data(), value);

  if (err != VmbErrorSuccess) {
    RCLCPP_ERROR(get_logger(), "%s failed with error %d", __FUNCTION__, err);
    return error{err};
  }

  return {};
}

result<std::string> VimbaXCamera::feature_enum_get(const std::string_view & name) const
{
  return feature_enum_get(name, camera_handle_);
}

result<std::string> VimbaXCamera::feature_enum_get(
  const std::string_view & name,
  VmbHandle_t handle) const
{
  RCLCPP_INFO(get_logger(), "%s('%s')", __FUNCTION__, name.data());

  const char * value{nullptr};
  auto const err = api_->FeatureEnumGet(handle, name.data(), &value);

  if (err != VmbErrorSuccess) {
    RCLCPP_ERROR(get_logger(), "%s failed with error %d", __FUNCTION__, err);
    return error{err};
  }

  return std::string{value};
}

result<void>
VimbaXCamera::feature_enum_set(const std::string_view & name, const std::string_view & value) const
{
  RCLCPP_INFO(get_logger(), "%s('%s', '%s')", __FUNCTION__, name.data(), value.data());
  auto const err =
    api_->FeatureEnumSet(camera_handle_, name.data(), value.data());

  if (err != VmbErrorSuccess) {
    RCLCPP_ERROR(get_logger(), "%s failed with error %d", __FUNCTION__, err);
    return error{err};
  }

  return {};
}

result<std::array<std::vector<std::string>, 2>>
VimbaXCamera::feature_enum_info_get(const std::string_view & name) const
{
  RCLCPP_INFO(get_logger(), "%s('%s')", __FUNCTION__, name.data());
  uint32_t numFound{};
  bool available{false};
  std::array<std::vector<std::string>, 2> values;  // 0: possibleValues, 1: availableValues

  auto err =
    api_->FeatureEnumRangeQuery(camera_handle_, name.data(), nullptr, 0, &numFound);

  if (err != VmbErrorSuccess) {
    RCLCPP_ERROR(get_logger(), "%s failed with error %d", __FUNCTION__, err);
    return error{err};
  }

  std::vector<const char *> enumEntries(numFound);

  err =
    api_->FeatureEnumRangeQuery(camera_handle_, name.data(), &enumEntries[0], numFound, &numFound);

  if (err != VmbErrorSuccess) {
    RCLCPP_ERROR(get_logger(), "%s failed with error %d", __FUNCTION__, err);
    return error{err};
  }

  for (auto const & entry : enumEntries) {
    values[0].push_back(std::string(entry));

    err =
      api_->FeatureEnumIsAvailable(camera_handle_, name.data(), entry, &available);

    if (err == VmbErrorSuccess && available) {
      values[1].push_back(std::string(entry));
    }
  }

  return values;
}

result<int64_t> VimbaXCamera::feature_enum_as_int_get(
  const std::string_view & name,
  const std::string_view & option) const
{
  RCLCPP_INFO(get_logger(), "%s('%s', '%s')", __FUNCTION__, name.data(), option.data());
  int64_t value{-1};

  auto const err = api_->FeatureEnumAsInt(
    camera_handle_, name.data(), option.data(),
    reinterpret_cast<VmbInt64_t *>(&value));

  if (err != VmbErrorSuccess) {
    RCLCPP_ERROR(
      get_logger(), "%s failed to convert enum %s option %s to int with error %d",
      __FUNCTION__, name.data(), option.data(), err);

    return error{err};
  }

  return value;
}

result<std::string>
VimbaXCamera::feature_enum_as_string_get(const std::string_view & name, const int64_t value) const
{
  RCLCPP_INFO(get_logger(), "%s('%s', %ld)", __FUNCTION__, name.data(), value);
  std::string option;
  const char * stringValue;

  auto const err = api_->FeatureEnumAsString(
    camera_handle_, name.data(), value,
    reinterpret_cast<const char **>(&stringValue));

  if (err != VmbErrorSuccess) {
    RCLCPP_ERROR(
      get_logger(), "Failed to convert enum %s option %s to int with %d",
      name.data(), option.data(), err);

    return error{err};
  } else {
    option.assign(stringValue);
  }

  return option;
}

result<std::vector<unsigned char>>
VimbaXCamera::feature_raw_get(const std::string_view & name) const
{
  RCLCPP_INFO(get_logger(), "%s('%s')", __FUNCTION__, name.data());

  uint32_t length{};
  uint32_t size_filled{};

  auto err =
    api_->FeatureRawLengthQuery(camera_handle_, name.data(), &length);

  if (err != VmbErrorSuccess) {
    RCLCPP_ERROR(get_logger(), "%s failed with error %d", __FUNCTION__, err);
    return error{err};
  }

  std::vector<unsigned char> buffer(length);
  err =
    api_->FeatureRawGet(
    camera_handle_,
    name.data(),
    reinterpret_cast<char *>(&buffer[0]),
    length,
    &size_filled);

  if (err != VmbErrorSuccess) {
    RCLCPP_ERROR(get_logger(), "%s failed with error %d", __FUNCTION__, err);
    return error{err};
  }

  return buffer;
}

result<void> VimbaXCamera::feature_raw_set(
  const std::string_view & name,
  const std::vector<uint8_t> buffer) const
{
  RCLCPP_INFO(
    get_logger(), "%s('%s', buffer.size()=%ld)", __FUNCTION__, name.data(), buffer.size());

  auto const err =
    api_->FeatureRawSet(
    camera_handle_,
    name.data(),
    reinterpret_cast<const char *>(buffer.data()),
    static_cast<uint32_t>(buffer.size()));

  if (err != VmbErrorSuccess) {
    RCLCPP_ERROR(get_logger(), "%s failed with error %d", __FUNCTION__, err);
    return error{err};
  }

  return {};
}

result<uint32_t> VimbaXCamera::feature_raw_info_get(const std::string_view & name) const
{
  RCLCPP_INFO(get_logger(), "%s('%s')", __FUNCTION__, name.data());

  uint32_t value{};

  auto err =
    api_->FeatureRawLengthQuery(camera_handle_, name.data(), &value);

  if (err != VmbErrorSuccess) {
    RCLCPP_ERROR(get_logger(), "%s failed with error %d", __FUNCTION__, err);
    return error{err};
  }

  return value;
}

result<std::array<bool, 2>>
VimbaXCamera::feature_access_mode_get(const std::string_view & name) const
{
  RCLCPP_INFO(get_logger(), "%s('%s')", __FUNCTION__, name.data());

  std::array<bool, 2> value;

  auto const err =
    api_->FeatureAccessQuery(camera_handle_, name.data(), &value[0], &value[1]);

  if (err != VmbErrorSuccess) {
    RCLCPP_ERROR(get_logger(), "%s failed with error %d", __FUNCTION__, err);
    return error{err};
  }

  return value;
}

result<std::vector<feature_info>>
VimbaXCamera::feature_info_query_list(const std::vector<std::string> & names) const
{
  std::vector<feature_info> infos;

  for (auto name : names) {
    VmbFeatureInfo featureInfo{};
    feature_info info{};

    auto const err =
      api_->FeatureInfoQuery(camera_handle_, name.data(), &featureInfo, sizeof(featureInfo));

    if (err != VmbErrorSuccess) {
      RCLCPP_ERROR(get_logger(), "Reading feature info for '%s' failed with %d", name.data(), err);
      return error{err};
    }
    info.name = std::string(featureInfo.name ? featureInfo.name : "");
    info.category = std::string(featureInfo.category ? featureInfo.category : "");
    info.display_name = std::string(featureInfo.displayName ? featureInfo.displayName : "");
    info.sfnc_namespace = std::string(featureInfo.sfncNamespace ? featureInfo.sfncNamespace : "");
    info.unit = std::string(featureInfo.unit ? featureInfo.unit : "");
    info.data_type = static_cast<uint32_t>(featureInfo.featureDataType);
    info.flags.flag_none = featureInfo.featureDataType == 0;
    info.flags.flag_read =
      (featureInfo.featureDataType & VmbFeatureFlagsRead) == VmbFeatureFlagsRead;
    info.flags.flag_write = (
      featureInfo.featureDataType & VmbFeatureFlagsWrite) == VmbFeatureFlagsWrite;
    info.flags.flag_volatile =
      (featureInfo.featureDataType & VmbFeatureFlagsVolatile) == VmbFeatureFlagsVolatile;
    info.flags.flag_modify_write =
      (featureInfo.featureDataType & VmbFeatureFlagsModifyWrite) == VmbFeatureFlagsModifyWrite;
    info.polling_time = featureInfo.pollingTime;

    infos.push_back(info);
  }

  return infos;
}

result<VmbPixelFormatType> VimbaXCamera::get_pixel_format() const
{
  auto const featureInfoOpt = feature_info_query(SFNCFeatures::PixelFormat);

  if (!featureInfoOpt) {
    return featureInfoOpt.error();
  } else if (featureInfoOpt->sfncNamespace == nullptr) {
    return error{VmbErrorInvalidAddress};
  } else if (std::string_view{featureInfoOpt->sfncNamespace} != "Standard") {
    return error{VmbErrorNotAvailable};
  }

  auto const currentFormatStr = feature_enum_get(SFNCFeatures::PixelFormat);

  if (!currentFormatStr) {
    return currentFormatStr.error();
  }

  auto const currentFormat = feature_enum_as_int_get(SFNCFeatures::PixelFormat, *currentFormatStr);

  if (!currentFormat) {
    return currentFormat.error();
  }

  return static_cast<VmbPixelFormatType>(*currentFormat);
}

result<VmbFeatureInfo> VimbaXCamera::feature_info_query(const std::string_view & name) const
{
  VmbFeatureInfo featureInfo{};

  auto const err =
    api_->FeatureInfoQuery(camera_handle_, name.data(), &featureInfo, sizeof(featureInfo));

  if (err != VmbErrorSuccess) {
    RCLCPP_ERROR(get_logger(), "Reading feature info for '%s' failed with %d", name.data(), err);
    return error{err};
  }

  return featureInfo;
}

result<void> VimbaXCamera::settings_load(const std::string_view & fileName)
{
  fs::path settings_file_path{fileName};

  if (!fs::exists(settings_file_path)) {
    return error{VmbErrorNotFound};
  }

  auto const presist_settings = get_default_feature_persist_settings();

  auto const err = api_->SettingsLoad(
    camera_handle_,
    settings_file_path.c_str(),
    &presist_settings,
    sizeof(presist_settings));

  if (err != VmbErrorSuccess) {
    return error{err};
  }

  return {};
}

result<void> VimbaXCamera::settings_save(const std::string_view & fileName)
{
  fs::path settings_file_path{fileName};

  if (settings_file_path.extension() != ".xml") {
    return error{VmbErrorInvalidValue};
  }

  if (!fs::exists(settings_file_path.parent_path())) {
    return error{VmbErrorNotFound};
  }

  auto const presist_settings = get_default_feature_persist_settings();

  auto const err = api_->SettingsSave(
    camera_handle_,
    settings_file_path.c_str(),
    &presist_settings,
    sizeof(presist_settings));

  if (err != VmbErrorSuccess) {
    return error{err};
  }

  return {};
}

VmbFeaturePersistSettings VimbaXCamera::get_default_feature_persist_settings() const
{
  return {
    VmbFeaturePersistType::VmbFeaturePersistNoLUT,
    VmbModulePersistFlagsType::VmbModulePersistFlagsRemoteDevice,
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

  info.streaming = streaming_;

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


  auto const trigger_mode = feature_enum_get(SFNCFeatures::TriggerMode);
  if (!trigger_mode) {
    return trigger_mode.error();
  } else {
    info.trigger_mode = *trigger_mode;
  }

  auto const trigger_source = feature_enum_get(SFNCFeatures::TriggerSource);
  if (!trigger_source) {
    if (trigger_source.error().code != VmbErrorNotAvailable) {
      return trigger_source.error();
    }
  } else {
    info.trigger_source = *trigger_source;
  }

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


  return info;
}

bool VimbaXCamera::is_streaming() const
{
  return streaming_;
}

result<std::shared_ptr<VimbaXCamera::Frame>> VimbaXCamera::Frame::create(
  std::shared_ptr<VimbaXCamera> camera,
  size_t size,
  size_t alignment)
{
  auto const pixelFormat = camera->get_pixel_format();

  if (!pixelFormat) {
    return pixelFormat.error();
  }

  uint32_t const bpp = (*pixelFormat >> 16) & 0xFF;

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

  auto const allocMode =
    (real_size == aligned_size) ? AllocationMode::kByImage : AllocationMode::kByTl;

  std::shared_ptr<VimbaXCamera::Frame> frame(new VimbaXCamera::Frame{camera, allocMode});

  if (allocMode == AllocationMode::kByTl) {
    frame->data.resize(real_size);

    frame->vmb_frame_.buffer = nullptr;
    frame->vmb_frame_.bufferSize = size;
  } else {
    frame->data.resize(size);

    frame->vmb_frame_.buffer = frame->data.data();
    frame->vmb_frame_.bufferSize = frame->data.size();
  }

  frame->step = line;

  auto announceError =
    camera->api_->FrameAnnounce(camera->camera_handle_, &frame->vmb_frame_, sizeof(vmb_frame_));

  if (announceError != VmbErrorSuccess) {
    return error{announceError};
  }

  return frame;
}

void VimbaXCamera::Frame::vmb_frame_callback(
  const VmbHandle_t, const VmbHandle_t, VmbFrame_t * frame)
{
  auto * ptr = reinterpret_cast<VimbaXCamera::Frame *>(frame->context[0]);
  auto sharedPtr = ptr->shared_from_this();

  if (frame->receiveStatus == VmbFrameStatusType::VmbFrameStatusComplete) {
    sharedPtr->on_frame_ready();
  } else {
    RCLCPP_WARN(get_logger(), "Frame with status %d received", frame->receiveStatus);
    sharedPtr->queue();
  }
}

void VimbaXCamera::Frame::on_frame_ready()
{
  encoding = get_image_encoding();
  width = vmb_frame_.width;
  height = vmb_frame_.height;
  is_bigendian = false;
  header.frame_id = std::to_string(vmb_frame_.frameID);
  std::chrono::nanoseconds vmbTimeStamp{timestamp_to_ns(vmb_frame_.timestamp)};
  auto const seconds = std::chrono::floor<std::chrono::seconds>(vmbTimeStamp);
  auto const nanoseconds =
    std::chrono::duration_cast<std::chrono::nanoseconds>(vmbTimeStamp - seconds);
  header.stamp.sec = int32_t(seconds.count());
  header.stamp.nanosec = nanoseconds.count();


  transform();

  if (callback_) {
    callback_(shared_from_this());
  }
}

uint64_t VimbaXCamera::Frame::timestamp_to_ns(uint64_t timestamp)
{
  if (!camera_.expired()) {
    auto camera = camera_.lock();


    if (camera->timestamp_frequency_) {
      RCLCPP_DEBUG(get_logger(), "Using timestamp frequnency %ld", *camera->timestamp_frequency_);

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

VimbaXCamera::Frame::Frame(std::shared_ptr<VimbaXCamera> camera, AllocationMode allocationMode)
: camera_{camera}, allocation_mode_{allocationMode}
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

}  // namespace vimbax_camera
