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

#include <rclcpp/rclcpp.hpp>

#include <vimbax_camera/vimbax_camera.hpp>
#include <vimbax_camera_msgs/srv/feature_int.hpp>

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
  auto const cameraInfo = query_camera_info();
  if (cameraInfo) {
    RCLCPP_INFO(
      get_logger(), "Opened camera info model name: %s, camera name: %s, serial: %s",
      cameraInfo->modelName, cameraInfo->cameraName, cameraInfo->serialString);
  }
}

VimbaXCamera::~VimbaXCamera()
{
  stop_streaming();

  if (api_ && camera_handle_) {
    api_->CameraClose(camera_handle_);
    camera_handle_ = nullptr;
  }
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

    for (auto & frame : frames_) {
      auto newFrame = Frame::create(shared_from_this(), payloadSize, 128);

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

  auto const acqStopError = feature_command_run(SFNCFeatures::AcquisitionStop);
  if (!acqStopError) {
    RCLCPP_ERROR(get_logger(), "Acquisition stop failed with %d", acqStopError.error().code);
    return acqStopError.error();
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
  auto const runError = api_->FeatureCommandRun(camera_handle_, name.data());

  if (runError != VmbErrorSuccess) {
    return error{runError};
  }

  bool done{false};
  api_->FeatureCommandIsDone(camera_handle_, name.data(), &done);

  while (!done) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    api_->FeatureCommandIsDone(camera_handle_, name.data(), &done);
  }

  return {};
}

result<int64_t> VimbaXCamera::feature_int_get(const std::string_view & name) const
{
  RCLCPP_INFO(get_logger(), "%s(%s)", __FUNCTION__, name.data());
  int64_t value{};
  auto const err =
    api_->FeatureIntGet(camera_handle_, name.data(), reinterpret_cast<VmbInt64_t *>(&value));

  if (err != VmbErrorSuccess) {
    RCLCPP_ERROR(get_logger(), "%s failed with error %d", __FUNCTION__, err);
    return error{err};
  }

  return value;
}

result<void> VimbaXCamera::feature_int_set(const std::string_view & name, const int64_t value) const
{
  RCLCPP_INFO(get_logger(), "%s(%s, %ld)", __FUNCTION__, name.data(), value);
  auto const err =
    api_->FeatureIntSet(camera_handle_, name.data(), value);

  if (err != VmbErrorSuccess) {
    RCLCPP_ERROR(get_logger(), "%s failed with error %d", __FUNCTION__, err);
    return error{err};
  }

  return {};
}

result<_Float64> VimbaXCamera::feature_float_get(const std::string_view & name) const
{
  RCLCPP_INFO(get_logger(), "%s(%s)", __FUNCTION__, name.data());

  _Float64 value{};
  auto const err =
    api_->FeatureFloatGet(camera_handle_, name.data(), reinterpret_cast<_Float64 *>(&value));

  if (err != VmbErrorSuccess) {
    RCLCPP_ERROR(get_logger(), "%s failed with error %d", __FUNCTION__, err);
    return error{err};
  }

  return value;
}

result<void> VimbaXCamera::feature_float_set(const std::string_view & name, const _Float64 value) const
{
  RCLCPP_INFO(get_logger(), "%s(%s, %lf)", __FUNCTION__, name.data(), value);
  auto const err =
    api_->FeatureFloatSet(camera_handle_, name.data(), value);

  if (err != VmbErrorSuccess) {
    RCLCPP_ERROR(get_logger(), "%s failed with error %d", __FUNCTION__, err);
    return error{err};
  }

  return {};
}

result<std::string> VimbaXCamera::feature_string_get(const std::string_view & name) const
{
  RCLCPP_INFO(get_logger(), "%s(%s)", __FUNCTION__, name.data());

  uint32_t size_filled{};
  std::string value;

  auto err =
    api_->FeatureStringGet(camera_handle_, name.data(), nullptr, 0, &size_filled);

  if (err != VmbErrorSuccess) {
    RCLCPP_ERROR(get_logger(), "%s failed with error %d", __FUNCTION__, err);
    return error{err};
  }
  else
  {
    char* buf = static_cast<char*>(malloc(size_filled));

    err =
      api_->FeatureStringGet(camera_handle_, name.data(), buf, size_filled, &size_filled);

    if (err == VmbErrorSuccess)
    {
      value.assign(buf, size_filled);;
    }

    free(buf);
    buf = nullptr;

    if (err != VmbErrorSuccess)
    {
      RCLCPP_ERROR(get_logger(), "%s failed with error %d", __FUNCTION__, err);
      return error{err};
    }
  }

  return value;
}

result<void> VimbaXCamera::feature_string_set(const std::string_view & name, const std::string_view value) const
{
  RCLCPP_INFO(get_logger(), "%s(%s, %s)", __FUNCTION__, name.data(), value.data());
  auto const err =
    api_->FeatureStringSet(camera_handle_, name.data(), value.data());

  if (err != VmbErrorSuccess) {
    RCLCPP_ERROR(get_logger(), "%s failed with error %d", __FUNCTION__, err);
    return error{err};
  }

  return {};
}

result<bool> VimbaXCamera::feature_bool_get(const std::string_view & name) const
{
  RCLCPP_INFO(get_logger(), "%s(%s)", __FUNCTION__, name.data());

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
  RCLCPP_INFO(get_logger(), "%s(%s, %d)", __FUNCTION__, name.data(), value);
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
  RCLCPP_INFO(get_logger(), "%s(%s)", __FUNCTION__, name.data());

  const char * value;
  auto const err = api_->FeatureEnumGet(camera_handle_, name.data(), &value);

  if (err != VmbErrorSuccess) {
    RCLCPP_ERROR(get_logger(), "%s failed with error %d", __FUNCTION__, err);
    return error{err};
  }

  return std::string{value};
}

result<void> VimbaXCamera::feature_enum_set(const std::string_view & name, const std::string_view & value) const
{
  RCLCPP_INFO(get_logger(), "%s(%s, %s)", __FUNCTION__, name.data(), value.data());
  auto const err =
    api_->FeatureEnumSet(camera_handle_, name.data(), value.data());

  if (err != VmbErrorSuccess) {
    RCLCPP_ERROR(get_logger(), "%s failed with error %d", __FUNCTION__, err);
    return error{err};
  }

  return {};
}

result<int64_t> VimbaXCamera::feature_enum_as_int_get(
  const std::string_view & name,
  const std::string_view & option) const
{
  RCLCPP_INFO(get_logger(), "%s(%s, %s)", __FUNCTION__, name.data(), option.data());
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

result<std::string> VimbaXCamera::feature_enum_as_string_get(const std::string_view & name, const int64_t value) const
{
  RCLCPP_INFO(get_logger(), "%s(%s, %ld)", __FUNCTION__, name.data(), value);
  std::string option;
  const char *stringValue;

  auto const err = api_->FeatureEnumAsString(
    camera_handle_, name.data(), value,
    reinterpret_cast<const char **>(&stringValue));

  if (err != VmbErrorSuccess) {
    RCLCPP_ERROR(
      get_logger(), "Failed to convert enum %s option %s to int with %d",
      name.data(), option.data(), err);

    return error{err};
  }
  else
  {
    option.assign(stringValue);
  }

  return option;
}

result<std::vector<unsigned char>> VimbaXCamera::feature_raw_get(const std::string_view & name) const
{
  RCLCPP_INFO(get_logger(), "%s(%s)", __FUNCTION__, name.data());
 
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
    api_->FeatureRawGet(camera_handle_, name.data(), reinterpret_cast<char*>(&buffer[0]), length, &size_filled);

  if (err != VmbErrorSuccess)
  {
    RCLCPP_ERROR(get_logger(), "%s failed with error %d", __FUNCTION__, err);
    return error{err};
  }

  return buffer;
}

result<void> VimbaXCamera::feature_raw_set(const std::string_view & name, const std::vector<unsigned char> buffer) const
{
  RCLCPP_INFO(get_logger(), "%s(%s, buffer.size()=%ld)", __FUNCTION__, name.data(), buffer.size());
 
  auto const err =
    api_->FeatureRawSet(camera_handle_, name.data(), reinterpret_cast<const char*>(buffer.data()), static_cast<uint32_t>(buffer.size()));

  if (err != VmbErrorSuccess) {
    RCLCPP_ERROR(get_logger(), "%s failed with error %d", __FUNCTION__, err);
    return error{err};
  }

  return {};
}

result<std::array<bool,2>> VimbaXCamera::feature_access_mode_get(const std::string_view & name) const
{
  RCLCPP_INFO(get_logger(), "%s(%s)", __FUNCTION__, name.data());
 
  std::array<bool,2> value;
  
  auto const err =
    api_->FeatureAccessQuery(camera_handle_, name.data(), &value[0], &value[1]);

  if (err != VmbErrorSuccess) {
    RCLCPP_ERROR(get_logger(), "%s failed with error %d", __FUNCTION__, err);
    return error{err};
  }

  return value;
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
    RCLCPP_ERROR(get_logger(), "Reading feature info for %s failed with %d", name.data(), err);
    return error{err};
  }

  return featureInfo;
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
  size_t const realSize = *height * line;

  auto const allocMode = (realSize == size) ? AllocationMode::kByImage : AllocationMode::kByTl;

  std::shared_ptr<VimbaXCamera::Frame> frame(new VimbaXCamera::Frame{camera, allocMode});

  if (allocMode == AllocationMode::kByTl) {
    frame->data.resize(realSize);

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
  std::chrono::microseconds vmbTimeStamp{vmb_frame_.timestamp};
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