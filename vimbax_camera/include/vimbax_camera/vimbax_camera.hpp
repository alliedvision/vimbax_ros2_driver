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

#ifndef VIMBAX_CAMERA__VIMBAX_CAMERA_HPP_
#define VIMBAX_CAMERA__VIMBAX_CAMERA_HPP_

#include <string>
#include <memory>
#include <functional>
#include <optional>
#include <vector>

#include <rclcpp/logger.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <vimbax_camera/result.hpp>
#include <vimbax_camera/loader/vmbc_api.hpp>
#include <vimbax_camera/vimbax_camera_helper.hpp>


namespace vimbax_camera
{
class VimbaXCamera : public std::enable_shared_from_this<VimbaXCamera>
{
public:
  class Frame : public sensor_msgs::msg::Image, public std::enable_shared_from_this<Frame>
  {
    /* *INDENT-OFF* */
  public:
    /* *INDENT-ON* */
    static result<std::shared_ptr<Frame>> create(
      std::shared_ptr<VimbaXCamera> camera, size_t size, size_t alignment = 1);

    ~Frame();

    Frame(const Frame &) = delete;
    Frame & operator=(const Frame &) = delete;

    void set_callback(std::function<void(std::shared_ptr<Frame>)> callback);

    int32_t queue() const;

    std::string get_image_encoding() const;

    int64_t get_frame_id() const;
    /* *INDENT-OFF* */
  private:
    /* *INDENT-ON* */
    enum class AllocationMode
    {
      kByImage,
      kByTl,
    };

    static void vmb_frame_callback(const VmbHandle_t, const VmbHandle_t, VmbFrame_t * frame);
    void on_frame_ready();
    void transform();

    Frame(std::shared_ptr<VimbaXCamera> camera, AllocationMode allocationMode);

    std::function<void(std::shared_ptr<Frame>)> callback_;
    std::weak_ptr<VimbaXCamera> camera_;
    VmbFrame vmb_frame_;

    AllocationMode allocation_mode_;
  };

  struct SFNCFeatures
  {
    static constexpr std::string_view PixelFormat = "PixelFormat";
    static constexpr std::string_view AcquisitionStart = "AcquisitionStart";
    static constexpr std::string_view AcquisitionStop = "AcquisitionStop";
    static constexpr std::string_view Width = "Width";
    static constexpr std::string_view Height = "Height";
  };

  static std::shared_ptr<VimbaXCamera> open(
    std::shared_ptr<VmbCAPI> api,
    const std::string & name = {});

  ~VimbaXCamera();

  VimbaXCamera(const VimbaXCamera &) = delete;
  VimbaXCamera & operator=(const VimbaXCamera &) = delete;

  result<void> start_streaming(
    int bufferCount,
    std::function<void(std::shared_ptr<Frame>)> onFrame,
    bool startAcquisition = true);
  result<void> stop_streaming();

  result<VmbCameraInfo> query_camera_info() const;

  // Feature access
  result<bool> feature_command_is_done(const std::string_view & name) const;
  result<void> feature_command_run(const std::string_view & name) const;
  result<int64_t> feature_int_get(const std::string_view & name) const;
  result<void> feature_int_set(const std::string_view & name, const int64_t value) const;
  result<_Float64> feature_float_get(const std::string_view & name) const;
  result<void> feature_float_set(const std::string_view & name, const _Float64 value) const;
  result<std::string> feature_string_get(const std::string_view & name) const;
  result<void> feature_string_set(const std::string_view & name, const std::string_view value) const;
  result<bool> feature_bool_get(const std::string_view & name) const;
  result<void> feature_bool_set(const std::string_view & name, const bool value) const;
  result<std::string> feature_enum_get(const std::string_view & name) const;
  result<void> feature_enum_set(const std::string_view & name, const std::string_view & value) const;
  result<int64_t> feature_enum_as_int_get(const std::string_view & name, const std::string_view & option) const;
  result<std::string> feature_enum_as_string_get(const std::string_view & name, const int64_t value) const;

  result<VmbPixelFormatType> get_pixel_format() const;

  result<VmbFeatureInfo> feature_info_query(const std::string_view & name) const;

  bool is_streaming() const;

private:
  explicit VimbaXCamera(std::shared_ptr<VmbCAPI> api, VmbHandle_t cameraHandle);

  std::shared_ptr<VmbCAPI> api_;
  VmbHandle_t camera_handle_;
  std::vector<std::shared_ptr<Frame>> frames_;
  bool streaming_{false};
};

}  // namespace vimbax_camera

#endif  // VIMBAX_CAMERA__VIMBAX_CAMERA_HPP_
