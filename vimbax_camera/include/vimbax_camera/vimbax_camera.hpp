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

#ifndef VIMBAX_CAMERA__VIMBAX_CAMERA_HPP_
#define VIMBAX_CAMERA__VIMBAX_CAMERA_HPP_

#include <string>
#include <memory>
#include <functional>
#include <optional>
#include <vector>
#include <queue>
#include <utility>
#include <unordered_map>

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

    uint64_t get_timestamp_ns() const;

    void on_frame_ready();
    /* *INDENT-OFF* */
  private:
    /* *INDENT-ON* */
    enum class AllocationMode
    {
      kByImage,
      kByTl,
    };

    static void vmb_frame_callback(const VmbHandle_t, const VmbHandle_t, VmbFrame_t * frame);

    void transform();
    uint64_t timestamp_to_ns(uint64_t timestamp) const;

    Frame(std::shared_ptr<VimbaXCamera> camera, AllocationMode allocation_mode);

    std::function<void(std::shared_ptr<Frame>)> callback_;
    std::weak_ptr<VimbaXCamera> camera_;
    VmbFrame vmb_frame_;

    AllocationMode allocation_mode_;
  };


  enum class Module : std::size_t
  {
    System,
    Interface,
    LocalDevice,
    RemoteDevice,
    Stream,
    ModuleMax
  };

  enum class StreamState
  {
    kStopped,
    kStarting,
    kActive,
    kStopping,
  };

  struct TriggerInfo
  {
    TriggerInfo(std::string selector, std::string mode, std::string source)
    : selector(selector), mode(mode), source(source) {}

    std::string selector;
    std::string mode;
    std::string source;
  };

  struct Info
  {
    std::string display_name;
    std::string model_name;
    std::string firmware_version;
    std::string device_id;
    std::string device_user_id;
    std::string device_serial_number;
    std::string interface_id;
    std::string transport_layer_id;
    std::optional<std::string> ip_address{std::nullopt};
    std::optional<std::string> mac_address{std::nullopt};
    bool streaming;
    uint32_t width;
    uint32_t height;
    double frame_rate;
    std::string pixel_format;
    std::vector<TriggerInfo> trigger_info;
  };

  static std::shared_ptr<VimbaXCamera> open(
    std::shared_ptr<VmbCAPI> api,
    const std::string & name = {});

  ~VimbaXCamera();

  VimbaXCamera(const VimbaXCamera &) = delete;
  VimbaXCamera & operator=(const VimbaXCamera &) = delete;

  result<void> start_streaming(
    int buffer_count,
    std::function<void(std::shared_ptr<Frame>)> on_frame,
    bool start_acquisition = true);
  result<void> stop_streaming();

  bool is_alive() const;
  bool has_feature(const std::string_view & name, const Module module = Module::RemoteDevice) const;

  // Feature access
  result<std::vector<std::string>> features_list_get(
    const Module module = Module::RemoteDevice) const;

  result<bool> feature_command_is_done(
    const std::string_view & name,
    const Module module = Module::RemoteDevice) const;
  result<void> feature_command_run(
    const std::string_view & name,
    const std::optional<std::chrono::milliseconds> & timeout = std::nullopt,
    const Module module = Module::RemoteDevice) const;

  result<int64_t> feature_int_get(
    const std::string_view & name,
    const Module module = Module::RemoteDevice) const;
  result<void> feature_int_set(
    const std::string_view & name,
    const int64_t value,
    const Module module = Module::RemoteDevice) const;
  result<std::array<int64_t, 3>> feature_int_info_get(
    const std::string_view & name,
    const Module module = Module::RemoteDevice) const;

  result<_Float64> feature_float_get(
    const std::string_view & name,
    const Module module = Module::RemoteDevice) const;
  result<void> feature_float_set(
    const std::string_view & name,
    const _Float64 value,
    const Module module = Module::RemoteDevice) const;
  result<feature_float_info> feature_float_info_get(
    const std::string_view & name,
    const Module module = Module::RemoteDevice) const;

  result<std::string> feature_string_get(
    const std::string_view & name,
    const Module module = Module::RemoteDevice) const;
  result<void> feature_string_set(
    const std::string_view & name,
    const std::string_view value,
    const Module module = Module::RemoteDevice) const;
  result<uint32_t> feature_string_info_get(
    const std::string_view & name,
    const Module module = Module::RemoteDevice) const;

  result<bool> feature_bool_get(
    const std::string_view & name,
    const Module module = Module::RemoteDevice) const;
  result<void> feature_bool_set(
    const std::string_view & name,
    const bool value,
    const Module module = Module::RemoteDevice) const;

  result<std::string> feature_enum_get(
    const std::string_view & name,
    const Module module = Module::RemoteDevice) const;
  result<void> feature_enum_set(
    const std::string_view & name,
    const std::string_view & value,
    const Module module = Module::RemoteDevice) const;
  result<std::array<std::vector<std::string>, 2>> feature_enum_info_get(
    const std::string_view & name,
    const Module module = Module::RemoteDevice) const;
  result<int64_t> feature_enum_as_int_get(
    const std::string_view & name,
    const std::string_view & option,
    const Module module = Module::RemoteDevice) const;
  result<std::string> feature_enum_as_string_get(
    const std::string_view & name,
    const int64_t value,
    const Module module = Module::RemoteDevice) const;

  result<std::vector<unsigned char>> feature_raw_get(
    const std::string_view & name,
    const Module module = Module::RemoteDevice) const;
  result<void> feature_raw_set(
    const std::string_view & name,
    const std::vector<uint8_t> buffer,
    const Module module = Module::RemoteDevice) const;
  result<uint32_t> feature_raw_info_get(
    const std::string_view & name,
    const Module module = Module::RemoteDevice) const;

  result<std::array<bool, 2>> feature_access_mode_get(
    const std::string_view & name,
    const Module module = Module::RemoteDevice) const;

  result<std::vector<feature_info>> feature_info_query_list(
    const std::vector<std::string> & names,
    const Module module = Module::RemoteDevice) const;


  result<VmbPixelFormatType> get_pixel_format() const;

  result<VmbCameraInfo> query_camera_info() const;

  result<VmbFeatureInfo> feature_info_query(
    const std::string_view & name,
    Module module = Module::RemoteDevice) const;

  result<void> settings_load(const std::string_view & file_name);

  result<void> settings_save(const std::string_view & file_name);

  result<Info> camera_info_get() const;

  bool is_streaming() const;

  result<void> feature_invalidation_register(
    const std::string_view & name,
    std::function<void(const std::string &)> callback);

  result<void> feature_invalidation_unregister(const std::string_view & name);

  using EventMetaDataList = std::vector<std::pair<std::string, std::string>>;

  result<EventMetaDataList> get_event_meta_data(const std::string_view & name);

private:
  explicit VimbaXCamera(std::shared_ptr<VmbCAPI> api, VmbHandle_t camera_handle);

  static void on_feature_invalidation(VmbHandle_t, const char * name, void * context);

  void initialize_feature_map(Module module);

  constexpr VmbHandle_t get_module_handle(Module module) const;

  VmbFeatureInfo get_feature_info(
    const std::string & name,
    Module module = Module::RemoteDevice) const;

  result<void> feature_command_run(
    const std::string_view & name, VmbHandle_t handle,
    const std::optional<std::chrono::milliseconds> & timeout = std::nullopt) const;

  result<int64_t> feature_int_get(const std::string_view & name, VmbHandle_t handle) const;

  result<_Float64> feature_float_get(const std::string_view & name, VmbHandle_t handle) const;

  result<std::string> feature_enum_get(const std::string_view & name, VmbHandle_t handle) const;

  result<std::string> feature_string_get(const std::string_view & name, VmbHandle_t handle) const;

  VmbFeaturePersistSettings get_default_feature_persist_settings() const;

  std::shared_ptr<VmbCAPI> api_;
  VmbHandle_t camera_handle_;
  std::vector<std::shared_ptr<Frame>> frames_;
  std::atomic<StreamState> stream_state_{StreamState::kStopped};
  bool is_valid_pixel_format(VmbPixelFormatType pixel_format);
  VmbCameraInfo camera_info_;
  std::optional<uint64_t> timestamp_frequency_;
  std::unordered_map<std::string, std::function<void(const std::string &)>> invalidation_callbacks_;

  std::mutex invalidation_callbacks_mutex_{};

  std::array<std::unordered_map<std::string, VmbFeatureInfo>,
    std::size_t(Module::ModuleMax)> feature_info_map_;
  std::array<std::unordered_multimap<std::string, std::string>,
    std::size_t(Module::ModuleMax)> feature_category_map_;

  mutable std::mutex frame_ready_queue_mutex_;
  std::condition_variable frame_ready_cv_;
  std::queue<std::shared_ptr<Frame>> frame_ready_queue_;
  std::shared_ptr<std::thread> frame_processing_thread_;
  std::atomic_bool frame_processing_enable_{false};
};

}  // namespace vimbax_camera

#endif  // VIMBAX_CAMERA__VIMBAX_CAMERA_HPP_
