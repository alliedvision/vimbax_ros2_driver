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

#include <filesystem>

#include <rclcpp/logging.hpp>

#include <vimbax_camera/vimbax_camera_helper.hpp>
#include <vimbax_camera/loader/vmbc_api.hpp>

#define str(s) #s
#define LOAD_FUNC(instance, name) \
  do { \
    auto const funcPtr = \
      reinterpret_cast<decltype(& Vmb ## name)>(library->resolve_symbol(str(Vmb ## name))); \
    if (funcPtr == nullptr) { \
      return {}; \
    } \
    instance->name = funcPtr; \
  } while (0)


namespace fs = std::filesystem;

namespace vimbax_camera
{

using helper::get_logger;
using helper::vmb_error_to_string;

std::weak_ptr<VmbCAPI> VmbCAPI::instance_{};

static std::string_view getenv_safe(const std::string & name)
{
  auto const val = getenv(name.c_str());

  return val == nullptr ? "" : val;
}

static std::vector<std::string_view> split_string(const std::string_view & str, const char c)
{
  if (str.empty()) {
    return {};
  }

  std::vector<std::string_view> parts{};
  std::size_t start = 0;
  std::size_t end;

  do {
    end = str.find(c, start);

    auto const part = str.substr(start, end);
    if (!part.empty()) {
      parts.push_back(part);
    }
    start = end + 1;
  } while (end != std::string::npos);

  return parts;
}

static std::unique_ptr<LoadedLibrary> load_vmbc_library(
  std::shared_ptr<LibraryLoader> library_loader)
{
  auto const lib_name = library_loader->build_library_name("VmbC");

  auto const vimbax_home = getenv_safe("VIMBA_X_HOME");

  auto const vimbax_home_parts = split_string(vimbax_home, ':');
  for (auto const & part : vimbax_home_parts) {
    fs::path const part_path{part};
    if (!fs::exists(part_path)) {
      RCLCPP_WARN(get_logger(), "VIMBA_X_HOME variable pointing to not existing directory");
      continue;
    }

    fs::path const vmbc_path = canonical(part_path / "api" / "lib") / lib_name;
    if (fs::exists(vmbc_path)) {
      RCLCPP_DEBUG(get_logger(), "Loading library %s by VimbaX home", vmbc_path.c_str());
      return library_loader->open(vmbc_path);
    }
  }

  auto const gentl_search_path = getenv_safe("GENICAM_GENTL64_PATH");

  auto const gentl_search_path_parts = split_string(gentl_search_path, ':');
  for (auto const & part : gentl_search_path_parts) {
    auto const lib_dir = fs::path{part} / ".." / "api" / "lib";
    if (!fs::exists(lib_dir)) {
      continue;
    }

    fs::path const vmbc_path = fs::canonical(lib_dir) / lib_name;
    if (fs::exists(vmbc_path)) {
      RCLCPP_DEBUG(get_logger(), "Loading library %s by TL search path", vmbc_path.c_str());
      return library_loader->open(vmbc_path);
    }
  }

  RCLCPP_DEBUG(get_logger(), "Loading library %s", lib_name.c_str());
  return library_loader->open(lib_name);
}

std::shared_ptr<VmbCAPI> VmbCAPI::get_instance(
  const std::string & tl_search_path,
  std::shared_ptr<LibraryLoader> library_loader)
{
  if (instance_.expired()) {
    if (!library_loader) {
      return {};
    }

    std::shared_ptr<VmbCAPI> instance{new VmbCAPI};

    auto library = load_vmbc_library(library_loader);

    if (!library) {
      RCLCPP_ERROR(get_logger(), "Failed to load VmbC library");
      return nullptr;
    }

    LOAD_FUNC(instance, CameraClose);
    LOAD_FUNC(instance, CameraInfoQuery);
    LOAD_FUNC(instance, CameraInfoQueryByHandle);
    LOAD_FUNC(instance, CameraOpen);
    LOAD_FUNC(instance, CamerasList);
    LOAD_FUNC(instance, CaptureEnd);
    LOAD_FUNC(instance, CaptureFrameQueue);
    LOAD_FUNC(instance, CaptureFrameWait);
    LOAD_FUNC(instance, CaptureQueueFlush);
    LOAD_FUNC(instance, CaptureStart);
    LOAD_FUNC(instance, ChunkDataAccess);
    LOAD_FUNC(instance, FeatureAccessQuery);
    LOAD_FUNC(instance, FeatureBoolGet);
    LOAD_FUNC(instance, FeatureBoolSet);
    LOAD_FUNC(instance, FeatureCommandIsDone);
    LOAD_FUNC(instance, FeatureCommandRun);
    LOAD_FUNC(instance, FeatureEnumAsInt);
    LOAD_FUNC(instance, FeatureEnumAsString);
    LOAD_FUNC(instance, FeatureEnumEntryGet);
    LOAD_FUNC(instance, FeatureEnumGet);
    LOAD_FUNC(instance, FeatureEnumIsAvailable);
    LOAD_FUNC(instance, FeatureEnumRangeQuery);
    LOAD_FUNC(instance, FeatureEnumSet);
    LOAD_FUNC(instance, FeatureFloatGet);
    LOAD_FUNC(instance, FeatureFloatIncrementQuery);
    LOAD_FUNC(instance, FeatureFloatRangeQuery);
    LOAD_FUNC(instance, FeatureFloatSet);
    LOAD_FUNC(instance, FeatureInfoQuery);
    LOAD_FUNC(instance, FeatureIntGet);
    LOAD_FUNC(instance, FeatureIntIncrementQuery);
    LOAD_FUNC(instance, FeatureIntRangeQuery);
    LOAD_FUNC(instance, FeatureIntSet);
    LOAD_FUNC(instance, FeatureIntValidValueSetQuery);
    LOAD_FUNC(instance, FeatureInvalidationRegister);
    LOAD_FUNC(instance, FeatureInvalidationUnregister);
    LOAD_FUNC(instance, FeatureListSelected);
    LOAD_FUNC(instance, FeatureRawGet);
    LOAD_FUNC(instance, FeatureRawLengthQuery);
    LOAD_FUNC(instance, FeatureRawSet);
    LOAD_FUNC(instance, FeatureStringGet);
    LOAD_FUNC(instance, FeatureStringMaxlengthQuery);
    LOAD_FUNC(instance, FeatureStringSet);
    LOAD_FUNC(instance, FeaturesList);
    LOAD_FUNC(instance, FrameAnnounce);
    LOAD_FUNC(instance, FrameRevoke);
    LOAD_FUNC(instance, FrameRevokeAll);
    LOAD_FUNC(instance, InterfacesList);
    LOAD_FUNC(instance, MemoryRead);
    LOAD_FUNC(instance, MemoryWrite);
    LOAD_FUNC(instance, PayloadSizeGet);
    LOAD_FUNC(instance, SettingsLoad);
    LOAD_FUNC(instance, SettingsSave);
    LOAD_FUNC(instance, Shutdown);
    LOAD_FUNC(instance, Startup);
    LOAD_FUNC(instance, TransportLayersList);
    LOAD_FUNC(instance, VersionQuery);

    instance->library_handle_ = std::move(library);

    auto const tl_search_path_ptr = tl_search_path.empty() ? nullptr : tl_search_path.c_str();
    auto const startup_res = instance->Startup(tl_search_path_ptr);
    if (startup_res != VmbErrorSuccess) {
      RCLCPP_ERROR(get_logger(), "VmbStartup failed with %d", startup_res);
      return {};
    }

    instance_ = instance;

    RCLCPP_DEBUG(get_logger(), "VmbC loading complete");
    return instance;
  } else {
    return instance_.lock();
  }
}

VmbCAPI::~VmbCAPI()
{
  if (Shutdown) {
    Shutdown();
  }
}

result<std::vector<VmbInterfaceInfo>> VmbCAPI::interface_list_get() const
{
  uint32_t interface_list_length{};

  auto const length_err = InterfacesList(
    nullptr,
    0,
    &interface_list_length,
    sizeof(VmbInterfaceInfo));

  if (length_err != VmbErrorSuccess) {
    return error{length_err};
  }

  std::vector<VmbInterfaceInfo> interface_list{};
  interface_list.resize(interface_list_length);

  auto const list_err = InterfacesList(
    interface_list.data(),
    interface_list.size(),
    &interface_list_length,
    sizeof(VmbInterfaceInfo));

  if (list_err != VmbErrorSuccess) {
    return error{list_err};
  }

  assert(interface_list.size() == interface_list.size());

  return interface_list;
}

result<int64_t> VmbCAPI::feature_int_get(VmbHandle_t handle, const std::string_view & name) const
{
  RCLCPP_DEBUG(get_logger(), "%s('%s')", __FUNCTION__, name.data());

  VmbInt64_t value{};

  auto const err = FeatureIntGet(handle, name.data(), &value);
  if (err != VmbErrorSuccess) {
    RCLCPP_ERROR(
      get_logger(), "%s failed with error %d (%s)", __FUNCTION__, err,
      vmb_error_to_string(err).data());
    return error{err};
  }

  return int64_t(value);
}

result<void> VmbCAPI::feature_int_set(
  VmbHandle_t handle,
  const std::string_view & name,
  const int64_t value) const
{
  RCLCPP_DEBUG(get_logger(), "%s('%s', %ld)", __FUNCTION__, name.data(), value);

  auto const err = FeatureIntSet(handle, name.data(), value);

  if (err != VmbErrorSuccess) {
    RCLCPP_ERROR(
      get_logger(), "%s failed with error %d (%s)", __FUNCTION__, err,
      vmb_error_to_string(err).data());
    return error{err};
  }

  return {};
}


result<std::string> VmbCAPI::feature_string_get(
  VmbHandle_t handle,
  const std::string_view & name) const
{
  RCLCPP_DEBUG(get_logger(), "%s('%s')", __FUNCTION__, name.data());

  uint32_t size_filled{};
  std::string value;

  auto err = FeatureStringGet(handle, name.data(), nullptr, 0, &size_filled);

  if (err != VmbErrorSuccess) {
    RCLCPP_ERROR(
      get_logger(), "%s failed with error %d (%s)", __FUNCTION__, err,
      vmb_error_to_string(err).data());
    return error{err};
  } else {
    char * buf = static_cast<char *>(malloc(size_filled));

    err = FeatureStringGet(handle, name.data(), buf, size_filled, &size_filled);

    if (err == VmbErrorSuccess) {
      value.assign(buf, size_filled);
    }

    free(buf);
    buf = nullptr;

    if (err != VmbErrorSuccess) {
      RCLCPP_ERROR(
        get_logger(), "%s failed with error %d (%s)", __FUNCTION__, err,
        vmb_error_to_string(err).data());
      return error{err};
    }
  }

  return value;
}

result<std::array<int64_t, 3>> VmbCAPI::feature_int_info_get(
  VmbHandle_t handle,
  const std::string_view & name) const
{
  RCLCPP_DEBUG(get_logger(), "%s('%s')", __FUNCTION__, name.data());

  std::array<int64_t, 3> value;
  auto err =
    FeatureIntRangeQuery(
    handle,
    name.data(),
    reinterpret_cast<VmbInt64_t *>(&value[0]),
    reinterpret_cast<VmbInt64_t *>(&value[1]));

  if (err != VmbErrorSuccess) {
    return error{err};
  }

  err =
    FeatureIntIncrementQuery(
    handle,
    name.data(),
    reinterpret_cast<VmbInt64_t *>(&value[2]));

  if (err != VmbErrorSuccess) {
    return error{err};
  }

  return value;
}

}  // namespace vimbax_camera
