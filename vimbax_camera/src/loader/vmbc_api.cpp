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

}  // namespace vimbax_camera
