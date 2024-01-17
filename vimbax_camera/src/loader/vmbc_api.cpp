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

std::shared_ptr<VmbCAPI> VmbCAPI::defaultInstance_{};

static std::string_view getenv_safe(const std::string & name)
{
  auto const val = getenv(name.c_str());

  return val == nullptr ? "" : val;
}

static std::vector<std::string_view> splitString(const std::string_view & str, const char c)
{
  if (str.empty()) {
    return {};
  }

  std::vector<std::string_view> resList{};
  std::size_t start = 0;
  std::size_t end;

  do {
    end = str.find(c, start);

    auto const part = str.substr(start, end);
    if (!part.empty()) {
      resList.push_back(part);
    }
    start = end + 1;
  } while (end != std::string::npos);

  return resList;
}

static std::unique_ptr<LoadedLibrary> loadVmbCLibrary(std::shared_ptr<LibraryLoader> libraryLoader)
{
  auto const libName = libraryLoader->build_library_name("VmbC");

  auto const vimbaXHome = getenv_safe("VIMBA_X_HOME");

  auto const vimbaXHomeParts = splitString(vimbaXHome, ':');
  for (auto const & part : vimbaXHomeParts) {
    fs::path vmbcPath = canonical((fs::path{part} / "api" / "lib")) / libName;
    if (fs::exists(vmbcPath)) {
      return libraryLoader->open(vmbcPath);
    }
  }

  auto const tlSearchPath = getenv_safe("GENICAM_GENTL64_PATH");

  auto const tlSearchPathParts = splitString(tlSearchPath, ':');
  for (auto const & part : tlSearchPathParts) {
    fs::path vmbcPath = canonical((fs::path{part} / ".." / "api" / "lib")) / libName;
    if (fs::exists(vmbcPath)) {
      return libraryLoader->open(vmbcPath);
    }
  }

  return libraryLoader->open(libName);
}

std::shared_ptr<VmbCAPI> VmbCAPI::get_default()
{
  if (!defaultInstance_) {
    defaultInstance_ = load(LibraryLoader::get_default());
  }

  return defaultInstance_;
}

std::shared_ptr<VmbCAPI> VmbCAPI::load(std::shared_ptr<LibraryLoader> libraryLoader)
{
  std::shared_ptr<VmbCAPI> instance{new VmbCAPI};

  auto library = loadVmbCLibrary(libraryLoader);

  if (!library) {
    return nullptr;
  }

  LOAD_FUNC(instance, Startup);
  LOAD_FUNC(instance, Shutdown);
  LOAD_FUNC(instance, CamerasList);
  LOAD_FUNC(instance, CameraOpen);
  LOAD_FUNC(instance, CameraClose);
  LOAD_FUNC(instance, CameraInfoQuery);

  instance->libraryHandle_ = std::move(library);

  return instance;
}

}  // namespace vimbax_camera
