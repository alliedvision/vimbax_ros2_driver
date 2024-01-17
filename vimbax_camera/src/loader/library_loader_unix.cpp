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

#include <dlfcn.h>

#include <rclcpp/logging.hpp>

#include <vimbax_camera/loader/library_loader.hpp>

namespace vimbax_camera
{
static std::shared_ptr<LibraryLoader> g_defaultLoader{};

class LoadedLibraryUnix : public LoadedLibrary
{
public:
  explicit LoadedLibraryUnix(void * dlHandle)
  : dlHandle_{dlHandle} {}

  ~LoadedLibraryUnix() override
  {
    dlclose(dlHandle_);
  }

  void * resolve_symbol(const std::string & name) override
  {
    auto symbolPtr = dlsym(dlHandle_, name.c_str());

    if (symbolPtr != nullptr) {
      return symbolPtr;
    }

    RCLCPP_ERROR(
      rclcpp::get_logger("LoadedLibraryUnix"), "Loading symbol %s failed with: %s",
      name.c_str(), dlerror());

    return nullptr;
  }

private:
  void * dlHandle_;
};

class LibraryLoaderUnix : public LibraryLoader
{
public:
  std::unique_ptr<LoadedLibrary> open(const std::string & name) override
  {
    void * handle = dlopen(name.c_str(), RTLD_LOCAL | RTLD_LAZY);
    if (handle != nullptr) {
      return std::make_unique<LoadedLibraryUnix>(handle);
    }

    RCLCPP_ERROR(
      rclcpp::get_logger("LibraryLoaderUnix"), "Loading shared library %s failed with: %s",
      name.c_str(), dlerror());

    return {};
  }

  std::string build_library_name(const std::string & name) override
  {
    return std::string{"lib" + name + ".so"};
  }
};

std::shared_ptr<LibraryLoader> LibraryLoader::get_default()
{
  if (!g_defaultLoader) {
    g_defaultLoader = std::make_shared<LibraryLoaderUnix>();
  }

  return g_defaultLoader;
}

}  // namespace vimbax_camera
