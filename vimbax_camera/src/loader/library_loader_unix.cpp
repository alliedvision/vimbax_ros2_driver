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

#include <dlfcn.h>

#include <rclcpp/logging.hpp>

#include <vimbax_camera/vimbax_camera_helper.hpp>
#include <vimbax_camera/loader/library_loader.hpp>

namespace vimbax_camera
{
using helper::get_logger;

static std::shared_ptr<LibraryLoader> g_default_loader{};

class LoadedLibraryUnix : public LoadedLibrary
{
public:
  explicit LoadedLibraryUnix(void * dl_handle)
  : dl_handle_{dl_handle} {}

  ~LoadedLibraryUnix() override
  {
    dlclose(dl_handle_);
  }

  void * resolve_symbol(const std::string & name) override
  {
    auto symbolPtr = dlsym(dl_handle_, name.c_str());

    if (symbolPtr != nullptr) {
      return symbolPtr;
    }

    RCLCPP_ERROR(get_logger(), "Loading symbol %s failed with: %s", name.c_str(), dlerror());

    return nullptr;
  }

private:
  void * dl_handle_;
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
      get_logger(), "Loading shared library %s failed with: %s", name.c_str(), dlerror());

    return {};
  }

  std::string build_library_name(const std::string & name) override
  {
    return std::string{"lib" + name + ".so"};
  }
};

std::shared_ptr<LibraryLoader> LibraryLoader::get_default()
{
  if (!g_default_loader) {
    g_default_loader = std::make_shared<LibraryLoaderUnix>();
  }

  return g_default_loader;
}

}  // namespace vimbax_camera
