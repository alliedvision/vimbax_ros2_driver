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

#ifndef VIMBAX_CAMERA__LOADER__VMBC_API_HPP_
#define VIMBAX_CAMERA__LOADER__VMBC_API_HPP_

#include <VmbC/VmbC.h>

#include <functional>
#include <memory>

#include <vimbax_camera/loader/library_loader.hpp>

namespace vimbax_camera
{

template<typename Ret, typename ... Args>
struct FunctionPtr;

template<typename Ret, typename ... Args>
struct FunctionPtr<Ret(Args...)>
{
  using RawPtr = Ret (*)(Args...);

  FunctionPtr()
  : raw{nullptr}
  {
  }

  explicit FunctionPtr(RawPtr ptr)
  : raw{ptr}
  {
  }

  FunctionPtr & operator=(RawPtr ptr)
  {
    raw = ptr;
    return *this;
  }

  operator bool() const
  {
    return raw != nullptr;
  }

  Ret operator()(Args... args) const
  {
    if (raw == nullptr) {
      throw std::invalid_argument("Function pointer not valid");
    }

    return raw(args ...);
  }

  RawPtr raw;
};

class VmbCAPI
{
public:
  static std::shared_ptr<VmbCAPI> get_default();
  static std::shared_ptr<VmbCAPI> load(std::shared_ptr<LibraryLoader> libraryLoader);

  FunctionPtr<decltype(VmbStartup)> Startup;
  FunctionPtr<decltype(VmbShutdown)> Shutdown;

  FunctionPtr<decltype(VmbCamerasList)> CamerasList;
  FunctionPtr<decltype(VmbCameraOpen)> CameraOpen;
  FunctionPtr<decltype(VmbCameraClose)> CameraClose;

  FunctionPtr<decltype(VmbCameraInfoQuery)> CameraInfoQuery;

private:
  VmbCAPI() = default;

  std::unique_ptr<LoadedLibrary> libraryHandle_;

  static std::shared_ptr<VmbCAPI> defaultInstance_;
};

}  // namespace vimbax_camera

#endif  // VIMBAX_CAMERA__LOADER__VMBC_API_HPP_
