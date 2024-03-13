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

#include <vmbc_interface/VmbC.h>

#include <functional>
#include <memory>
#include <string>
#include <stdexcept>

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
  static std::shared_ptr<VmbCAPI> get_instance(
    const std::string & searchPath = {},
    std::shared_ptr<LibraryLoader> libraryLoader = LibraryLoader::get_default());

  ~VmbCAPI();

  FunctionPtr<decltype(VmbCameraClose)> CameraClose;
  FunctionPtr<decltype(VmbCameraInfoQuery)> CameraInfoQuery;
  FunctionPtr<decltype(VmbCameraInfoQueryByHandle)> CameraInfoQueryByHandle;
  FunctionPtr<decltype(VmbCameraOpen)> CameraOpen;
  FunctionPtr<decltype(VmbCamerasList)> CamerasList;
  FunctionPtr<decltype(VmbCaptureEnd)> CaptureEnd;
  FunctionPtr<decltype(VmbCaptureFrameQueue)> CaptureFrameQueue;
  FunctionPtr<decltype(VmbCaptureFrameWait)> CaptureFrameWait;
  FunctionPtr<decltype(VmbCaptureQueueFlush)> CaptureQueueFlush;
  FunctionPtr<decltype(VmbCaptureStart)> CaptureStart;
  FunctionPtr<decltype(VmbChunkDataAccess)> ChunkDataAccess;
  FunctionPtr<decltype(VmbFeatureAccessQuery)> FeatureAccessQuery;
  FunctionPtr<decltype(VmbFeatureBoolGet)> FeatureBoolGet;
  FunctionPtr<decltype(VmbFeatureBoolSet)> FeatureBoolSet;
  FunctionPtr<decltype(VmbFeatureCommandIsDone)> FeatureCommandIsDone;
  FunctionPtr<decltype(VmbFeatureCommandRun)> FeatureCommandRun;
  FunctionPtr<decltype(VmbFeatureEnumAsInt)> FeatureEnumAsInt;
  FunctionPtr<decltype(VmbFeatureEnumAsString)> FeatureEnumAsString;
  FunctionPtr<decltype(VmbFeatureEnumEntryGet)> FeatureEnumEntryGet;
  FunctionPtr<decltype(VmbFeatureEnumGet)> FeatureEnumGet;
  FunctionPtr<decltype(VmbFeatureEnumIsAvailable)> FeatureEnumIsAvailable;
  FunctionPtr<decltype(VmbFeatureEnumRangeQuery)> FeatureEnumRangeQuery;
  FunctionPtr<decltype(VmbFeatureEnumSet)> FeatureEnumSet;
  FunctionPtr<decltype(VmbFeatureFloatGet)> FeatureFloatGet;
  FunctionPtr<decltype(VmbFeatureFloatIncrementQuery)> FeatureFloatIncrementQuery;
  FunctionPtr<decltype(VmbFeatureFloatRangeQuery)> FeatureFloatRangeQuery;
  FunctionPtr<decltype(VmbFeatureFloatSet)> FeatureFloatSet;
  FunctionPtr<decltype(VmbFeatureInfoQuery)> FeatureInfoQuery;
  FunctionPtr<decltype(VmbFeatureIntGet)> FeatureIntGet;
  FunctionPtr<decltype(VmbFeatureIntIncrementQuery)> FeatureIntIncrementQuery;
  FunctionPtr<decltype(VmbFeatureIntRangeQuery)> FeatureIntRangeQuery;
  FunctionPtr<decltype(VmbFeatureIntSet)> FeatureIntSet;
  FunctionPtr<decltype(VmbFeatureIntValidValueSetQuery)> FeatureIntValidValueSetQuery;
  FunctionPtr<decltype(VmbFeatureInvalidationRegister)> FeatureInvalidationRegister;
  FunctionPtr<decltype(VmbFeatureInvalidationUnregister)> FeatureInvalidationUnregister;
  FunctionPtr<decltype(VmbFeatureListSelected)> FeatureListSelected;
  FunctionPtr<decltype(VmbFeatureRawGet)> FeatureRawGet;
  FunctionPtr<decltype(VmbFeatureRawLengthQuery)> FeatureRawLengthQuery;
  FunctionPtr<decltype(VmbFeatureRawSet)> FeatureRawSet;
  FunctionPtr<decltype(VmbFeatureStringGet)> FeatureStringGet;
  FunctionPtr<decltype(VmbFeatureStringMaxlengthQuery)> FeatureStringMaxlengthQuery;
  FunctionPtr<decltype(VmbFeatureStringSet)> FeatureStringSet;
  FunctionPtr<decltype(VmbFeaturesList)> FeaturesList;
  FunctionPtr<decltype(VmbFrameAnnounce)> FrameAnnounce;
  FunctionPtr<decltype(VmbFrameRevoke)> FrameRevoke;
  FunctionPtr<decltype(VmbFrameRevokeAll)> FrameRevokeAll;
  FunctionPtr<decltype(VmbInterfacesList)> InterfacesList;
  FunctionPtr<decltype(VmbMemoryRead)> MemoryRead;
  FunctionPtr<decltype(VmbMemoryWrite)> MemoryWrite;
  FunctionPtr<decltype(VmbPayloadSizeGet)> PayloadSizeGet;
  FunctionPtr<decltype(VmbSettingsLoad)> SettingsLoad;
  FunctionPtr<decltype(VmbSettingsSave)> SettingsSave;

  FunctionPtr<decltype(VmbTransportLayersList)> TransportLayersList;
  FunctionPtr<decltype(VmbVersionQuery)> VersionQuery;

private:
  VmbCAPI() = default;


  // Startup and Shutdown are private, because the lifecycle is managed by the class itself.
  FunctionPtr<decltype(VmbStartup)> Startup;
  FunctionPtr<decltype(VmbShutdown)> Shutdown;

  std::unique_ptr<LoadedLibrary> libraryHandle_;

  static std::weak_ptr<VmbCAPI> instance_;
};

}  // namespace vimbax_camera

#endif  // VIMBAX_CAMERA__LOADER__VMBC_API_HPP_
