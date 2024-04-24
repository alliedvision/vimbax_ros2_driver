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

#ifndef VIMBAX_CAMERA__LOADER__VMBC_API_HPP_
#define VIMBAX_CAMERA__LOADER__VMBC_API_HPP_

#include <VmbC/VmbC.h>

#include <functional>
#include <memory>
#include <string>
#include <stdexcept>
#include <vector>

#include <vimbax_camera/loader/library_loader.hpp>
#include <vimbax_camera/result.hpp>

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
    const std::string & search_path = {},
    std::shared_ptr<LibraryLoader> library_loader = LibraryLoader::get_default());

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

  result<std::vector<VmbInterfaceInfo>> interface_list_get() const;

  result<int64_t> feature_int_get(
    VmbHandle_t handle,
    const std::string_view & name) const;

  result<void> feature_int_set(
    VmbHandle_t handle,
    const std::string_view & name,
    const int64_t value) const;

  result<std::array<int64_t, 3>> feature_int_info_get(
    VmbHandle_t handle,
    const std::string_view & name) const;

  result<std::string> feature_string_get(
    VmbHandle_t handle,
    const std::string_view & name) const;

private:
  VmbCAPI() = default;


  // Startup and Shutdown are private, because the lifecycle is managed by the class itself.
  FunctionPtr<decltype(VmbStartup)> Startup;
  FunctionPtr<decltype(VmbShutdown)> Shutdown;

  std::unique_ptr<LoadedLibrary> library_handle_;

  static std::weak_ptr<VmbCAPI> instance_;
};

}  // namespace vimbax_camera

#endif  // VIMBAX_CAMERA__LOADER__VMBC_API_HPP_
