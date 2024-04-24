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

#include <map>

#include "api_mock.hpp"

std::weak_ptr<APIMock> g_api_mock;


static VmbError_t VmbVersionQueryMock(
  VmbVersionInfo_t * versionInfo, VmbUint32_t sizeofVersionInfo)
{
  auto mock = g_api_mock.lock();
  return mock->VersionQuery(versionInfo, sizeofVersionInfo);
}
static VmbError_t VmbStartupMock(
  const VmbFilePathChar_t * pathConfiguration)
{
  auto mock = g_api_mock.lock();
  return mock->Startup(pathConfiguration);
}
void VmbShutdownMock()
{
  auto mock = g_api_mock.lock();
  return mock->Shutdown();
}
static VmbError_t VmbCamerasListMock(
  VmbCameraInfo_t * cameraInfo, VmbUint32_t listLength,
  VmbUint32_t * numFound, VmbUint32_t sizeofCameraInfo)
{
  auto mock = g_api_mock.lock();
  return mock->CamerasList(cameraInfo, listLength, numFound, sizeofCameraInfo);
}
static VmbError_t VmbCameraInfoQueryByHandleMock(
  VmbHandle_t cameraHandle, VmbCameraInfo_t * info,
  VmbUint32_t sizeofCameraInfo)
{
  auto mock = g_api_mock.lock();
  return mock->CameraInfoQueryByHandle(cameraHandle, info, sizeofCameraInfo);
}
static VmbError_t VmbCameraInfoQueryMock(
  const char * idString, VmbCameraInfo_t * info,
  VmbUint32_t sizeofCameraInfo)
{
  auto mock = g_api_mock.lock();
  return mock->CameraInfoQuery(idString, info, sizeofCameraInfo);
}
static VmbError_t VmbCameraOpenMock(
  const char * idString, VmbAccessMode_t accessMode,
  VmbHandle_t * cameraHandle)
{
  auto mock = g_api_mock.lock();
  return mock->CameraOpen(idString, accessMode, cameraHandle);
}
static VmbError_t VmbCameraCloseMock(
  const VmbHandle_t cameraHandle)
{
  auto mock = g_api_mock.lock();
  return mock->CameraClose(cameraHandle);
}
static VmbError_t VmbFeaturesListMock(
  VmbHandle_t handle, VmbFeatureInfo_t * featureInfoList,
  VmbUint32_t listLength, VmbUint32_t * numFound, VmbUint32_t sizeofFeatureInfo)
{
  auto mock = g_api_mock.lock();
  return mock->FeaturesList(handle, featureInfoList, listLength, numFound, sizeofFeatureInfo);
}
static VmbError_t VmbFeatureInfoQueryMock(
  const VmbHandle_t handle, const char * name,
  VmbFeatureInfo_t * featureInfo, VmbUint32_t sizeofFeatureInfo)
{
  auto mock = g_api_mock.lock();
  return mock->FeatureInfoQuery(handle, name, featureInfo, sizeofFeatureInfo);
}
static VmbError_t VmbFeatureListSelectedMock(
  const VmbHandle_t handle, const char * name,
  VmbFeatureInfo_t * featureInfoList, VmbUint32_t listLength,
  VmbUint32_t * numFound, VmbUint32_t sizeofFeatureInfo)
{
  auto mock = g_api_mock.lock();
  return mock->FeatureListSelected(
    handle, name, featureInfoList,
    listLength, numFound, sizeofFeatureInfo);
}
static VmbError_t VmbFeatureAccessQueryMock(
  const VmbHandle_t handle, const char * name,
  VmbBool_t * isReadable, VmbBool_t * isWriteable)
{
  auto mock = g_api_mock.lock();
  return mock->FeatureAccessQuery(handle, name, isReadable, isWriteable);
}
static VmbError_t VmbFeatureIntGetMock(
  const VmbHandle_t handle, const char * name,
  VmbInt64_t * value)
{
  auto mock = g_api_mock.lock();
  return mock->FeatureIntGet(handle, name, value);
}
static VmbError_t VmbFeatureIntSetMock(
  const VmbHandle_t handle, const char * name,
  VmbInt64_t value)
{
  auto mock = g_api_mock.lock();
  return mock->FeatureIntSet(handle, name, value);
}
static VmbError_t VmbFeatureIntRangeQueryMock(
  const VmbHandle_t handle, const char * name,
  VmbInt64_t * min, VmbInt64_t * max)
{
  auto mock = g_api_mock.lock();
  return mock->FeatureIntRangeQuery(handle, name, min, max);
}
static VmbError_t VmbFeatureIntIncrementQueryMock(
  const VmbHandle_t handle, const char * name,
  VmbInt64_t * value)
{
  auto mock = g_api_mock.lock();
  return mock->FeatureIntIncrementQuery(handle, name, value);
}
static VmbError_t VmbFeatureIntValidValueSetQueryMock(
  const VmbHandle_t handle, const char * name,
  VmbInt64_t * buffer, VmbUint32_t bufferSize, VmbUint32_t * setSize)
{
  auto mock = g_api_mock.lock();
  return mock->FeatureIntValidValueSetQuery(
    handle, name, buffer,
    bufferSize, setSize);
}
static VmbError_t VmbFeatureFloatGetMock(
  const VmbHandle_t handle, const char * name,
  double * value)
{
  auto mock = g_api_mock.lock();
  return mock->FeatureFloatGet(handle, name, value);
}
static VmbError_t VmbFeatureFloatSetMock(
  const VmbHandle_t handle, const char * name, double value)
{
  auto mock = g_api_mock.lock();
  return mock->FeatureFloatSet(handle, name, value);
}
static VmbError_t VmbFeatureFloatRangeQueryMock(
  const VmbHandle_t handle, const char * name,
  double * min, double * max)
{
  auto mock = g_api_mock.lock();
  return mock->FeatureFloatRangeQuery(handle, name, min, max);
}
static VmbError_t VmbFeatureFloatIncrementQueryMock(
  const VmbHandle_t handle, const char * name,
  VmbBool_t * hasIncrement, double * value)
{
  auto mock = g_api_mock.lock();
  return mock->FeatureFloatIncrementQuery(handle, name, hasIncrement, value);
}
static VmbError_t VmbFeatureEnumGetMock(
  const VmbHandle_t handle, const char * name,
  const char ** value)
{
  auto mock = g_api_mock.lock();
  return mock->FeatureEnumGet(handle, name, value);
}
static VmbError_t VmbFeatureEnumSetMock(
  const VmbHandle_t handle, const char * name,
  const char * value)
{
  auto mock = g_api_mock.lock();
  return mock->FeatureEnumSet(handle, name, value);
}
static VmbError_t VmbFeatureEnumRangeQueryMock(
  const VmbHandle_t handle, const char * name,
  const char ** nameArray, VmbUint32_t arrayLength, VmbUint32_t * numFound)
{
  auto mock = g_api_mock.lock();
  return mock->FeatureEnumRangeQuery(handle, name, nameArray, arrayLength, numFound);
}
static VmbError_t VmbFeatureEnumIsAvailableMock(
  const VmbHandle_t handle, const char * name,
  const char * value, VmbBool_t * isAvailable)
{
  auto mock = g_api_mock.lock();
  return mock->FeatureEnumIsAvailable(handle, name, value, isAvailable);
}
static VmbError_t VmbFeatureEnumAsIntMock(
  const VmbHandle_t handle, const char * name,
  const char * value, VmbInt64_t * intVal)
{
  auto mock = g_api_mock.lock();
  return mock->FeatureEnumAsInt(handle, name, value, intVal);
}
static VmbError_t VmbFeatureEnumAsStringMock(
  VmbHandle_t handle, const char * name,
  VmbInt64_t intValue, const char ** stringValue)
{
  auto mock = g_api_mock.lock();
  return mock->FeatureEnumAsString(handle, name, intValue, stringValue);
}
static VmbError_t VmbFeatureEnumEntryGetMock(
  const VmbHandle_t handle, const char * featureName,
  const char * entryName, VmbFeatureEnumEntry_t * featureEnumEntry,
  VmbUint32_t sizeofFeatureEnumEntry)
{
  auto mock = g_api_mock.lock();
  return mock->FeatureEnumEntryGet(
    handle, featureName, entryName,
    featureEnumEntry, sizeofFeatureEnumEntry);
}
static VmbError_t VmbFeatureStringGetMock(
  const VmbHandle_t handle, const char * name,
  char * buffer, VmbUint32_t bufferSize, VmbUint32_t * sizeFilled)
{
  auto mock = g_api_mock.lock();
  return mock->FeatureStringGet(handle, name, buffer, bufferSize, sizeFilled);
}
static VmbError_t VmbFeatureStringSetMock(
  const VmbHandle_t handle, const char * name,
  const char * value)
{
  auto mock = g_api_mock.lock();
  return mock->FeatureStringSet(handle, name, value);
}
static VmbError_t VmbFeatureStringMaxlengthQueryMock(
  const VmbHandle_t handle, const char * name,
  VmbUint32_t * maxLength)
{
  auto mock = g_api_mock.lock();
  return mock->FeatureStringMaxlengthQuery(handle, name, maxLength);
}
static VmbError_t VmbFeatureBoolGetMock(
  const VmbHandle_t handle, const char * name,
  VmbBool_t * value)
{
  auto mock = g_api_mock.lock();
  return mock->FeatureBoolGet(handle, name, value);
}
static VmbError_t VmbFeatureBoolSetMock(
  const VmbHandle_t handle, const char * name,
  VmbBool_t value)
{
  auto mock = g_api_mock.lock();
  return mock->FeatureBoolSet(handle, name, value);
}
static VmbError_t VmbFeatureCommandRunMock(
  const VmbHandle_t handle, const char * name)
{
  auto mock = g_api_mock.lock();
  return mock->FeatureCommandRun(handle, name);
}
static VmbError_t VmbFeatureCommandIsDoneMock(
  const VmbHandle_t handle, const char * name,
  VmbBool_t * isDone)
{
  auto mock = g_api_mock.lock();
  return mock->FeatureCommandIsDone(handle, name, isDone);
}
static VmbError_t VmbFeatureRawGetMock(
  const VmbHandle_t handle, const char * name, char * buffer,
  VmbUint32_t bufferSize, VmbUint32_t * sizeFilled)
{
  auto mock = g_api_mock.lock();
  return mock->FeatureRawGet(handle, name, buffer, bufferSize, sizeFilled);
}
static VmbError_t VmbFeatureRawSetMock(
  const VmbHandle_t handle, const char * name,
  const char * buffer, VmbUint32_t bufferSize)
{
  auto mock = g_api_mock.lock();
  return mock->FeatureRawSet(handle, name, buffer, bufferSize);
}
static VmbError_t VmbFeatureRawLengthQueryMock(
  const VmbHandle_t handle, const char * name,
  VmbUint32_t * length)
{
  auto mock = g_api_mock.lock();
  return mock->FeatureRawLengthQuery(handle, name, length);
}
static VmbError_t VmbFeatureInvalidationRegisterMock(
  VmbHandle_t handle, const char * name,
  VmbInvalidationCallback callback, void * userContext)
{
  auto mock = g_api_mock.lock();
  return mock->FeatureInvalidationRegister(handle, name, callback, userContext);
}
static VmbError_t VmbFeatureInvalidationUnregisterMock(
  VmbHandle_t handle, const char * name,
  VmbInvalidationCallback callback)
{
  auto mock = g_api_mock.lock();
  return mock->FeatureInvalidationUnregister(handle, name, callback);
}
static VmbError_t VmbPayloadSizeGetMock(
  VmbHandle_t handle, VmbUint32_t * payloadSize)
{
  auto mock = g_api_mock.lock();
  return mock->PayloadSizeGet(handle, payloadSize);
}
static VmbError_t VmbFrameAnnounceMock(
  VmbHandle_t handle, const VmbFrame_t * frame,
  VmbUint32_t sizeofFrame)
{
  auto mock = g_api_mock.lock();
  return mock->FrameAnnounce(handle, frame, sizeofFrame);
}
static VmbError_t VmbFrameRevokeMock(
  VmbHandle_t handle, const VmbFrame_t * frame)
{
  auto mock = g_api_mock.lock();
  return mock->FrameRevoke(handle, frame);
}
static VmbError_t VmbFrameRevokeAllMock(
  VmbHandle_t handle)
{
  auto mock = g_api_mock.lock();
  return mock->FrameRevokeAll(handle);
}
static VmbError_t VmbCaptureStartMock(
  VmbHandle_t handle)
{
  auto mock = g_api_mock.lock();
  return mock->CaptureStart(handle);
}
static VmbError_t VmbCaptureEndMock(
  VmbHandle_t handle)
{
  auto mock = g_api_mock.lock();
  return mock->CaptureEnd(handle);
}
static VmbError_t VmbCaptureFrameQueueMock(
  VmbHandle_t handle, const VmbFrame_t * frame,
  VmbFrameCallback callback)
{
  auto mock = g_api_mock.lock();
  return mock->CaptureFrameQueue(handle, frame, callback);
}
static VmbError_t VmbCaptureFrameWaitMock(
  const VmbHandle_t handle, const VmbFrame_t * frame,
  VmbUint32_t timeout)
{
  auto mock = g_api_mock.lock();
  return mock->CaptureFrameWait(handle, frame, timeout);
}
static VmbError_t VmbCaptureQueueFlushMock(
  VmbHandle_t handle)
{
  auto mock = g_api_mock.lock();
  return mock->CaptureQueueFlush(handle);
}
static VmbError_t VmbTransportLayersListMock(
  VmbTransportLayerInfo_t * transportLayerInfo,
  VmbUint32_t listLength, VmbUint32_t * numFound, VmbUint32_t sizeofTransportLayerInfo)
{
  auto mock = g_api_mock.lock();
  return mock->TransportLayersList(
    transportLayerInfo, listLength, numFound,
    sizeofTransportLayerInfo);
}
static VmbError_t VmbInterfacesListMock(
  VmbInterfaceInfo_t * interfaceInfo, VmbUint32_t listLength,
  VmbUint32_t * numFound, VmbUint32_t sizeofInterfaceInfo)
{
  auto mock = g_api_mock.lock();
  return mock->InterfacesList(interfaceInfo, listLength, numFound, sizeofInterfaceInfo);
}
static VmbError_t VmbMemoryReadMock(
  const VmbHandle_t handle, VmbUint64_t address,
  VmbUint32_t bufferSize, char * dataBuffer, VmbUint32_t * sizeComplete)
{
  auto mock = g_api_mock.lock();
  return mock->MemoryRead(handle, address, bufferSize, dataBuffer, sizeComplete);
}
static VmbError_t VmbMemoryWriteMock(
  const VmbHandle_t handle, VmbUint64_t address,
  VmbUint32_t bufferSize, const char * dataBuffer, VmbUint32_t * sizeComplete)
{
  auto mock = g_api_mock.lock();
  return mock->MemoryWrite(handle, address, bufferSize, dataBuffer, sizeComplete);
}
static VmbError_t VmbSettingsSaveMock(
  VmbHandle_t handle, const VmbFilePathChar_t * filePath,
  const VmbFeaturePersistSettings_t * settings, VmbUint32_t sizeofSettings)
{
  auto mock = g_api_mock.lock();
  return mock->SettingsSave(handle, filePath, settings, sizeofSettings);
}
static VmbError_t VmbSettingsLoadMock(
  VmbHandle_t handle, const VmbFilePathChar_t * filePath,
  const VmbFeaturePersistSettings_t * settings, VmbUint32_t sizeofSettings)
{
  auto mock = g_api_mock.lock();
  return mock->SettingsLoad(handle, filePath, settings, sizeofSettings);
}
static VmbError_t VmbChunkDataAccessMock(
  const VmbFrame_t * frame,
  VmbChunkAccessCallback chunkAccessCallback, void * userContext)
{
  auto mock = g_api_mock.lock();
  return mock->ChunkDataAccess(frame, chunkAccessCallback, userContext);
}

static const std::map<std::string, void *> mock_ptr_map = {
  {"VmbCameraClose", reinterpret_cast<void *>(&VmbCameraCloseMock)},
  {"VmbCameraInfoQuery", reinterpret_cast<void *>(&VmbCameraInfoQueryMock)},
  {"VmbCameraInfoQueryByHandle", reinterpret_cast<void *>(&VmbCameraInfoQueryByHandleMock)},
  {"VmbCameraOpen", reinterpret_cast<void *>(&VmbCameraOpenMock)},
  {"VmbCamerasList", reinterpret_cast<void *>(&VmbCamerasListMock)},
  {"VmbCaptureEnd", reinterpret_cast<void *>(&VmbCaptureEndMock)},
  {"VmbCaptureFrameQueue", reinterpret_cast<void *>(&VmbCaptureFrameQueueMock)},
  {"VmbCaptureFrameWait", reinterpret_cast<void *>(&VmbCaptureFrameWaitMock)},
  {"VmbCaptureQueueFlush", reinterpret_cast<void *>(&VmbCaptureQueueFlushMock)},
  {"VmbCaptureStart", reinterpret_cast<void *>(&VmbCaptureStartMock)},
  {"VmbChunkDataAccess", reinterpret_cast<void *>(&VmbChunkDataAccessMock)},
  {"VmbFeatureAccessQuery", reinterpret_cast<void *>(&VmbFeatureAccessQueryMock)},
  {"VmbFeatureBoolGet", reinterpret_cast<void *>(&VmbFeatureBoolGetMock)},
  {"VmbFeatureBoolSet", reinterpret_cast<void *>(&VmbFeatureBoolSetMock)},
  {"VmbFeatureCommandIsDone", reinterpret_cast<void *>(&VmbFeatureCommandIsDoneMock)},
  {"VmbFeatureCommandRun", reinterpret_cast<void *>(&VmbFeatureCommandRunMock)},
  {"VmbFeatureEnumAsInt", reinterpret_cast<void *>(&VmbFeatureEnumAsIntMock)},
  {"VmbFeatureEnumAsString", reinterpret_cast<void *>(&VmbFeatureEnumAsStringMock)},
  {"VmbFeatureEnumEntryGet", reinterpret_cast<void *>(&VmbFeatureEnumEntryGetMock)},
  {"VmbFeatureEnumGet", reinterpret_cast<void *>(&VmbFeatureEnumGetMock)},
  {"VmbFeatureEnumIsAvailable", reinterpret_cast<void *>(&VmbFeatureEnumIsAvailableMock)},
  {"VmbFeatureEnumRangeQuery", reinterpret_cast<void *>(&VmbFeatureEnumRangeQueryMock)},
  {"VmbFeatureEnumSet", reinterpret_cast<void *>(&VmbFeatureEnumSetMock)},
  {"VmbFeatureFloatGet", reinterpret_cast<void *>(&VmbFeatureFloatGetMock)},
  {"VmbFeatureFloatIncrementQuery",
    reinterpret_cast<void *>(&VmbFeatureFloatIncrementQueryMock)},
  {"VmbFeatureFloatRangeQuery", reinterpret_cast<void *>(&VmbFeatureFloatRangeQueryMock)},
  {"VmbFeatureFloatSet", reinterpret_cast<void *>(&VmbFeatureFloatSetMock)},
  {"VmbFeatureInfoQuery", reinterpret_cast<void *>(&VmbFeatureInfoQueryMock)},
  {"VmbFeatureIntGet", reinterpret_cast<void *>(&VmbFeatureIntGetMock)},
  {"VmbFeatureIntIncrementQuery", reinterpret_cast<void *>(&VmbFeatureIntIncrementQueryMock)},
  {"VmbFeatureIntRangeQuery", reinterpret_cast<void *>(&VmbFeatureIntRangeQueryMock)},
  {"VmbFeatureIntSet", reinterpret_cast<void *>(&VmbFeatureIntSetMock)},
  {"VmbFeatureIntValidValueSetQuery",
    reinterpret_cast<void *>(&VmbFeatureIntValidValueSetQueryMock)},
  {"VmbFeatureInvalidationRegister",
    reinterpret_cast<void *>(&VmbFeatureInvalidationRegisterMock)},
  {"VmbFeatureInvalidationUnregister",
    reinterpret_cast<void *>(&VmbFeatureInvalidationUnregisterMock)},
  {"VmbFeatureListSelected", reinterpret_cast<void *>(&VmbFeatureListSelectedMock)},
  {"VmbFeatureRawGet", reinterpret_cast<void *>(&VmbFeatureRawGetMock)},
  {"VmbFeatureRawLengthQuery", reinterpret_cast<void *>(&VmbFeatureRawLengthQueryMock)},
  {"VmbFeatureRawSet", reinterpret_cast<void *>(&VmbFeatureRawSetMock)},
  {"VmbFeatureStringGet", reinterpret_cast<void *>(&VmbFeatureStringGetMock)},
  {"VmbFeatureStringMaxlengthQuery",
    reinterpret_cast<void *>(&VmbFeatureStringMaxlengthQueryMock)},
  {"VmbFeatureStringSet", reinterpret_cast<void *>(&VmbFeatureStringSetMock)},
  {"VmbFeaturesList", reinterpret_cast<void *>(&VmbFeaturesListMock)},
  {"VmbFrameAnnounce", reinterpret_cast<void *>(&VmbFrameAnnounceMock)},
  {"VmbFrameRevoke", reinterpret_cast<void *>(&VmbFrameRevokeMock)},
  {"VmbFrameRevokeAll", reinterpret_cast<void *>(&VmbFrameRevokeAllMock)},
  {"VmbInterfacesList", reinterpret_cast<void *>(&VmbInterfacesListMock)},
  {"VmbMemoryRead", reinterpret_cast<void *>(&VmbMemoryReadMock)},
  {"VmbMemoryWrite", reinterpret_cast<void *>(&VmbMemoryWriteMock)},
  {"VmbPayloadSizeGet", reinterpret_cast<void *>(&VmbPayloadSizeGetMock)},
  {"VmbSettingsLoad", reinterpret_cast<void *>(&VmbSettingsLoadMock)},
  {"VmbSettingsSave", reinterpret_cast<void *>(&VmbSettingsSaveMock)},
  {"VmbShutdown", reinterpret_cast<void *>(&VmbShutdownMock)},
  {"VmbStartup", reinterpret_cast<void *>(&VmbStartupMock)},
  {"VmbTransportLayersList", reinterpret_cast<void *>(&VmbTransportLayersListMock)},
  {"VmbVersionQuery", reinterpret_cast<void *>(&VmbVersionQueryMock)},
};


void * APIMock::get_function_ptr(const std::string & name)
{
  return mock_ptr_map.at(name);
}


std::shared_ptr<APIMock> APIMock::get_instance()
{
  if (g_api_mock.expired()) {
    auto instance = std::make_shared<APIMock>();

    g_api_mock = instance;

    return instance;
  } else {
    return g_api_mock.lock();
  }
}
