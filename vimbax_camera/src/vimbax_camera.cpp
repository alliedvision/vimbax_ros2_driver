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

#include <optional>

#include <rclcpp/rclcpp.hpp>

#include <vimbax_camera/vimbax_camera.hpp>

namespace vimbax_camera
{

std::unique_ptr<VimbaXCamera> VimbaXCamera::open(
  std::shared_ptr<VmbCAPI> api,
  const std::string & name)
{
  auto const logger = rclcpp::get_logger("vimbax_camera");

  auto openCamera =
    [&](const std::string & idStr) -> std::optional<VmbHandle_t> {
      VmbHandle_t cameraHandle;
      auto const openError =
        api->CameraOpen(idStr.c_str(), VmbAccessModeType::VmbAccessModeExclusive, &cameraHandle);

      if (openError != VmbErrorSuccess) {
        RCLCPP_ERROR(logger, "Failed to open camera %s with %d", idStr.c_str(), openError);
        return std::nullopt;
      }

      return cameraHandle;
    };

  auto listCameras =
    [&]() -> std::vector<VmbCameraInfo_t> {
      uint32_t availableCamerasCount{0};
      auto const countError = api->CamerasList(nullptr, 0, &availableCamerasCount, 0);

      if (countError != VmbErrorSuccess) {
        RCLCPP_ERROR(logger, "Reading camera list size failed with %d", countError);
        return {};
      }

      std::vector<VmbCameraInfo_t> cameraList{};
      cameraList.resize(availableCamerasCount);
      uint32_t camerasFound = 0;

      auto const error = api->CamerasList(
        cameraList.data(), availableCamerasCount, &camerasFound, sizeof(VmbCameraInfo_t));

      if (error != VmbErrorSuccess) {
        RCLCPP_ERROR(logger, "List first camera failed with %d", error);
        return {};
      }

      return cameraList;
    };

  if (name.empty()) {
    RCLCPP_INFO(logger, "No camera requested opening first available");

    auto const availableCameras = listCameras();

    if (availableCameras.empty()) {
      RCLCPP_ERROR(logger, "List cameras returned 0");
      return nullptr;
    }

    for (auto const & cam : availableCameras) {
      if ((cam.permittedAccess & VmbAccessModeExclusive) != 0) {
        RCLCPP_INFO(logger, "Try opening camera with extended id %s", cam.cameraIdExtended);

        auto const optHandle = openCamera(cam.cameraIdExtended);

        if (optHandle) {
          return std::unique_ptr<VimbaXCamera>(new VimbaXCamera{api, *optHandle});
        }
      }
    }

    RCLCPP_ERROR(logger, "No camera available!");

    return nullptr;
  } else {
    auto const optHandle = openCamera(name);

    if (optHandle) {
      return std::unique_ptr<VimbaXCamera>(new VimbaXCamera{api, *optHandle});
    }

    auto const availableCameras = listCameras();

    // Try open by serial number
    if (!availableCameras.empty()) {
      for (auto const & camera : availableCameras) {
        if (camera.serialString == name && (camera.permittedAccess & VmbAccessModeExclusive) != 0) {
          auto const optHandleSerial = openCamera(camera.cameraIdExtended);

          if (optHandleSerial) {
            return std::unique_ptr<VimbaXCamera>(new VimbaXCamera{api, *optHandleSerial});
          }
        }
      }
    }

    RCLCPP_ERROR(logger, "Failed to open given camera %s", name.c_str());
  }

  return nullptr;
}

VimbaXCamera::VimbaXCamera(std::shared_ptr<VmbCAPI> api, VmbHandle_t cameraHandle)
: api_{std::move(api)}, cameraHandle_{cameraHandle}, logger_{rclcpp::get_logger("vimbax_camera")}
{
  VmbCameraInfo_t cameraInfo{};
  auto const error = api_->CameraInfoQueryByHandle(cameraHandle_, &cameraInfo, sizeof(cameraInfo));
  if (error == VmbErrorSuccess) {
    RCLCPP_INFO(
      logger_, "Opened camera info model name: %s, camera name: %s, serial: %s",
      cameraInfo.modelName, cameraInfo.cameraName, cameraInfo.serialString);
  }
}

VimbaXCamera::~VimbaXCamera()
{
  if (api_ && cameraHandle_) {
    api_->CameraClose(cameraHandle_);
    cameraHandle_ = nullptr;
  }
}
}  // namespace vimbax_camera
