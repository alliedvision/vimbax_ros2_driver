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


#ifndef VIMBAX_CAMERA__VIMBAX_CAMERA_HPP_
#define VIMBAX_CAMERA__VIMBAX_CAMERA_HPP_

#include <string>
#include <memory>

#include <rclcpp/logger.hpp>

#include <vimbax_camera/loader/vmbc_api.hpp>

namespace vimbax_camera
{
class VimbaXCamera
{
public:
  static std::unique_ptr<VimbaXCamera> open(
    std::shared_ptr<VmbCAPI> api,
    const std::string & name = {});

  ~VimbaXCamera();

  VimbaXCamera(const VimbaXCamera &) = delete;
  VimbaXCamera & operator=(const VimbaXCamera &) = delete;

private:
  explicit VimbaXCamera(std::shared_ptr<VmbCAPI> api, VmbHandle_t cameraHandle);

  std::shared_ptr<VmbCAPI> api_;
  VmbHandle_t cameraHandle_;
  rclcpp::Logger logger_;
};

}  // namespace vimbax_camera

#endif  // VIMBAX_CAMERA__VIMBAX_CAMERA_HPP_
