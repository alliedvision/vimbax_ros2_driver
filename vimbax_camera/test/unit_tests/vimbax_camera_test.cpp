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

#include <vimbax_camera/vimbax_camera.hpp>

#include <gmock/gmock.h>

#include "mocks/library_loader_mock.hpp"

using ::vimbax_camera::VmbCAPI;
using ::vimbax_camera::VimbaXCamera;

using ::testing::_;
using ::testing::AtLeast;
using ::testing::Return;
using ::testing::ByMove;
using ::testing::Eq;


class VimbaXCameraTest : public testing::Test
{
protected:
  void SetUp() override
  {
    auto loaderMock = std::make_shared<MockLibraryLoader>();

    api_mock_ = APIMock::get_instance();

    EXPECT_CALL(*loaderMock, build_library_name(_)).Times(1)
    .WillRepeatedly(Return("VmbCTest"));

    EXPECT_CALL(*loaderMock, open("VmbCTest")).Times(1)
    .WillRepeatedly(
      [](const std::string &) {
        auto libraryMock = std::make_unique<MockLoadedLibrary>();
        EXPECT_CALL(*libraryMock, resolve_symbol(_)).Times(AtLeast(1));
        return libraryMock;
      });


    EXPECT_CALL(*api_mock_, Startup(_)).Times(1);
    EXPECT_CALL(*api_mock_, Shutdown()).Times(1);
    auto instance = VmbCAPI::get_instance({}, loaderMock);

    if (instance) {
      api_ = instance;
    }

    Test::SetUp();
  }

  void TearDown() override
  {
    api_.reset();
    api_mock_.reset();

    Test::TearDown();
  }

  std::shared_ptr<VmbCAPI> api_;
  std::shared_ptr<APIMock> api_mock_;
};

TEST_F(VimbaXCameraTest, open_first_camera)
{
  uint64_t dummyHandle{};

  std::array<VmbCameraInfo, 2> availableCameras = {
    VmbCameraInfo{
      "testCam1Id",
      "testCam1ExtId",
      "Test Camera 1",
      "Test Camera",
      "1234",
      nullptr,
      nullptr,
      nullptr,
      nullptr,
      0,
      VmbAccessModeRead | VmbAccessModeFull | VmbAccessModeExclusive
    },
    VmbCameraInfo{
      "testCam2Id",
      "testCam2ExtId",
      "Test Camera 2",
      "Test Camera",
      "5678",
      nullptr,
      nullptr,
      nullptr,
      nullptr,
      0,
      VmbAccessModeRead | VmbAccessModeFull | VmbAccessModeExclusive
    }
  };

  EXPECT_CALL(*api_mock_, CamerasList).Times(AtLeast(1))
  .WillRepeatedly(
    [&](
      VmbCameraInfo_t * cameraInfo,
      VmbUint32_t listLength,
      VmbUint32_t * numFound,
      VmbUint32_t sizeofCameraInfo) -> VmbError_t {
      EXPECT_NE(numFound, nullptr);

      if (cameraInfo == nullptr) {
        *numFound = availableCameras.size();
        return VmbErrorSuccess;
      }

      if (listLength < availableCameras.size()) {
        return VmbErrorMoreData;
      }


      EXPECT_EQ(sizeofCameraInfo, sizeof(VmbCameraInfo));
      if (sizeofCameraInfo == sizeof(VmbCameraInfo)) {
        memcpy(cameraInfo, availableCameras.data(), sizeofCameraInfo * availableCameras.size());
        *numFound = availableCameras.size();
        return VmbErrorSuccess;
      }

      return VmbErrorUnknown;
    });

  auto const extIdStr = std::string{availableCameras[0].cameraIdExtended};
  EXPECT_CALL(*api_mock_, CameraOpen(Eq(extIdStr), _, _)).Times(1)
  .WillOnce(
    [&](auto, auto, auto cameraHandle) -> VmbError_t {
      *cameraHandle = reinterpret_cast<VmbHandle_t>(&dummyHandle);
      return VmbErrorSuccess;
    });

  EXPECT_CALL(*api_mock_, CameraInfoQueryByHandle(&dummyHandle, _, _))
  .Times(1).WillOnce(
    [&](auto, auto infoPtr, auto infoSize) -> VmbError_t {
      EXPECT_EQ(infoSize, sizeof(VmbCameraInfo));

      if (infoSize == sizeof(VmbCameraInfo)) {
        *infoPtr = availableCameras[0];
        return VmbErrorSuccess;
      }

      return VmbErrorUnknown;
    });

  EXPECT_CALL(*api_mock_, CameraClose(&dummyHandle)).Times(1);

  auto camera = VimbaXCamera::open(api_);

  EXPECT_TRUE(camera);
}

TEST_F(VimbaXCameraTest, open_second_camera)
{
  uint64_t dummyHandle{};

  std::array<VmbCameraInfo, 2> availableCameras = {
    VmbCameraInfo{
      "testCam1Id",
      "testCam1ExtId",
      "Test Camera 1",
      "Test Camera",
      "1234",
      nullptr,
      nullptr,
      nullptr,
      nullptr,
      0,
      VmbAccessModeRead
    },
    VmbCameraInfo{
      "testCam2Id",
      "testCam2ExtId",
      "Test Camera 2",
      "Test Camera",
      "5678",
      nullptr,
      nullptr,
      nullptr,
      nullptr,
      0,
      VmbAccessModeRead | VmbAccessModeFull | VmbAccessModeExclusive
    }
  };

  EXPECT_CALL(*api_mock_, CamerasList).Times(AtLeast(1))
  .WillRepeatedly(
    [&](
      VmbCameraInfo_t * cameraInfo,
      VmbUint32_t listLength,
      VmbUint32_t * numFound,
      VmbUint32_t sizeofCameraInfo) -> VmbError_t {
      EXPECT_NE(numFound, nullptr);

      if (cameraInfo == nullptr) {
        *numFound = availableCameras.size();
        return VmbErrorSuccess;
      }

      if (listLength < availableCameras.size()) {
        return VmbErrorMoreData;
      }


      EXPECT_EQ(sizeofCameraInfo, sizeof(VmbCameraInfo));
      if (sizeofCameraInfo == sizeof(VmbCameraInfo)) {
        memcpy(cameraInfo, availableCameras.data(), sizeofCameraInfo * availableCameras.size());
        *numFound = availableCameras.size();
        return VmbErrorSuccess;
      }

      return VmbErrorUnknown;
    });

  auto const extIdStr = std::string{availableCameras[1].cameraIdExtended};
  EXPECT_CALL(*api_mock_, CameraOpen(Eq(extIdStr), _, _)).Times(1)
  .WillOnce(
    [&](auto, auto, auto cameraHandle) -> VmbError_t {
      *cameraHandle = reinterpret_cast<VmbHandle_t>(&dummyHandle);
      return VmbErrorSuccess;
    });

  EXPECT_CALL(*api_mock_, CameraInfoQueryByHandle(&dummyHandle, _, _))
  .Times(1).WillOnce(
    [&](auto, auto infoPtr, auto infoSize) -> VmbError_t {
      EXPECT_EQ(infoSize, sizeof(VmbCameraInfo));

      if (infoSize == sizeof(VmbCameraInfo)) {
        *infoPtr = availableCameras[1];
        return VmbErrorSuccess;
      }

      return VmbErrorUnknown;
    });

  EXPECT_CALL(*api_mock_, CameraClose(&dummyHandle)).Times(1);

  auto camera = VimbaXCamera::open(api_);

  EXPECT_TRUE(camera);
}


TEST_F(VimbaXCameraTest, open_camera_no_access)
{
  std::array<VmbCameraInfo, 2> availableCameras = {
    VmbCameraInfo{
      "testCam1Id",
      "testCam1ExtId",
      "Test Camera 1",
      "Test Camera",
      "1234",
      nullptr,
      nullptr,
      nullptr,
      nullptr,
      0,
      VmbAccessModeRead
    },
    VmbCameraInfo{
      "testCam2Id",
      "testCam2ExtId",
      "Test Camera 2",
      "Test Camera",
      "5678",
      nullptr,
      nullptr,
      nullptr,
      nullptr,
      0,
      VmbAccessModeRead
    }
  };

  EXPECT_CALL(*api_mock_, CamerasList).Times(AtLeast(1))
  .WillRepeatedly(
    [&](
      VmbCameraInfo_t * cameraInfo,
      VmbUint32_t listLength,
      VmbUint32_t * numFound,
      VmbUint32_t sizeofCameraInfo) -> VmbError_t {
      EXPECT_NE(numFound, nullptr);

      if (cameraInfo == nullptr) {
        *numFound = availableCameras.size();
        return VmbErrorSuccess;
      }

      if (listLength < availableCameras.size()) {
        return VmbErrorMoreData;
      }


      EXPECT_EQ(sizeofCameraInfo, sizeof(VmbCameraInfo));
      if (sizeofCameraInfo == sizeof(VmbCameraInfo)) {
        memcpy(cameraInfo, availableCameras.data(), sizeofCameraInfo * availableCameras.size());
        *numFound = availableCameras.size();
        return VmbErrorSuccess;
      }

      return VmbErrorUnknown;
    });

  EXPECT_CALL(*api_mock_, CameraOpen(_, _, _)).Times(0);

  EXPECT_CALL(*api_mock_, CameraInfoQueryByHandle(_, _, _))
  .Times(0);

  auto camera = VimbaXCamera::open(api_);

  EXPECT_FALSE(camera);
}

TEST_F(VimbaXCameraTest, open_camera_no_available)
{
  EXPECT_CALL(*api_mock_, CamerasList).Times(AtLeast(1))
  .WillRepeatedly(
    [&](
      VmbCameraInfo_t *,
      VmbUint32_t,
      VmbUint32_t * numFound,
      VmbUint32_t) -> VmbError_t {
      EXPECT_NE(numFound, nullptr);

      if (numFound != nullptr) {
        *numFound = 0;
        return VmbErrorSuccess;
      }

      return VmbErrorUnknown;
    });

  EXPECT_CALL(*api_mock_, CameraOpen(_, _, _)).Times(0);

  EXPECT_CALL(*api_mock_, CameraInfoQueryByHandle(_, _, _))
  .Times(0);

  auto camera = VimbaXCamera::open(api_);

  EXPECT_FALSE(camera);
}

TEST_F(VimbaXCameraTest, open_by_id_fail)
{
  std::string const cameraIdStr = "testCamera1ById";

  EXPECT_CALL(*api_mock_, CamerasList).Times(0);

  EXPECT_CALL(*api_mock_, CameraInfoQueryByHandle(_, _, _))
  .Times(0);

  EXPECT_CALL(*api_mock_, CameraOpen(Eq(cameraIdStr), _, _))
  .Times(1).WillOnce(Return(VmbErrorUnknown));

  auto camera = VimbaXCamera::open(api_, cameraIdStr);

  EXPECT_FALSE(camera);
}

TEST_F(VimbaXCameraTest, open_by_id_sucess)
{
  uint64_t dummyHandle{};
  std::string const cameraIdStr = "testCamera1ById";

  EXPECT_CALL(*api_mock_, CamerasList).Times(0);

  EXPECT_CALL(*api_mock_, CameraClose(&dummyHandle)).Times(1);

  EXPECT_CALL(*api_mock_, CameraInfoQueryByHandle(&dummyHandle, _, _))
  .Times(1).WillOnce(Return(VmbErrorSuccess));

  EXPECT_CALL(*api_mock_, CameraOpen(Eq(cameraIdStr), _, _))
  .Times(1).WillOnce(
    [&](auto, auto, auto handle) -> VmbError_t {
      *handle = &dummyHandle;
      return VmbErrorSuccess;
    });

  auto camera = VimbaXCamera::open(api_, cameraIdStr);

  EXPECT_TRUE(camera);
}


int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
