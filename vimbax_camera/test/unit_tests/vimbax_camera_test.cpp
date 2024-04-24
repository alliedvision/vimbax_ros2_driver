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

#include <vimbax_camera/vimbax_camera.hpp>
#include <vimbax_camera/vimbax_camera_helper.hpp>

#include <gmock/gmock.h>

#include <filesystem>
#include <fstream>

#include "mocks/library_loader_mock.hpp"

using ::vimbax_camera::VmbCAPI;
using ::vimbax_camera::VimbaXCamera;
using ::vimbax_camera::SFNCFeatures;

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


    EXPECT_CALL(*api_mock_, FeatureCommandIsDone).WillRepeatedly(
      [](auto, auto, bool * done) -> VmbError_t {
        *done = true;
        return VmbErrorSuccess;
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

  std::array<VmbHandle_t, 1> stream_handles_{};
};

class VimbaXCameraOpenedTest : public VimbaXCameraTest
{
protected:
  void SetUp() override
  {
    VimbaXCameraTest::SetUp();

    std::string const cameraIdStr = "testcam";

    EXPECT_CALL(*api_mock_, CamerasList).Times(AtLeast(1))
    .WillRepeatedly(
      [&](auto, auto, auto numFound, auto) -> VmbError_t {
        EXPECT_NE(numFound, nullptr);

        if (numFound != nullptr) {
          *numFound = 0;
          return VmbErrorSuccess;
        }

        return VmbErrorUnknown;
      });

    EXPECT_CALL(*api_mock_, CameraClose(&dummy_handle_)).Times(1);

    EXPECT_CALL(*api_mock_, CameraInfoQueryByHandle(&dummy_handle_, _, _))
    .Times(AtLeast(1)).WillRepeatedly(
      [&](auto, VmbCameraInfo_t * ptr, auto) -> VmbError_t {
        ptr->modelName = "TestCamera";
        ptr->cameraName = "TestCamera";
        ptr->serialString = "1234";
        ptr->streamHandles = stream_handles_.data();
        ptr->streamCount = uint32_t(stream_handles_.size());
        return VmbErrorSuccess;
      });

    EXPECT_CALL(*api_mock_, CameraOpen(Eq(cameraIdStr), _, _))
    .Times(1).WillOnce(
      [&](auto, auto, auto handle) -> VmbError_t {
        *handle = &dummy_handle_;
        return VmbErrorSuccess;
      });

    camera_ = VimbaXCamera::open(api_, cameraIdStr);

    EXPECT_CALL(
      *api_mock_,
      FeatureInfoQuery(&dummy_handle_, Eq(SFNCFeatures::PixelFormat), _, _))
    .Times(AtLeast(0)).WillRepeatedly(
      [&](auto, auto, VmbFeatureInfo_t * info, auto infoSize) -> int32_t {
        if (infoSize != sizeof(VmbFeatureInfo)) {
          return VmbErrorStructSize;
        }
        info->sfncNamespace = "Standard";
        return VmbErrorSuccess;
      });

    EXPECT_CALL(
      *api_mock_, FeatureEnumGet(&dummy_handle_, Eq(SFNCFeatures::PixelFormat), _))
    .Times(AtLeast(0)).WillRepeatedly(
      [&](auto, auto, const char ** ptr) -> int32_t {
        *ptr = "Test";
        return VmbErrorSuccess;
      });

    EXPECT_CALL(
      *api_mock_,
      FeatureEnumAsInt(
        &dummy_handle_,
        Eq(SFNCFeatures::PixelFormat),
        Eq(std::string("Test")), _))
    .Times(AtLeast(0)).WillRepeatedly(
      [&](auto, auto, auto, VmbInt64_t * iptr) -> int32_t {
        *iptr = test_format_;
        return VmbErrorSuccess;
      });

    EXPECT_CALL(
      *api_mock_, FeatureIntGet(_, Eq(SFNCFeatures::Width), _)).Times(AtLeast(0))
    .WillRepeatedly(
      [&](auto, auto, VmbInt64_t * ptr) -> int32_t {
        *ptr = test_width_;
        return VmbErrorSuccess;
      });
    EXPECT_CALL(
      *api_mock_, FeatureIntGet(_, Eq(SFNCFeatures::Height), _)).Times(AtLeast(0))
    .WillRepeatedly(
      [&](auto, auto, VmbInt64_t * ptr) -> int32_t {
        *ptr = test_height_;
        return VmbErrorSuccess;
      });

    ASSERT_TRUE(camera_);
  }

  void TearDown() override
  {
    camera_.reset();
    VimbaXCameraTest::TearDown();
  }

  std::shared_ptr<VimbaXCamera> camera_;
  int32_t test_format_ = VmbPixelFormatRgb8;
  int32_t test_width_ = 1500;
  int32_t test_height_ = 1500;
  int32_t test_line_ = test_width_ * 3;
  int64_t test_size_ = test_line_ * test_height_;

  uint64_t dummy_handle_{};
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
      stream_handles_.data(),
      uint32_t(stream_handles_.size()),
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
      stream_handles_.data(),
      uint32_t(uint32_t(stream_handles_.size())),
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
  .Times(AtLeast(1)).WillRepeatedly(
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
      stream_handles_.data(),
      uint32_t(stream_handles_.size()),
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
      stream_handles_.data(),
      uint32_t(stream_handles_.size()),
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
  .Times(AtLeast(1)).WillRepeatedly(
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
      stream_handles_.data(),
      uint32_t(stream_handles_.size()),
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
      stream_handles_.data(),
      uint32_t(stream_handles_.size()),
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
    [&](auto, auto, auto numFound, auto) -> VmbError_t {
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

  EXPECT_CALL(*api_mock_, CamerasList).Times(AtLeast(1))
  .WillRepeatedly(
    [&](auto, auto, auto numFound, auto) -> VmbError_t {
      EXPECT_NE(numFound, nullptr);

      if (numFound != nullptr) {
        *numFound = 0;
        return VmbErrorSuccess;
      }

      return VmbErrorUnknown;
    });

  EXPECT_CALL(*api_mock_, CameraInfoQueryByHandle(_, _, _))
  .Times(0);

  EXPECT_CALL(*api_mock_, CameraOpen(Eq(cameraIdStr), _, _))
  .Times(1).WillOnce(Return(VmbErrorUnknown));

  auto camera = VimbaXCamera::open(api_, cameraIdStr);

  EXPECT_FALSE(camera);
}

TEST_F(VimbaXCameraTest, open_by_id_success_fallback)
{
  uint64_t dummyHandle{};
  std::string const cameraIdStr = "testCam3ExtId";

  EXPECT_CALL(*api_mock_, CamerasList).Times(AtLeast(1))
  .WillRepeatedly(
    [&](auto, auto, auto numFound, auto) -> VmbError_t {
      EXPECT_NE(numFound, nullptr);

      if (numFound != nullptr) {
        *numFound = 0;
        return VmbErrorSuccess;
      }

      return VmbErrorUnknown;
    });

  EXPECT_CALL(*api_mock_, CameraClose(&dummyHandle)).Times(1);

  EXPECT_CALL(*api_mock_, CameraInfoQueryByHandle(&dummyHandle, _, _))
  .Times(AtLeast(1)).WillRepeatedly(
    [&](auto, VmbCameraInfo_t * ptr, auto) {
      ptr->streamHandles = stream_handles_.data();
      ptr->streamCount = uint32_t(stream_handles_.size());
      return VmbErrorSuccess;
    });

  EXPECT_CALL(*api_mock_, CameraOpen(Eq(cameraIdStr), _, _))
  .Times(1).WillOnce(
    [&](auto, auto, auto handle) -> VmbError_t {
      *handle = &dummyHandle;
      return VmbErrorSuccess;
    });

  auto camera = VimbaXCamera::open(api_, cameraIdStr);

  EXPECT_TRUE(camera);
}


TEST_F(VimbaXCameraTest, open_by_id_success)
{
  uint64_t dummyHandle{};
  std::string const cameraIdStr = "testCam2ExtId";

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
      stream_handles_.data(),
      uint32_t(stream_handles_.size()),
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
      stream_handles_.data(),
      uint32_t(stream_handles_.size()),
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

  EXPECT_CALL(*api_mock_, CameraClose(&dummyHandle)).Times(1);

  EXPECT_CALL(*api_mock_, CameraInfoQueryByHandle(&dummyHandle, _, _))
  .Times(AtLeast(1)).WillRepeatedly(
    [&](auto, VmbCameraInfo_t * info_ptr, auto) -> VmbError_t {
      *info_ptr = availableCameras[1];
      return VmbErrorSuccess;
    });

  EXPECT_CALL(*api_mock_, CameraOpen(Eq(cameraIdStr), _, _))
  .Times(1).WillOnce(
    [&](auto, auto, auto handle) -> VmbError_t {
      *handle = &dummyHandle;
      return VmbErrorSuccess;
    });

  auto camera = VimbaXCamera::open(api_, cameraIdStr);

  EXPECT_TRUE(camera);
}

TEST_F(VimbaXCameraTest, open_by_serial)
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
      stream_handles_.data(),
      uint32_t(stream_handles_.size()),
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
      stream_handles_.data(),
      uint32_t(stream_handles_.size()),
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
  EXPECT_CALL(*api_mock_, CameraOpen(_, _, _)).Times(1)
  .WillOnce(
    [&](auto, auto, auto cameraHandle) -> VmbError_t {
      *cameraHandle = reinterpret_cast<VmbHandle_t>(&dummyHandle);
      return VmbErrorSuccess;
    });

  EXPECT_CALL(*api_mock_, CameraInfoQueryByHandle(&dummyHandle, _, _))
  .Times(AtLeast(1)).WillRepeatedly(
    [&](auto, auto infoPtr, auto infoSize) -> VmbError_t {
      EXPECT_EQ(infoSize, sizeof(VmbCameraInfo));

      if (infoSize == sizeof(VmbCameraInfo)) {
        *infoPtr = availableCameras[0];
        return VmbErrorSuccess;
      }

      return VmbErrorUnknown;
    });

  EXPECT_CALL(*api_mock_, CameraClose(&dummyHandle)).Times(1);

  auto camera = VimbaXCamera::open(api_, availableCameras[1].serialString);

  EXPECT_TRUE(camera);
}

TEST_F(VimbaXCameraOpenedTest, frame_create_announce_fail)
{
  auto const announceError = VmbErrorMoreData;

  EXPECT_CALL(*api_mock_, FrameAnnounce).Times(1).WillOnce(Return(announceError));

  auto const frameRes = VimbaXCamera::Frame::create(camera_, test_size_);

  EXPECT_FALSE(frameRes);
  EXPECT_EQ(frameRes.error().code, announceError);
}

TEST_F(VimbaXCameraOpenedTest, frame_create_format_info_fail)
{
  auto const infoError = VmbErrorMoreData;

  EXPECT_CALL(*api_mock_, FeatureInfoQuery(_, Eq(SFNCFeatures::PixelFormat), _, _))
  .Times(1).WillOnce(Return(infoError));

  EXPECT_CALL(*api_mock_, FrameAnnounce).Times(0);

  auto const frameRes = VimbaXCamera::Frame::create(camera_, test_size_);

  EXPECT_FALSE(frameRes);
  EXPECT_EQ(frameRes.error().code, infoError);
}

TEST_F(VimbaXCameraOpenedTest, frame_create_success_by_announce)
{
  EXPECT_CALL(*api_mock_, FrameAnnounce).Times(1)
  .WillOnce(
    [&](auto, const VmbFrame * frame, auto frameSize) -> int32_t {
      EXPECT_EQ(frameSize, sizeof(VmbFrame));

      EXPECT_EQ(frame->bufferSize, test_size_);
      EXPECT_NE(frame->buffer, nullptr);

      return VmbErrorSuccess;
    });

  auto const frameRes = VimbaXCamera::Frame::create(camera_, test_size_);

  EXPECT_TRUE(frameRes);
  EXPECT_NE(frameRes->get(), nullptr);
}

TEST_F(VimbaXCameraOpenedTest, frame_create_success_by_alloc_and_announce)
{
  auto const size = test_size_ + 128;

  EXPECT_CALL(*api_mock_, FrameAnnounce).Times(1)
  .WillOnce(
    [&](auto, const VmbFrame * frame, auto frameSize) -> int32_t {
      EXPECT_EQ(frameSize, sizeof(VmbFrame));

      EXPECT_EQ(frame->bufferSize, size);
      EXPECT_EQ(frame->buffer, nullptr);

      return VmbErrorSuccess;
    });

  auto const frameRes = VimbaXCamera::Frame::create(camera_, size);

  EXPECT_TRUE(frameRes);
  EXPECT_NE(frameRes->get(), nullptr);
}

TEST_F(VimbaXCameraOpenedTest, settings_save_invalid_directory)
{
  auto const temp_path = std::filesystem::temp_directory_path();
  auto const test_xml_path = temp_path / "invalid" / "invalid.xml";

  auto const result = camera_->settings_save(test_xml_path.string());

  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().code, VmbErrorNotFound);
}

TEST_F(VimbaXCameraOpenedTest, settings_save_invalid_file_name)
{
  auto const temp_path = std::filesystem::temp_directory_path();
  auto const test_xml_path = temp_path / "invalid" / "invalid.json";

  auto const result = camera_->settings_save(test_xml_path.string());

  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().code, VmbErrorInvalidValue);
}

TEST_F(VimbaXCameraOpenedTest, settings_save_valid_error)
{
  auto const error = VmbErrorCustom;
  auto const temp_path = std::filesystem::temp_directory_path();
  auto const test_xml_path = temp_path / "valid.xml";

  if (std::filesystem::exists(test_xml_path)) {
    std::filesystem::remove(test_xml_path);
  }

  std::ofstream stream{test_xml_path};
  stream.close();

  EXPECT_CALL(*api_mock_, SettingsSave(_, Eq(test_xml_path.string()), _, _)).Times(1)
  .WillOnce(Return(error));

  auto const result = camera_->settings_save(test_xml_path.string());

  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().code, error);
}


TEST_F(VimbaXCameraOpenedTest, settings_save_valid_ok)
{
  auto const temp_path = std::filesystem::temp_directory_path();
  auto const test_xml_path = temp_path / "valid.xml";

  if (std::filesystem::exists(test_xml_path)) {
    std::filesystem::remove(test_xml_path);
  }

  std::ofstream stream{test_xml_path};
  stream.close();

  EXPECT_CALL(*api_mock_, SettingsSave(_, Eq(test_xml_path.string()), _, _)).Times(1)
  .WillOnce(Return(VmbErrorSuccess));

  auto const result = camera_->settings_save(test_xml_path.string());

  ASSERT_TRUE(result);
}

TEST_F(VimbaXCameraOpenedTest, settings_load_invalid)
{
  auto const temp_path = std::filesystem::temp_directory_path();
  auto const test_xml_path = temp_path / "invalid" / "invalid.xml";

  auto const result = camera_->settings_load(test_xml_path.string());

  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().code, VmbErrorNotFound);
}

TEST_F(VimbaXCameraOpenedTest, settings_load_valid_error)
{
  auto const error = VmbErrorCustom;
  auto const temp_path = std::filesystem::temp_directory_path();
  auto const test_xml_path = temp_path / "valid.xml";

  if (std::filesystem::exists(test_xml_path)) {
    std::filesystem::remove(test_xml_path);
  }

  std::ofstream stream{test_xml_path};
  stream.close();

  EXPECT_CALL(*api_mock_, SettingsLoad(_, Eq(test_xml_path.string()), _, _)).Times(1)
  .WillOnce(Return(error));

  auto const result = camera_->settings_load(test_xml_path.string());

  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().code, error);
}


TEST_F(VimbaXCameraOpenedTest, settings_load_valid_ok)
{
  auto const temp_path = std::filesystem::temp_directory_path();
  auto const test_xml_path = temp_path / "valid.xml";

  if (std::filesystem::exists(test_xml_path)) {
    std::filesystem::remove(test_xml_path);
  }

  std::ofstream stream{test_xml_path};
  stream.close();

  EXPECT_CALL(*api_mock_, SettingsLoad(_, Eq(test_xml_path.string()), _, _)).Times(1)
  .WillOnce(Return(VmbErrorSuccess));

  auto const result = camera_->settings_load(test_xml_path.string());

  ASSERT_TRUE(result);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
