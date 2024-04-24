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

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <vimbax_camera/loader/vmbc_api.hpp>

#include "mocks/library_loader_mock.hpp"

using ::vimbax_camera::LibraryLoader;
using ::vimbax_camera::LoadedLibrary;
using ::vimbax_camera::VmbCAPI;

using ::testing::_;
using ::testing::AtLeast;
using ::testing::Return;
using ::testing::ByMove;

class APILoader : public testing::Test
{
protected:
  void SetUp() override
  {
    auto path = getenv("GENICAM_GENTL64_PATH");
    if (path != nullptr) {
      gentl_path_ = path;
    }

    Test::SetUp();
  }
  void TearDown() override
  {
    Test::TearDown();

    setenv("GENICAM_GENTL64_PATH", gentl_path_.c_str(), 1);
  }

private:
  std::string gentl_path_;
};

TEST_F(APILoader, library_load_fail)
{
  ASSERT_EQ(setenv("GENICAM_GENTL64_PATH", "", 1), 0);

  auto mock = std::make_shared<MockLibraryLoader>();
  EXPECT_CALL(*mock, build_library_name(_)).Times(1)
  .WillOnce(Return("VmbCTest"));
  EXPECT_CALL(*mock, open("VmbCTest")).Times(1);


  auto api = VmbCAPI::get_instance({}, mock);

  ASSERT_FALSE(api);
}

TEST_F(APILoader, library_load_env)
{
  ASSERT_EQ(setenv("GENICAM_GENTL64_PATH", "", 1), 0);

  auto mock = std::make_shared<MockLibraryLoader>();
  EXPECT_CALL(*mock, build_library_name(_)).Times(1)
  .WillOnce(Return("VmbCTest"));
  EXPECT_CALL(*mock, open("VmbCTest")).Times(1);


  auto api = VmbCAPI::get_instance({}, mock);

  ASSERT_FALSE(api);
}

TEST_F(APILoader, symbol_load_fail)
{
  ASSERT_EQ(setenv("GENICAM_GENTL64_PATH", "", 1), 0);

  auto loaderMock = std::make_shared<MockLibraryLoader>();
  auto libraryMock = std::make_unique<MockLoadedLibrary>();
  EXPECT_CALL(*libraryMock, resolve_symbol(_)).Times(1)
  .WillOnce(Return(nullptr));
  EXPECT_CALL(*loaderMock, build_library_name(_)).Times(1)
  .WillOnce(Return("VmbCTest"));
  EXPECT_CALL(*loaderMock, open("VmbCTest")).Times(1)
  .WillOnce(Return(ByMove(std::move(libraryMock))));

  auto api = VmbCAPI::get_instance({}, loaderMock);

  ASSERT_FALSE(api);
}

TEST_F(APILoader, load_sucess)
{
  ASSERT_EQ(setenv("GENICAM_GENTL64_PATH", "", 1), 0);

  auto loaderMock = std::make_shared<MockLibraryLoader>();

  auto apiMock = APIMock::get_instance();
  EXPECT_CALL(*apiMock, Startup(_)).Times(1);
  EXPECT_CALL(*apiMock, Shutdown()).Times(1);

  EXPECT_CALL(*loaderMock, build_library_name(_)).Times(1)
  .WillOnce(Return("VmbCTest"));

  EXPECT_CALL(*loaderMock, open("VmbCTest")).Times(1)
  .WillRepeatedly(
    [](const std::string &) {
      auto libraryMock = std::make_unique<MockLoadedLibrary>();
      EXPECT_CALL(*libraryMock, resolve_symbol(_)).Times(AtLeast(1));
      return libraryMock;
    });


  auto api = VmbCAPI::get_instance({}, loaderMock);

  ASSERT_TRUE(api);
}


TEST_F(APILoader, load_sucess_same_instance)
{
  ASSERT_EQ(setenv("GENICAM_GENTL64_PATH", "", 1), 0);

  auto loaderMock = std::make_shared<MockLibraryLoader>();

  auto apiMock = APIMock::get_instance();
  EXPECT_CALL(*apiMock, Startup(_)).Times(1);
  EXPECT_CALL(*apiMock, Shutdown()).Times(1);

  EXPECT_CALL(*loaderMock, build_library_name(_)).Times(1)
  .WillRepeatedly(Return("VmbCTest"));

  EXPECT_CALL(*loaderMock, open("VmbCTest")).Times(1)
  .WillRepeatedly(
    [](const std::string &) {
      auto libraryMock = std::make_unique<MockLoadedLibrary>();
      EXPECT_CALL(*libraryMock, resolve_symbol(_)).Times(AtLeast(1));
      return libraryMock;
    });

  auto instance1 = VmbCAPI::get_instance({}, loaderMock);

  ASSERT_TRUE(instance1);

  auto instance2 = VmbCAPI::get_instance({}, loaderMock);

  EXPECT_EQ(instance1.get(), instance2.get());

  auto instance3 = VmbCAPI::get_instance();

  EXPECT_EQ(instance1.get(), instance3.get());

  auto instance4 = VmbCAPI::get_instance({}, nullptr);

  EXPECT_EQ(instance1.get(), instance4.get());
}

TEST_F(APILoader, load_destory)
{
  ASSERT_EQ(setenv("GENICAM_GENTL64_PATH", "", 1), 0);

  auto loaderMock = std::make_shared<MockLibraryLoader>();

  auto apiMock = APIMock::get_instance();


  EXPECT_CALL(*loaderMock, build_library_name(_)).Times(2)
  .WillRepeatedly(Return("VmbCTest"));

  EXPECT_CALL(*loaderMock, open("VmbCTest")).Times(2)
  .WillRepeatedly(
    [](const std::string &) {
      auto libraryMock = std::make_unique<MockLoadedLibrary>();
      EXPECT_CALL(*libraryMock, resolve_symbol(_)).Times(AtLeast(1));
      return libraryMock;
    });


  EXPECT_CALL(*apiMock, Startup(_)).Times(1);
  auto instance = VmbCAPI::get_instance({}, loaderMock);

  ASSERT_TRUE(instance);
  EXPECT_TRUE(instance.unique());

  EXPECT_CALL(*apiMock, Shutdown()).Times(1);
  auto instance_ptr = instance.get();

  instance.reset();

  auto null_instance = VmbCAPI::get_instance({}, nullptr);

  ASSERT_FALSE(null_instance);
  EXPECT_EQ(null_instance.get(), nullptr);
  EXPECT_NE(null_instance.get(), instance_ptr);

  EXPECT_CALL(*apiMock, Startup(_)).Times(1);
  auto new_instance = VmbCAPI::get_instance({}, loaderMock);

  ASSERT_TRUE(new_instance);
  EXPECT_TRUE(new_instance.unique());

  EXPECT_CALL(*apiMock, Shutdown()).Times(1);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
