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

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <vimbax_camera/loader/vmbc_api.hpp>

#include "api_mock/api_mock.hpp"

using ::vimbax_camera::LibraryLoader;
using ::vimbax_camera::LoadedLibrary;
using ::vimbax_camera::VmbCAPI;

using ::testing::_;
using ::testing::AtLeast;
using ::testing::Return;
using ::testing::ByMove;

struct MockLibraryLoader : public LibraryLoader
{
  MOCK_METHOD(std::unique_ptr<LoadedLibrary>, open, (const std::string & name), (override));
  MOCK_METHOD(std::string, build_library_name, (const std::string & name), (override));
};

struct MockLoadedLibrary : public LoadedLibrary
{
  MockLoadedLibrary()
  {
    ON_CALL(*this, resolve_symbol).WillByDefault(
      [](const std::string & name) {
        return APIMock::get_function_ptr(name);
      });
  }

  MOCK_METHOD(void *, resolve_symbol, (const std::string & name), (override));
};

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


  auto api = VmbCAPI::get_instance(mock);

  ASSERT_FALSE(api);
}

TEST_F(APILoader, library_load_env)
{
  ASSERT_EQ(setenv("GENICAM_GENTL64_PATH", "", 1), 0);

  auto mock = std::make_shared<MockLibraryLoader>();
  EXPECT_CALL(*mock, build_library_name(_)).Times(1)
  .WillOnce(Return("VmbCTest"));
  EXPECT_CALL(*mock, open("VmbCTest")).Times(1);


  auto api = VmbCAPI::get_instance(mock);

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

  auto api = VmbCAPI::get_instance(loaderMock);

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


  auto api = VmbCAPI::get_instance(loaderMock);

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

  auto instance1 = VmbCAPI::get_instance(loaderMock);

  ASSERT_TRUE(instance1);

  auto instance2 = VmbCAPI::get_instance(loaderMock);

  EXPECT_EQ(instance1.get(), instance2.get());

  auto instance3 = VmbCAPI::get_instance();

  EXPECT_EQ(instance1.get(), instance3.get());

  auto instance4 = VmbCAPI::get_instance(nullptr);

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
  auto instance = VmbCAPI::get_instance(loaderMock);

  ASSERT_TRUE(instance);
  EXPECT_TRUE(instance.unique());

  EXPECT_CALL(*apiMock, Shutdown()).Times(1);
  auto instance_ptr = instance.get();

  instance.reset();

  auto null_instance = VmbCAPI::get_instance(nullptr);

  ASSERT_FALSE(null_instance);
  EXPECT_EQ(null_instance.get(), nullptr);
  EXPECT_NE(null_instance.get(), instance_ptr);

  EXPECT_CALL(*apiMock, Startup(_)).Times(1);
  auto new_instance = VmbCAPI::get_instance(loaderMock);

  ASSERT_TRUE(new_instance);
  EXPECT_TRUE(new_instance.unique());

  EXPECT_CALL(*apiMock, Shutdown()).Times(1);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
