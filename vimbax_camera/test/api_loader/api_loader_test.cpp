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

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <vimbax_camera/loader/vmbc_api.hpp>

using ::vimbax_camera::LibraryLoader;
using ::vimbax_camera::LoadedLibrary;
using ::vimbax_camera::VmbCAPI;

using ::testing::_;
using ::testing::AtLeast;
using ::testing::Return;
using ::testing::ByMove;

VmbError_t VmbStartupMock(const VmbFilePathChar_t * path)
{
  if (path == nullptr) {
    return -2;
  }

  std::string_view view{path};
  if (view == "ok") {
    return 0;
  } else if (view == "error") {
    return -1;
  } else {
    return -3;
  }
}

void VmbShutdownMock()
{
}

static const std::map<std::string, void *> apiFuncMap = {
  {"VmbStartup", reinterpret_cast<void *>(&VmbStartupMock)},
  {"VmbShutdown", reinterpret_cast<void *>(&VmbShutdownMock)},
};

struct MockLibraryLoader : public LibraryLoader
{
  MOCK_METHOD(std::unique_ptr<LoadedLibrary>, open, (const std::string & name), (override));
  MOCK_METHOD(std::string, build_library_name, (const std::string & name), (override));
};

struct MockLoadedLibrary : public LoadedLibrary
{
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


  auto api = VmbCAPI::load(mock);

  ASSERT_FALSE(api);
}

TEST_F(APILoader, library_load_env)
{
  ASSERT_EQ(setenv("GENICAM_GENTL64_PATH", "", 1), 0);

  auto mock = std::make_shared<MockLibraryLoader>();
  EXPECT_CALL(*mock, build_library_name(_)).Times(1)
  .WillOnce(Return("VmbCTest"));
  EXPECT_CALL(*mock, open("VmbCTest")).Times(1);


  auto api = VmbCAPI::load(mock);

  ASSERT_FALSE(api);
}

TEST_F(APILoader, symbol_load_fail)
{
  ASSERT_EQ(setenv("GENICAM_GENTL64_PATH", "", 1), 0);

  auto loaderMock = std::make_shared<MockLibraryLoader>();
  auto libraryMock = std::make_unique<MockLoadedLibrary>();
  EXPECT_CALL(*libraryMock, resolve_symbol(_)).Times(1);
  EXPECT_CALL(*loaderMock, build_library_name(_)).Times(1)
  .WillOnce(Return("VmbCTest"));
  EXPECT_CALL(*loaderMock, open("VmbCTest")).Times(1)
  .WillOnce(Return(ByMove(std::move(libraryMock))));

  auto api = VmbCAPI::load(loaderMock);

  ASSERT_FALSE(api);
}

TEST_F(APILoader, load_sucess_startup_shutdown)
{
  ASSERT_EQ(setenv("GENICAM_GENTL64_PATH", "", 1), 0);

  int dummy{};

  auto loaderMock = std::make_shared<MockLibraryLoader>();
  auto libraryMock = std::make_unique<MockLoadedLibrary>();

  EXPECT_CALL(*libraryMock, resolve_symbol(_)).Times(AtLeast(1))
  .WillRepeatedly(
    [&](auto & name) -> void * {
      try {
        return apiFuncMap.at(name);
      } catch (...) {
        return &dummy;
      }
    });

  EXPECT_CALL(*loaderMock, build_library_name(_)).Times(1)
  .WillOnce(Return("VmbCTest"));

  EXPECT_CALL(*loaderMock, open("VmbCTest")).Times(1)
  .WillOnce(Return(ByMove(std::move(libraryMock))));

  auto api = VmbCAPI::load(loaderMock);

  ASSERT_TRUE(api);

  EXPECT_TRUE(api->Startup);
  EXPECT_TRUE(api->Shutdown);


  EXPECT_EQ(api->Startup.raw, &VmbStartupMock);
  EXPECT_EQ(api->Shutdown.raw, &VmbShutdownMock);

  EXPECT_EQ(api->Startup("ok"), 0);
  EXPECT_EQ(api->Startup("error"), -1);
  EXPECT_EQ(api->Startup(nullptr), -2);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
