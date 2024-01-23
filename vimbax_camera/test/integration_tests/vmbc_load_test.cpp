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

#include <filesystem>

#include <vimbax_camera/loader/vmbc_api.hpp>

using vimbax_camera::VmbCAPI;

namespace fs = std::filesystem;

class VmbCAPITest : public testing::Test
{
  void SetUp() override
  {
    auto gentl_path = getenv("GENICAM_GENTL64_PATH");
    if (gentl_path != nullptr) {
      gentl_path_ = gentl_path;
      setenv("GENICAM_GENTL64_PATH", "", 1);
      std::cout << "GENICAM_GENTL64_PATH" << getenv("GENICAM_GENTL64_PATH") << std::endl;
    }

    auto vimbax_home = getenv("VIMBA_X_HOME");
    if (vimbax_home != nullptr) {
      vimbax_home_ = vimbax_home;
      setenv("VIMBA_X_HOME", "", 1);
      std::cout << "VIMBA_X_HOME" << getenv("VIMBA_X_HOME") << std::endl;
    }

    if (vimbax_home_.empty()) {
      GTEST_SKIP() << "Environment variable VIMBA_X_HOME not set";
    }

    if (!fs::exists(vimbax_home_)) {
      GTEST_SKIP() << "Environment variable VIMBA_X_HOME pointing to not existing directory";
    }

    if (gentl_path_.empty()) {
      gentl_path_ = vimbax_home_ / "cti";
    }

    Test::SetUp();
  }
  void TearDown() override
  {
    Test::TearDown();

    setenv("GENICAM_GENTL64_PATH", gentl_path_.c_str(), 1);
    setenv("VIMBA_X_HOME", vimbax_home_.c_str(), 1);
  }

protected:
  std::string gentl_path_;
  fs::path vimbax_home_;
};

TEST_F(VmbCAPITest, load_by_vimabx_home)
{
  setenv("VIMBA_X_HOME", vimbax_home_.c_str(), 1);

  // API loading should fail, because GENICAM_GENTL64_PATH is not set
  auto api_fail = VmbCAPI::get_instance();

  EXPECT_FALSE(api_fail);

  auto api_ok = VmbCAPI::get_instance(gentl_path_);

  EXPECT_TRUE(api_ok);

  EXPECT_TRUE(api_ok->VersionQuery);
}

TEST_F(VmbCAPITest, load_by_gentl_path)
{
  setenv("GENICAM_GENTL64_PATH", (vimbax_home_ / "cti").c_str(), 1);

  auto api_ok = VmbCAPI::get_instance();

  EXPECT_TRUE(api_ok);

  EXPECT_TRUE(api_ok->VersionQuery);
}

TEST_F(VmbCAPITest, load_by_library_path)
{
  // API loading should fail, because GENICAM_GENTL64_PATH is not set
  auto api_fail = VmbCAPI::get_instance();

  EXPECT_FALSE(api_fail);

  auto api_ok = VmbCAPI::get_instance(gentl_path_);

  EXPECT_TRUE(api_ok);

  EXPECT_TRUE(api_ok->VersionQuery);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
