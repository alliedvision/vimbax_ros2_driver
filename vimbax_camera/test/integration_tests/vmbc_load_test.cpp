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
