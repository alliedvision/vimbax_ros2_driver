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

#include <vimbax_camera/loader/library_loader.hpp>

using ::vimbax_camera::LibraryLoader;
using ::vimbax_camera::LoadedLibrary;

TEST(library_loader, single_default_instance)
{
#ifdef __unix
  auto loader = LibraryLoader::get_default();
  ASSERT_TRUE(loader);

  auto loader2 = LibraryLoader::get_default();
  ASSERT_TRUE(loader2);

  ASSERT_EQ(loader.get(), loader2.get());
#endif
}

TEST(library_loader, build_library_name)
{
#ifdef __unix
  auto loader = LibraryLoader::get_default();
  ASSERT_TRUE(loader);

  ASSERT_EQ(loader->build_library_name("test"), "libtest.so");
#endif
}

TEST(library_loader, open_error_by_name)
{
  auto loader = LibraryLoader::get_default();
  ASSERT_TRUE(loader);

  auto const fullLibraryName = loader->build_library_name("Missing");
  auto const library = loader->open(fullLibraryName);

  ASSERT_FALSE(library);
}

TEST(library_loader, open_error_by_path)
{
  auto loader = LibraryLoader::get_default();
  ASSERT_TRUE(loader);

  auto const fullLibraryName = loader->build_library_name("Missing");
  auto const library = loader->open("./" + fullLibraryName);

  ASSERT_FALSE(library);
}

TEST(library_loader, open_by_path)
{
  auto loader = LibraryLoader::get_default();
  ASSERT_TRUE(loader);

  auto const fullLibraryName = loader->build_library_name("test_lib");
  auto const library = loader->open("./" + fullLibraryName);

  ASSERT_TRUE(library);
}

TEST(library_loader, open_by_name)
{
  auto loader = LibraryLoader::get_default();
  ASSERT_TRUE(loader);

  auto const fullLibraryName = loader->build_library_name("test_lib");
  auto const library = loader->open(fullLibraryName);

  ASSERT_TRUE(library);
}

TEST(library_loader, invalid_symbol)
{
  auto loader = LibraryLoader::get_default();
  ASSERT_TRUE(loader);

  auto const fullLibraryName = loader->build_library_name("test_lib");
  auto const library = loader->open(fullLibraryName);

  ASSERT_TRUE(library);

  auto const symbol = library->resolve_symbol("invalid");

  ASSERT_EQ(symbol, nullptr);
}

TEST(library_loader, valid_symbol)
{
  auto loader = LibraryLoader::get_default();
  ASSERT_TRUE(loader);

  auto const fullLibraryName = loader->build_library_name("test_lib");
  auto const library = loader->open(fullLibraryName);

  ASSERT_TRUE(library);

  auto const symbol = library->resolve_symbol("test_add_func");

  ASSERT_NE(symbol, nullptr);
}

TEST(library_loader, test_func_call)
{
  auto loader = LibraryLoader::get_default();
  ASSERT_TRUE(loader);

  auto const fullLibraryName = loader->build_library_name("test_lib");
  auto const library = loader->open(fullLibraryName);

  ASSERT_TRUE(library);

  auto const symbol = library->resolve_symbol("test_add_func");

  ASSERT_NE(symbol, nullptr);

  auto const test_func = reinterpret_cast<int (*)(int, int)>(symbol);

  ASSERT_NE(test_func, nullptr);

  unsigned int seed = time(nullptr);

  int const i1 = rand_r(&seed), i2 = rand_r(&seed);

  auto const result = test_func(i1, i2);

  EXPECT_EQ(result, i1 + i2);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
