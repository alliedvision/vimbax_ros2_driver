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
