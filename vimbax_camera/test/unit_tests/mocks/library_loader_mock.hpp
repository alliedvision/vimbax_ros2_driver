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

#ifndef UNIT_TESTS__MOCKS__LIBRARY_LOADER_MOCK_HPP_
#define UNIT_TESTS__MOCKS__LIBRARY_LOADER_MOCK_HPP_

#include <gmock/gmock.h>

#include <string>
#include <memory>

#include <vimbax_camera/loader/vmbc_api.hpp>

#include "api_mock.hpp"

using ::vimbax_camera::LibraryLoader;
using ::vimbax_camera::LoadedLibrary;

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

#endif  // UNIT_TESTS__MOCKS__LIBRARY_LOADER_MOCK_HPP_
