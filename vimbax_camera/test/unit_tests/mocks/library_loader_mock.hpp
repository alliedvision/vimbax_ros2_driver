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
