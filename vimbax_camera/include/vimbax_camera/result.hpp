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

#ifndef VIMBAX_CAMERA__RESULT_HPP_
#define VIMBAX_CAMERA__RESULT_HPP_

#include <cstdint>
#include <variant>
#include <utility>
#include <string>

#include <vimbax_camera/vimbax_camera_helper.hpp>


#include <vimbax_camera_msgs/msg/error.hpp>


namespace vimbax_camera
{
struct error
{
  int32_t code;

  vimbax_camera_msgs::msg::Error to_error_msg() const
  {
    return vimbax_camera_msgs::msg::Error().set__code(code)
           .set__text(std::string{vimbax_camera::helper::vmb_error_to_string(code)});
  }
};

template<typename T>
class result
{
public:
  result(const T & value)  // NOLINT
  : value_error_{value}
  {
  }

  result(const T && value)  // NOLINT
  : value_error_{std::move(value)}
  {
  }

  result(const error & error)  // NOLINT
  : value_error_{error}
  {
  }

  operator bool() const
  {
    return std::holds_alternative<T>(value_error_);
  }

  const T & operator*() const
  {
    return std::get<T>(value_error_);
  }

  const T * operator->() const
  {
    return &std::get<T>(value_error_);
  }

  const vimbax_camera::error & error() const
  {
    return std::get<vimbax_camera::error>(value_error_);
  }

private:
  std::variant<T, vimbax_camera::error> value_error_;
};

template<>
class result<void>
{
public:
  result()  // NOLINT
  : error_{std::nullopt}
  {
  }

  result(const vimbax_camera::error & error)  // NOLINT
  : error_{error}
  {
  }

  operator bool() const
  {
    return !error_;
  }

  const vimbax_camera::error & error() const
  {
    return *error_;
  }

private:
  std::optional<vimbax_camera::error> error_;
};
}  // namespace vimbax_camera
#endif  // VIMBAX_CAMERA__RESULT_HPP_
