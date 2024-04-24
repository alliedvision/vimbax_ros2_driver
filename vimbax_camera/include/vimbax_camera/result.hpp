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
