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

#if defined(__x86_64__) && defined(__GNUC__)
#include <immintrin.h>
#define ATTRIBUTE_TARGET(tgt) __attribute__((target(#tgt)))
#define USE_X86_SIMD 1
#elif defined(__aarch64__) && defined(__GNUC__)
#include <arm_neon.h>
#define ATTRIBUTE_TARGET(tgt)
#define USE_AARCH64_SIMD 1
#else
#define ATTRIBUTE_TARGET(tgt)
#endif

#include <VmbC/VmbCommonTypes.h>

#include <vimbax_camera/vimbax_camera_helper.hpp>


namespace vimbax_camera::helper
{
static rclcpp::Node::WeakPtr g_activeNode{};

rclcpp::Logger get_logger()
{
  if (!g_activeNode.expired()) {
    return g_activeNode.lock()->get_logger();
  }

  return rclcpp::get_logger("vimbax_camera_fb");
}

rclcpp::Node::SharedPtr create_node(const std::string & name, const rclcpp::NodeOptions & options)
{
  auto node = rclcpp::Node::make_shared(name, options);
  g_activeNode = node;
  return node;
}

static void left_shift16_default(void * out, const void * in, ssize_t size, const int shift)
{
  auto u16in = reinterpret_cast<const uint16_t *>(in);
  auto u16out = reinterpret_cast<uint16_t *>(out);
  for (uint32_t i = 0; i < size / 2; i++) {
    *u16out = (*u16in << shift);
    u16in++;
    u16out++;
  }
}

#ifdef USE_AARCH64_SIMD
void left_shift16(void * out, const void * in, size_t size, int shift)
{
  int16x8_t vecShift = {int16_t(shift), int16_t(shift), int16_t(shift), int16_t(shift),
    int16_t(shift), int16_t(shift), int16_t(shift), int16_t(shift)};

  auto div = ldiv(size, 16);
  auto u128in = reinterpret_cast<const uint16x8_t *>(in);
  auto u128out = reinterpret_cast<uint16x8_t *>(out);
  for (uint32_t i = 0; i < div.quot; i++) {
    *u128out = vshlq_u16(*u128in, vecShift);
    u128in++;
    u128out++;
  }

  if (div.quot != 0) {
    left_shift16_default(u128out, u128in, div.rem, shift);
  }
}
#else
ATTRIBUTE_TARGET(default)
void left_shift16(void * out, const void * in, size_t size, int shift)
{
  left_shift16_default(out, in, size, shift);
}
#endif

#ifdef USE_X86_SIMD
ATTRIBUTE_TARGET(avx2)
void left_shift16(void * out, const void * in, ssize_t size, const int shift)
{
  auto div = ldiv(size, 32);
  auto u8in = reinterpret_cast<const uint8_t *>(in);
  auto u8out = reinterpret_cast<uint8_t *>(out);
  for (uint32_t i = 0; i < div.quot; i++) {
    __m256i value;
    memcpy(&value, u8in, 32);
    auto res = _mm256_slli_epi16(value, shift);
    memcpy(u8out, &res, 32);
    u8in += 32;
    u8out += 32;
  }

  if (div.quot != 0) {
    left_shift16_default(u8out, u8in, div.rem, shift);
  }
}

ATTRIBUTE_TARGET(sse2)
void left_shift16(void * out, const void * in, ssize_t size, const int shift)
{
  auto div = ldiv(size, 16);
  auto u128in = reinterpret_cast<const __m128i *>(in);
  auto u128out = reinterpret_cast<__m128i *>(out);
  for (uint32_t i = 0; i < div.quot; i++) {
    *u128out = _mm_slli_epi16(*u128in, shift);
    u128in++;
    u128out++;
  }

  if (div.quot != 0) {
    left_shift16_default(u128out, u128in, div.rem, shift);
  }
}
#endif


std::string_view vmb_error_to_string(int32_t error_code)
{
  switch (error_code) {
    case VmbErrorSuccess:
      return "VmbErrorSuccess";
    case VmbErrorInternalFault:
      return "VmbErrorInternalFault";
    case VmbErrorApiNotStarted:
      return "VmbErrorApiNotStarted";
    case VmbErrorNotFound:
      return "VmbErrorNotFound";
    case VmbErrorBadHandle:
      return "VmbErrorBadHandle";
    case VmbErrorDeviceNotOpen:
      return "VmbErrorDeviceNotOpen";
    case VmbErrorInvalidAccess:
      return "VmbErrorInvalidAccess";
    case VmbErrorBadParameter:
      return "VmbErrorBadParameter";
    case VmbErrorStructSize:
      return "VmbErrorStructSize";
    case VmbErrorMoreData:
      return "VmbErrorMoreData";
    case VmbErrorWrongType:
      return "VmbErrorWrongType";
    case VmbErrorInvalidValue:
      return "VmbErrorInvalidValue";
    case VmbErrorTimeout:
      return "VmbErrorTimeout";
    case VmbErrorOther:
      return "VmbErrorOther";
    case VmbErrorResources:
      return "VmbErrorResources";
    case VmbErrorInvalidCall:
      return "VmbErrorInvalidCall";
    case VmbErrorNoTL:
      return "VmbErrorNoTL";
    case VmbErrorNotImplemented:
      return "VmbErrorNotImplemented";
    case VmbErrorNotSupported:
      return "VmbErrorNotSupported";
    case VmbErrorIncomplete:
      return "VmbErrorIncomplete";
    case VmbErrorIO:
      return "VmbErrorIO";
    case VmbErrorValidValueSetNotPresent:
      return "VmbErrorValidValueSetNotPresent";
    case VmbErrorGenTLUnspecified:
      return "VmbErrorGenTLUnspecified";
    case VmbErrorUnspecified:
      return "VmbErrorUnspecified";
    case VmbErrorBusy:
      return "VmbErrorBusy";
    case VmbErrorNoData:
      return "VmbErrorNoData";
    case VmbErrorParsingChunkData:
      return "VmbErrorParsingChunkData";
    case VmbErrorInUse:
      return "VmbErrorInUse";
    case VmbErrorUnknown:
      return "VmbErrorUnknown";
    case VmbErrorXml:
      return "VmbErrorXml";
    case VmbErrorNotAvailable:
      return "VmbErrorNotAvailable";
    case VmbErrorNotInitialized:
      return "VmbErrorNotInitialized";
    case VmbErrorInvalidAddress:
      return "VmbErrorInvalidAddress";
    case VmbErrorAlready:
      return "VmbErrorAlready";
    case VmbErrorNoChunkData:
      return "VmbErrorNoChunkData";
    case VmbErrorUserCallbackException:
      return "VmbErrorUserCallbackException";
    case VmbErrorFeaturesUnavailable:
      return "VmbErrorFeaturesUnavailable";
    case VmbErrorTLNotFound:
      return "VmbErrorTLNotFound";
    case VmbErrorAmbiguous:
      return "VmbErrorAmbiguous";
    case VmbErrorRetriesExceeded:
      return "VmbErrorRetriesExceeded";
    case VmbErrorInsufficientBufferCount:
      return "VmbErrorInsufficientBufferCount";
    default:
      return "VmbErrorCustom";
  }
}

}  // namespace vimbax_camera::helper
