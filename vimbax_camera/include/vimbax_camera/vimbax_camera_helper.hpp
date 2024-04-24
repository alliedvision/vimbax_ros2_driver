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

#ifndef VIMBAX_CAMERA__VIMBAX_CAMERA_HELPER_HPP_
#define VIMBAX_CAMERA__VIMBAX_CAMERA_HELPER_HPP_

#include <string>

#include <rclcpp/rclcpp.hpp>


struct feature_float_info
{
  _Float64 min;
  _Float64 max;
  _Float64 inc;
  bool inc_available;
};

struct feature_flags
{
  bool flag_none;
  bool flag_read;
  bool flag_write;
  bool flag_volatile;
  bool flag_modify_write;
};
struct feature_info
{
  std::string name;
  std::string category;
  std::string display_name;
  std::string sfnc_namespace;
  std::string unit;
  uint32_t data_type;           // Data type corresponding to VmbFeatureDataType
  feature_flags flags;
  uint32_t polling_time;
};

namespace vimbax_camera
{
struct SFNCFeatures
{
  static constexpr std::string_view PixelFormat = "PixelFormat";
  static constexpr std::string_view AcquisitionStart = "AcquisitionStart";
  static constexpr std::string_view AcquisitionStop = "AcquisitionStop";
  static constexpr std::string_view Width = "Width";
  static constexpr std::string_view Height = "Height";
  static constexpr std::string_view TriggerMode = "TriggerMode";
  static constexpr std::string_view TriggerSource = "TriggerSource";
  static constexpr std::string_view TriggerSelector = "TriggerSelector";
  static constexpr std::string_view DeviceFirmwareVersion = "DeviceFirmwareVersion";
  static constexpr std::string_view DeviceUserId = "DeviceUserID";
  static constexpr std::string_view AcquisitionFrameRate = "AcquisitionFrameRate";
  static constexpr std::string_view DeviceTimestampFrequency = "DeviceTimestampFrequency";
  static constexpr std::string_view GVSPAdjustPacketSize = "GVSPAdjustPacketSize";

  static constexpr std::string_view InterfaceId = "InterfaceID";
  static constexpr std::string_view TransportLayerId = "TLID";

  static constexpr std::string_view GevDeviceIPAddress = "GevDeviceIPAddress";
  static constexpr std::string_view GevDeviceMACAddress = "GevDeviceMACAddress";

  static constexpr std::string_view StreamBufferAlignment = "StreamBufferAlignment";

  static constexpr std::string_view EventSelector = "EventSelector";
  static constexpr std::string_view EventNotification = "EventNotification";
  static constexpr std::string_view EventCameraDiscovery = "EventCameraDiscovery";
  static constexpr std::string_view EventCameraDiscoveryCameraID = "EventCameraDiscoveryCameraID";
  static constexpr std::string_view EventCameraDiscoveryType = "EventCameraDiscoveryType";
};
}  // namespace vimbax_camera

namespace vimbax_camera::helper
{

std::string_view vmb_error_to_string(int32_t error_code);

rclcpp::Logger get_logger();

rclcpp::Node::SharedPtr create_node(const std::string & name, const rclcpp::NodeOptions & options);

void left_shift16(void * out, const void * in, size_t size, int shift);
}  // namespace vimbax_camera::helper

#endif  // VIMBAX_CAMERA__VIMBAX_CAMERA_HELPER_HPP_
