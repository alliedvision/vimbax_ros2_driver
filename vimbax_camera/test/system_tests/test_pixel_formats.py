# Copyright (c) 2024 Allied Vision Technologies GmbH. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Allied Vision Technologies GmbH nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


# ROS client lib
import rclpy
from rclpy.service import Service
from rclpy.time import Time
from sensor_msgs.msg import Image

from time import sleep

# pytest libs
import pytest
import launch_pytest
import launch_ros
from launch import LaunchDescription
from launch.actions import ExecuteProcess

# VimbaX_Camera msgs
from vimbax_camera_msgs.srv import (
    FeatureEnumInfoGet,
    FeatureEnumSet,
    StreamStartStop,
    FeatureCommandRun,
)
from test_helper import check_error

from typing import List
from conftest import TestNode


import logging

LOGGER = logging.getLogger()

# The required formats are listed in requirement UNIRT-1118
REQUIRED_PIXEL_FORMATS = [
    "Mono8",
    "Mono12",
    "Mono16",
    "RGB8",
    "BGR8",
    "BayerRG16",
    "BayerRG12",
    "BayerRG10",
    "BayerRG8",
    "BayerBG16",
    "BayerBG12",
    "BayerBG10",
    "BayerBG8",
    "BayerGB16",
    "BayerGB12",
    "BayerGB10",
    "BayerGB8",
    "BayerGR16",
    "BayerGR12",
    "BayerGR10",
    "BayerGR8",
    "YCbCr422_8_CbYCrY",
]

PFNC_TO_ROS = {
    "Mono8": "mono8",
    "Mono16": "mono16",
    "Mono12": "mono16",
    "BGR8": "bgr8",
    "RGB8": "rgb8",
    "BayerBG8": "bayer_bggr8",
    "BayerGB8": "bayer_gbrg8",
    "BayerRG8": "bayer_rggb8",
    "BayerGR8": "bayer_grbg8",
    "BayerRG16": "bayer_rggb16",
    "BayerRG12": "bayer_rggb16",
    "BayerRG10": "bayer_rggb16",
    "BayerBG16": "bayer_bggr16",
    "BayerBG12": "bayer_bggr16",
    "BayerBG10": "bayer_bggr16",
    "BayerGB16": "bayer_gbrg16",
    "BayerGB12": "bayer_gbrg16",
    "BayerGB10": "bayer_gbrg16",
    "BayerGR16": "bayer_grbg16",
    "BayerGR12": "bayer_grbg16",
    "BayerGR10": "bayer_grbg16",
    "YCbCr422_8_CbYCrY": "yuv422",
}


class PixelFormatTestNode(TestNode):
    """Custom ROS2 Node to make testing easier."""

    def __init__(self, name: str, test_node_name: str, timeout_sec: float = 10.0):
        super().__init__(name, test_node_name)
        self.__rcl_timeout_sec = float(timeout_sec)
        self.__enum_info_get_srv: Service = self.create_client(
            srv_type=FeatureEnumInfoGet, srv_name=f"/{test_node_name}/features/enum_info_get"
        )
        self.__enum_set_srv: Service = self.create_client(
            srv_type=FeatureEnumSet, srv_name=f"/{test_node_name}/features/enum_set"
        )
        self.__stream_start_srv: Service = self.create_client(
            srv_type=StreamStartStop, srv_name=f"/{test_node_name}/stream_start"
        )
        self.__stream_stop_srv: Service = self.create_client(
            srv_type=StreamStartStop, srv_name=f"/{test_node_name}/stream_stop"
        )

        self.subscribe_image_raw()

        # Magic timeout value
        assert self.__enum_info_get_srv.wait_for_service(timeout_sec=self.__rcl_timeout_sec)
        assert self.__enum_set_srv.wait_for_service(timeout_sec=self.__rcl_timeout_sec)
        assert self.__stream_start_srv.wait_for_service(timeout_sec=self.__rcl_timeout_sec)
        assert self.__stream_stop_srv.wait_for_service(timeout_sec=self.__rcl_timeout_sec)

    def stop_stream(self) -> StreamStartStop.Response:
        return self.call_service_sync(self.__stream_stop_srv, StreamStartStop.Request())

    def start_stream(self) -> StreamStartStop.Response:
        return self.call_service_sync(self.__stream_start_srv, StreamStartStop.Request())

    def get_supported_pixel_formats(self) -> List[str]:
        """Receives the list of available pixel formats from the camera."""
        req: FeatureEnumInfoGet.Request = FeatureEnumInfoGet.Request()
        req.feature_name = "PixelFormat"
        res: FeatureEnumInfoGet.Response = self.call_service_sync(self.__enum_info_get_srv, req)
        check_error(res.error)

        return res.available_values

    def set_pixel_format(self, format: str) -> FeatureEnumSet.Response:
        """Set the pixel format published by the camera."""
        req: FeatureEnumSet.Request = FeatureEnumSet.Request()
        req.feature_name = "PixelFormat"
        req.value = format
        return self.call_service_sync(self.__enum_set_srv, req)

    def get_latest_image(self) -> Image:
        self.clear_queue()
        return self.wait_for_frame(timeout=self.__rcl_timeout_sec)


@pytest.fixture(scope="class")
def pixel_test_node(launch_context):
    rclpy.init()

    test_node: PixelFormatTestNode = PixelFormatTestNode(
        "pytest_client_node", "test_pixel_formats", timeout_sec=10.0
    )

    enum_set_client = test_node.create_client(
        FeatureEnumSet, "/test_pixel_formats/features/enum_set"
    )
    command_run_client = test_node.create_client(
        FeatureCommandRun, "/test_pixel_formats/features/command_run"
    )
    enum_set_client.wait_for_service(10)
    command_run_client.wait_for_service(10)

    test_node.call_service_sync(
        enum_set_client,
        FeatureEnumSet.Request(feature_name="UserSetSelector", value="UserSetDefault"),
    )

    test_node.call_service_sync(
        command_run_client, FeatureCommandRun.Request(feature_name="UserSetLoad")
    )

    yield test_node

    rclpy.shutdown()


@launch_pytest.fixture(scope="class")
def vimbax_camera_node_class_scope():
    return LaunchDescription(
        [
            ExecuteProcess(
                cmd=["ros2", "node", "list", "--all"],
                shell=True,
                output="both",
            ),
            launch_ros.actions.Node(
                package="vimbax_camera",
                namespace="/test_pixel_formats",
                executable="vimbax_camera_node",
                parameters=[{
                    "use_ros_time": True
                }],
                name="test_pixel_formats",
            ),
            launch_pytest.actions.ReadyToTest(),
        ]
    )


@pytest.mark.launch(fixture=vimbax_camera_node_class_scope)
class TestPixelFormat:
    """One VimbaXCamera node is started for all tests."""

    @pytest.mark.parametrize("format", REQUIRED_PIXEL_FORMATS)
    def test_format(self, format, launch_context, pixel_test_node: PixelFormatTestNode):

        # The PixelFormat cannot be changed while the camera is streaming
        check_error(pixel_test_node.stop_stream().error)

        # We can only test the formats required and supported by the attached camera
        if not (format in pixel_test_node.get_supported_pixel_formats()):
            pytest.skip(f"{format} is not supported by current camera")
            return

        # Set the pixel format
        LOGGER.info(f"Testing format: {format}")

        check_error(pixel_test_node.set_pixel_format(format).error)
        check_error(pixel_test_node.start_stream().error)

        # Discard images that were taken before the settings change
        ts = pixel_test_node.get_clock().now()
        image: Image = pixel_test_node.get_latest_image()
        while image is not None and Time.from_msg(image.header.stamp).nanoseconds < ts.nanoseconds:
            image = pixel_test_node.get_latest_image()

        check_error(pixel_test_node.stop_stream().error)
        # Assert the pixel format of the image matches the requested format
        assert image is not None
        # Because the ROS and PFNC formats differ in naming the encoding needs to be translated
        assert image.encoding == PFNC_TO_ROS[format]
        sleep(1)

    def test_invalid_value(self, launch_context, pixel_test_node):

        pixel_test_node.stop_stream()
        # This should fail
        res = pixel_test_node.set_pixel_format("")
        error_msg: str = "Unexpected error: {} ({}); Expected -11 (VmbErrorInvalidValue)"
        assert res.error.code == -11, error_msg.format(res.error.code, res.error.text)
