# Copyright 2024 Allied Vision Technologies GmbH. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# ROS client lib
import rclpy
from rclpy.service import Service
from sensor_msgs.msg import Image

from time import sleep

# pytest libs
import pytest

# VimbaX_Camera msgs
from vimbax_camera_msgs.srv import FeatureEnumInfoGet, FeatureEnumSet, StreamStartStop
from test_helper import check_error

from conftest import vimbax_camera_node, TestNode

from typing import List


import logging
LOGGER = logging.getLogger(__name__)

# The required formats are listed in requirement UNIRT-1118
REQUIRED_PIXEL_FORMATS = {
    "Mono8",
    "Mono16",
    "Mono12",
    "BGR8",
    "RGB8",
    "BayerBG8",
    "BayerGB8",
    "BayerRG8",
    "BayerRG16",
    "BayerRG12",
    "BayerRG10",
    "BayerBG16",
    "BayerBG12",
    "BayerBG10",
    "BayerGB16",
    "BayerGB12",
    "BayerGB10",
    "BayerGR16",
    "BayerGR12",
    "BayerGR10",
    "YCBCR422_8",
}

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
    "YCbCr422_8": "yuv422",
}


@pytest.fixture(autouse=True)
def init_and_shutdown_ros():
    rclpy.init()

    # The test is run here
    yield

    rclpy.shutdown()


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

        # Magic timeout value
        assert self.__enum_info_get_srv.wait_for_service(timeout_sec=self.__rcl_timeout_sec)
        assert self.__enum_set_srv.wait_for_service(timeout_sec=self.__rcl_timeout_sec)
        assert self.__stream_start_srv.wait_for_service(timeout_sec=self.__rcl_timeout_sec)
        assert self.__stream_stop_srv.wait_for_service(timeout_sec=self.__rcl_timeout_sec)

    def __call_service_sync(self, srv: Service, request):
        return srv.call(request)

    def stop_stream(self) -> StreamStartStop.Response:
        return self.__call_service_sync(self.__stream_stop_srv, StreamStartStop.Request())

    def start_stream(self) -> StreamStartStop.Response:
        return self.__call_service_sync(self.__stream_start_srv, StreamStartStop.Request())

    def get_supported_pixel_formats(self) -> List[str]:
        """Receives the list of available pixel formats from the camera."""
        req: FeatureEnumInfoGet.Request = FeatureEnumInfoGet.Request()
        req.feature_name = "PixelFormat"
        res: FeatureEnumInfoGet.Response = self.__call_service_sync(self.__enum_info_get_srv, req)
        check_error(res.error)

        return res.available_values

    def set_pixel_format(self, format: str) -> FeatureEnumSet.Response:
        """Set the pixel format published by the camera."""
        req: FeatureEnumSet.Request = FeatureEnumSet.Request()
        req.feature_name = "PixelFormat"
        req.value = format
        return self.__call_service_sync(self.__enum_set_srv, req)

    def get_latest_image(self) -> Image:
        """Spins the default context until an Image is received from the Camera."""
        return self.wait_for_frame(self.__rcl_timeout_sec)


@pytest.mark.launch(fixture=vimbax_camera_node)
def test_pixel_formats(launch_context, node_test_id, camera_test_node_name):

    node: PixelFormatTestNode = PixelFormatTestNode(
        f"pytest_client_node_{node_test_id}", camera_test_node_name, timeout_sec=5.0)

    # We can only test the formats required and supported by the attached camera
    available_formats = REQUIRED_PIXEL_FORMATS.intersection(
        set(node.get_supported_pixel_formats())
    )

    LOGGER.info(f"Available Pixel Formats: {available_formats}")

    for format in available_formats:
        # Set the pixel format
        LOGGER.info(f"Testing format: {format}")

        check_error(node.set_pixel_format(format).error)
        node.clear_queue()
        node.subscribe_image_raw()

        image: Image = node.get_latest_image()

        node.unsubscribe_image_raw()
        # Assert the pixel format of the image matches the requested format
        assert image is not None
        # Because the ROS and PFNC formats differ in naming the encoding needs to be translated
        assert image.encoding == PFNC_TO_ROS[format]
        sleep(1)


@pytest.mark.launch(fixture=vimbax_camera_node)
def test_invalid_pixel_format(launch_context, node_test_id, camera_test_node_name):

    node: PixelFormatTestNode = PixelFormatTestNode(
        f"pytest_client_node_{node_test_id}", camera_test_node_name, timeout_sec=5.0)
    node.stop_stream()
    # This should fail
    res = node.set_pixel_format("")
    error_msg: str = "Unexpected error: {} ({}); Expected -11 (VmbErrorInvalidValue)"
    assert res.error.code == -11,  error_msg.format(res.error.code, res.error.text)
