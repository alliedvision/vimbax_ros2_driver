# ROS client lib
import rclpy
from rclpy.node import Node
from rclpy.service import Service
from rclpy import Future
from rclpy.subscription import Subscription
import launch
from launch_ros import actions
from sensor_msgs.msg import Image

# pytest libs
import pytest
import launch_pytest

# VimbaX_Camera msgs
from vimbax_camera_msgs.srv import FeatureEnumInfoGet, FeatureEnumSet, StreamStartStop

from typing import List


import logging
LOGGER = logging.getLogger(__name__)

# The required formats are listed in requirement UNIRT-1118
REQUIRED_PIXEL_FORMATS = {
        'Mono8', 
        'Mono16', 
        'Mono12', 
        'BGR8', 
        'RGB8', 
        'BayerBG8',
        'BayerGB8',
        'BayerRG8',
        'BayerRG16',
        'BayerRG12',
        'BayerRG10',
        'BayerBG16',
        'BayerBG12',
        'BayerBG10',
        'BayerGB16',
        'BayerGB12',
        'BayerGB10',
        'BayerGR16',
        'BayerGR12',
        'BayerGR10',
        'YCBCR422_8'}

PFNC_TO_ROS = {
        'Mono8' : 'mono8', 
        'Mono16': 'mono16', 
        'Mono12': 'mono16', 
        'BGR8'  : 'bgr8', 
        'RGB8'  : 'rgb8', 
        'BayerBG8'  : 'bayer_bggr8',
        'BayerGB8'  : 'bayer_gbrg8',
        'BayerRG8'  : 'bayer_rggb8', #TODO: Maybe add bayer_grbg8 after requirement clarification
        'BayerRG16' : 'bayer_rggb16',
        'BayerRG12' : 'bayer_rggb16',
        'BayerRG10' : 'bayer_rggb16',
        'BayerBG16' : 'bayer_bggr16',
        'BayerBG12' : 'bayer_bggr16',
        'BayerBG10' : 'bayer_bggr16',
        'BayerGB16' : 'bayer_gbrg16',
        'BayerGB12' : 'bayer_gbrg16',
        'BayerGB10' : 'bayer_gbrg16',
        'BayerGR16' : 'bayer_grbg16',
        'BayerGR12' : 'bayer_grbg16',
        'BayerGR10' : 'bayer_grbg16',
        'YCBCR422_8': 'yuv422'}



test_node_name: str = "vimbax_camera_pytest"

# Fixture to launch the vimbax_camera_node
@launch_pytest.fixture
def vimbax_camera_node(): 
    """Launch the vimbax_camera_node """
    
    return launch.LaunchDescription([
        actions.Node(
            package='vimbax_camera',
            # namespace='avt_vimbax',
            executable='vimbax_camera_node',
            name=test_node_name
        ),
        # Tell launch when to start the test
        # If no ReadyToTest action is added, one will be appended automatically.
        launch_pytest.actions.ReadyToTest()
    ])

@pytest.fixture(autouse=True)
def init_and_shutdown_ros():
    rclpy.init()
    
    # The test is run here
    yield

    rclpy.shutdown()

class PixelFormatTestNode(Node):
    """ Custom ROS2 Node to make testing easier """
    def __init__(self, name: str, timeout_sec: float = 10.0):
        super().__init__(name)
        self.__rcl_timeout_sec = float(timeout_sec)
        self.__enum_info_get_srv: Service = self.create_client(srv_type=FeatureEnumInfoGet, srv_name=f"/{test_node_name}/features/enum_info_get")
        self.__enum_set_srv: Service = self.create_client(srv_type=FeatureEnumSet, srv_name=f"/{test_node_name}/features/enum_set")
        self.__stream_start_srv: Service = self.create_client(srv_type=StreamStartStop, srv_name=f"/{test_node_name}/stream_start")
        self.__stream_stop_srv: Service = self.create_client(srv_type=StreamStartStop, srv_name=f"/{test_node_name}/stream_stop")
        self.__image_future: Future = Future()
        self.__image_sub: Subscription = self.create_subscription(Image, f"/{test_node_name}/image_raw", lambda msg: self.__image_future.set_result(msg), 0)

        # Magic timeout value
        assert self.__enum_info_get_srv.wait_for_service(timeout_sec=self.__rcl_timeout_sec);
        assert self.__enum_set_srv.wait_for_service(timeout_sec=self.__rcl_timeout_sec);
        assert self.__stream_start_srv.wait_for_service(timeout_sec=self.__rcl_timeout_sec);
        assert self.__stream_stop_srv.wait_for_service(timeout_sec=self.__rcl_timeout_sec);

    def __call_service_sync(self, srv: Service, request):
        future = srv.call_async(request)
        rclpy.spin_until_future_complete(node=self, future=future, timeout_sec=self.__rcl_timeout_sec)
        return future.result()

    def stop_stream(self) -> StreamStartStop.Response:
        return self.__call_service_sync(self.__stream_stop_srv, StreamStartStop.Request())

    def start_stream(self) -> StreamStartStop.Response:
        return self.__call_service_sync(self.__stream_start_srv, StreamStartStop.Request())

    def get_supported_pixel_formats(self) -> List[str]:
        """ Receives the list of available pixel formats from the camera """
        req: FeatureEnumInfoGet.Request = FeatureEnumInfoGet.Request()
        req.feature_name = "PixelFormat"
        res: FeatureEnumInfoGet.Response = self.__call_service_sync(self.__enum_info_get_srv, req)

        return res.available_values

    def set_pixel_format(self, format: str) -> FeatureEnumSet.Response:
        """ Sets the pixel format """
        req: FeatureEnumSet.Request = FeatureEnumSet.Request()
        req.feature_name = "PixelFormat"
        req.value = format
        return self.__call_service_sync(self.__enum_set_srv, req)

    def get_latest_image(self) -> Image:
        """ Spins the default context until an Image is received from the Camera and returns the Image """
        # Clear the future to receive a new Image
        self.__image_future = rclpy.Future()
        rclpy.spin_until_future_complete(node=self, future=self.__image_future, timeout_sec=self.__rcl_timeout_sec)
        return self.__image_future.result()


@pytest.mark.launch(fixture=vimbax_camera_node)
def test_pixel_formats(launch_context):

    node: PixelFormatTestNode = PixelFormatTestNode("pytest_client_node", timeout_sec=5.0)

    # The PixelFormat cannot be changed while the camera is streaming
    assert node.stop_stream().error == 0

    # We can only test the formats required and supported by the attached camera
    available_formats = REQUIRED_PIXEL_FORMATS.intersection(set(node.get_supported_pixel_formats()))

    LOGGER.info(f"Available Pixel Formats: {available_formats}")

    for format in available_formats:
        # Set the pixel format 
        LOGGER.info(f"Testing format: {format}")

        assert node.set_pixel_format(format).error.code == 0
        assert node.start_stream().error == 0

        image: Image = node.get_latest_image()

        assert node.stop_stream().error == 0
        # Assert the pixel format of the image matches the requested format
        assert not image is None
        # Because the ROS and PFNC formats differ in naming the encoding needs to be translated
        assert image.encoding == PFNC_TO_ROS[format]

@pytest.mark.launch(fixture=vimbax_camera_node)
def test_invalid_pixel_format(launch_context):

    node: PixelFormatTestNode = PixelFormatTestNode("pytest_client_node", timeout_sec=5.0)
    node.stop_stream()
    # This should fail
    assert node.set_pixel_format("").error != 0
