# ROS client lib
import rclpy
from rclpy.node import Node
import launch
from launch_ros import actions
from sensor_msgs.msg import Image

# pytest libs
import pytest
import launch_pytest

# VimbaX_Camera msgs
from vimbax_camera_msgs.srv import FeatureEnumInfoGet, FeatureEnumSet, StreamStartStop

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
        'BayerRG8'  : 'bayer_rggb8', #TODO: Maybe add bayer_grbg8 after req clarification
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
    """Launch a simple process to print 'hello_world'."""
    
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


def call_service_sync(node: Node, service: rclpy.service.Service, request):
    future = service.call_async(request)
    rclpy.spin_until_future_complete(node=node, future=future)
    return future.result()

@pytest.mark.launch(fixture=vimbax_camera_node)
def test_pixel_formats(launch_context):

    # TODO: Find a way to assert that the node launched
    # Init ros
    rclpy.init()

    # Create the needed service client
    node: Node = rclpy.create_node("pytest_client_node")
    enum_info_get_srv: rclpy.service.Service = node.create_client(srv_type=FeatureEnumInfoGet, srv_name=f"/{test_node_name}/features/enum_info_get")
    enum_set_srv: rclpy.service.Service = node.create_client(srv_type=FeatureEnumSet, srv_name=f"/{test_node_name}/features/enum_set")
    stream_start_srv: rclpy.service.Service = node.create_client(srv_type=StreamStartStop, srv_name=f"/{test_node_name}/stream_start")
    stream_stop_srv: rclpy.service.Service = node.create_client(srv_type=StreamStartStop, srv_name=f"/{test_node_name}/stream_stop")

    # Magic timeout value
    assert enum_info_get_srv.wait_for_service(timeout_sec=10);
    assert enum_set_srv.wait_for_service(timeout_sec=10);
    assert stream_start_srv.wait_for_service(timeout_sec=10);
    assert stream_stop_srv.wait_for_service(timeout_sec=10);

    # Subscribe and receive 1 image
    image: Image = None
    def image_cb(msg: Image):
        nonlocal image
        image = msg
    img_sub = node.create_subscription(Image, f"/{test_node_name}/image_raw", image_cb, 0)
    # Timeout is necessary because we might block for ever if no messages are received
    rclpy.spin_once(node, timeout_sec=1.0)
    
    assert call_service_sync(node, stream_stop_srv, StreamStartStop.Request()).error == 0
    print("Stopped stream")

    # Get the list of available pixel formats
    req: FeatureEnumInfoGet.Request = FeatureEnumInfoGet.Request()
    req.feature_name = "PixelFormat"
    res: FeatureEnumInfoGet.Response = call_service_sync(node, enum_info_get_srv, req)

    # We can only test the formats required and supported by the attached camera
    available_formats = REQUIRED_PIXEL_FORMATS.intersection(set(res.available_values))

    print(f"Available Pixel Formats: {available_formats}")
    
    # If we dont reset image to None here the subscriber might have already received an image with an unknown image format
    image = None

    for format in available_formats:
        # Set the pixel format 
        req: FeatureEnumSet.Request = FeatureEnumSet.Request()
        req.feature_name = "PixelFormat"
        req.value = format
        res: FeatureEnumSet.Response = call_service_sync(node, enum_set_srv, req)
        print(f"Set format: {format}")
        assert res.error.code == 0

        # Start stream for one frame
        assert call_service_sync(node, stream_start_srv, StreamStartStop.Request()).error == 0
        print("Started stream")

        # TODO: timeout
        # Give ROS time to process callbacks until we receive an Image
        while image is None:
            rclpy.spin_once(node)
            print("Spinning")

        assert call_service_sync(node, stream_stop_srv, StreamStartStop.Request()).error == 0
        print("Stopped stream")

        # Assert the pixel format of the image matches the requested format
        assert not image is None
        assert image.encoding == PFNC_TO_ROS[format]

        image = None
    
    # Shutdown ros
    rclpy.shutdown()
