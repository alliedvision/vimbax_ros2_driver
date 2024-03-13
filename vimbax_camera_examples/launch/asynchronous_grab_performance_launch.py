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

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
            name='image_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[
                ComposableNode(
                    package='vimbax_camera',
                    plugin='vimbax_camera::VimbaXCameraNode',
                    name='vimbax_camera',
                    namespace='vimbax_camera',
                    extra_arguments=[{'use_intra_process_comms': True}]),
                ComposableNode(
                    package='vimbax_camera_examples',
                    plugin='vimbax_camera_examples::AsynchronousGrabPerformance',
                    name='asynchronous_grab_performance',
                    remappings=[('/image', '/vimbax_camera/image_raw')],
                    parameters=[{'history': 'keep_last'}],
                    extra_arguments=[{'use_intra_process_comms': True}]),
            ],
            output='both',
    )

    return launch.LaunchDescription([container])
