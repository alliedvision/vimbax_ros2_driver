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

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vimbax_camera',
            namespace='vimbax_camera_calib',
            executable='vimbax_camera_node',
            name='vimbax_camera_calib'
        ),
        Node(
            package='camera_calibration',
            executable='cameracalibrator',
            remappings=[
                ('/image', '/vimbax_camera_calib/image_raw'),
                ('/camera/set_camera_info', '/vimbax_camera_calib/set_camera_info'),
            ],
            arguments=["--size", "9x6", "--square", "0.108"]
        )
    ])
