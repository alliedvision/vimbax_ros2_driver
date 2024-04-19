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
            namespace='vimbax_camera_test',
            executable='vimbax_camera_node',
            name='vimbax_camera_test',
            parameters=[{
                # "camera_id": "00:0f:31:00:0e:2f"
                # "camera_id": "00:0F-31-00-0E-2F"
                # "camera_id": "169.254.103.205"
                # "camera_id": "DEV_000F31000E2F"
            }]
        )
    ])
