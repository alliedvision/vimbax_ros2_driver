# Vimba X ROS 2 camera driver

## Compability
- ROS 2 humble
- Nvidia Jetpack 5.x (arm64)
- Ubuntu 22.04 (x86_64)
- Allied Vision Alvium cameras
- Tested ROS 2 RMW implementation: rmw_cyclone_dds 

## Prerequisites
- ROS 2 humble is installed on the system as defined by the [ROS 2 installation instructions](https://docs.ros.org/en/humble/Installation.html)
- For NVIDIA Jetson boards please follow the [NVIDIA ISAAC ROS installation guide](https://nvidia-isaac-ros.github.io/getting_started/isaac_ros_buildfarm_cdn.html#setup)
- For running the system tests make sure the package "ros-humble-launch-pytests" is installed on your system.
- [Vimba X 2023-4](https://www.alliedvision.com/en/products/software/vimba-x-sdk/) or later
- For CSI cameras make sure to install the drivers available on [github](https://github.com/alliedvision/linux_nvidia_jetson)

## Installation
Download the debian package from the release page and install it using the following command:
```
sudo apt install ros-humble-rmw-cyclonedds-cpp
sudo apt install ./ros-humble-vimbax-camera-driver.deb 
```

## Getting started

Setup the ROS 2 environment:
```shell
source /opt/ros/humble/setup.bash
```

Change the the ROS 2 middleware to cyclonedds, because the default middleware is causing some issues. See [known issuse](#known-issues) for more details.:
```shell
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```
To get maximum performance please also apply the settings described in [DDS Tuning](https://docs.ros.org/en/humble/How-To-Guides/DDS-tuning.html).
Especially the settings for the Cyclone DDS section are important, because otherwise you might loose frames. 

To start the Vimba X ROS 2 node run:
```shell
ros2 run vimbax_camera vimbax_camera_node
```
By default the Vimba X ROS 2 node will open the first available camera. If you want to open a specific camera, you can use *camera_id* parameter by adding `--ros-args -p camera_id:=<camera to open>`. The *camera_id* can be the device id, extended device id, serial number, ip address or mac address. The
node will always open the camera in exclusive access mode. If no camera is available or the specified camera was opened by another application, the node startup fails and an error message is printed.
The node will publish all topics and service under the namespace `vimbax_camera_<pid>`, where `pid`
is the process id of the camera node process.

An example can be started by running:
```shell
ros2 run vimbax_camera_examples <example name> vimbax_camera_<pid>
```

The Vimba X ROS2 node must have been started before.

The following examples are available:
- asynchronous_grab: Stream images from the camera node and print image info to console.
- asynchronous_grab_performance: High performance streaming example.
- asynchronous_grab_opencv: Stream images from the camera node and display them using opencv imshow.
- event_viewer: Show GenICam events on the console.
- feature_command_execute: How to run a command feature.
- feature_get: How to get a feature value.
- feature_info_get: How to get the type specific feature information.
- feature_set: Change the value of a feature.
- list_features: How to list all available features.
- settings_load_save: Load or save the camera settings to an xml file.
- status_get: How to use the status service to get the current camera status.
- camera_connected: Reads if the camera is currently connected or not. 
- connection_observer: Periodically calls the connected service and prints a message if the connections status has changed.

## Build Instructions
1. Setup the ROS2 environment
    ```shell
    source /opt/ros/humble/setup.bash
    ```

2. Initialize and update rosdep if not already done
    ```shell
    rosdep init
    rosdep update
    sudo apt-get update
    ```

3. Create a workspace
    ```shell
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    ```

4. Clone this repository into the workspace
    ```shell
    git clone ...
    ```
5. Install dependencies
    ```shell
    cd ~/ros2_ws/
    rosdep install --from-path src --ignore-src
    ```

6. Run the build
    ```shell
    cd ~/ros2_ws/
    colcon build --cmake-args -DVMB_DIR=<path to VimbaX installation>
    ```
    The optional VMB_DIR cmake argument can be used to specify the path to the Vimba X installation
    used for testing.

7. Run unit tests (optional)
    ```shell
    colcon test
    ```

8. Print test results (optional)
    ```shell
    colcon test-result --verbose
    ```

## Supported pixel formats
The following PFNC pixel formats are supported:
- Mono8
- Mono12/16
- BGR8
- RGB8
- BayerBG8
- BayerGB8
- BayerRG8
- BayerGR8
- BayerRG10/12
- BayerBG10/12
- BayerGB10/12
- BayerGR10/12
- BayerRG16
- BayerBG16
- BayerGB16
- BayerGR16
- Any 8-bit YCbCr422 format (e.g. YCbCr422_8)

If an unsupported pixel format is used the stream will not start.

## Automatic stream

The streaming automatically starts if a node subscribes to the *image_raw* or any other image
topic of the camera node. The streaming automatically stops if a node unsubscribes from the
*image_raw* or any other image topic of the camera node.
The automatic stream start/stop can be enabled and disabled using the
*autostream* parameter.

## GenICam events

GenICam events and feature invalidations can be used using the vimbax_camera_events package. For more details please look into the events examples in the vimbax_camera_examples package.

## Camera disconnect and reconnect

If a camera (GigE or USB) is disconnected while the camera node is already running, the node
will wait for the camera to reappear and then reconnect to it. If the camera was streaming
while it is disconnected, the stream will be restarted after the camera is reconnected.
Only if [automatic stream](#automatic-stream) is enabled.

## Parameters

| Name | Description |
|------|-------------|
| camera_id | Id of camera to open. Can be the device id, extended device id, serial number, ip or mac address. |
| settings_file | Path to xml settings file to load on startup. <br> **The file must point to a valid file on system that the node runs on.** |
| buffer_count | Number of buffers used for streaming. <br> **Can't be change during streaming.** |
| autostream | When set to 1 the [automatic stream](#automatic-stream) is enabled. |
| camera_frame_id | ROS 2 frame id of the camera. |
| camera_info_url | Url to ROS 2 camera info file. |
| command_feature_timeout | Timeout for command features. |
| use_ros_time | Use ros2 timestamp in Image message header instead of camera timestamp | 

## Common message types

### vimbax_camera_msgs/Error

| Name | Type | Description |
|------|------|-------------|
| code | int32 | Error code of the operation. <br> 0 on success. |
| text | string | Error code as a string.  <br> Only valid if the *code* is not 0. |

### vimbax_camera_msgs/FeatureFlags
| Name | Type | Description |
|------|------|-------------|
| flag_none | bool | No additional information is provided |
| flag_read | bool | Static info about read access. Current status depends on access mode, check with the [access_mode_get](#camera-node-nsfeaturesaccess_mode_get) service |
| flag_write | bool | Static info about write access. Current status depends on access mode, check with the [access_mode_get](#camera-node-nsfeaturesaccess_mode_get) service |
| flag_volatile | bool | Value may change at any time |
| flag_modify_write | bool | Value may change after a write |

### vimbax_camera_msgs/FeatureInfo
| Name | Type | Description |
|------|------|-------------|
| name | string | Name of the feature. |
| category | string | Feature category. |
| display_name | string | Display name of the feature. |
| sfnc_namespace | string | SFNC namespace of the feature |
| unit | string | Unit of the feature. |
| data_type | uint32 | Feature data type: <br> 0: Unknown <br> 1: Int <br> 2: Float <br> 3: Enum <br> 4: String <br> 5: Bool <br> 6: Command <br> 7: Raw <br> 8: None |
| flags | [FeatureFlags](#vimbax_camera_msgsfeatureflags) | Access flags for this feature |
| polling_time | uint32 | Predefined polling time for volatile features |

### vimbax_camera_msgs/FeatureModule
| Name | Type | Description |
|------|------|-------------|
| id | uint8 | Id of the GenTL module which should be accessed. See table below for valid module ids. |

| Id | Constant | Module |
|----|----------|--------|
| 0 | MODULE_REMOTE_DEVICE | Remote Device / Camera |
| 1 | MODULE_SYSTEM | System / Transport Layer |
| 2 | MODULE_INTERFACE | Interface |
| 3 | MODULE_LOCAL_DEVICE | Local Device |
| 4 | MODULE_STREAM | Stream 0 |

## vimbax_camera_msgs/TriggerInfo
| Name | Type | Description |
|------|------|-------------|
| selector | string | Trigger selector for which the mode and source was read. |
| mode | string | Trigger mode of the given selector. |
| source | string | Trigger source of the given selector. |

## Available services

### /\<camera node ns>/feature_info_query
#### Description

Query the feature information for the features given in *feature_names*. If the *feature_names*
is empty, then the information for all features will be returned.


#### Request

| Name | Type | Description |
|------|------|-------------|
|feature_names| string[] | Names of features to query
|feature_module| [FeatureModule](#vimbax_camera_msgsfeaturemodule) | GenTL module to access | 

#### Response

| Name | Type | Description |
|------|------|-------------|
| feature_info | [FeatureInfo](#vimbax_camera_msgsfeatureinfo)[] | List of feature infos |
| error | [Error](#vimbax_camera_msgserror) | Result of the operation |

### /\<camera node ns>/features/access_mode_get
#### Description

This service reads the current access mode of the feature *feature_name*

#### Request

| Name | Type | Description |
|------|------|-------------|
| feature_name | string | Name of the feature for getting the current access mode |
| feature_module | [FeatureModule](#vimbax_camera_msgsfeaturemodule) | GenTL module to access | 

#### Response

| Name | Type | Description |
|------|------|-------------|
| is_readable | bool | True if the feature can currently be read otherwise false |
| is_writeable | bool | True if the feature can currently be written otherwise false |
| error | [Error](#vimbax_camera_msgserror)  | Result of the request |

### /\<camera node ns>/features/bool_get
#### Description

Reads the current value of the bool feature *feature_name*

#### Request

| Name | Type | Description |
|------|------|-------------|
| feature_name | string | Name of the feature |
| feature_module | [FeatureModule](#vimbax_camera_msgsfeaturemodule) | GenTL module to access | 

#### Response

| Name | Type | Description |
|------|------|-------------|
| value | bool | Current bool value of the feature |
| error | [Error](#vimbax_camera_msgserror) | Result of the feature read |

### /\<camera node ns>/features/bool_set
#### Description

Set the value of the bool feature *feature_name* to *value*

#### Request

| Name | Type | Description |
|------|------|-------------|
| feature_name | string | Name of the feature |
| value | bool | New feature value |
| feature_module | [FeatureModule](#vimbax_camera_msgsfeaturemodule) | GenTL module to access | 

#### Response

| Name | Type | Description |
|------|------|-------------|
| error | [Error](#vimbax_camera_msgserror) | Result of the feature write |

### /\<camera node ns>/features/command_is_done
#### Description

Check if the command feature *feature_name* has finished.

#### Request

| Name | Type | Description |
|------|------|-------------|
| feature_name | string | Name fo the feature |
| feature_module | [FeatureModule](#vimbax_camera_msgsfeaturemodule) | GenTL module to access | 

#### Response

| Name | Type | Description |
|------|------|-------------|
| is_done | bool | True if the command feature execution has finished |
| error | [Error](#vimbax_camera_msgserror) | Result of the operation |

### /\<camera node ns>/features/command_run
#### Description

Run the command feature *feature_name* and wait until it's done, so a call to is_done is not needed.

#### Request

| Name | Type | Description |
|------|------|-------------|
| feature_name | string | Name fo the feature |
| feature_module | [FeatureModule](#vimbax_camera_msgsfeaturemodule) | GenTL module to access | 


#### Response

| Name | Type | Description |
|------|------|-------------|
| error | [Error](#vimbax_camera_msgserror) | Result of the feature execution |

### /\<camera node ns>/features/enum_as_int_get
#### Description

Get the corresponding integer value for enum option *option* of feature *feature_name*

#### Request

| Name | Type | Description |
|------|------|-------------|
| feature_name | string | Name of the feature |
| option | string | Enum option |
| feature_module | [FeatureModule](#vimbax_camera_msgsfeaturemodule) | GenTL module to access | 

#### Response

| Name | Type | Description |
|------|------|-------------|
| value | int64 | Integer value of option |
| error | [Error](#vimbax_camera_msgserror) | Result of the operation |

### /\<camera node ns>/features/enum_as_string_get
#### Description

Get the enum string representation for the enum integer value *value* of the feature *feature_name*

#### Request

| Name | Type | Description |
|------|------|-------------|
| feature_name | string | Name of the feature |
| value | int64 | Integer value of the enum feature |
| feature_module | [FeatureModule](#vimbax_camera_msgsfeaturemodule) | GenTL module to access | 

#### Response

| Name | Type | Description |
|------|------|-------------|
| option | string | String representation of the enum value |
| error | [Error](#vimbax_camera_msgserror) | Result of the operation |

### /\<camera node ns>/features/enum_get
#### Description

Read the current option of the enum feature *feature_name*

#### Request

| Name | Type | Description |
|------|------|-------------|
| feature_name | string | Name of the feature |
| feature_module | [FeatureModule](#vimbax_camera_msgsfeaturemodule) | GenTL module to access | 

#### Response

| Name | Type | Description |
|------|------|-------------|
| option | string | Current enum feature option |
| error | [Error](#vimbax_camera_msgserror) | Result of the operation |

### /\<camera node ns>/features/enum_info_get
#### Description

Get the type specific feature info of the enum feature *feature_name*.

#### Request

| Name | Type | Description |
|------|------|-------------|
| feature_name | string | Name of the feature |
| feature_module | [FeatureModule](#vimbax_camera_msgsfeaturemodule) | GenTL module to access | 

#### Response

| Name | Type | Description |
|------|------|-------------|
| possible_values | string[] | List of all existing enum options |
| available_values | string[] | List of the currently available enum options |
| error | [Error](#vimbax_camera_msgserror) | Result of the operation |

### /\<camera node ns>/features/enum_set
#### Description

Sets the value of the enum feature *feature_name* to *value*

#### Request

| Name | Type | Description |
|------|------|-------------|
| feature_name | string | Name of the enum feature to change |
| value | string | Enum option to set |
| feature_module | [FeatureModule](#vimbax_camera_msgsfeaturemodule) | GenTL module to access | 

#### Response

| Name | Type | Description |
|------|------|-------------|
| error | [Error](#vimbax_camera_msgserror) | Result of the operation |

### /\<camera node ns>/features/float_get
#### Description

Reads the current value of the float feature *feature_name*

#### Request

| Name | Type | Description |
|------|------|-------------|
| feature_name | string | Name of the float feature to read |
| feature_module | [FeatureModule](#vimbax_camera_msgsfeaturemodule) | GenTL module to access | 

#### Response

| Name | Type | Description |
|------|------|-------------|
| value | float64 | Current value of the float feature |
| error | [Error](#vimbax_camera_msgserror) | Result of the operation |

### /\<camera node ns>/features/float_info_get
#### Description

Get the type specific feature information (limits) of the float feature *feature_name*

#### Request

| Name | Type | Description |
|------|------|-------------|
| feature_name | string | Name of the float feature |
| feature_module | [FeatureModule](#vimbax_camera_msgsfeaturemodule) | GenTL module to access | 

#### Response

| Name | Type | Description |
|------|------|-------------|
| min | float64 | Minimum value of the feature |
| max | float64 | Maximum value of the feature |
| inc | float64 | Increment of the feature |
| inc_available | bool | True when the *inc* field contains a valid value otherwise false |
| error | [Error](#vimbax_camera_msgserror) | Result of the operation |

### /\<camera node ns>/features/float_set
#### Description

Set the value of the float feature *feature_name* to *value*

#### Request

| Name | Type | Description |
|------|------|-------------|
| feature_name | string | Name of the float feature to change |
| value | float64 | New value of the float feature |
| feature_module | [FeatureModule](#vimbax_camera_msgsfeaturemodule) | GenTL module to access | 

#### Response

| Name | Type | Description |
|------|------|-------------|
| error | [Error](#vimbax_camera_msgserror) | Result of the operation |

### /\<camera node ns>/features/int_get
#### Description

Read the current value of the int feature *feature_name*

#### Request

| Name | Type | Description |
|------|------|-------------|
| feature_name | string | Name of the int feature to read |
| feature_module | [FeatureModule](#vimbax_camera_msgsfeaturemodule) | GenTL module to access | 

#### Response

| Name | Type | Description |
|------|------|-------------|
| value | int64 | Current value of the int feature |
| error | [Error](#vimbax_camera_msgserror) | Result of the operation |

### /\<camera node ns>/features/int_info_get
#### Description

Get the type specific feature information (limits) of the int feature *feature_name*

#### Request

| Name | Type | Description |
|------|------|-------------|
| feature_name | string | Name of the int feature |
| feature_module | [FeatureModule](#vimbax_camera_msgsfeaturemodule) | GenTL module to access | 

#### Response

| Name | Type | Description |
|------|------|-------------|
| min | int64 | Minimum feature value |
| max | int64 | Maximum feature value |
| inc | int64 | Increment of the feature |
| error | [Error](#vimbax_camera_msgserror) | Result of the operation |

### /\<camera node ns>/features/int_set
#### Description

Set the value of the int feature *feature_name* to *value*

#### Request

| Name | Type | Description |
|------|------|-------------|
| feature_name | string | Name of the int feature to change |
| value | int64 | New value of the int feature |
| feature_module | [FeatureModule](#vimbax_camera_msgsfeaturemodule) | GenTL module to access | 

#### Response

| Name | Type | Description |
|------|------|-------------|
| error | [Error](#vimbax_camera_msgserror) | Result of the operation |

### /\<camera node ns>/features/list_get
#### Description

Get a list of all available feature names

#### Request

| Name | Type | Description |
|------|------|-------------|
| feature_module | [FeatureModule](#vimbax_camera_msgsfeaturemodule) | GenTL module to access | 

#### Response

| Name | Type | Description |
|------|------|-------------|
| feature_list | string[] | List containing all feature names of the camera |
| error | [Error](#vimbax_camera_msgserror) | Result of the operation |

### /\<camera node ns>/features/raw_get
#### Description

Get the data of the raw feature *feature_name*

#### Request

| Name | Type | Description |
|------|------|-------------|
| feature_name | string | Name of the raw feature |
| feature_module | [FeatureModule](#vimbax_camera_msgsfeaturemodule) | GenTL module to access | 

#### Response

| Name | Type | Description |
|------|------|-------------|
| buffer | byte[] | Contains the data of the raw feature |
| buffer_size | uint32 | Length of the feature data |
| error | [Error](#vimbax_camera_msgserror) | Result of the operation |

### /\<camera node ns>/features/raw_info_get
#### Description

Get the type specific feature information of the raw feature *feature_name*

#### Request

| Name | Type | Description |
|------|------|-------------|
| feature_name | string | Name of the raw feature |
| feature_module | [FeatureModule](#vimbax_camera_msgsfeaturemodule) | GenTL module to access | 

#### Response

| Name | Type | Description |
|------|------|-------------|
| max_length | int64 | Maximum length of the raw feature data |
| error | [Error](#vimbax_camera_msgserror) | Result of the operation |

### /\<camera node ns>/features/raw_set
#### Description

Set the raw feature *feature_name* to the data in *buffer*

#### Request

| Name | Type | Description |
|------|------|-------------|
| feature_name | string | Name of the raw feature to change |
| buffer | byte[] | New data of the raw feature |
| feature_module | [FeatureModule](#vimbax_camera_msgsfeaturemodule) | GenTL module to access | 

#### Response

| Name | Type | Description |
|------|------|-------------|
| error | [Error](#vimbax_camera_msgserror) | Result of the operation |

### /\<camera node ns>/features/string_get
#### Description

Get the current value of the string feature *feature_name*

#### Request

| Name | Type | Description |
|------|------|-------------|
| feature_name | string | Name of the string feature to read |
| feature_module | [FeatureModule](#vimbax_camera_msgsfeaturemodule) | GenTL module to access | 

#### Response

| Name | Type | Description |
|------|------|-------------|
| value | string | Current value of the string feature |
| error | [Error](#vimbax_camera_msgserror) | Result of the operation |

### /\<camera node ns>/features/string_info_get
#### Description

Get the type specific feature info of the string feature *feature_name*

#### Request

| Name | Type | Description |
|------|------|-------------|
| feature_name | string | Name of the string feature |
| feature_module | [FeatureModule](#vimbax_camera_msgsfeaturemodule) | GenTL module to access | 

#### Response

| Name | Type | Description |
|------|------|-------------|
| max_length | int64 | Maximum length of the string value |
| error | [Error](#vimbax_camera_msgserror) | Result of the operation |

### /\<camera node ns>/features/string_set
#### Description

Set the string feature *feature_name* to *value*

#### Request

| Name | Type | Description |
|------|------|-------------|
| feature_name | string | Name of the string feature to change |
| value | string | New value of the string feature |
| feature_module | [FeatureModule](#vimbax_camera_msgsfeaturemodule) | GenTL module to access | 

#### Response

| Name | Type | Description |
|------|------|-------------|
| error | [Error](#vimbax_camera_msgserror) | Result of the operation |

### /\<camera node ns>/settings/load
#### Description

Load the camera setting from the xml file *filename*.

**The path to the settings file must point to an existing file system that the node runs on.**

#### Request

| Name | Type | Description |
|------|------|-------------|
| filename | string | Path to the xml file to load |

#### Response

| Name | Type | Description |
|------|------|-------------|
| error | [Error](#vimbax_camera_msgserror) | Result of the operation |

### /\<camera node ns>/settings/save
#### Description

Save the camera setting to the xml file *filename*.

**The path to the settings file must point to an existing directory system that the node runs on.**

#### Request

| Name | Type | Description |
|------|------|-------------|
| filename | string | Path to the file where the settings should be saved. |

#### Response

| Name | Type | Description |
|------|------|-------------|
| error | [Error](#vimbax_camera_msgserror) | Result of the operation |

### /\<camera node ns>/status
#### Description

Get status information of the connected camera.

#### Request

| Name | Type | Description |
|------|------|-------------|

#### Response

| Name | Type | Description |
|------|------|-------------|
| error | [Error](#vimbax_camera_msgserror) | Result of the operation |
| display_name | string | Display name of the camera |
| model_name | string | Model name of the camera |
| device_firmware_version | string | Firmware version of the camera |
| device_id | string | Device id of the camera. Corresponds to the camera feature "DeviceFirmwareVersion". |
| device_user_id | string | Device user id of the camera. Corresponds to the camera feature "DeviceUserID". |
| device_serial_number | string | Serial number of the camera |
| interface_id | string | Id of the GenTL interface module |
| transport_layer_id | string | Id of the GenTL transport layer module |
| streaming | bool | True if the camera is currently streaming otherwise false |
| width | uint32 | Currently set width of the image. Equals the value of the camera feature "Width" |
| height | uint32 | Currently set height of the image. Equals the value of the camera feature "Height" |
| frame_rate | float64 | Currently set frame rate of the camera |
| pixel_format | string | Currently set pixel format. Equals the camera feature "PixelFormat" |
| trigger_info | [TriggerInfo](#vimbax_camera_msgstriggerinfo) | Trigger information |
| ip_address | string | IP address of the camera. <br> **Only valid for GigE vision cameras**
| mac_address | string | MAC address of the camera. <br> **Only valid for GigE vision cameras**

### /\<camera node ns>/stream_start
#### Description

Start the streaming of camera images.

#### Request

| Name | Type | Description |
|------|------|-------------|

#### Response

| Name | Type | Description |
|------|------|-------------|
| error | [Error](#vimbax_camera_msgserror) | Result of the operation |

### /\<camera node ns>/stream_stop
#### Description

Stop the streaming of the camera.

#### Request

| Name | Type | Description |
|------|------|-------------|

#### Response

| Name | Type | Description |
|------|------|-------------|
| error | [Error](#vimbax_camera_msgserror) | Result of the operation |

### /\<camera node ns>/connected
#### Description

Read wether the camera is connected or not.

#### Request

| Name | Type | Description |
|------|------|-------------|

#### Response

| Name | Type | Description |
|------|------|-------------|
| connected | bool | True when the camera is connected otherwise false. |

## Troubleshooting

### Finding and listing cameras
For listing all available cameras please use the [list cameras example](https://docs.alliedvision.com/Vimba_X/Vimba_X_DeveloperGuide/examplesOverview.html#list-cameras) from Vimba X SDK installation.
After running the example you can use the printed `Camera Id` or `Serial Number` as *camera_id*
parameter for opening a specific camera.

### Camera calibration
If an error message regarding a missing camera calibration file appears it can be ignored.
For more information see [ROS2 camera calibration documentation](https://docs.ros.org/en/rolling/p/camera_calibration/tutorial_mono.html).

### Images lost in ROS 2 
If you are losing image and don't see any error messages, please make sure that you have applied all settings from the [DDS Tuning Guide](https://docs.ros.org/en/humble/How-To-Guides/DDS-tuning.html).

### Known issues
- When using the default ros 2 middleware rmw_fastrtps_cpp the may get unresponsive sporadically if you very often subscribe and unsubscribe to the image_raw topic. This happens due to a deadlock in the middleware implementation. Therefore it is recommended to use rmw_cyclonedds_cpp as ros 2 middleware instead. 
- Due to a bug in Vimba X 2023-4, opening a camera by ip address does only work correctly when the camera was discovered before. Otherwise the node might not work correctly. 


### Debug Instructions (VS Code)
* Install ROS extension (from Microsoft) within VS Code
* Type CTRL-SHIFT-p to open the command palette
* Search and execute "ROS: Start"
* Search and execute "ROS: Update C++ properties"
* Type CTRL-SHIFT-b and select "colcon" to build the project
* Type CTRL-SHIFT-d and select "ROS: Launch" to start debug session

## Beta Disclaimer

Please be aware that all code revisions not explicitly listed in the Github Release section are
considered a **Beta Version**.

For Beta Versions, the following applies in addition to the BSD 3 Clause License:

THE SOFTWARE IS PRELIMINARY AND STILL IN TESTING AND VERIFICATION PHASE AND IS PROVIDED ON AN “AS
IS” AND “AS AVAILABLE” BASIS AND IS BELIEVED TO CONTAIN DEFECTS. THE PRIMARY PURPOSE OF THIS EARLY
ACCESS IS TO OBTAIN FEEDBACK ON PERFORMANCE AND THE IDENTIFICATION OF DEFECTS IN THE SOFTWARE,
HARDWARE AND DOCUMENTATION.

