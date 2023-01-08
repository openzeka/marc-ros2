# MIT License

# Copyright (c) 2020 Hongrui Zheng

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration


def generate_launch_description():

    rviz_config_dir = os.path.join(
        get_package_share_directory("sllidar_ros2"), "rviz", "sllidar_ros2.rviz"
    )

    vesc_config = os.path.join(
        get_package_share_directory("f1tenth_stack"), "config", "vesc.yaml"
    )
    sensors_config = os.path.join(
        get_package_share_directory("f1tenth_stack"), "config", "sensors.yaml"
    )
    mux_config = os.path.join(
        get_package_share_directory("f1tenth_stack"), "config", "mux.yaml"
    )

    vesc_la = DeclareLaunchArgument(
        "vesc_config",
        default_value=vesc_config,
        description="Descriptions for vesc configs",
    )
    sensors_la = DeclareLaunchArgument(
        "sensors_config",
        default_value=sensors_config,
        description="Descriptions for sensor configs",
    )
    mux_la = DeclareLaunchArgument(
        "mux_config",
        default_value=mux_config,
        description="Descriptions for ackermann mux configs",
    )

    ld = LaunchDescription([vesc_la, sensors_la, mux_la])

    ackermann_to_vesc_node = Node(
        package="vesc_ackermann",
        executable="ackermann_to_vesc_node",
        name="ackermann_to_vesc_node",
        parameters=[LaunchConfiguration("vesc_config")],
    )
    vesc_to_odom_node = Node(
        package="vesc_ackermann",
        executable="vesc_to_odom_node",
        name="vesc_to_odom_node",
        parameters=[LaunchConfiguration("vesc_config")],
    )
    vesc_driver_node = Node(
        package="vesc_driver",
        executable="vesc_driver_node",
        name="vesc_driver_node",
        parameters=[LaunchConfiguration("vesc_config")],
    )

    ackermann_mux_node = Node(
        package="ackermann_mux",
        executable="ackermann_mux",
        name="ackermann_mux",
        parameters=[LaunchConfiguration("mux_config")],
        remappings=[("ackermann_cmd_out", "ackermann_drive")],
    )
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_baselink_to_laser",
        arguments=["0.27", "0.0", "0.11", "0.0", "0.0", "0.0", "base_link", "laser"],
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy",
    )

    # Camera model (force value)
    camera_model = "zed2"

    # ZED Wrapper node
    zed_wrapper_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            [
                get_package_share_directory("zed_wrapper"),
                "/launch/include/zed_camera.launch.py",
            ]
        ),
        launch_arguments={"camera_model": camera_model}.items(),
    )

    rplidar_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            [
                get_package_share_directory("sllidar_ros2"),
                "/launch/sllidar_s1_launch.py",
            ]
        ),
    )

    # finalize
    ld.add_action(ackermann_to_vesc_node)
    ld.add_action(vesc_to_odom_node)
    ld.add_action(vesc_driver_node)
    # ld.add_action(throttle_interpolator_node)
    ld.add_action(joy_node)
    ld.add_action(ackermann_mux_node)
    ld.add_action(static_tf_node)
    ld.add_action(zed_wrapper_launch)
    ld.add_action(rplidar_launch)

    return ld
