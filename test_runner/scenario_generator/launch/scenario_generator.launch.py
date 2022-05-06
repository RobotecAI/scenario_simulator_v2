#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Launch description for scenario generator."""

# Copyright (c) 2022 Tier IV, Inc. All rights reserved.
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
#
# Co-developed by Tier IV, Inc. and Robotec.AI sp. z o.o.

import os

from ament_index_python import get_package_share_directory

from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    scenario_generator_pkg_prefix = get_package_share_directory('scenario_generator')
    rviz_cfg_path = os.path.join(scenario_generator_pkg_prefix, 'rviz/scenario_generator.rviz')

    lanelet_map_path = LaunchConfiguration('lanelet_map_path')
    lanelet_map_path_arg = DeclareLaunchArgument('lanelet_map_path', default_value='default_lanelet.osm')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', str(rviz_cfg_path)]
    )

    scenario_generator_node = Node(
        package='scenario_generator',
        executable='scenario_generator',
        parameters=[
            {'lanelet_map_path': lanelet_map_path}
        ]
    )

    return LaunchDescription([
        lanelet_map_path_arg,
        rviz,
        scenario_generator_node
    ])
