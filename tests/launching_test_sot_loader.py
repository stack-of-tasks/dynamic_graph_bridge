# Copyright 2021 LAAS-CNRS
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

import os
import sys
import unittest

import ament_index_python

import launch
import launch.actions

import launch_testing
import launch_testing.actions
from launch.substitutions import PathJoinSubstitution
from launch_testing.asserts import assertSequentialStdout

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

import pytest


@pytest.mark.launch_test
def generate_test_description():
    ld = LaunchDescription()

    robot_description_content_path =  PathJoinSubstitution(
                [
                    get_package_share_directory("dynamic_graph_bridge"),
                    "urdf",
                    "dgb_minimal_robot.urdf",
                ]
            )
    robot_description_content = open(robot_description_content_path.perform(None)).read()

    params = { "state_vector_map": [ "joint1", "joint2"],
               "robot_description": robot_description_content};

    terminating_process = Node(
         package="dynamic_graph_bridge",
         executable="test_sot_loader",
         parameters=[params],
    )

    return (
        launch.LaunchDescription([
            terminating_process,
            launch_testing.util.KeepAliveProc(),
            launch_testing.actions.ReadyToTest(),
        ]), locals()
    )

class TestSotLoaderBasic(unittest.TestCase):

    def test_termination(self, terminating_process, proc_info):
        proc_info.assertWaitForShutdown(process=terminating_process, timeout=(10))

@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):

    def test_exit_code(self, proc_info):
        # Check that all processes in the launch (in this case, there's just one) exit
        # with code 0
        launch_testing.asserts.assertExitCodes(proc_info)
