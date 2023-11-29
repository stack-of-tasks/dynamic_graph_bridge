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
"""Module to test sot_loader."""

import unittest
from pathlib import Path

import launch
import launch.actions
import launch_testing
import launch_testing.actions
import pytest
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node


@pytest.mark.launch_test
def generate_test_description():
    """Load a simple urdf, parameters and the library."""
    ld = LaunchDescription()
    print("After launch description")
    robot_description_content_path = PathJoinSubstitution(
        [
            get_package_share_directory("dynamic_graph_bridge"),
            "urdf",
            "dgb_minimal_robot.urdf",
        ],
    )
    print("Before robot_description_content")
    robot_description_content_path = Path(robot_description_content_path.perform(None))
    robot_description_content = robot_description_content_path.open().read()
    print("Before params")
    params = {
        "state_vector_map": ["joint1", "joint2"],
        "robot_description": robot_description_content,
    }

    terminating_process = Node(
        package="dynamic_graph_bridge",
        executable="test_sot_loader",
        output="screen",
        emulate_tty=True,
        parameters=[params],
    )

    return (
        launch.LaunchDescription(
            [
                terminating_process,
                launch_testing.util.KeepAliveProc(),
                launch_testing.actions.ReadyToTest(),
            ],
        ),
        locals(),
    )


class TestSotLoaderBasic(unittest.TestCase):
    """Loads the sot_loader."""

    def test_termination(self, terminating_process, proc_info):
        """Calls the decorator generate_test_description."""
        proc_info.assertWaitForShutdown(process=terminating_process, timeout=(60))


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    """Handle the test result."""

    def test_exit_code(self, proc_info):
        """Check that all processes in the launch exit with code 0."""
        launch_testing.asserts.assertExitCodes(proc_info)
