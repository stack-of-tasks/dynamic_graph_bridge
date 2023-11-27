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
"""Module to charge a urdf and run a sot_loader_basic class."""

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
    """Run a node in dynamic_graph_bridge package.

    It also set parameters state_vector_map and robot_description
    """
    print("start generate_test_description")
    ld = LaunchDescription()

    robot_description_pathjs_str = PathJoinSubstitution(
        [
            get_package_share_directory("dynamic_graph_bridge"),
            "urdf",
            "dgb_minimal_robot.urdf",
        ],
    )
    robot_description_path = Path(robot_description_pathjs_str.perform(None))
    robot_description_content = robot_description_path.open().read()
    print("After building robot_description_content")
    terminating_process = Node(
        package="dynamic_graph_bridge",
        executable="test_sot_loader_basic",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"state_vector_map": ["joint1", "joint2"]},
            {"robot_description": robot_description_content},
        ],
        arguments=["--ros-args", "--log-level", "INFO"],
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
    """Main class to start the test."""

    def test_termination(self, terminating_process, proc_info):
        """Runs the generate_test_description() function."""
        proc_info.assertWaitForShutdown(process=terminating_process, timeout=(60))


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    """Class to handle the test result."""

    def test_exit_code(self, proc_info):
        """Check that all processes in the launch exit with code 0."""
        launch_testing.asserts.assertExitCodes(proc_info)
