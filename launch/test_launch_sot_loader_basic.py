from launch import LaunchDescription
from launch_ros.actions import Node
from launch import LaunchContext
from launch.substitutions import TextSubstitution

from launch_ros.utilities import evaluate_parameters
from launch_ros.utilities import normalize_parameters


def generate_launch_description():
    ld = LaunchDescription()

    params = { "state_vector_map": [ "joint1", "joint2"] };
    test_sot_loader_basic_node = Node(
        package="dynamic_graph_bridge",
        executable="test_sot_loader_basic",
        parameters=[params],
    )

    
    ld.add_action(test_sot_loader_basic_node)

    return ld
