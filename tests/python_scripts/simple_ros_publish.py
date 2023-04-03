"""Simple module to test interaction with the ROS 2 world."""

from dynamic_graph import plug
from dynamic_graph.ros.ros_publish import RosPublish
from dynamic_graph.ros.tests.impl_test_sot_mock_device import ImplTestSotMockDevice

# Create a topic from the SoT to the ROS world
ros_publish = RosPublish("rosPublish")

name = "control"
ros_publish.add("vector", name + "_ros", name)

device_sot_mock = ImplTestSotMockDevice("device_sot_mock")
plug(device_sot_mock.control, ros_publish.signal(name + "_ros"))
