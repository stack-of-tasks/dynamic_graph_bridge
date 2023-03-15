from dynamic_graph.ros.ros_publish import RosPublish

# Create a topic from the SoT to the ROS world
ros_publish = RosPublish("rosPublish")

name="control"
ros_publish.add("vector", name + "_ros", name)

device_sot_mock= ImplTestSotMockDevice("device_sot_mock")
plug(device_sot_mock.control, ros_import.signal(name + "_ros"))
