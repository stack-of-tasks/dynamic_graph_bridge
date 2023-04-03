"""@package dynamic_graph_bridge.

@file ros.py
@author Maximilien Naveau (maximilien.naveau@gmail.com)
@license License BSD-3-Clause
@copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
@date 2019-05-22

@brief This file defines a class that create ros entities for the interaction
between ROS and the dynamic graph

"""

from dynamic_graph_bridge.dynamic_graph.ros_entities import RosPublish, RosSubscribe


class Ros:
    """Store ros_publish and ros_subscribe."""

    def __init__(self, device, suffix=""):
        """Store objects needed for ROS."""
        self.device = device
        self.ros_publish = RosPublish("ros_publish" + suffix)
        self.ros_subscribe = RosSubscribe("ros_subscribe" + suffix)

        # make sure that the publishing is done by plugging the refresh
        # (trigger) to the device periodic call system
        self.device.after.addDownsampledSignal(self.ros_publish.name + ".trigger", 1)
        # not needed for self.ros_subscribe and self.ros_subscribe as they get
        # the information from ROS, they do not have output signals
