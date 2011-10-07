#!/usr/bin/python

try:
    from dynamic_graph.sot.dynamics.tools import *
except:
    print("This test requires sot-dynamic.")
    exit(42)

from dynamic_graph.ros import RosJointState

rjs = RosJointState('rosjointstate')
plug(robot.device.state, rjs.state)
rjs.trigger.recompute(rjs.trigger.time + 1)
