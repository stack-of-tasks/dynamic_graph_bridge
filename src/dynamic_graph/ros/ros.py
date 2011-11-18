from ros_import import RosImport
from ros_export import RosExport
from ros_joint_state import RosJointState

from dynamic_graph import plug

class Ros(object):
    device = None
    rosImport = None
    rosExport = None
    rosJointState = None

    def __init__(self, robot, suffix = ''):
        self.robot = robot
        self.rosImport = RosImport('rosImport{0}'.format(suffix))
        self.rosExport = RosExport('rosExport{0}'.format(suffix))
        self.rosJointState = RosJointState('rosJointState{0}'.format(suffix))

        plug(self.robot.device.state, self.rosJointState.state)
        self.robot.device.after.addSignal('{0}.trigger'.format(self.rosImport.name))
        self.robot.device.after.addSignal('{0}.trigger'.format(self.rosJointState.name))

        # Export base_footprint aka dynamic-graph world frame.
        self.rosImport.add('matrixHomoStamped', 'base_footprint', 'base_footprint')
        plug(robot.dynamic.signal('left-ankle'), self.rosImport.base_footprint)

        # Export base_link aka waist frame.
        self.rosImport.add('matrixHomoStamped', 'base_link', 'base_link')
        plug(robot.dynamic.waist, self.rosImport.base_link)

        # Export center of mass position.
        self.rosImport.add('vector3Stamped', 'com', 'com')
        plug(robot.dynamic.com, self.rosImport.com)

        # Export ZMP position.
        self.rosImport.add('vector3Stamped', 'zmp', 'zmp')
        plug(robot.dynamic.zmp, self.rosImport.zmp)
