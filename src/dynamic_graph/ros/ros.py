from ros_import import RosImport
from ros_export import RosExport
from ros_joint_state import RosJointState

from dynamic_graph import plug

class Ros(object):
    device = None
    rosImport = None
    rosExport = None
    rosJointState = None

    def __init__(self, device, suffix = ''):
        self.device = device
        self.rosImport = RosImport('rosImport{0}'.format(suffix))
        self.rosExport = RosExport('rosExport{0}'.format(suffix))
        self.rosJointState = RosJointState('rosJointState{0}'.format(suffix))

        plug(self.device.state, self.rosJointState.state)
        self.device.after.addSignal('{0}.trigger'.format(self.rosImport.name))
        self.device.after.addSignal('{0}.trigger'.format(self.rosJointState.name))
