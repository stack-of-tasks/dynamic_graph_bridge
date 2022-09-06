from .ros_publish import RosPublish
from .ros_subscribe import RosSubscribe
from .ros_time import RosTime


class Ros(object):
    device = None
    rosPublish = None
    rosSubscribe = None

    # aliases, for retro compatibility
    rosImport = None
    rosExport = None

    def __init__(self, robot, suffix=""):
        self.robot = robot
        self.rosPublish = RosPublish("rosPublish{0}".format(suffix))
        self.rosSubscribe = RosSubscribe("rosSubscribe{0}".format(suffix))
        self.rosTime = RosTime("rosTime{0}".format(suffix))

        self.robot.device.after.addSignal("{0}.trigger".format(self.rosPublish.name))

        # aliases, for retro compatibility
        self.rosImport = self.rosPublish
        self.rosExport = self.rosSubscribe
