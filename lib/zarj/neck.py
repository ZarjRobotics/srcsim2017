"""
    Logic to control the Neck
"""
import rospy
from ihmc_msgs.msg import NeckTrajectoryRosMessage
from .utils import log

class Neck(object):
    """ Control of the neck """

    def __init__(self, zarj_os):
        log("Zarj neck initialization begin")

        self.zarj = zarj_os
        self.publishers = []
        self.MOVE_TIME = 1.0
        self.neck_publisher = rospy.Publisher(
            "/ihmc_ros/{0}/control/neck_trajectory".format(self.zarj.robot_name),
            NeckTrajectoryRosMessage, queue_size=1)
        self.publishers.append(self.neck_publisher)
        self.zarj.wait_for_publishers(self.publishers)

        log("Zarj neck initialization completed")

    def __del__(self):
        self.zarj.close_publishers(self.publishers)

    def neck_control(self, positions, wait=True):
        """ Control all 3 points of the head:
             lowerNeckPitch - 0 to 0.5 - looks down
             neckYaw -  -1.0 to 1.0 - looks left and right
             upperNeckPitch - -0.5 to 0.0 - looks up
        """
        trajectories = []

        for p in positions:
            trajectories.append(self.zarj.make_joint(self.MOVE_TIME, p))

        msg = NeckTrajectoryRosMessage()
        msg.joint_trajectory_messages = trajectories
        msg.unique_id = self.zarj.uid

        self.neck_publisher.publish(msg)
        if wait:
            rospy.sleep(self.MOVE_TIME + 0.1)

    def head_down(self, wait=True):
        """ head only down """
        log("Head down");
        self.neck_control([ 0.5, 0.0, 0.0 ], True)

    def head_forward(self, wait=True):
        """ head only forward"""
        log("Head forward");
        self.neck_control([ 0.0, 0.0, 0.0 ], True)
