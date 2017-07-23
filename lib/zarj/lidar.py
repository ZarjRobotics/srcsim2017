"""
    Lidar for the robot
"""

import rospy
from sensor_msgs.msg import LaserScan
from copy import deepcopy

__all__ = ['Lidar']

class Lidar(object):
    """ Give information about the lidar """

    def __init__(self):
        rospy.loginfo("Zarj lidar initialization begin")
        self.scan = None
        self.sub = rospy.Subscriber("/multisense/lidar_scan",
                                    LaserScan, self.recv_scan)
        rospy.loginfo("Zarj lidar initialization finished")

    def __del__(self):
        if self.sub is not None:
            self.sub.unregister()

    def recv_scan(self, msg):
        """ Handle a laser scan """
        self.scan = deepcopy(msg)

    def last_scan(self):
        """ Handle a laser scan """
        self.scan = None
        while self.scan is None:
            rospy.sleep(0.3)
        return deepcopy(self.scan)
