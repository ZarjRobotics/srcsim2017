"""
    This is the core of the ZARJ robot.
    All hail ZARJ!
"""

import time
from .eyes import Eyes
from .lidar import Lidar
from .tfzarj import TfZarj
from .walk import Walk
from .pelvis import Pelvis
from .neck import Neck
from .navigation import Navigation
from .zps import ZPS
from .hands import Hands
from .utils import log

import rospy
from ihmc_msgs.msg import OneDoFJointTrajectoryRosMessage
from ihmc_msgs.msg import TrajectoryPoint1DRosMessage
from ihmc_msgs.msg import NeckTrajectoryRosMessage
from ihmc_msgs.msg import StopAllTrajectoryRosMessage
from ihmc_msgs.msg import GoHomeRosMessage

USE_ROSPY = True

__all__ = ['ZarjOS']


class ZarjOS(object):
    """ The core of ZARJ's being """

    def __init__(self):
        log("ZarjOS initialization begin")

        self.msg_uid = int(time.time())
        if USE_ROSPY:
            self.robot_name = rospy.get_param('/ihmc_ros/robot_name')
        else:
            self.robot_name = 'ZARJ'
        log('  name: {0}'.format(self.robot_name))

        self._eyes = None
        self._lidar = None
        self._transform = None
        self._walk = None
        self._pelvis = None
        self._neck = None
        self._navigation = None
        self._ZPS = None
        self._hands = None

        #  We need transforms right from the start
        self._transform = TfZarj(self.robot_name)
        self._transform.init_transforms()

        self.stop_publisher = rospy.Publisher(
            "/ihmc_ros/{0}/control/stop_all_trajectories".format(self.robot_name),
            StopAllTrajectoryRosMessage, queue_size=1)

        self.home_publisher = rospy.Publisher(
            "/ihmc_ros/{0}/control/go_home".format(self.robot_name),
            GoHomeRosMessage, queue_size=1)

        self.publishers = [self.stop_publisher, self.home_publisher]
        self.wait_for_publishers(self.publishers)

        log("ZarjOS initialization finished")


    def stop_all_moves(self):
        msg = StopAllTrajectoryRosMessage()
        msg.unique_id = self.uid
        self.stop_publisher.publish(msg)
        if self._walk is not None:
            self._walk.abort()
        if self._ZPS is not None:
            self._ZPS.all_stop()

    @property
    def transform(self):
        """ Transform package """
        return self._transform

    @property
    def walk(self):
        """ walking controller """
        if self._walk is None:
            self._walk = Walk(self)
        return self._walk

    @property
    def pelvis(self):
        """ pelvis controller """
        if self._pelvis is None:
            self._pelvis = Pelvis(self)
        return self._pelvis

    @property
    def neck(self):
        """ pelvis controller """
        if self._neck is None:
            self._neck = Neck(self)
        return self._neck

    @property
    def eyes(self):
        """ Eyeballs """
        if self._eyes is None:
            self._eyes = Eyes()
        return self._eyes

    @property
    def lidar(self):
        """ Laser range finder """
        if self._lidar is None:
            self._lidar = Lidar()
        return self._lidar

    @property
    def navigation(self):
        """ Navigation """
        if self._navigation is None:
            self._navigation = Navigation(self)
        return self._navigation

    @property
    def zps(self):
        """ ZPS """
        if self._ZPS is None:
            self._ZPS = ZPS(self)
        return self._ZPS

    @property
    def hands(self):
        """ hands and arms controller """
        if self._hands is None:
            self._hands = Hands(self)
        return self._hands

    @property
    def uid(self):
        """ Get the next uid """
        self.msg_uid += 1
        return self.msg_uid

    @staticmethod
    def make_joint(joint_time, position):
        """ Build a joint tragectory message """
        point = TrajectoryPoint1DRosMessage()
        point.time = joint_time
        point.position = position
        point.velocity = 0

        joint = OneDoFJointTrajectoryRosMessage()
        joint.trajectory_points = [point]
        joint.unique_id = -1

        return joint

    @staticmethod
    def wait_for_publishers(publishers):
        """ Wait until a set of publishers are being listened to """
        while any([p.get_num_connections() == 0 for p in publishers]):
            log('waiting for subscribers: ' + ', '.join(sorted([
                p.name for p in publishers if p.get_num_connections() == 0])))
            rospy.sleep(0.1)

    @staticmethod
    def close_publishers(publishers):
        """ Close out a set of publishers """
        for p in publishers:
            p.unregister()

    def go_home(self, side, part):
        """ Send a go home """
        msg = GoHomeRosMessage()
        msg.unique_id = self.uid
        msg.body_part = part
        msg.robot_side = side
        msg.trajectory_time = 0.2
        self.home_publisher.publish(msg)
