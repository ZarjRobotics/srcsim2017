#!/usr/bin/env python
""" Team ZARJ grab test """

# Just turn the torso

import rospy
import sys
from copy import deepcopy

sys.path.append('../lib')
sys.path.append('./lib')

from zarj.zarjos import ZarjOS
from srcsim.msg import Task

from ihmc_msgs.msg import SE3TrajectoryPointRosMessage
from ihmc_msgs.msg import HandTrajectoryRosMessage

from geometry_msgs.msg import Vector3
from numpy import dot, sign
from tf.transformations import quaternion_matrix

class Main(object):
    """ Class to perform all main logic. """

    def __init__(self, name):
        rospy.init_node(name)

        if not rospy.has_param('/ihmc_ros/robot_name'):
            rospy.logerr("Cannot run team_zarj_main.py, missing parameters!")
            rospy.logerr("Missing parameter '/ihmc_ros/robot_name'")

        else:
            self.zarj = ZarjOS()
            self.hand_trajectory_publisher = rospy.Publisher(
                "/ihmc_ros/{0}/control/hand_trajectory".format(self.zarj.robot_name),
                HandTrajectoryRosMessage, queue_size=10)

    def task_status(self, status_msg):
        """ Handle a task status messasge from srcsim """
        #rospy.loginfo("Chatty status")

    def run(self, angle):
        """ Run our code """
        rospy.loginfo("Start test code")
        self.task_subscriber = rospy.Subscriber("/srcsim/finals/task", Task, self.task_status)

        rate = rospy.Rate(1) # 10hz
        if self.task_subscriber.get_num_connections() == 0:
            rospy.loginfo('waiting for task publisher...')
            while self.task_subscriber.get_num_connections() == 0:
                rate.sleep()
        rospy.loginfo('...done.  Moving on to main action.')

        self.zarj.pelvis.turn_body_to(angle)
    

if __name__ == '__main__':
    arg1 = 0.0
    if len(sys.argv) > 1:
        arg1 = float(sys.argv[1])
    try:
        Main('turnbody').run(arg1)
    except rospy.ROSInterruptException:
        pass
