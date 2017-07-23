#!/usr/bin/env python
""" Team ZARJ competition mainline """

import rospy
import sys

sys.path.append('../lib')
sys.path.append('./lib')

from zarj.zarjos import ZarjOS
from srcsim.msg import Task

class Main(object):
    """ Class to perform all main logic. """

    def __init__(self, name):
        rospy.init_node(name)

        if not rospy.has_param('/ihmc_ros/robot_name'):
            rospy.logerr("Cannot run team_zarj_main.py, missing parameters!")
            rospy.logerr("Missing parameter '/ihmc_ros/robot_name'")

        else:
            self.zarj_os = ZarjOS()

    def task_status(self, status_msg):
        """ Handle a task status messasge from srcsim """
        #rospy.loginfo("Chatty status")

    def run(self, distance, angle, snap_to, options):
        """ Run our code """
        rospy.loginfo("Start test code")
        self.task_subscriber = rospy.Subscriber("/srcsim/finals/task", Task, self.task_status)

        rate = rospy.Rate(1) # 10hz
        if self.task_subscriber.get_num_connections() == 0:
            rospy.loginfo('waiting for task publisher...')
            while self.task_subscriber.get_num_connections() == 0:
                rate.sleep()
    
        if distance > 0.0001 or distance < -0.005:
            self.zarj_os.walk.forward(distance, True)
            while not self.zarj_os.walk.walk_is_done():
                rospy.sleep(0.01)

        if abs(angle) > 0.0 or "turn" in options:
            align = "align" in options
            small_forward = 0.005 if align else None
            self.zarj_os.walk.turn(angle, snap_to, align_to_snap = align, small_forward = small_forward)
            while not self.zarj_os.walk.walk_is_done():
                rospy.sleep(0.01)


if __name__ == '__main__':
    arg1 = 0.0
    arg2 = 30.0
    arg3 = 0
    arg4 = ""
    if len(sys.argv) > 1:
        arg1 = float(sys.argv[1])
    if len(sys.argv) > 2:
        arg2 = float(sys.argv[2])
    if len(sys.argv) > 3:
        arg3 = float(sys.argv[3])
    if len(sys.argv) > 4:
        arg4 = sys.argv[4]
    try:
        Main('team_zarj_test').run(arg1, arg2, arg3, arg4)
    except rospy.ROSInterruptException:
        pass
