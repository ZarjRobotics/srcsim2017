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
        self.zarj_os = ZarjOS()
        if not rospy.has_param('/ihmc_ros/robot_name'):
            rospy.logerr("Cannot run team_zarj_main.py, missing parameters!")
            rospy.logerr("Missing parameter '/ihmc_ros/robot_name'")
        else:
            self.zarj_os = ZarjOS()

    def task_status(self, status_msg):
        return

    def run(self, foot_type, x, y, angle):
        rospy.loginfo("Start test code")
        self.task_subscriber = rospy.Subscriber("/srcsim/finals/task", Task, self.task_status)

        rate = rospy.Rate(1) # 10hz
        if self.task_subscriber.get_num_connections() == 0:
            rospy.loginfo('waiting for task publisher...')
            while self.task_subscriber.get_num_connections() == 0:
                rate.sleep()
        ft = "left" if "l" in foot_type else "right"
        self.zarj_os.walk.move_foot(ft, x, y, angle)

if __name__ == '__main__':
    arg1 = "left"
    arg2 = 0.01
    arg3 = 0
    arg4 = 0
    if len(sys.argv) > 1:
        arg1 = sys.argv[1]
    if len(sys.argv) > 2:
        arg2 = float(sys.argv[2])
    if len(sys.argv) > 3:
        arg3 = float(sys.argv[3])
    if len(sys.argv) > 4:
        arg4 = float(sys.argv[4])
    try:
        Main('team_zarj_test').run(arg1, arg2, arg3, arg4)
    except rospy.ROSInterruptException:
        pass
