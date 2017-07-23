#!/usr/bin/env python
""" Team ZARJ task test utility """

import rospy
import sys
import os
import threading

from srcsim.msg import Task

class Main(object):
    """ Class to perform all main logic. """

    def __init__(self, name):
        rospy.init_node(name)

        self.task = None

        if not rospy.has_param('/ihmc_ros/robot_name'):
            rospy.logerr("Cannot run, missing /ihmc_ros/robot_name")

    def recv_task(self, msg):
        """ Handle a task message """
        self.task = msg

    def run(self, task, checkpoint):
        """ Run our code """
        #print("Start looking for a task message")

        sub = rospy.Subscriber(
            "/srcsim/finals/task", Task, self.recv_task)

        rate = rospy.Rate(10) # 10hz
        for i in range(10):
            if sub.get_num_connections() == 0:
                rate.sleep()

        if sub.get_num_connections() == 0:
            sub.unregister()
            #print("No connection found; not done.")
            return 1

        rospy.sleep(0.5)

        sub.unregister()
        if self.task is None:
            print("No task info received")
            return 2

        if self.task.task > int(task):
            print "Finished; task is now {}".format(self.task.task)
            return 0

        if self.task.task == int(task) and self.task.current_checkpoint >= int(checkpoint):
            print "Finished; task is now {}; checkpoint now {}".format(self.task.task, self.task.current_checkpoint)
            return 0

        #print "Failure; task {} checkpoint {}".format(self.task.task, self.task.current_checkpoint)
        return 2

def suicide():
    print "No connection made to ros in 60 seconds"
    os._exit(3)

if __name__ == '__main__':
    threading.Timer(60, suicide).start()
    try:
        if len(sys.argv) < 3:
            print "Error: specify task and checkpoint."
            rc = -1
        else:
            rc = Main('team_zarj_finished').run(sys.argv[1], sys.argv[2])
    except:
        print "Aborted"
        rc = -1
    os._exit(rc)
