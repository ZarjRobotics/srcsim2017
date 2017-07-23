#!/usr/bin/env python
""" Team ZARJ harness test utility """

import rospy
import sys
import os
import threading

from srcsim.msg import Harness

class Main(object):
    """ Class to perform all main logic. """

    def __init__(self, name):
        rospy.init_node(name)

        self.harness = None

        if not rospy.has_param('/ihmc_ros/robot_name'):
            rospy.logerr("Cannot run, missing /ihmc_ros/robot_name")

    def recv_harness(self, msg):
        """ Handle a harness message """
        self.harness = msg.status

    def run(self):
        """ Run our code """
        #print("Start looking for a harness")

        sub = rospy.Subscriber(
            "/srcsim/finals/harness", Harness, self.recv_harness)

        rate = rospy.Rate(10) # 10hz
        for i in range(10):
            if sub.get_num_connections() == 0:
                rate.sleep()

        if sub.get_num_connections() == 0:
            sub.unregister()
            #print("No connection found; not harnessed.")
            return 1

        rospy.sleep(0.5)

        # Okay, if we did not get a harness message at all,
        #  then let's chill for a while.  If we still get nothing,
        #  assume that we missed the '0' message
        if self.harness is None:
            rospy.sleep(2.0)

        sub.unregister()
        if self.harness == 0:
            print("Actually heard the harnessed message!")
            return 0
        if self.harness is None:
            print("Connected, and no harness messages; assuming harnessed at {}".format(rospy.get_time()))
            return 0

        #print("Harness currently {} at {}".format(self.harness, rospy.get_time()))
        return 2

def suicide():
    print "No connection made to ros in 60 seconds"
    os._exit(3)

if __name__ == '__main__':
    threading.Timer(60, suicide).start()
    try:
        rc = Main('team_zarj_harness').run()
    except:
        print "Aborted"
        rc = -1
    os._exit(rc)
