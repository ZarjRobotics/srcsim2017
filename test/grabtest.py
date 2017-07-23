#!/usr/bin/env python
""" Team ZARJ grab test """

# Misc hand tests

import rospy
import sys
from copy import deepcopy

sys.path.append('../lib')
sys.path.append('./lib')

from zarj.zarjos import ZarjOS
from zarj.tf_conv import v_to_msg_v

from math import sin, cos, acos, sqrt, degrees, radians


class Main(object):
    """ Class to perform all main logic. """

    def __init__(self, name):
        rospy.init_node(name)

        if not rospy.has_param('/ihmc_ros/robot_name'):
            rospy.logerr("Cannot run team_zarj_main.py, missing parameters!")
            rospy.logerr("Missing parameter '/ihmc_ros/robot_name'")

        else:
            self.zarj = ZarjOS()
            #self.hand_trajectory_publisher = rospy.Publisher(
            #    "/ihmc_ros/{0}/control/hand_trajectory".format(self.zarj.robot_name),
            #    HandTrajectoryRosMessage, queue_size=10)

    def task_status(self, status_msg):
        """ Handle a task status messasge from srcsim """
        #rospy.loginfo("Chatty status")


    def cmd(self, x, y, z, orientation):
        rospy.loginfo("Start test code")
        sidename = 'right'
        if orientation == 'ypr':
            self.zarj.hands.move_with_yaw_pitch_roll(sidename, v_to_msg_v([0,0,0]), [x, y, z])
        else:
            self.zarj.hands.move_hand_center_rel(sidename, v_to_msg_v([x, y, z]), orientation)

if __name__ == '__main__':
    arg1 = 0.0
    arg2 = 0.0
    arg3 = 0.0
    arg4 = ""
    if len(sys.argv) == 1:
        print """
        The general format of the command is:
          ./grabtest.py x_offset y_offset z_offset orientation_name
        The offsets move the hand in the reference frame of the torso. The orientation_name
        can be:
            <blank> -- No value indicates no change in orientation.
            none -- Zero orientation. Leaves hand in handshake position pointing forward from
                front of robot.
            faceup -- Turns hand faceup.
            facedown -- Turns hand facedown.
        However, if the orientation name is *ypr* (for yaw, pitch, roll) then the x_offset, y_offset,
        and z_offset numbers are interpreted as yaw, pitch, and roll respetively in degrees. Here are some
        example commands.
            
            ./grabtest.py 0.1 0 0 none  # Moves the hand forward from the front of the robot and puts into handshake position.
            ./grabtest.py 0 0 0.2       # Moves the hand up by 20cm and leaves hand orientation alone.
            ./grabtest.py 20 0 0 ypr    # Rotates the hand to the left 20 degrees.
            ./grabtest.py 0 20 0 ypr    # Rotates the hand downward by 20 degrees.
            ./grabtest.py 0 0 20 ypr    # Rolls the hand to the right (left thumb of right hand goes from left to up).
        """
        exit(0)
    if len(sys.argv) > 1:
        arg1 = float(sys.argv[1])
    if len(sys.argv) > 2:
        arg2 = float(sys.argv[2])
    if len(sys.argv) > 3:
        arg3 = float(sys.argv[3])
    if len(sys.argv) > 4:
        arg4 = sys.argv[4]
    try:
        Main('grab_test').cmd(arg1, arg2, arg3, arg4)
    except rospy.ROSInterruptException:
        pass
