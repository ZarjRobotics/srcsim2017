#!/usr/bin/env python

# This is just a copy of the walk_test.py script, hacked
#  to prove I can...

import time
import rospy
import tf
import tf2_ros
import numpy
from copy import deepcopy

from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion

from ihmc_msgs.msg import GoHomeRosMessage
from ihmc_msgs.msg import TrajectoryPoint1DRosMessage
from ihmc_msgs.msg import OneDoFJointTrajectoryRosMessage
from ihmc_msgs.msg import HandTrajectoryRosMessage
from ihmc_msgs.msg import WholeBodyTrajectoryRosMessage
from ihmc_msgs.msg import HandDesiredConfigurationRosMessage
from ihmc_msgs.msg import ArmTrajectoryRosMessage
from ihmc_msgs.msg import SE3TrajectoryPointRosMessage
from ihmc_msgs.msg import FootstepStatusRosMessage
from ihmc_msgs.msg import FootstepDataListRosMessage
from ihmc_msgs.msg import FootstepDataRosMessage

LEFT = 0
RIGHT = 1

ROBOT_NAME = None

id = 0

def uid():
    global id
    id += 1
    return id

def goHome():
    msg = GoHomeRosMessage()

    msg.robot_side = msg.RIGHT
    msg.body_part = msg.ARM
    msg.trajectory_time = 0.5
    msg.unique_id = uid()

    goHomePublisher.publish(msg)

    msg.robot_side = msg.LEFT
    msg.unique_id = uid()
    rospy.loginfo('goHome: uid' + str(msg.unique_id))
    goHomePublisher.publish(msg)


def do_a_little_dance():
    wholemsg = WholeBodyTrajectoryRosMessage()
    wholemsg.unique_id = uid()

    rhandWorld = tfBuffer.lookup_transform('world', 'rightPalm', rospy.Time())
    lhandWorld = tfBuffer.lookup_transform('world', 'leftPalm', rospy.Time())

    hand_pt1 = SE3TrajectoryPointRosMessage()
    hand_pt1.time = 1.0
    hand_pt1.position = deepcopy(rhandWorld.transform.translation)
    hand_pt1.orientation = rhandWorld.transform.rotation
    hand_pt1.position.x += 0.5
    hand_pt1.position.y -= 0.25
    hand_pt1.position.z += 0.5
    hand_pt2 = SE3TrajectoryPointRosMessage()
    hand_pt2.time = 2.0  # How long to take to move the arm
    hand_pt2.position = deepcopy(rhandWorld.transform.translation)
    hand_pt2.orientation = rhandWorld.transform.rotation
    hand_pt2.position.x -= 0.5
    hand_pt2.position.y -= 0.25
    hand_pt2.position.z -= 0.5
    hand_pt3 = SE3TrajectoryPointRosMessage()
    hand_pt3.time = 3.0  # How long to take to move the arm
    hand_pt3.position = deepcopy(rhandWorld.transform.translation)
    hand_pt3.orientation = rhandWorld.transform.rotation
    hand_pt3.position.x += 0.25
    hand_pt3.position.y += 0.25
    hand_pt3b = SE3TrajectoryPointRosMessage()
    hand_pt3b.time = 4.0  # How long to take to move the arm
    hand_pt3b.position = deepcopy(rhandWorld.transform.translation)
    hand_pt3b.orientation = rhandWorld.transform.rotation
    hand_pt3b.position.x -= 0.25
    hand_pt3b.position.y += 0.25
    hand_pt3b.position.z += 1
    wholemsg.right_hand_trajectory_message.taskspace_trajectory_points = [hand_pt1, hand_pt2, hand_pt3, hand_pt3b]


    hand_pt4 = SE3TrajectoryPointRosMessage()
    hand_pt4.time = 1.0  # How long to take to move the arm
    hand_pt4.position = deepcopy(lhandWorld.transform.translation)
    hand_pt4.orientation = lhandWorld.transform.rotation
    hand_pt4.position.x -= 0.5
    hand_pt4.position.y += 0.25
    hand_pt4.position.z -= 0.5
    hand_pt5 = SE3TrajectoryPointRosMessage()
    hand_pt5.time = 2.0  # How long to take to move the arm
    hand_pt5.position = deepcopy(lhandWorld.transform.translation)
    hand_pt5.orientation = lhandWorld.transform.rotation
    hand_pt5.position.x += 0.5
    hand_pt5.position.y += 0.25
    hand_pt5.position.z += 0.5
    hand_pt6 = SE3TrajectoryPointRosMessage()
    hand_pt6.time = 3.0  # How long to take to move the arm
    hand_pt6.position = deepcopy(lhandWorld.transform.translation)
    hand_pt6.orientation = lhandWorld.transform.rotation
    hand_pt6.position.x += 0.25
    hand_pt6.position.y -= 0.25
    hand_pt7 = SE3TrajectoryPointRosMessage()
    hand_pt7.time = 4.0  # How long to take to move the arm
    hand_pt7.position = deepcopy(lhandWorld.transform.translation)
    hand_pt7.orientation = lhandWorld.transform.rotation
    hand_pt7.position.x -= 0.25
    hand_pt7.position.y += 0.25
    hand_pt7.position.z += 1
    wholemsg.left_hand_trajectory_message.taskspace_trajectory_points = [hand_pt4, hand_pt5, hand_pt6, hand_pt7]

    wholemsg.right_hand_trajectory_message.base_for_control = wholemsg.right_hand_trajectory_message.WORLD
    wholemsg.right_hand_trajectory_message.execution_mode = wholemsg.right_hand_trajectory_message.OVERRIDE
    wholemsg.right_hand_trajectory_message.unique_id = wholemsg.unique_id

    wholemsg.left_hand_trajectory_message.base_for_control = wholemsg.left_hand_trajectory_message.WORLD
    wholemsg.left_hand_trajectory_message.execution_mode = wholemsg.left_hand_trajectory_message.OVERRIDE
    wholemsg.left_hand_trajectory_message.unique_id = wholemsg.unique_id

    wholemsg.left_hand_trajectory_message.robot_side = 0
    wholemsg.right_hand_trajectory_message.robot_side = 1
    wholemsg.left_arm_trajectory_message.robot_side = 0
    wholemsg.right_arm_trajectory_message.robot_side = 1
    wholemsg.left_foot_trajectory_message.robot_side = 0
    wholemsg.right_foot_trajectory_message.robot_side = 1

    #wholemsg.chest_trajectory_message
    #wholemsg.pelvis_trajectory_message

    rospy.loginfo('Positioning Body: uid' + str(wholemsg.unique_id))
    BodyPublisher.publish(wholemsg)

if __name__ == '__main__':
    try:
        rospy.init_node('team_zarj_qual2')

        if not rospy.has_param('/ihmc_ros/robot_name'):
            rospy.logerr("Cannot run team_zarj_qual2.py, missing parameters!")
            rospy.logerr("Missing parameter '/ihmc_ros/robot_name'")

        else:
            ROBOT_NAME = rospy.get_param('/ihmc_ros/robot_name')
            BodyPublisher = rospy.Publisher("/ihmc_ros/{0}/control/whole_body_trajectory".format(ROBOT_NAME), WholeBodyTrajectoryRosMessage, queue_size=10)
            goHomePublisher = rospy.Publisher("/ihmc_ros/{0}/control/go_home".format(ROBOT_NAME), GoHomeRosMessage, queue_size=10)


            tfBuffer = tf2_ros.Buffer()
            tfListener = tf2_ros.TransformListener(tfBuffer)

            rate = rospy.Rate(10) # 10hz
            time.sleep(2)
            time.sleep(1)
            id = rospy.get_rostime().secs

            if not rospy.is_shutdown():
                goHome()
                time.sleep(2)
                do_a_little_dance()
            else:
                if not rospy.has_param(left_foot_frame_parameter_name):
                    rospy.logerr("Missing parameter {0}".format(left_foot_frame_parameter_name))
                if not rospy.has_param(right_foot_frame_parameter_name):
                    rospy.logerr("Missing parameter {0}".format(right_foot_frame_parameter_name))

    except rospy.ROSInterruptException:
        pass

    rospy.signal_shutdown("done")
