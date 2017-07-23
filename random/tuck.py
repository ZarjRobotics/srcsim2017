#!/usr/bin/env python

# Experimentations in tucking arms

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
stage = 0

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

def makeJoint(time, position):
    point = TrajectoryPoint1DRosMessage()
    point.time = time
    point.position = position

    joint = OneDoFJointTrajectoryRosMessage()
    joint.trajectory_points = [ point ]

    return joint

def tuck_left_arm(time):
    joints = []
    joints.append(makeJoint(time, -0.2000))     # ShoulderPitch
    joints.append(makeJoint(time, -1.5182))     # ShoulderRoll
    joints.append(makeJoint(time, 0.41400))     # ShoulderYaw
    joints.append(makeJoint(time, -1.5121))     # ElbowPitch
    joints.append(makeJoint(time, 0.00504))    # ForearmYaw (JPW experiment)
    joints.append(makeJoint(time, 0.00021))    # WristRoll
    joints.append(makeJoint(time, 0.00283))    # WristPitch
    return joints

def tuck_right_arm(time):
    joints = []
    joints.append(makeJoint(time, -0.2000))     # ShoulderPitch
    joints.append(makeJoint(time,  1.5182))     # ShoulderRoll
    joints.append(makeJoint(time, 0.41400))     # ShoulderYaw
    joints.append(makeJoint(time,  1.5121))     # ElbowPitch
    joints.append(makeJoint(time, 0.00504))    # ForearmYaw (JPW experiment)
    joints.append(makeJoint(time, -0.00021))    # WristRoll
    joints.append(makeJoint(time, -0.00283))    # WristPitch
    return joints

def tuck_arms(time):
    wholemsg = WholeBodyTrajectoryRosMessage()
    wholemsg.unique_id = uid()

    rjoints = tuck_right_arm(time)

    wholemsg.right_arm_trajectory_message.joint_trajectory_messages = rjoints
    wholemsg.right_arm_trajectory_message.execution_mode = 0
    wholemsg.right_arm_trajectory_message.unique_id = wholemsg.unique_id

    ljoints = tuck_left_arm(time)

    wholemsg.left_arm_trajectory_message.joint_trajectory_messages = ljoints
    wholemsg.left_arm_trajectory_message.execution_mode = 0
    wholemsg.left_arm_trajectory_message.unique_id = wholemsg.unique_id

    wholemsg.left_hand_trajectory_message.robot_side = 0
    wholemsg.right_hand_trajectory_message.robot_side = 1
    wholemsg.left_arm_trajectory_message.robot_side = 0
    wholemsg.right_arm_trajectory_message.robot_side = 1
    wholemsg.left_foot_trajectory_message.robot_side = 0
    wholemsg.right_foot_trajectory_message.robot_side = 1

    rospy.loginfo('Tucking arm: uid' + str(wholemsg.unique_id))
    BodyPublisher.publish(wholemsg)

if __name__ == '__main__':
    try:
        rospy.init_node('team_zarj_tuck')

        if not rospy.has_param('/ihmc_ros/robot_name'):
            rospy.logerr("Cannot run tuck.py, missing parameters!")
            rospy.logerr("Missing parameter '/ihmc_ros/robot_name'")

        else:
            ROBOT_NAME = rospy.get_param('/ihmc_ros/robot_name')
            BodyPublisher = rospy.Publisher("/ihmc_ros/{0}/control/whole_body_trajectory".format(ROBOT_NAME), WholeBodyTrajectoryRosMessage, queue_size=10)
            goHomePublisher = rospy.Publisher("/ihmc_ros/{0}/control/go_home".format(ROBOT_NAME), GoHomeRosMessage, queue_size=10)

            tfBuffer = tf2_ros.Buffer()
            tfListener = tf2_ros.TransformListener(tfBuffer)

            id = rospy.get_rostime().secs
            rospy.sleep(0.5)

            if not rospy.is_shutdown():
                goHome()
                rospy.sleep(0.25)
                tuck_arms(0.4)
                rospy.sleep(0.25)

    except rospy.ROSInterruptException:
        pass

    rospy.signal_shutdown("done")
