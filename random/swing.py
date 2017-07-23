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

def make_forward_arm(time, side):
    joints = []
    joints.append(makeJoint(time, -1.02))     # ShoulderPitch
    if side:
        joints.append(makeJoint(time, 0.95))     # ShoulderRoll
    else:
        joints.append(makeJoint(time, -0.95))     # ShoulderRoll
    if side:
        joints.append(makeJoint(time, 0.02))     # ShoulderYaw
    else:
        joints.append(makeJoint(time, -0.02))     # ShoulderYaw
    if side:
        joints.append(makeJoint(time, 1.57))     # ElbowPitch
    else:
        joints.append(makeJoint(time, -1.57))     # ElbowPitch
    #unused
    joints.append(makeJoint(time, 1.7))    # ForearmYaw (JPW experiment)
    joints.append(makeJoint(time, 0.2))    # WristRoll
    joints.append(makeJoint(time, 0.2))    # WristPitch
    return joints

def make_backward_arm(time, side):
    joints = []
    joints.append(makeJoint(time, 1.02))     # ShoulderPitch
    if side:
        joints.append(makeJoint(time, 0.95))     # ShoulderRoll
    else:
        joints.append(makeJoint(time, -0.95))     # ShoulderRoll
    if side:
        joints.append(makeJoint(time, 0.02))     # ShoulderYaw
    else:
        joints.append(makeJoint(time, -0.02))     # ShoulderYaw
    if side:
        joints.append(makeJoint(time, 1.57))     # ElbowPitch
    else:
        joints.append(makeJoint(time, -1.57))     # ElbowPitch
    #unused
    joints.append(makeJoint(time, 1.7))    # ForearmYaw (JPW experiment)
    joints.append(makeJoint(time, 0.2))    # WristRoll
    joints.append(makeJoint(time, 0.2))    # WristPitch
    return joints

def swing_arms():
    global stage
    wholemsg = WholeBodyTrajectoryRosMessage()
    wholemsg.unique_id = uid()

    stage = stage + 1

    side = stage%2

    #Shoulder
    if side:
        rjoints = make_forward_arm(0.4, 1)
    else:
        rjoints = make_backward_arm(0.4, 1)

    wholemsg.right_arm_trajectory_message.joint_trajectory_messages = rjoints
    wholemsg.right_arm_trajectory_message.execution_mode = 0
    wholemsg.right_arm_trajectory_message.unique_id = wholemsg.unique_id

    #Shoulder
    if side:
        ljoints = make_backward_arm(0.4, 0)
    else:
        ljoints = make_forward_arm(0.4, 0)

    wholemsg.left_arm_trajectory_message.joint_trajectory_messages = ljoints
    wholemsg.left_arm_trajectory_message.execution_mode = 0
    wholemsg.left_arm_trajectory_message.unique_id = wholemsg.unique_id

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
                while True:
                    swing_arms()
                    time.sleep(3)
                if not rospy.has_param(left_foot_frame_parameter_name):
                    rospy.logerr("Missing parameter {0}".format(left_foot_frame_parameter_name))
                if not rospy.has_param(right_foot_frame_parameter_name):
                    rospy.logerr("Missing parameter {0}".format(right_foot_frame_parameter_name))

    except rospy.ROSInterruptException:
        pass

    rospy.signal_shutdown("done")
