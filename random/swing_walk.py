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
from std_msgs.msg import String

lf_start_position = Vector3()
lf_start_orientation = Quaternion()
rf_start_position = Vector3()
rf_start_orientation = Quaternion()

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

def lift_right_foot():
    global rf_start_position
    global rf_start_orientation

    wholemsg = WholeBodyTrajectoryRosMessage()
    wholemsg.unique_id = uid()

    wholemsg.left_hand_trajectory_message.robot_side = 0
    wholemsg.right_hand_trajectory_message.robot_side = 1
    wholemsg.left_arm_trajectory_message.robot_side = 0
    wholemsg.right_arm_trajectory_message.robot_side = 1
    wholemsg.left_foot_trajectory_message.robot_side = 0
    wholemsg.right_foot_trajectory_message.robot_side = 1

    wholemsg.right_foot_trajectory_message.unique_id = wholemsg.unique_id
    wholemsg.right_foot_trajectory_message.previous_message_id = wholemsg.unique_id -1
    wholemsg.left_foot_trajectory_message.previous_message_id = wholemsg.unique_id -1
    wholemsg.right_foot_trajectory_message.execution_mode = 0
    wholemsg.left_foot_trajectory_message.execution_mode = 0

    zero = Vector3(0.0, 0.0, 0.0)

    p1 = SE3TrajectoryPointRosMessage()
    p1.time = .45
    p1.position = deepcopy(rf_start_position)
    p1.position.z += 0.05
    p1.position.x += 0
    p1.orientation = deepcopy(rf_start_orientation)
    p1.linear_velocity = zero
    p1.angular_velocity = zero

    p1a = SE3TrajectoryPointRosMessage()
    p1a.time = .85
    p1a.position = deepcopy(rf_start_position)
    p1a.position.z += 0.05
    p1a.position.x += 0.6
    p1a.orientation = deepcopy(rf_start_orientation)
    p1a.linear_velocity = zero
    p1a.angular_velocity = zero


    p2 = SE3TrajectoryPointRosMessage()
    p2.time = .95
    p2.position = deepcopy(rf_start_position)
    p2.position.z -= 0.01
    p2.position.x += 0.6
    p2.orientation = deepcopy(rf_start_orientation)
    p2.linear_velocity = zero
    p2.angular_velocity = zero

    wholemsg.right_foot_trajectory_message.taskspace_trajectory_points = [ p1, p1a, p2 ]

    rjoints = make_backward_arm(2.0, 1)

    wholemsg.right_arm_trajectory_message.joint_trajectory_messages = rjoints
    wholemsg.right_arm_trajectory_message.execution_mode = 0
    wholemsg.right_arm_trajectory_message.unique_id = wholemsg.unique_id
    wholemsg.right_arm_trajectory_message.previous_message_id = wholemsg.unique_id-1

    ljoints = make_forward_arm(2.0, 0)

    wholemsg.left_arm_trajectory_message.joint_trajectory_messages = ljoints
    wholemsg.left_arm_trajectory_message.execution_mode = 0
    wholemsg.left_arm_trajectory_message.unique_id = wholemsg.unique_id
    wholemsg.left_arm_trajectory_message.previous_message_id = wholemsg.unique_id-1

    rospy.loginfo('JPW lifting right foot: uid' + str(wholemsg.unique_id))

    BodyPublisher.publish(wholemsg)

def lift_left_foot(): 
    global lf_start_position
    global lf_start_orientation

    wholemsg = WholeBodyTrajectoryRosMessage()
    wholemsg.unique_id = uid()

    wholemsg.left_hand_trajectory_message.robot_side = 0
    wholemsg.right_hand_trajectory_message.robot_side = 1
    wholemsg.left_arm_trajectory_message.robot_side = 0
    wholemsg.right_arm_trajectory_message.robot_side = 1
    wholemsg.left_foot_trajectory_message.robot_side = 0
    wholemsg.right_foot_trajectory_message.robot_side = 1

    wholemsg.left_foot_trajectory_message.unique_id = wholemsg.unique_id
    wholemsg.left_foot_trajectory_message.execution_mode = 0

    zero = Vector3(0.0, 0.0, 0.0)

    p3 = SE3TrajectoryPointRosMessage()
    p3.time = 0.25
    p3.position = deepcopy(lf_start_position)
    p3.position.z += 0.05
    p3.position.x += 0
    p3.orientation = deepcopy(lf_start_orientation)
    p3.linear_velocity = zero
    p3.angular_velocity = zero

    p3a = SE3TrajectoryPointRosMessage()
    p3a.time = 0.45
    p3a.position = deepcopy(lf_start_position)
    p3a.position.z += 0.05
    p3a.position.x += 0.3
    p3a.orientation = deepcopy(lf_start_orientation)
    p3a.linear_velocity = zero
    p3a.angular_velocity = zero


    p4 = SE3TrajectoryPointRosMessage()
    p4.time = 0.65
    p4.position = deepcopy(lf_start_position)
    p4.position.z -= 0.01
    p4.position.x += 0.3
    p4.orientation = deepcopy(lf_start_orientation)
    p4.linear_velocity = zero
    p4.angular_velocity = zero

    wholemsg.left_foot_trajectory_message.taskspace_trajectory_points = [ p3, p3a, p4 ]

    rjoints = make_forward_arm(2.00, 1)

    wholemsg.right_arm_trajectory_message.joint_trajectory_messages = rjoints
    wholemsg.right_arm_trajectory_message.execution_mode = 0
    wholemsg.right_arm_trajectory_message.unique_id = wholemsg.unique_id

    ljoints = make_backward_arm(2.00, 0)

    wholemsg.left_arm_trajectory_message.joint_trajectory_messages = ljoints
    wholemsg.left_arm_trajectory_message.execution_mode = 0
    wholemsg.left_arm_trajectory_message.unique_id = wholemsg.unique_id


    rospy.loginfo('JPW lifting left foot: uid' + str(wholemsg.unique_id))

    BodyPublisher.publish(wholemsg)

# Creates footstep with the current position and orientation of the foot.
def lookupFeet(name, tfBuffer):
    global lf_start_position
    global lf_start_orientation
    global rf_start_position
    global rf_start_orientation

    lfp = "/ihmc_ros/{0}/left_foot_frame_name".format(name)
    rfp = "/ihmc_ros/{0}/right_foot_frame_name".format(name)

    if rospy.has_param(rfp) and rospy.has_param(lfp):
        lfname = rospy.get_param(lfp)
        rfname = rospy.get_param(rfp)

        leftFootWorld = tfBuffer.lookup_transform('world', lfname, rospy.Time())
        lf_start_orientation = leftFootWorld.transform.rotation
        lf_start_position = leftFootWorld.transform.translation
    
        rightFootWorld = tfBuffer.lookup_transform('world', rfname, rospy.Time())
        rf_start_orientation = rightFootWorld.transform.rotation
        rf_start_position = rightFootWorld.transform.translation


def statusRec(data):
    print "status:", str(data)

if __name__ == '__main__':
    try:
        rospy.init_node('zarj_walk_test')

        if not rospy.has_param('/ihmc_ros/robot_name'):
            rospy.logerr("Cannot run team_zarj_qual2.py, missing parameters!")
            rospy.logerr("Missing parameter '/ihmc_ros/robot_name'")

        else:
            robot_name = rospy.get_param('/ihmc_ros/robot_name')
            BodyPublisher = rospy.Publisher("/ihmc_ros/{0}/control/whole_body_trajectory".format(robot_name), WholeBodyTrajectoryRosMessage, queue_size=10)
            goHomePublisher = rospy.Publisher("/ihmc_ros/{0}/control/go_home".format(robot_name), GoHomeRosMessage, queue_size=10)
            #sub = rospy.Subscriber("/ihmc_ros/valkyrie/output/robot_motion_status", String, statusRec)

            tfBuffer = tf2_ros.Buffer()
            tfListener = tf2_ros.TransformListener(tfBuffer)

            rate = rospy.Rate(10) # 10hz
            id = rospy.get_rostime().secs

            # make sure the simulation is running otherwise wait
            if BodyPublisher.get_num_connections() == 0:
                rospy.loginfo('waiting for subscriber...')
                while BodyPublisher.get_num_connections() == 0:
                    rate.sleep()

            lookupFeet(robot_name, tfBuffer)

            if not rospy.is_shutdown():
                #goHome()
                #rospy.sleep(5)
                lift_left_foot()
                # JPW looks like we need to wait until robot_motion_status is standing,
                #   otherwise it is ignored.
                rospy.sleep(3.0)
                lift_right_foot()
                #rospy.sleep(2.0)

    except rospy.ROSInterruptException:
        pass

    rospy.signal_shutdown("done")
