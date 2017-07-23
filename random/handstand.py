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
from std_msgs.msg import String

from ihmc_msgs.msg import AbortWalkingRosMessage
from ihmc_msgs.msg import EndEffectorLoadBearingRosMessage
from ihmc_msgs.msg import StopAllTrajectoryRosMessage
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

lf_start_position = Vector3()
lf_start_orientation = Quaternion()
rf_start_position = Vector3()
rf_start_orientation = Quaternion()

id = 0
stage = 0
motion_status = ""

def uid():
    global id
    id += 1
    return id

def lift_both_feet():
    global rf_start_position
    global rf_start_orientation
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

    #wholemsg.right_foot_trajectory_message.unique_id = wholemsg.unique_id
    #wholemsg.right_foot_trajectory_message.execution_mode = wholemsg.right_foot_trajectory_message.OVERRIDE

    zero = Vector3(0.0, 0.0, 0.0)

    p1 = SE3TrajectoryPointRosMessage()
    p1.time = .25
    p1.position = deepcopy(rf_start_position)
    p1.position.z += 0.05
    p1.position.x += 0.2
    p1.orientation = deepcopy(rf_start_orientation)
    #p1.orientation.z += 0.2
    p1.linear_velocity = zero
    p1.angular_velocity = zero

    p2 = SE3TrajectoryPointRosMessage()
    p2.time = .5
    p2.position = deepcopy(rf_start_position)
    p2.position.z -= 0.01
    p2.position.x += 0.4
    p2.orientation = deepcopy(rf_start_orientation)
    #p2.orientation.z += 0.2
    p2.linear_velocity = zero
    p2.angular_velocity = zero

    #wholemsg.right_foot_trajectory_message.taskspace_trajectory_points = [ p1, p2 ]

    wholemsg.left_foot_trajectory_message.unique_id = wholemsg.unique_id
    wholemsg.left_foot_trajectory_message.execution_mode = wholemsg.left_foot_trajectory_message.OVERRIDE

    pstand = SE3TrajectoryPointRosMessage()
    pstand.time = 0.5
    pstand.position = deepcopy(lf_start_position)
    pstand.orientation = deepcopy(lf_start_orientation)
    pstand.linear_velocity = zero
    pstand.angular_velocity = zero

    p3 = SE3TrajectoryPointRosMessage()
    p3.time = 0.75
    p3.position = deepcopy(lf_start_position)
    p3.position.z += 0.05
    p3.position.x += 0.2
    p3.orientation = deepcopy(lf_start_orientation)
    #p3.orientation.z += 0.2
    p3.linear_velocity = zero
    p3.angular_velocity = zero

    p4 = SE3TrajectoryPointRosMessage()
    p4.time = 1.0
    p4.position = deepcopy(lf_start_position)
    p4.position.z -= 0.01
    p4.position.x += 0.4
    p4.orientation = deepcopy(lf_start_orientation)
    #p4.orientation.z += 0.2

    p4.linear_velocity = zero
    p4.angular_velocity = zero

    wholemsg.left_foot_trajectory_message.taskspace_trajectory_points = [ p3, p4 ]

    rospy.loginfo('JPW lifting both feet: uid' + str(wholemsg.unique_id))

    BodyPublisher.publish(wholemsg)

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
    wholemsg.right_foot_trajectory_message.execution_mode = wholemsg.right_foot_trajectory_message.OVERRIDE

    zero = Vector3(0.0, 0.0, 0.0)

    p1 = SE3TrajectoryPointRosMessage()
    p1.time = .25
    p1.position = deepcopy(rf_start_position)
    p1.position.z += 0.05
    p1.position.x += 0.2
    p1.orientation = deepcopy(rf_start_orientation)
    #p1.orientation.z += 0.2
    p1.linear_velocity = zero
    p1.angular_velocity = zero

    p2 = SE3TrajectoryPointRosMessage()
    p2.time = .5
    p2.position = deepcopy(rf_start_position)
    p2.position.z -= 0.01
    p2.position.x += 0.4
    p2.orientation = deepcopy(rf_start_orientation)
    #p2.orientation.z += 0.2
    p2.linear_velocity = zero
    p2.angular_velocity = zero

    wholemsg.right_foot_trajectory_message.taskspace_trajectory_points = [ p1, p2 ]

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
    wholemsg.left_foot_trajectory_message.execution_mode = wholemsg.left_foot_trajectory_message.OVERRIDE

    zero = Vector3(0.0, 0.0, 0.0)

    p3 = SE3TrajectoryPointRosMessage()
    p3.time = 0.25
    p3.position = deepcopy(lf_start_position)
    p3.position.z += 0.05
    p3.position.x += 0.2
    p3.orientation = deepcopy(lf_start_orientation)
    #p3.orientation.z += 0.2

    p3.linear_velocity = zero
    p3.angular_velocity = zero

    p4 = SE3TrajectoryPointRosMessage()
    p4.time = 0.5
    p4.position = deepcopy(lf_start_position)
    p4.position.z -= 0.01
    p4.position.x += 0.4
    p4.orientation = deepcopy(lf_start_orientation)
    #p4.orientation.z += 0.2

    p4.linear_velocity = zero
    p4.angular_velocity = zero

    wholemsg.left_foot_trajectory_message.taskspace_trajectory_points = [ p3, p4 ]

    rospy.loginfo('JPW lifting left foot: uid' + str(wholemsg.unique_id))

    BodyPublisher.publish(wholemsg)

def received_motion_status(msg):
    global motion_status
    if motion_status != msg:
        motion_status = msg
        rospy.loginfo("New motion status " + str(motion_status))

def abort_walking():
    msg = AbortWalkingRosMessage()
    msg.unique_id = uid()
    rospy.loginfo('JPW aborting walk: uid' + str(msg.unique_id))
    AbortPublisher.publish(msg)

def stop_all_trajectories():
    msg = StopAllTrajectoryRosMessage()
    msg.unique_id = uid()
    rospy.loginfo('JPW stopping all trajectories: uid' + str(msg.unique_id))
    StopPublisher.publish(msg)

def unload_right_foot():
    msg = EndEffectorLoadBearingRosMessage()
    msg.unique_id = uid()
    msg.robot_side = msg.RIGHT
    msg.end_effector = msg.FOOT
    msg.request = msg.UNLOAD
    rospy.loginfo('JPW unload right foot : uid' + str(msg.unique_id))
    EndEffectorPublisher.publish(msg)

def unload_left_foot():
    msg = EndEffectorLoadBearingRosMessage()
    msg.unique_id = uid()
    msg.robot_side = msg.LEFT
    msg.end_effector = msg.FOOT
    msg.request = msg.UNLOAD
    rospy.loginfo('JPW unload left foot : uid' + str(msg.unique_id))
    EndEffectorPublisher.publish(msg)

def stand_on_right_hand():
    msg = EndEffectorLoadBearingRosMessage()
    msg.unique_id = uid()
    msg.robot_side = msg.RIGHT
    msg.end_effector = msg.HAND
    msg.request = msg.LOAD
    rospy.loginfo('JPW standing on right hand : uid' + str(msg.unique_id))
    EndEffectorPublisher.publish(msg)

def stand_on_left_hand():
    msg = EndEffectorLoadBearingRosMessage()
    msg.unique_id = uid()
    msg.robot_side = msg.LEFT
    msg.end_effector = msg.HAND
    msg.request = msg.LOAD
    rospy.loginfo('JPW standing on left hand : uid' + str(msg.unique_id))
    EndEffectorPublisher.publish(msg)

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

if __name__ == '__main__':
    try:
        rospy.init_node('zarj_walk_test')

        if not rospy.has_param('/ihmc_ros/robot_name'):
            rospy.logerr("Cannot run team_zarj_qual2.py, missing parameters!")
            rospy.logerr("Missing parameter '/ihmc_ros/robot_name'")

        else:
            robot_name = rospy.get_param('/ihmc_ros/robot_name')
            BodyPublisher = rospy.Publisher("/ihmc_ros/{0}/control/whole_body_trajectory".format(robot_name), WholeBodyTrajectoryRosMessage, queue_size=20)
            AbortPublisher = rospy.Publisher("/ihmc_ros/{0}/control/abort_walking".format(robot_name), AbortWalkingRosMessage, queue_size=10)
            StopPublisher = rospy.Publisher("/ihmc_ros/{0}/control/stop_all_trajectories".format(robot_name), StopAllTrajectoryRosMessage, queue_size=10)
            EndEffectorPublisher = rospy.Publisher("/ihmc_ros/{0}/control/end_effector_load_bearing".format(robot_name), EndEffectorLoadBearingRosMessage, queue_size=10)
            motionSubscriber = rospy.Subscriber("/ihmc_ros/{0}/output/robot_motion_status".format(robot_name), String, received_motion_status)

            tfBuffer = tf2_ros.Buffer()
            tfListener = tf2_ros.TransformListener(tfBuffer)

            rate = rospy.Rate(10) # 10hz
            id = rospy.get_rostime().secs

            # make sure the simulation is running otherwise wait
            if BodyPublisher.get_num_connections() == 0:
                rospy.loginfo('waiting for subscriber...')
                while BodyPublisher.get_num_connections() == 0:
                    rate.sleep()

            rospy.loginfo('looking up feet...')
            lookupFeet(robot_name, tfBuffer)

            if not rospy.is_shutdown():
                #lift_right_foot()
                #rospy.sleep(2.0)
                #lift_left_foot()
                #rospy.sleep(2.0)
                #stand_on_left_foot()
                rospy.sleep(0.1)
                lift_right_foot()
                rospy.sleep(1.0)
                unload_left_foot()
                rospy.sleep(1.0)
                stand_on_right_hand()
                #lift_both_feet()
                # JPW looks like we need to wait until robot_motion_status is standing,
                #   otherwise it is ignored.
                rospy.sleep(4.0)

    except rospy.ROSInterruptException:
        pass

    rospy.signal_shutdown("done")
