#!/usr/bin/env python

# Team ZARJ Qualification 2 script

import time
import rospy
import os
import tf
import tf2_ros
import sys
import math

import numpy as np

import tf2_geometry_msgs
sys.path.append('../lib')
sys.path.append('./lib')
import zarj.tf
import zarj.eyes


from copy import deepcopy
from threading import Event

from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry

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

from std_msgs.msg import Int32


# Constants
LEFT = 0
RIGHT = 1
OVERRIDE = 0
QUEUE = 1

# Global variables
robot_name = None
lf_start_position = Vector3()
lf_start_orientation = Quaternion()
rf_start_position = Vector3()
rf_start_orientation = Quaternion()
first_x = -1.0
first_y = -1.0
right_palm_x = 0

id = 0
furthest_x = 0.0
start_time = 0.0
fell_down = False

standing_event = Event()
behavior = None

found_button = False

arm_style_names = [
    'mash',
    'king_tut',
    'aric_outreach']

arm_styles = [
    None,
    [ -1.4, 1.10, 1.65, 0.05, 1.7, 0.2, 0.2 ],
    [ -1.4, 0.75, 1.65, 0.55, 1.7, 0.2, 0.2 ]]

def uid():
    global id
    id += 1
    return id


def makeJoint(time, position):
    point = TrajectoryPoint1DRosMessage()
    point.time = time
    point.position = position

    joint = OneDoFJointTrajectoryRosMessage()
    joint.trajectory_points = [ point ]

    return joint

def make_arm_trajectory(side, mode, time, joints, previous_msg = None):
    trajectories = []
    for x in joints:
        trajectories.append(makeJoint(time, x))

    msg = ArmTrajectoryRosMessage()
    msg.robot_side = side
    msg.joint_trajectory_messages = trajectories
    msg.execution_mode = mode
    msg.unique_id = uid()
    if previous_msg is not None:
        msg.previous_message_id = previous_msg

    return msg


# The arm joints are as follows:
#   ShoulderPitch - immediate rotation, XXX TODO document more XXX
#   ShoulderRoll  - spin sindie first joint, affects left/right, 0 extended, 1.4 ahead
#   ShoulderYaw   - spinning at second joint, affects height.
#   ElbowPitch    - The orange joint.  lower is wider.
#   ForearmYaw, WristRoll, WristPitch - Don't seem to work, but values required
def plot_arms(backoff_time):
    king_tut = [ -1.4, 1.10, 1.65, 0.05, 1.7, 0.2, 0.2 ]
    aric_tut = [ -1.4, 0.75, 1.65, 0.55, 1.7, 0.2, 0.2 ]
    backoff = [ -1.1, 0.75, 1.65, 0.55, 1.7, 0.2, 0.2 ]
    backoff2 = [ -1.1, 1.25, 1.65, 0.85, 1.7, 0.2, 0.2 ]
    backoff3 = [ -1.1, 1.45, 1.0, 1.51, 1.7, 0.2, 0.2 ]
    leftarm = [ -1.4, -1.50, 0.2, -1.5121, 0.00504, 0.00021, 0.00283 ]

    #msg = make_arm_trajectory(RIGHT, OVERRIDE, 1.0, king_tut)
    msg = make_arm_trajectory(RIGHT, OVERRIDE, 1.0, aric_tut)
    rospy.loginfo('ArmTrajectory raise: uid' + str(msg.unique_id))
    armTrajectoryPublisher.publish(msg)

    # A delay in the first message seems to be required to get
    #  the queuing to work
    #rospy.sleep(0.1)
    rospy.sleep(0.1)

    # 12.5 worked with swing time 0.645, 13.0 was a hair high for 0.7
    #msg = make_arm_trajectory(RIGHT, QUEUE, 0.5, aric_tut, msg.unique_id)
    #hack
    msg = make_arm_trajectory(RIGHT, QUEUE, backoff_time, aric_tut, msg.unique_id)
    rospy.loginfo('ArmTrajectory hold : uid' + str(msg.unique_id))
    armTrajectoryPublisher.publish(msg)
    rospy.sleep(0.1)

    msg = make_arm_trajectory(RIGHT, QUEUE, 0.1, backoff, msg.unique_id)
    rospy.loginfo('ArmTrajectory backoff : uid' + str(msg.unique_id))
    armTrajectoryPublisher.publish(msg)
    rospy.sleep(0.1)

    msg = make_arm_trajectory(RIGHT, QUEUE, 0.1, backoff2, msg.unique_id)
    rospy.loginfo('ArmTrajectory backoff 2: uid' + str(msg.unique_id))
    armTrajectoryPublisher.publish(msg)
    rospy.sleep(0.1)

    msg = make_arm_trajectory(RIGHT, QUEUE, 0.2, backoff3, msg.unique_id)
    rospy.loginfo('ArmTrajectory backoff 3: uid' + str(msg.unique_id))
    armTrajectoryPublisher.publish(msg)
    rospy.sleep(0.1)

    msg = make_arm_trajectory(LEFT, OVERRIDE, 0.5, leftarm)
    rospy.loginfo('ArmTrajectory left arm: uid' + str(msg.unique_id))
    armTrajectoryPublisher.publish(msg)
    rospy.sleep(0.1)

 
def set_arms(style):
    if not style in arm_style_names:
        return
    index = arm_style_names.index(style)
    if arm_styles[index] is None:
        return
    msg = make_arm_trajectory(RIGHT, OVERRIDE, 1.0, arm_styles[index])
    rospy.loginfo('ArmTrajectory raise: uid' + str(msg.unique_id))
    armTrajectoryPublisher.publish(msg)

    # A delay in the first message seems to be required to get
    #  the queuing to work
    #rospy.sleep(0.1)

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

def mashButton():
    # Button pressed position gathered from Gazebo analysis
    #buttonPosition = Vector3(3.35, -0.73, 1.25)
    # we can subtract the length of the fingers... about 9 cm
    buttonPosition = Vector3(3.16, -0.73, 1.25)

    zarj_tf.transform_from_world(buttonPosition)
    handWorld = zarj_tf.lookup_frame_in_world('rightPalm', rospy.Time())

    hp1 = SE3TrajectoryPointRosMessage()
    hp1.time = 0.2
    hp1.position = deepcopy(handWorld.transform.translation)
    hp1.position.y = buttonPosition.y
    hp1.position.z = buttonPosition.z
    hp1.orientation = handWorld.transform.rotation

    hp2 = SE3TrajectoryPointRosMessage()
    hp2.time = 0.3
    hp2.position.x = buttonPosition.x
    hp2.position.y = buttonPosition.y
    hp2.position.z = buttonPosition.z
    hp2.orientation = handWorld.transform.rotation

    hp3 = SE3TrajectoryPointRosMessage()
    hp3.time = 0.6
    hp3.position = deepcopy(handWorld.transform.translation)
    hp3.position.y = buttonPosition.y
    hp3.position.z = buttonPosition.z
    hp3.orientation = handWorld.transform.rotation

    hp4 = SE3TrajectoryPointRosMessage()
    hp4.time = 0.8
    hp4.position = deepcopy(handWorld.transform.translation)
    hp4.orientation = handWorld.transform.rotation

    msg = HandTrajectoryRosMessage()
    msg.robot_side = msg.RIGHT
    msg.base_for_control = msg.WORLD
    msg.execution_mode = msg.OVERRIDE
    msg.taskspace_trajectory_points = [ hp1, hp2, hp3, hp4 ]
    msg.unique_id = uid()

    rospy.loginfo('AS Mash Button: uid' + str(msg.unique_id))
    handTrajectoryPublisher.publish(msg)

def lookup_feet(name, tf_buffer):
    global lf_start_position
    global lf_start_orientation
    global rf_start_position
    global rf_start_orientation

    lfp = "/ihmc_ros/{0}/left_foot_frame_name".format(name)
    rfp = "/ihmc_ros/{0}/right_foot_frame_name".format(name)

    if rospy.has_param(rfp) and rospy.has_param(lfp):
        lfname = rospy.get_param(lfp)
        rfname = rospy.get_param(rfp)

        leftFootWorld = tf_buffer.lookup_transform('world', lfname, rospy.Time())
        lf_start_orientation = leftFootWorld.transform.rotation
        lf_start_position = leftFootWorld.transform.translation

        rightFootWorld = tf_buffer.lookup_transform('world', rfname, rospy.Time())
        rf_start_orientation = rightFootWorld.transform.rotation
        rf_start_position = rightFootWorld.transform.translation


def create_one_footstep(side, offset):
    global lf_start_position
    global lf_start_orientation
    global rf_start_position
    global rf_start_orientation

    footstep = FootstepDataRosMessage()
    footstep.robot_side = side

    if side == LEFT:
        footstep.orientation = deepcopy(lf_start_orientation)
        footstep.location = deepcopy(lf_start_position)
    else:
        footstep.orientation = deepcopy(rf_start_orientation)
        footstep.location = deepcopy(rf_start_position)

    # JPW There are further parameters on feet that we
    #  should be able to play with.  This is supposedly the 'fast'
    #  trajectory
    footstep.trajectory_type = footstep.PUSH_RECOVERY

    footstep.location.x += offset[0]
    footstep.location.y += offset[1]
    footstep.location.z += offset[2]

    return footstep

def create_setup_footsteps(adjust_x=0.0, left_y=-0.08, stance_width=0.18):
    footsteps = []

    rightstep = FootstepDataRosMessage()
    rightstep.robot_side = RIGHT
    rightstep.orientation = deepcopy(rf_start_orientation)
    rightstep.location = deepcopy(rf_start_position)
    rightstep.trajectory_type = rightstep.PUSH_RECOVERY
    rightstep.origin = rightstep.AT_SOLE_FRAME
    rightstep.location.x = adjust_x
    rightstep.location.y = left_y - stance_width
    zarj_tf.transform_from_world(rightstep.location)
    footsteps.append(rightstep)

    leftstep = FootstepDataRosMessage()
    leftstep.robot_side = LEFT
    leftstep.orientation = deepcopy(lf_start_orientation)
    leftstep.location = deepcopy(lf_start_position)
    leftstep.trajectory_type = leftstep.PUSH_RECOVERY
    leftstep.origin = leftstep.AT_SOLE_FRAME
    leftstep.location.x = adjust_x
    leftstep.location.y = left_y
    zarj_tf.transform_from_world(leftstep.location)
    footsteps.append(leftstep)

    return footsteps

def create_footsteps(start_foot, stride, start_distance, max_distance):
    total_distance = start_distance
    foot = start_foot

    footsteps = []

    while (total_distance + .001) < max_distance:
        total_distance += stride
        if (total_distance >= max_distance):
            footsteps.append(create_one_footstep(foot, [max_distance, 0.0, 0.0]))
            break
        footsteps.append(create_one_footstep(foot, [total_distance, 0.0, 0.0]))
        if foot == LEFT:
            foot = RIGHT
        else:
            foot = LEFT

    return footsteps

def walk(transfer_time, swing_time, footsteps):
    msg = FootstepDataListRosMessage()

    msg.transfer_time = transfer_time
    msg.swing_time = swing_time
    msg.execution_mode = msg.OVERRIDE  # Override means replace others
    msg.unique_id = uid()

    msg.footstep_data_list = footsteps

    footStepListPublisher.publish(msg)
    rospy.loginfo('FootstepDataList: uid' + str(msg.unique_id))

def waitForFootsteps(numberOfSteps):
    global stepCounter
    while stepCounter < numberOfSteps:
        rate.sleep()
    rospy.loginfo('finished set of steps')

def recievedFootStepStatus(msg):
    global stepCounter
    if msg.status == 1:
        stepCounter += 1
        rospy.loginfo('completed step: ' + str(msg.footstep_index) + '; total: ' + str(stepCounter))


def receivedPose(msg):
    global first_x
    global first_y
    global furthest_x
    global start_time
    global fell_down
    global right_palm_x
    pose = msg.pose.pose
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    z = msg.pose.pose.position.z

    if first_x == -1.0:
        rospy.loginfo("Initial X Offset: " + str(x))
        first_x = x
    if first_y == -1.0:
        rospy.loginfo("Initial Y Offset: " + str(y))
        first_y = y
    if x > furthest_x:
        if furthest_x == 0:
            rospy.loginfo("Robot starting x: " + str(x))
        if x >= 0.5 and furthest_x < 0.5:
            rospy.loginfo("Robot crossed first line: " + str(x))
            start_time = rospy.get_time()
        if x >= 4.5 and furthest_x < 4.5:
            rospy.loginfo("Robot crossed second line: " + str(x))
            rospy.loginfo("Result: " + str(rospy.get_time() - start_time))

        furthest_x = x
    if z < 0.5 and not fell_down:
        fell_down = True
        rospy.loginfo("We fell down at:" + str(x))
        standing_event.set()

    if zarj_tf is not None:
        handWorld = zarj_tf.lookup_frame_in_world('rightPalm', rospy.Time())
        handpos = handWorld.transform.translation
        if handpos.x > 3.15 and right_palm_x < 3.15:
            print 'button?', handpos
        right_palm_x = handpos.x

def let_batch_know(signal):
    if 'BATCH_SIGNAL_DIR' in os.environ:
        open(os.environ['BATCH_SIGNAL_DIR'] + "/"+signal+".signal", "w")


    # JPW record of experiments.
    #   We seem to have three variables; transfer time, swing time,
    #    and stride length
    # Strides of 0.45 are too big; haven't tried in between
    #   The sample started with 1.5 for each
    #   .5 was working, .1 on both gets strange...
    #  transfer/swing/stride length - result
    #  .1/.5/.4 - robot fell down
    #  .3/.5/.4 - robot fell down
    #  .5/.5/.45 - robot falls down
    #  .2/.7/.45 - robot falls down
    #  .5/.5/.4 - 9.7 seconds (from walking forward to finished, 8 steps)
    # 1.5/1.5/.4 - 23.6 seconds
    #  .5/.4/.4 - robot fell down
    #  .2/.7/.4 - 8.013
    #  .2/.6/.4 - robot fell down
    #  .2/.65/.4 - 7.801  (hmm.  worked once, but not with arm up)
    #  .2/.7/.5 - fell down, but in maybe a promising way
    #  .3/.9/.8 - fell down spectacularly
    #  .1/.65/.4 - fell down

def zarj_run_qual2():
    global stepCounter
    stepCounter = 0

    transfer_time = 0.2
    swing_time = 0.7
    stride = 0.49

    delay_at_door = 0.355
    first_walk_distance = 2.9
    tuck_time = 0.1

    raise_arm = 'king_tut'
    left_foot_y = -0.04
    stance_width = 0.18

    backoff_time = 12.00

    if 'ZARJ_SWING_TIME' in os.environ:
        swing_time = float(os.environ['ZARJ_SWING_TIME'])
    if 'ZARJ_TRANSFER_TIME' in os.environ:
        transfer_time = float(os.environ['ZARJ_TRANSFER_TIME'])
    if 'ZARJ_STRIDE' in os.environ:
        stride = float(os.environ['ZARJ_STRIDE'])
    if 'ZARJ_FIRST_WALK_DISTANCE' in os.environ:
        first_walk_distance = float(os.environ['ZARJ_FIRST_WALK_DISTANCE'])
    if 'ZARJ_DELAY_AT_DOOR' in os.environ:
        delay_at_door = float(os.environ['ZARJ_DELAY_AT_DOOR'])
    if 'ZARJ_RAISE_ARM' in os.environ:
        raise_arm = os.environ['ZARJ_RAISE_ARM'].lower()
    if 'ZARJ_LEFT_FOOT_Y' in os.environ:
        left_foot_y = float(os.environ['ZARJ_LEFT_FOOT_Y'])
    if 'ZARJ_STANCE_WIDTH' in os.environ:
        stance_width = float(os.environ['ZARJ_STANCE_WIDTH'])
    if 'ZARJ_TUCK_TIME' in os.environ:
        tuck_time = float(os.environ['ZARJ_TUCK_TIME'])
    if 'ZARJ_BACKOFF_TIME' in os.environ:
        backoff_time = float(os.environ['ZARJ_BACKOFF_TIME'])
    rospy.loginfo("Configuration: {}, {}, {}, {}, {}, {}, {}, {}, {}, {}".format(
                  swing_time, transfer_time, stride, first_walk_distance, delay_at_door, tuck_time, raise_arm, left_foot_y, stance_width, backoff_time))

    steps = int(math.floor(first_walk_distance/stride))
    adjustment = first_walk_distance - (steps * stride)
    if adjustment > (stride / 2):
        adjustment = adjustment - stride
        steps += 1
    second_start = int(steps%2)
    rospy.loginfo(str(steps) + " steps")
    rospy.loginfo("second start " + str(second_start))
    rospy.loginfo("Adjustment: " + str(adjustment))

    rospy.sleep(0.05)
    if backoff_time > 0.1:
        plot_arms(backoff_time)

        # Quick hack - if the time is low, just return so we
        #   can examine the arm pose
        if backoff_time < 3.0:
            return
    else:
        set_arms(raise_arm)
    rospy.sleep(0.05)

    lookup_feet(robot_name, zarj_tf.tf_buffer)
    setup_footsteps = create_setup_footsteps(adjustment, left_foot_y, stance_width)
    walk(1.0, 1.5, setup_footsteps)
    if not standing_event.wait(60):
        rospy.loginfo("Error: setup footsteps did not complete.")
        let_batch_know("fail")
        return
    if fell_down:
        let_batch_know("fail")
        return
    standing_event.clear()
    rospy.sleep(0.01)

    handWorld = zarj_tf.lookup_frame_in_world('rightPalm', rospy.Time())
    handpos = handWorld.transform.translation
    print handpos

    lookup_feet(robot_name, zarj_tf.tf_buffer)

    first_walk_distance -= adjustment

    if backoff_time > 0.1:
        all_footsteps = create_footsteps(LEFT, stride, 0.0, 6.0)
        walk(transfer_time, swing_time, all_footsteps)
    else:
        door_footsteps = create_footsteps(LEFT, stride, 0.0, first_walk_distance)
        last_footsteps = create_footsteps(second_start, stride, first_walk_distance, 6.0)
        walk(transfer_time, swing_time, door_footsteps)

    if not standing_event.wait(88):
        let_batch_know("fail")
        rospy.loginfo("Error: first footsteps did not complete.")
        return
    standing_event.clear()
    if fell_down:
        let_batch_know("fail")
        return

    if backoff_time > 0.1:
        let_batch_know("done")
        return

    if raise_arm is 'mash':
        rospy.sleep(0.01)
        mashButton()
        rospy.sleep(1.0)

    if delay_at_door == 0.0:
        rospy.loginfo("Waiting for button push.")
        while right_palm_x < 3.15:
            rospy.sleep(0.01)
    else:
        rospy.sleep(delay_at_door - tuck_time)

    tuck_arms(tuck_time)
    rospy.sleep(0.01)

    walk(transfer_time, swing_time, last_footsteps)
    if not standing_event.wait(68):
        rospy.loginfo("Error: footsteps through the door did not complete.")
        let_batch_know("fail")
        return
    standing_event.clear()
    if fell_down:
        let_batch_know("fail")
        return

    let_batch_know("done")

def behaviorWatcher(data):
    global behavior
    if data != behavior:
        rospy.loginfo("Behavior switch to " + str(data))
        if data.data == 3 and behavior is not None and behavior.data == 4:
            standing_event.set()
        behavior = data

def eyes_found_red(name, center):
    global found_button
    # Button pressed position gathered from Gazebo analysis
    buttonPos = Vector3(3.35, -0.73, 1.25)
    offset = Vector3(center.point.x - buttonPos.x,
                     center.point.y - buttonPos.y,
                     center.point.z - buttonPos.z)
    zarj_eyes.remove_detection_request(name)
    zarj_tf.set_world_transform(offset)
    found_button = True

if __name__ == '__main__':
    try:
        rospy.init_node('team_zarj_qual2')

        if not rospy.has_param('/ihmc_ros/robot_name'):
            rospy.logerr("Cannot run team_zarj_qual2.py, missing parameters!")
            rospy.logerr("Missing parameter '/ihmc_ros/robot_name'")

        else:
            robot_name = rospy.get_param('/ihmc_ros/robot_name')

            right_foot_frame_parameter_name = "/ihmc_ros/{0}/right_foot_frame_name".format(robot_name)
            left_foot_frame_parameter_name = "/ihmc_ros/{0}/left_foot_frame_name".format(robot_name)

            if rospy.has_param(right_foot_frame_parameter_name) and rospy.has_param(left_foot_frame_parameter_name):
                control = "/ihmc_ros/{0}/control/".format(robot_name)
                output = "/ihmc_ros/{0}/output/".format(robot_name)

                BodyPublisher = rospy.Publisher(control + "whole_body_trajectory", WholeBodyTrajectoryRosMessage, queue_size=10)
                footStepListPublisher = rospy.Publisher(control + "footstep_list", FootstepDataListRosMessage, queue_size=10)
                handTrajectoryPublisher = rospy.Publisher(control + "hand_trajectory", HandTrajectoryRosMessage, queue_size=10)
                armTrajectoryPublisher = rospy.Publisher(control + "arm_trajectory", ArmTrajectoryRosMessage, queue_size=10)
                handDesiredConfigurationPublisher = rospy.Publisher(control + "hand_desired_configuration", HandDesiredConfigurationRosMessage, queue_size=10)
                goHomePublisher = rospy.Publisher(control + "go_home", GoHomeRosMessage, queue_size=10)

                zarj_tf = zarj.tf.TfZarj(robot_name)
                zarj_tf.init_transforms()

                # make sure our correlation_window_size is correct
                rospy.set_param('/multisense/camera/stereo_proc/correlation_window_size', 31)
                zarj_eyes = zarj.eyes.ZarjEyes('world', zarj_tf)

                lower_red = np.array([0, 100, 100])
                upper_red = np.array([10, 255, 255])
                red = zarj.eyes.DetectionRequest('red', lower_red, upper_red,
                    1, rospy.Duration.from_sec(0.1), eyes_found_red)
                zarj_eyes.add_detection_request(red)

                zarj_eyes.start_processing()

                rate = rospy.Rate(10) # 10hz

                while not found_button:
                    rospy.sleep(0.1)

                rospy.loginfo('World calibration done')


                footStepStatusSubscriber = rospy.Subscriber(output + "footstep_status", FootstepStatusRosMessage, recievedFootStepStatus)
                poseSubscriber = rospy.Subscriber(output + "robot_pose", Odometry, receivedPose)
                behaviorSubscriber = rospy.Subscriber(output + "behavior", Int32, behaviorWatcher)

                # make sure the simulation is running otherwise wait
                if footStepListPublisher.get_num_connections() == 0:
                    rospy.loginfo('waiting for subscriber...')
                    while footStepListPublisher.get_num_connections() == 0:
                        rate.sleep()

                if not rospy.is_shutdown():
                    zarj_run_qual2()
            else:
                if not rospy.has_param(left_foot_frame_parameter_name):
                    rospy.logerr("Missing parameter {0}".format(left_foot_frame_parameter_name))
                if not rospy.has_param(right_foot_frame_parameter_name):
                    rospy.logerr("Missing parameter {0}".format(right_foot_frame_parameter_name))

    except rospy.ROSInterruptException:
        pass
