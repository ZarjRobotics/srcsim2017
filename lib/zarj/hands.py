"""
    Hands and arms logic for the robot

"""
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_multiply
from tf.transformations import quaternion_about_axis
from .tf_conv import q_to_msg_q
from .tf_conv import msg_q_to_q
from .tf_conv import mult_q
from .tf_conv import msg_v_to_v
from .tf_conv import conjugate_q
from .tf_conv import v_to_msg_v
from .tf_conv import add_v
from .tf_conv import sub_v
from .tf_conv import rotate_v
from .tf_conv import q_to_yaw_pitch_roll
from .tf_conv import yaw_pitch_roll_to_q
from .tf_conv import fmt_q_as_dir_and_roll
from .tf_conv import fmt_q_as_ypr
from .tf_conv import fmt

from sensor_msgs.msg import JointState

from ihmc_msgs.msg import TrajectoryPoint1DRosMessage
from ihmc_msgs.msg import OneDoFJointTrajectoryRosMessage
from ihmc_msgs.msg import HandTrajectoryRosMessage
from ihmc_msgs.msg import HandDesiredConfigurationRosMessage
from ihmc_msgs.msg import ArmTrajectoryRosMessage
from ihmc_msgs.msg import SE3TrajectoryPointRosMessage

from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayDimension
import rospy

from .tfzarj import TfZarj
from .utils import log

from copy import deepcopy

import math
import time

LEFT = 0
RIGHT = 1
OVERRIDE = 0
QUEUE = 1

class Hands(object):
    """ This is the hands and arms package """
    # Constants


    def __init__(self, zarj_os):
        log("Zarj arms initialization begin")
        self.zarj = zarj_os
        self.ARM_MOVE_TIME = 1.0
        self.HAND_MOVE_TIME = 1.0
        self.HAND_TRAJECTORY_TIME = 2.0
        self.HAND_ROTATION_TIME = 4.0
        self.SQRT_TWO = math.sqrt(2.0)
        self.joint_states = {}
        self.publishers = []
        self.last_hands = [ [ 0.0, 0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0] ]
        self.last_arms = [ [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] ]
        self.finger_offset = [ Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0) ]

        # Hand joints
        #   ShoulderPitch - immediate rotation, from -2.9 to 2.0
        #   ShoulderRoll  - shoulder first joint, 0 extended, 1.5 closed. from -1.3 to 1.5.
        #   ShoulderYaw   - spinning at second joint.  -3.1 to 2.2.
        #   ElbowPitch    - The orange joint.  lower is wider. -.1 to 2.2
        #   ForearmYaw    - Spin of forearm.  -2.0 to 3.1
        #   WristRoll     - Side to side motion of wrist.  -.6 to .6
        #   WristPitch    - Up/down motion of wrist. -.5 to .4
        joint_names = ["ShoulderPitch", "ShoulderRoll", "ShoulderYaw", "ElbowPitch", "ForearmYaw", "WristRoll",
                       "WristPitch"]
        self.left_arm_joint_names = map(lambda x: "left" + x, joint_names)
        self.right_arm_joint_names = map(lambda x: "right" + x, joint_names)

        # Initialized when first request for hand states come in.
        self.hand_state_subscriber = None
        self.got_first_message = False

        self.hand_trajectory_publisher = rospy.Publisher(
            "/ihmc_ros/{0}/control/hand_trajectory".format(self.zarj.robot_name),
            HandTrajectoryRosMessage, queue_size=10)
        self.publishers.append(self.hand_trajectory_publisher)

        self.hand_desired_configuration_publisher = rospy.Publisher(
            "/ihmc_ros/{0}/control/hand_desired_configuration".format(self.zarj.robot_name),
            HandDesiredConfigurationRosMessage, queue_size=10)
        self.publishers.append(self.hand_desired_configuration_publisher)

        self.arm_trajectory_publisher = rospy.Publisher(
            "/ihmc_ros/{0}/control/arm_trajectory".format(self.zarj.robot_name),
            ArmTrajectoryRosMessage, queue_size=10)
        self.publishers.append(self.arm_trajectory_publisher)

        log("Zarj arms initialization completed")

        self.right_hand_position_controller_publisher = rospy.Publisher(
            "/right_hand_position_controller/command",
            Float64MultiArray, queue_size=10)
        self.publishers.append(self.right_hand_position_controller_publisher)

        self.left_hand_position_controller_publisher = rospy.Publisher(
            "/left_hand_position_controller/command",
            Float64MultiArray, queue_size=10)
        self.publishers.append(self.left_hand_position_controller_publisher)

        self.zarj.wait_for_publishers(self.publishers)

        log("Zarj hands initialization completed")

    def __del__(self):
        self.zarj.close_publishers(self.publishers)
        log("TODO - implement __del__ method.")

    def check_get_states_init(self):
        if self.hand_state_subscriber is None:
            self.hand_state_subscriber = rospy.Subscriber("/joint_states", JointState, self.recv_joint_state)
            rospy.sleep(0.1)

    def get_joint_states(self, sidename):
        self.check_get_states_init()
        if sidename == "left":
            return map(lambda x: {x: self.joint_states[x]}, self.left_arm_joint_names)
        elif sidename == "right":
            return map(lambda x: {x: self.joint_states[x]}, self.right_arm_joint_names)
        else:
            rospy.logerr("Unknown side {}".format(sidename))
            return

    def get_joint_values(self, sidename):
        self.check_get_states_init()
        if sidename == "left":
            return map(lambda x: self.joint_states[x][0], self.left_arm_joint_names)
        elif sidename == "right":
            return map(lambda x: self.joint_states[x][0], self.right_arm_joint_names)
        else:
            rospy.logerr("Unknown side {}".format(sidename))
            return

    def recv_joint_state(self, msg):
        if not self.got_first_message:
            log("Got first joint state message")
            self.got_first_message = True
        num_states = len(msg.name)
        for i in range(0, num_states):
            name = msg.name[i]
            position = msg.position[i]
            velocity = msg.velocity[i]
            effort = msg.effort[i]
            self.joint_states[name] = [position, velocity, effort]

        # The arm joints are as follows:
        #   ShoulderPitch - immediate rotation, from -2.9 to 2.0
        #   ShoulderRoll  - shoulder first joint, 0 extended, 1.5 closed. from -1.3 to 1.5.
        #   ShoulderYaw   - spinning at second joint.  -3.1 to 2.2.
        #   ElbowPitch    - The orange joint.  lower is wider. -.1 to 2.2
        #   ForearmYaw    - Spin of forearm.  -2.0 to 3.1
        #   WristRoll     - Side to side motion of wrist.  -.6 to .6
        #   WristPitch    - Up/down motion of wrist. -.5 to .4

    def make_arm_trajectory(self, side, mode, time, joints, previous_msg = None):
        trajectories = []
        for x in joints:
            trajectories.append(self.zarj.make_joint(time, x))

        msg = ArmTrajectoryRosMessage()
        msg.robot_side = side
        msg.joint_trajectory_messages = trajectories
        msg.execution_mode = mode
        msg.unique_id = self.zarj.uid
        if previous_msg is not None:
            msg.previous_message_id = previous_msg

        return msg

        # The arm joints are as follows:
        #   ThumbRoll     - Side to side motion of thumb.  0 to 1.8.
        #   ThumbPitch    - Closing of thumb.  0.0 to 0.6
        #   IndexPitch    - Closing of index finger.  0 to 1.1.
        #   MiddlePitch   - Closing of middle finger.  0 to 0.9.
        #   PinkyPitch    - Closing of pinky finger.  0 to 1.0.
    def make_hand_configuration(self, points):
        dim = MultiArrayDimension()
        dim.label = 'fingers'
        dim.size = 5
        dim.stride = 5
        msg = Float64MultiArray()
        msg.layout.dim = [dim]
        msg.layout.data_offset = 0
        msg.data = points
        return msg

    def make_hand_trajectory(self, sidename, position, orientation, time = 1.0,
            mode = HandTrajectoryRosMessage.OVERRIDE,
            base_for_control = HandTrajectoryRosMessage.WORLD, previous_msg = None):
        if sidename == 'left':
            side = ArmTrajectoryRosMessage.LEFT
        elif sidename == 'right':
            side = ArmTrajectoryRosMessage.RIGHT
        else:
            rospy.logerr("Unknown side {}".format(sidename))
            return

        point = SE3TrajectoryPointRosMessage()
        point.time = time
        point.position = position
        point.orientation = orientation

        msg = HandTrajectoryRosMessage()
        msg.robot_side = side
        msg.base_for_control = base_for_control
        msg.taskspace_trajectory_points = [ point ]
        msg.execution_mode = mode
        msg.unique_id = self.zarj.uid

        if previous_msg is not None:
            msg.previous_message_id = previous_msg

        return msg

    def set_arm_configuration(self, sidename, joints, movetime = None):
        if sidename == 'left':
            side = ArmTrajectoryRosMessage.LEFT
        elif sidename == 'right':
            side = ArmTrajectoryRosMessage.RIGHT
        else:
            rospy.logerr("Unknown side {}".format(sidename))
            return

        if movetime is None:
            movetime = self.ARM_MOVE_TIME

        for i in range(len(joints)):
            if math.isnan(joints[i]):
                joints[i] = self.last_arms[side][i]

        msg = self.make_arm_trajectory(side, ArmTrajectoryRosMessage.OVERRIDE, movetime, joints)
        self.last_arms[side] = deepcopy(joints)
        log('Setting {} arm to {}'.format(sidename, joints))
        self.arm_trajectory_publisher.publish(msg)
        rospy.sleep(movetime)

    def set_hand_configuration(self, sidename, joints, movetime = None):
        if sidename == 'left':
            pub = self.left_hand_position_controller_publisher
            side = LEFT
        elif sidename == 'right':
            pub = self.right_hand_position_controller_publisher
            side = RIGHT
        else:
            rospy.logerr("Unknown side {}".format(sidename))
            return

        if movetime is None:
            movetime = self.HAND_MOVE_TIME

        for i in range(len(joints)):
            if math.isnan(joints[i]):
                joints[i] = self.last_hands[side][i]

        msg = self.make_hand_configuration(joints)
        self.last_hands[side] = deepcopy(joints)
        log('Setting {} hand to {}'.format(sidename, joints))
        pub.publish(msg)
        rospy.sleep(movetime)

    # Get the current frame of reference for the center of the hand (somewhere in the middle of the palm).
    # The palm reference frame does *not* do this, it is actually close to the wrist. When hand trajectory
    # message is acted on it is done from this center of the hand location by the controller, and *not*
    # at the wrist. We do create a virtual rotation of the palm reference frame so that the x-axis
    # in zero rotation points forward along the hand from wrist to the tips of the fingers.
    def get_current_hand_center_transform(self, sidename, rel_to_frame_name = None):
        # Get current hand center position and orientation, the position defined as the reference frame
        # position for using the hand trajectory message.
        if sidename == 'left':
            framename = "leftPalm"
            offset = [0,0.1,0]
            from_palm_q = [0,0,self.SQRT_TWO/2,self.SQRT_TWO/2]
            # joint_states = map(lambda x: {x: self.joint_states[x]}, self.left_arm_joint_names)
        elif sidename == 'right':
            framename = "rightPalm"
            offset = [0,-0.1,0]
            from_palm_q = [0,0,-self.SQRT_TWO/2,self.SQRT_TWO/2]
            # joint_states = map(lambda x: {x: self.joint_states[x]}, self.right_arm_joint_names)
        else:
            rospy.logerr("Unknown side {}".format(sidename))
            return

        palm_frame = self.zarj.transform.lookup_transform('world', framename, rospy.Time())
        palm_transform = palm_frame.transform
        offset_to_center = rotate_v(offset, msg_q_to_q(palm_transform.rotation))
        center_position = add_v(msg_v_to_v(palm_transform.translation), offset_to_center)
        palm_q = msg_q_to_q(palm_transform.rotation)

        # Rotate so that going from wrist to finger tip is along x-axis is the zero orientation.
        along_hand_orientation_q = quaternion_multiply(msg_q_to_q(palm_transform.rotation),
                                                       from_palm_q)
        rpt_frame = 'world'
        if rel_to_frame_name is not None:
            rpt_frame = rel_to_frame_name
            frame_to = self.zarj.transform.lookup_transform('world', rel_to_frame_name, rospy.Time())
            transform_to = frame_to.transform
            rotation_conj_to = conjugate_q(msg_q_to_q(transform_to.rotation))
            translation_to = msg_v_to_v(transform_to.translation)
            center_position = rotate_v(sub_v(center_position, translation_to), rotation_conj_to)
            along_hand_orientation_q = mult_q(rotation_conj_to, along_hand_orientation_q)

        rospy.loginfo('Current {} hand center position in reference frame {} is {} and {}.'.format(
            sidename,
            rpt_frame,
            fmt(center_position),
            fmt_q_as_ypr(along_hand_orientation_q)))
        # rospy.loginfo('Current {} hand joint states: {}'.format(sidename, joint_states))
        palm_transform.translation = v_to_msg_v(center_position)
        palm_transform.rotation = q_to_msg_q(along_hand_orientation_q)

        return palm_transform

    # Rotation when using the hand trajectory message can take quite a lot of time.
    # Part of the problem appears to be the ambiguity in choices of how to achieve the rotation.
    # Some rotations which seem best solved by twisting the wrist will twist elbows and shoulders
    # instead and require much more motion than is optimal.
    def determine_hand_move_time(self, sidename, position, orientation, cur_transform):
        move_time = self.HAND_TRAJECTORY_TIME
        desired_q = msg_q_to_q(orientation)
        cur_q = msg_q_to_q(cur_transform.rotation)
        rotation_amount = quaternion_multiply(desired_q, conjugate_q(cur_q))
        rot_w = rotation_amount[3]
        if rot_w < 0: rot_w = -rot_w
        add_time = 0
        if rot_w < 0.85:
            add_time = self.HAND_ROTATION_TIME
        # if rot_w < 0.92
        #    add_time = self.HAND_ROTATION_TIME
        # if rot_w < 0.75:
        #    add_time += self.HAND_ROTATION_TIME
        if add_time > 0:
            rospy.loginfo('Doing significant {} hand rotation, '
                          'adding rotation wait of {}.'.format(sidename, add_time))
            move_time += add_time
        return move_time

    # Moves the hand center to an absolute world position (Vector3) and orientation (Quaternion).
    def move_hand_center_abs(self, sidename, position, orientation, move_time = None):
        desired_q = msg_q_to_q(orientation)
        # Use the hand trajectory message to move the hand
        if move_time is None:
            cur_transform = self.get_current_hand_center_transform(sidename)
            move_time = self.determine_hand_move_time(sidename, position, orientation, cur_transform)
        rospy.loginfo('Desired {} hand center orientation in world reference frame is {}.'.format(
            sidename, fmt_q_as_ypr(desired_q)))

        # Rotate from orientation based on x-axis being along hand to one where x-axis
        # is perpendicular to palm.
        if sidename == 'left':
            to_palm_q = [0, 0, -self.SQRT_TWO / 2, self.SQRT_TWO/2]
        elif sidename == 'right':
            to_palm_q = [0, 0, self.SQRT_TWO / 2, self.SQRT_TWO/2]
        else:
            rospy.logerr("Unknown side {}".format(sidename))
            return

        perpendicular_to_palm_q = quaternion_multiply(desired_q, to_palm_q)
        palm_orientation = q_to_msg_q(perpendicular_to_palm_q)
        # rospy.loginfo('Palm quaternion desired: ' + str(perpendicular_to_palm_q))
        msg = self.make_hand_trajectory(sidename, position, palm_orientation, self.HAND_TRAJECTORY_TIME)
        self.hand_trajectory_publisher.publish(msg)

        rospy.sleep(move_time + 0.5)

    # Moves the hand by a relative displacement and to a named orientation. If no orientation
    # is provided, then the orientation is not changed. The relative displacement is done
    # within the reference frame of the torso so Vector3(1,0,0) will move the hand forward
    # away from the front of the robot.
    def move_hand_center_rel(self, sidename, offset, orientation_name = "unchanged", move_time = None):
        current_center = self.get_current_hand_center_transform(sidename)
        cur_v = msg_v_to_v(current_center.translation)
        cur_q = msg_q_to_q(current_center.rotation)

        lf_frame = self.zarj.transform.lookup_transform('world', self.zarj.walk.lfname, rospy.Time())
        lf_q = msg_q_to_q(lf_frame.transform.rotation)

        apply_lf_transform = True
        offset_v = msg_v_to_v(offset)
        abs_offset_v = rotate_v(offset_v, lf_q)

        if orientation_name == "none":  # Same as handshake position.
            q = [0, 0, 0, 1]
        elif orientation_name == "faceup":
            # q = [0.5,-0.5,0.5,0.5] if doing direct in hand message.
            q = [self.SQRT_TWO/2, 0, 0, self.SQRT_TWO/2]
        elif orientation_name == "facedown":
            # q = [-0.5, 0.5, 0.5, 0.5] if doing direct in hand message.
            q = [-self.SQRT_TWO/2, 0, 0, self.SQRT_TWO/2]
        else:
            apply_lf_transform = False
            q = cur_q

        # print("----Torso_q: " + str(torso_q))
        if apply_lf_transform:
            if sidename == 'left':
                q = conjugate_q(q)
            rospy.loginfo('Asked {} hand center to move, relative to left foot, by {} and '
                          'to {}.'.format(sidename, offset_v, fmt_q_as_ypr(q)))
            # Apply torso rotation, and then our preferred rotation.
            # q = quaternion_multiply(torso_q, quaternion_multiply(q, conjugate_q(torso_q)))
            q = quaternion_multiply(lf_q, q)
        else:
            relative_q = quaternion_multiply(conjugate_q(lf_q), q)
            rospy.loginfo('Asked {} hand center to move, relative to left foot, by {} '
                          'and to stay at {}.'.format(sidename, offset_v, fmt_q_as_ypr(relative_q)))

        position = v_to_msg_v(add_v(cur_v, abs_offset_v))
        orientation = q_to_msg_q(q)
        # print("----Current q: " + str(cur_q))
        # print("----Orientation q: " + str(q))
        if move_time is None:
            move_time = self.determine_hand_move_time(sidename, position, orientation, current_center)
        self.move_hand_center_abs(sidename, position, orientation, move_time)

    # The value ypr is the array of desired changes in yaw, pitch and roll (in degrees)
    # Offset is a displacement in Vector3 representation and is relative to the frame of the torso.
    # Generally the hand trajectory message seems to do better with smaller rotations than large and
    # it appears to be quite good in giving highly precise orientations of the hand.
    def move_with_yaw_pitch_roll(self, sidename, offset, ypr, move_time = None):
        current_center = self.get_current_hand_center_transform(sidename)
        current_q = msg_q_to_q(current_center.rotation)
        cur_ypr = q_to_yaw_pitch_roll(current_q)
        # print "cur_ypr = " + str(cur_ypr)

        lf_frame = self.zarj.transform.lookup_transform('world', self.zarj.walk.lfname, rospy.Time())
        lf_q = msg_q_to_q(lf_frame.transform.rotation)

        new_ypr = add_v(cur_ypr, ypr)
        cur_v = msg_v_to_v(current_center.translation)
        offset_v = msg_v_to_v(offset)
        abs_offset_v = rotate_v(offset_v, lf_q)
        rospy.loginfo('Moving {} hand center by {} and rotating by {} to new yaw/pitch/roll '
                      'of {} in world frame'.format(sidename, fmt(offset_v), fmt(ypr), fmt(new_ypr)))
        new_q = yaw_pitch_roll_to_q(new_ypr)
        new_v = add_v(cur_v, abs_offset_v)
        new_orientation = q_to_msg_q(new_q)
        new_position = v_to_msg_v(new_v)
        if move_time is None:
            move_time = self.determine_hand_move_time(sidename, new_position, new_orientation, current_center)
        self.move_hand_center_abs(sidename, new_position, new_orientation, move_time)

    # The Hand Trajectory frame is unclear.  It doesn't appear to be
    #  the palm, or a finger, or anything.  The index finger is about
    #  the closest.  So what we do is we move the 'hand', and then
    #  keep a running offset between what we commanded and where the
    # index finger ended up, and use that as an adjustemnt.
    def move_index_finger(self, sidename, position, time = None, repeat = True):
        if sidename == 'left':
            framename = "leftIndexFingerPitch1Link"
            side = LEFT
        elif sidename == 'right':
            framename = "rightIndexFingerPitch1Link"
            side = RIGHT
        if time == None:
            time = self.ARM_MOVE_TIME

        original_frame = self.zarj.transform.lookup_transform('world', framename, rospy.Time())
        adjusted_pos = deepcopy(position)
        adjusted_pos.x += self.finger_offset[side].x
        adjusted_pos.y += self.finger_offset[side].y
        adjusted_pos.z += self.finger_offset[side].z
        msg = self.make_hand_trajectory(side, adjusted_pos, original_frame.transform.rotation, self.ARM_MOVE_TIME)
        self.hand_trajectory_publisher.publish(msg)
        log('Asked hand finger to {},{},{}'.format(adjusted_pos.x, adjusted_pos.y, adjusted_pos.z))
        rospy.sleep(time + 0.1)
        new_frame = self.zarj.transform.lookup_transform('world', framename, rospy.Time())
        self.finger_offset[side].x = adjusted_pos.x - new_frame.transform.translation.x
        self.finger_offset[side].y = adjusted_pos.y - new_frame.transform.translation.y
        self.finger_offset[side].z = adjusted_pos.z - new_frame.transform.translation.z
        if (abs(new_frame.transform.translation.x - position.x) > 0.01 or
            abs(new_frame.transform.translation.y - position.y) > 0.01 or
            abs(new_frame.transform.translation.z - position.z) > 0.01):
            log("Commanded move missed by {},{},{}".format(
                new_frame.transform.translation.x - position.x,
                new_frame.transform.translation.y - position.y,
                new_frame.transform.translation.z - position.z))
            if repeat:
                self.move_index_finger(sidename, position, time, False)


    def plot_arms(self):
        king_tut = [ -1.4, 1.10, 1.65, 0.05, 1.7, 0.2, 0.2 ]
        aric_tut = [ -1.4, 0.75, 1.65, 0.55, 1.7, 0.2, 0.2 ]
        backoff = [ -1.1, 0.75, 1.65, 0.55, 1.7, 0.2, 0.2 ]
        backoff2 = [ -1.1, 1.25, 1.65, 0.85, 1.7, 0.2, 0.2 ]
        backoff3 = [ -1.1, 1.45, 1.0, 1.51, 1.7, 0.2, 0.2 ]
        leftarm = [ -1.4, -1.50, 0.2, -1.5121, 0.00504, 0.00021, 0.00283 ]

#        msg = make_arm_trajectory(RIGHT, OVERRIDE, 1.0, king_tut)
        msg = self.make_arm_trajectory(RIGHT, OVERRIDE, 1.0, aric_tut)
        log('ArmTrajectory raise: uid' + str(msg.unique_id))
        self.arm_trajectory_publisher.publish(msg)

        # A delay in the first message seems to be required to get
        #  the queuing to work
        #rospy.sleep(0.1)
        rospy.sleep(0.1)

        msg = self.make_arm_trajectory(RIGHT, OVERRIDE, 1, aric_tut, msg.unique_id)
        log('ArmTrajectory hold : uid' + str(msg.unique_id))
        self.arm_trajectory_publisher.publish(msg)
        rospy.sleep(0.1)

        msg = self.make_arm_trajectory(RIGHT, OVERRIDE, 0.1, backoff, msg.unique_id)
        log('ArmTrajectory backoff : uid' + str(msg.unique_id))
        self.arm_trajectory_publisher.publish(msg)
        rospy.sleep(0.1)

        msg = self.make_arm_trajectory(RIGHT, OVERRIDE, 0.1, backoff2, msg.unique_id)
        log('ArmTrajectory backoff 2: uid' + str(msg.unique_id))
        self.arm_trajectory_publisher.publish(msg)
        rospy.sleep(0.1)

        msg = self.make_arm_trajectory(RIGHT, OVERRIDE, 0.2, backoff3, msg.unique_id)
        log('ArmTrajectory backoff 3: uid' + str(msg.unique_id))
        self.arm_trajectory_publisher.publish(msg)
        rospy.sleep(0.1)

        msg = self.make_arm_trajectory(LEFT, OVERRIDE, 0.5, leftarm)
        log('ArmTrajectory left arm: uid' + str(msg.unique_id))
        self.arm_trajectory_publisher.publish(msg)
        rospy.sleep(0.1)

     #  def move_hand(self):
    def mashButton(self):
        # Button pressed position gathered from Gazebo analysis
        #buttonPosition = Vector3(3.35, -0.73, 1.25)
        # we can subtract the length of the fingers... about 9 cm
        buttonPosition = Vector3(3.16, -0.73, 1.25)

        self.zarj.transform.transform_from_world(buttonPosition)
        handWorld = self.zarj.transform.lookup_transform('world', 'rightPalm', rospy.Time())

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
        msg.unique_id = self.zarj.uid

        log('AS Mash Button: uid' + str(msg.unique_id))
        self.hand_trajectory_publisher.publish(msg)

    def open_grasp(self, side):
# relying on val_grasping being installed & launched in terminal with roslaunch val_grasping
# val_grasping.launch
        msg = Float64MultiArray()
        msg.data=[0,0,0,0,0]
        if side is RIGHT:
            self.right_hand_position_controller_publisher.publish(msg)
            log('Right hand open grasp ')
        else :
            self.left_hand_position_controller_publisher.publish(msg)
            log('Left hand open grasp ')


    def close_grasp(self, side):
# relying on val_grasping being installed & launched in terminal with roslaunch val_grasping
# val_grasping.launch
        msg = Float64MultiArray()
        if side is RIGHT:
            msg.data=[1.4, 0.55, 1.1, 0.9, 1.0]
            self.right_hand_position_controller_publisher.publish(msg)
            log('Right hand close grasp ')
        else :
            msg.data=[1.4, -0.55, -1.1, -0.9, -1.0]
            self.left_hand_position_controller_publisher.publish(msg)
            log('Left hand close grasp ')


    def hand_point(self, side):
# relying on val_grasping being installed & launched in terminal with roslaunch val_grasping
# val_grasping.launch
        msg = Float64MultiArray()
        if side is RIGHT:
            msg.data=[1.4, 0.55, 0, 0.9, 1.0]
            self.right_hand_position_controller_publisher.publish(msg)
            log('Right hand point ')
        else :
            msg.data=[1.4, -0.55, 0, -0.9, -1.0]
            self.left_hand_position_controller_publisher.publish(msg)
            log('Left hand point ')

#   def move_arm(self):

 #  def move_hand(self):

    def test(self):
#       move_arm(self)
#       move_hand(self)
#       self.plot_arms()
        rospy.sleep(0.1)
#       self.mashButton()
        rospy.sleep(0.1)
        self.close_grasp(RIGHT)
        rospy.sleep(0.1)
        self.open_grasp(RIGHT)
        rospy.sleep(0.1)
        self.close_grasp(LEFT)
        rospy.sleep(0.1)
        self.open_grasp(LEFT)
        rospy.sleep(0.1)
        self.close_grasp(LEFT)
