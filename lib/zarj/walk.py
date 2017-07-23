"""
    Walking logic for the robot
"""

import rospy
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_multiply
from tf.transformations import quaternion_about_axis
from tf.transformations import quaternion_matrix

from ihmc_msgs.msg import FootstepDataListRosMessage
from ihmc_msgs.msg import FootstepDataRosMessage
from ihmc_msgs.msg import FootstepStatusRosMessage
from ihmc_msgs.msg import Point2dRosMessage
from ihmc_msgs.msg import AbortWalkingRosMessage
from std_msgs.msg import Int32

from .tf_conv import rotate_v
from .tf_conv import msg_v_to_v
from .tf_conv import v_to_msg_v
from .tf_conv import q_to_msg_q
from .tf_conv import add_v
from .tf_conv import compute_rot_angle

from .tf_conv import fmt
from math import cos
from math import sin



LEFT = FootstepDataRosMessage.LEFT
RIGHT = FootstepDataRosMessage.RIGHT

from copy import deepcopy
from math import sin, cos, acos, sqrt, degrees, radians
from numpy import dot, sign

from .utils import log
from .confused import ZarjConfused

def invert_foot(foot):
    """ Utility to switch feet """
    if foot == FootstepDataRosMessage.LEFT:
        return FootstepDataRosMessage.RIGHT
    else:
        return FootstepDataRosMessage.LEFT


class Walk(object):
    """ This is the walking package """
    # Constants
    DEFAULT_STRIDE = 0.4
    SIDE_STRIDE = 0.2
    DEFAULT_TRANSFER_TIME = 0.4
    DEFAULT_SWING_TIME = 0.8

    FOOT_LENGTH = 0.27
    FOOT_WIDTH = 0.16

    DEFAULT_STANCE_WIDTH = 0.25

    # Toggle this to see if things get better or worse with using trajectory for
    # for right foot step up.
    USE_STEP_UP_TRAJECTORY = True

    def __init__(self, zarj_os):
        log("Zarj walk initialization begin")
        self.zarj = zarj_os
        self.steps = 0
        self.publishers = []
        self.planned_steps = 0
        self.stride = self.DEFAULT_STRIDE
        self.transfer_time = self.DEFAULT_TRANSFER_TIME
        self.swing_time = self.DEFAULT_SWING_TIME
        self.stance_width = self.DEFAULT_STANCE_WIDTH
        self.lf_start_position = Vector3()
        self.lf_start_orientation = Quaternion()
        self.rf_start_position = Vector3()
        self.rf_start_orientation = Quaternion()
        self.behavior = 0
        self.start_walk = None
        self.walk_failure = None
        self.last_footstep = None

        lfp = "/ihmc_ros/{0}/left_foot_frame_name".format(self.zarj.robot_name)
        rfp = "/ihmc_ros/{0}/right_foot_frame_name".format(self.zarj.robot_name)

        if rospy.has_param(rfp) and rospy.has_param(lfp):
            self.lfname = rospy.get_param(lfp)
            self.rfname = rospy.get_param(rfp)

        if self.lfname == None or self.rfname == None:
            log("Zarj could not find left or right foot name")
            raise RuntimeError

        self.footstep_list_publisher = rospy.Publisher(
            "/ihmc_ros/{0}/control/footstep_list".format(self.zarj.robot_name),
            FootstepDataListRosMessage, queue_size=10)
        self.publishers.append(self.footstep_list_publisher)

        self.footstep_status_subscriber = rospy.Subscriber(
            "/ihmc_ros/{0}/output/footstep_status".format(self.zarj.robot_name),
            FootstepStatusRosMessage, self.receive_footstep_status)

        self.behavior_subscriber = rospy.Subscriber(
            "/ihmc_ros/{0}/output/behavior".format(self.zarj.robot_name),
            Int32, self.receive_behavior)

        self.abort_publisher = rospy.Publisher(
            "/ihmc_ros/{0}/control/abort_walking".format(self.zarj.robot_name),
            AbortWalkingRosMessage, queue_size=1)
        self.publishers.append(self.abort_publisher)

        self.lookup_feet()
        self.zarj.wait_for_publishers(self.publishers)

        log("Zarj walk initialization completed")

    def __del__(self):
        log("TODO - implement __del__ method.")
        self.zarj.close_publishers(self.publishers)

    def lookup_feet(self, report=False):
        """ Find where our feet are in the world """
        world = self.zarj.transform.lookup_transform('world', self.lfname, rospy.Time())
        self.lf_start_orientation = world.transform.rotation
        self.lf_start_position = world.transform.translation

        world = self.zarj.transform.lookup_transform('world', self.rfname, rospy.Time())
        self.rf_start_orientation = world.transform.rotation
        self.rf_start_position = world.transform.translation

        # Create some reports on current position relative to close 45 degree multiple angle
        # This is being used to try to prove that Valkyrie gets confused about its position
        # and angle at times.
        if report:
            lf_angle = compute_rot_angle(self.lf_start_orientation)
            rf_angle = compute_rot_angle(self.rf_start_orientation)
            rounded_angle = round(lf_angle/45.0) * 45.0
            un_rotate = quaternion_about_axis(radians(-rounded_angle), [0, 0, 1])
            lf_v = msg_v_to_v(self.lf_start_position)
            rf_v = msg_v_to_v(self.rf_start_position)
            lf_rel_v = rotate_v(lf_v, un_rotate)
            rf_rel_v = rotate_v(rf_v, un_rotate)
            log("*View From {} Deg, lf pos={},angle={}, rf pos={},angle={}".format(
                int(rounded_angle), fmt(lf_rel_v), fmt(lf_angle - rounded_angle),
                fmt(rf_rel_v), fmt(rf_angle - rounded_angle)
            ))


    def undo_transform_offset(self, offset):
        """  Transform an offset relative to the left foot in world coordinates to a relative offset """
        quaternion = [-self.lf_start_orientation.x, -self.lf_start_orientation.y, -self.lf_start_orientation.z,
                      self.lf_start_orientation.w]
        return rotate_v(offset, quaternion)

    def transform_left_offset(self, offset):
        """  Transform an offset relative to the left foot into world coordinates """
        quaternion = [
            self.lf_start_orientation.x, self.lf_start_orientation.y, self.lf_start_orientation.z,
            self.lf_start_orientation.w]
        return rotate_v(offset, quaternion)

    def transform_right_offset(self, offset):
        """  Transform an offset relative to the right foot into world coordinates """
        quaternion = [
            self.lf_start_orientation.x, self.lf_start_orientation.y, self.lf_start_orientation.z,
            self.lf_start_orientation.w]
        return rotate_v(offset, quaternion)

    def rotate_vector(self, vector, quaternion):
        """ Deprecated method """
        return rotate_v(vector, quaternion)

    def receive_behavior(self, msg):
        """ Gets an Int based on the status of the robot """
        self.behavior = msg.data
        if self.walk_failure is not None:
            return
        if self.start_walk is not None:
            if self.behavior == 4:
                log("Walk started")
                self.start_walk = None
                if self.last_footstep is None:
                    self.last_footstep = rospy.get_time()
            else:
                now = rospy.get_time()
                if now > self.start_walk + 10.0:
                    self.walk_failure = 'WALK FAILED: Asked to walk, and ' \
                                        'never started'
                    log(self.walk_failure)
        if self.last_footstep is not None:
            now = rospy.get_time()
            if now > self.last_footstep + 10.0:
                self.walk_failure = 'WALK FAILED: footstep {} is taking over ' \
                                    '10 seconds to complete'.format(self.steps)
                log(self.walk_failure)

    def receive_footstep_status(self, msg):
        """ Get our current footstep status """
        if msg.status == 1:
            self.steps += 1
            log('Completed step: {} of {}; total received {}.'.format(
                msg.footstep_index, self.planned_steps, self.steps))
            if self.steps < self.planned_steps:
                self.last_footstep = rospy.get_time()
            else:
                self.last_footstep = None

    def walk_is_done(self):
        """ See if we have completed all of our steps """
        if self.walk_failure is not None:
            self.start_walk = None
            self.last_footstep = None
            raise ZarjConfused(self.walk_failure)
        if self.steps >= self.planned_steps and self.behavior == 3:
            return True
        return False

    def create_one_abs_footstep(self, side, location, orientation):
        """ Simple create of a footstep to move a foot to a precise location."""
        footstep = FootstepDataRosMessage()
        footstep.robot_side = side
        footstep.location = location
        footstep.orientation = orientation
        footstep.trajectory_type = FootstepDataRosMessage.DEFAULT
        # Create report of what we are doing relative to left foot.
        cur_pos = self.lf_start_position
        cur_angle = compute_rot_angle(self.lf_start_orientation)
        cur_pos_diff = [location.x - cur_pos.x, location.y - cur_pos.y, 0]
        lf_rot_pos_diff = self.undo_transform_offset(cur_pos_diff)
        new_angle = compute_rot_angle(orientation)
        if side == FootstepDataRosMessage.LEFT: footstep_type = "left"
        else: footstep_type = "right"
        log("Moving {} foot relative to (starting) left foot to position {} and angle {}.".format(
            footstep_type, fmt(lf_rot_pos_diff), fmt(new_angle - cur_angle)
        ))
        return footstep

    def create_one_footstep(self, side, offset, angle=None, world=False):
        """ Create one footstep. """
        footstep = FootstepDataRosMessage()
        footstep.robot_side = side
        up = offset[2]
        intermediate_pt = None
        world_intermediate = None
        world_intermediate2 = None
        # if up > 0.05 and side == FootstepDataRosMessage.RIGHT and self.USE_RIGHT_STEP_UP_TRAJECTORY:
        if up > 0.05 and self.USE_STEP_UP_TRAJECTORY:
            # Note, how we use 0.06 or 0.03 forward, which would intuitively seem wrong, but it produces the desired
            # effect.
            xoffset = 0.06 if side == FootstepDataRosMessage.LEFT else 0.03
            intermediate_pt = [xoffset, 0, up + 0.05]

        if side == FootstepDataRosMessage.LEFT:
            footstep_type = "left"
            footstep.orientation = deepcopy(self.lf_start_orientation)
            footstep.location = deepcopy(self.lf_start_position)
            if not world:
                world_offset = self.transform_left_offset(offset)
                if intermediate_pt is not None:
                    log("Using waypoints to move left foot to avoid stair")
                    lf_start_v = msg_v_to_v(self.lf_start_position)
                    world_intermediate = add_v(lf_start_v, self.transform_left_offset(intermediate_pt))
                    world_intermediate2 = add_v(lf_start_v, world_offset)
            else:
                world_offset = offset
        else:
            footstep_type = "right"
            footstep.orientation = deepcopy(self.rf_start_orientation)
            footstep.location = deepcopy(self.rf_start_position)
            if not world:
                world_offset = self.transform_right_offset(offset)
                if intermediate_pt is not None:
                    log("Using waypoints to move right foot to avoid stair")
                    rf_start_v = msg_v_to_v(self.rf_start_position)
                    world_intermediate = add_v(rf_start_v, self.transform_right_offset(intermediate_pt))
                    world_intermediate2 = add_v(rf_start_v, world_offset)
            else:
                world_offset = offset
        if world_intermediate is None:
            footstep.trajectory_type = FootstepDataRosMessage.DEFAULT
        else:
            footstep.trajectory_type = FootstepDataRosMessage.CUSTOM
            footstep.trajectory_waypoints = [v_to_msg_v(world_intermediate), v_to_msg_v(world_intermediate2)]
        reference_foot_angle = compute_rot_angle(self.lf_start_orientation)

        if not world:
            log("Preparing " + footstep_type + " foot (left foot rotated=" + fmt(reference_foot_angle) +
                "), moving " + fmt(offset))
        else:
            log("Preparing " + footstep_type + " foot, move (world) " + fmt(offset))

        footstep.location.x += world_offset[0]
        footstep.location.y += world_offset[1]
        footstep.location.z += world_offset[2]

        if angle is not None:
            # Rotate about z-axis.
            rotate = quaternion_about_axis(radians(angle), [0, 0, 1])
            original = [footstep.orientation.x, footstep.orientation.y, footstep.orientation.z,
                        footstep.orientation.w]
            new = quaternion_multiply(rotate, original)
            log("Rotating " + footstep_type + " foot by " + fmt(angle))
            footstep.orientation = Quaternion(new[0], new[1], new[2], new[3])

        return footstep

    def create_footsteps(self, distance, start_foot=LEFT,
            square_up=False, stride=DEFAULT_STRIDE, start=0.0):
        """ Create a series of footsteps. """

        sign = 1.0
        if distance < 0:
            sign = -1.0
            distance = distance * sign
            start = start * sign

        total_distance = start
        foot = start_foot

        footsteps = []

        while total_distance + .001 < distance:
            total_distance += stride
            if total_distance + 0.001 >= distance:
                footsteps.append(self.create_one_footstep(foot, [sign * distance, 0.0, 0.0]))
                break
            footsteps.append(self.create_one_footstep(foot, [sign * total_distance, 0.0, 0.0]))
            foot = invert_foot(foot)

        if square_up:
            foot = invert_foot(foot)
            footsteps.append(self.create_one_footstep(foot, [sign * distance, 0.0, 0.0]))

        return footsteps

    def set_stride(self, stride=DEFAULT_STRIDE):
        """ Set the default stride """
        self.stride = stride

    def set_transfer_time(self, transfer_time=DEFAULT_TRANSFER_TIME):
        """ Set the default transfer time """
        self.transfer_time = transfer_time

    def set_swing_time(self, swing_time=DEFAULT_SWING_TIME):
        """ Set the default swing time """
        self.swing_time = swing_time

    def set_stance_width(self, stance_width=DEFAULT_STANCE_WIDTH):
        """ Set the default stance_width """
        self.stance_width = stance_width

    def get_oriented_lf_forward_distance(self, snap_to = 45):
        """
        Gets left foot's forward distance (measured from 0,0 in world coordinates)
        relative to its orientation angle snapped to a multiple of
        a snap_to angle.
        """
        self.lookup_feet(report=True)
        left_ft_angle = compute_rot_angle(self.lf_start_orientation)
        right_ft_angle = compute_rot_angle(self.rf_start_orientation)
        rounded_angle = left_ft_angle
        if snap_to is not None and snap_to > 0.5:
            # Round to nearest snap_to degree multiple.
            rounded_angle = round(left_ft_angle/snap_to) * snap_to
        x = self.lf_start_position.x
        y = self.lf_start_position.y
        # print "At lf forward distance, x = " + fmt(x) + ", y = " + fmt(y)
        c = cos(rounded_angle)
        s = sin(rounded_angle)
        return c*x + s*y

    def forward(self, distance, square_up=True, start_foot=LEFT):
        """ Walk forward the given distance """
        msg = FootstepDataListRosMessage()
        msg.default_transfer_time = self.transfer_time
        msg.default_swing_time = self.swing_time

        stride = self.stride
        if distance < 0 and stride > 0.3:
            log('Walking backward exceeds safe stride')
            stride = 0.3

        msg.execution_mode = msg.OVERRIDE  # Override means replace others
        msg.unique_id = self.zarj.uid

        # TODO - this may be nice to rethink
        self.lookup_feet(report=True)

        msg.footstep_data_list = self.create_footsteps(distance, start_foot, square_up, self.stride)

        self.steps = 0
        self.planned_steps = len(msg.footstep_data_list)
        self.walk_failure = None
        self.start_walk = rospy.get_time()
        self.footstep_list_publisher.publish(msg)
        log('Move ' + fmt(distance) + ' FootstepDataList: uid ' + str(msg.unique_id))

    def turn(self, orig_angle, snap_to = 15, small_forward = None, report = True, align_to_snap = False):
        """ Turn to the given angle.  Note for a larger angle that we will take one step
        a bit to the side, before returning, so be aware that we need
        some space to turn. Angle is positive for counter clockwise. The align_to_snap, if set to true,
        essentially prevents the feet from rotating around a center, but instead just tries to fix
        the current orientation of the feet, using the most forward foot as the basis of this correction. The
        problem is that if you the left foot is rotated -5 degrees from optimal on a stair, then
        doing a normal +5 degree rotation of the feet to correct it will cause the left foot to backwards by 0.012,
        which can cause stair failure. """
        self.lookup_feet(report = report)
        left_ft_angle = compute_rot_angle(self.lf_start_orientation)

        angle = orig_angle
        align_angle = left_ft_angle
        if snap_to is not None and snap_to > 0.5:
            total_angle = left_ft_angle + orig_angle
            # Round to nearest snap_to degree multiple.
            rounded_angle = round(total_angle/snap_to) * snap_to
            snap_diff = rounded_angle - total_angle
            angle += snap_diff
            if align_to_snap:
                align_angle = rounded_angle
            log("Applying 'snap_to' of " + fmt(snap_to) + " to current left foot angle " + fmt(left_ft_angle) +
                " creating angle diff " + fmt(snap_diff) +
                " to change turn angle " + fmt(orig_angle) + " to turn angle " + fmt(angle))
            log("Small forward={}, align_angle={}".format(fmt(small_forward), fmt(align_angle)))

        if angle > 180:
            rospy.logerr("Error: angle cannot exceed 180 degrees")
            return

        if angle < -180:
            rospy.logerr("Error: angle cannot go below -180 degrees")
            return

        # We use the "left foot dictates forward position" logic.
        # In the reference frame of the left foot, we use the left foots forward position
        # as the average, instead of averaging it with the right foot. In climbing stairs,
        # the left foot has a more reliable relative "forward" position.
        lf_to_rf_diffx = self.rf_start_position.x - self.lf_start_position.x
        lf_to_rf_diffy = self.rf_start_position.y - self.lf_start_position.y

        # View diff relative to left foot.
        undo_align_rotation = quaternion_about_axis(radians(-align_angle), [0, 0 , 1])
        rf_rel = rotate_v([lf_to_rf_diffx, lf_to_rf_diffy, 0.0], undo_align_rotation)

        # The nice thing about doing math in the left foot frame (with a small adjustement
        # for small change in alignment angle), is that the left foot
        # has 0,0,0 as its coordinates, making all the math focus on the right foot's
        # relative location.
        rf_rel_x = rf_rel[0]
        rf_rel_y = rf_rel[1]

        # See if we are asked to make a small adjustment forward (for making sure we stay stuck up against stairs
        # tables, and walls).
        # The following conditional test is based on observation and reports of errors, and where the feet
        # appear to be when the errors happen.
        rel_avg_x = 0
        if align_to_snap and small_forward is not None and small_forward > 0.002 and rf_rel_x > small_forward:
            # This is the left foot slipped backwards scenario.
            log("Choosing right foot as forward position, it is {} more forward, "
                "ignoring small_forward".format(fmt(rf_rel_x)))
            rel_avg_x = rf_rel_x + 0.003
        elif small_forward is not None:
            rel_avg_x += small_forward
        rel_avg_y = (0 + rf_rel_y)/2.0 # Notice simple averaging logic for relative y location.

        # print "lf_start_position=" + str(self.lf_start_position) + ",rf_start_position=" + str(self.rf_start_position)
        # print "rel_avg_x=" + str(rel_avg_x) + ",rel_avg_y=" + str(rel_avg_y)

        # Transform average back to world coordinates.
        align_rotation = quaternion_about_axis(radians(align_angle), [0, 0 , 1])
        avg_diff_v = rotate_v([rel_avg_x, rel_avg_y, 0], align_rotation)
        avgx = self.lf_start_position.x + avg_diff_v[0]
        avgy = self.lf_start_position.y + avg_diff_v[1]

        # Compute distance between the feet. Useful for figuring out how much we can do without having
        # to do a side step.
        current_distance = sqrt(rf_rel_x * rf_rel_x + rf_rel_y * rf_rel_y)
        distance = self.stance_width # distance from center point between feet # sqrt(xterm + yterm)
        correction_factor = 1.02 # Feet end up not quite as far apart as expected by computations.
        distance_ratio = current_distance/distance
        choose_left = True
        if abs(angle) < 10 or align_to_snap:
            # Choose first foot to move by which one is further behind.
            # The not pivot foot moves first. Pivot foot should be the
            # one that is further ahead of the other. On stairs, this can
            # make a difference, especially if the left foot slips backwards.
            if rf_rel_x > 0:
                choose_left = False
        elif angle > 0:
            choose_left = False

        if choose_left:
            foot_type = "left"
            pivot_foot = FootstepDataRosMessage.LEFT
            pivot_angle = compute_rot_angle(self.lf_start_orientation)
            pivot_start_pos = self.lf_start_position
            not_pivot_angle = compute_rot_angle(self.rf_start_orientation)
            not_pivot_start_pos = self.rf_start_position
            foot_rel_offset = [0, -distance * correction_factor, 0]
            sidestep_rel_offset = [-self.FOOT_LENGTH / 2.0, -distance * correction_factor, 0]
        else:
            foot_type = "right"
            pivot_foot = FootstepDataRosMessage.RIGHT
            pivot_angle = compute_rot_angle(self.rf_start_orientation)
            pivot_start_pos = self.rf_start_position
            not_pivot_angle = compute_rot_angle(self.lf_start_orientation)
            not_pivot_start_pos = self.lf_start_position
            foot_rel_offset = [0, distance * correction_factor, 0]
            sidestep_rel_offset = [-self.FOOT_LENGTH / 2.0, distance * correction_factor, 0]

        # print "pivot_angle=" + str(pivot_angle) + ",not_pivot_angle=" + str(not_pivot_angle)

        msg = FootstepDataListRosMessage()
        msg.default_transfer_time = self.transfer_time
        msg.default_swing_time = self.swing_time

        msg.execution_mode = msg.OVERRIDE  # Override means replace others
        msg.unique_id = self.zarj.uid

        total_rot_q = quaternion_about_axis(radians(angle + left_ft_angle), [0, 0 , 1])
        abs_offset = rotate_v(foot_rel_offset, total_rot_q)
        xoffset = abs_offset[0]
        yoffset = abs_offset[1]

        xo2 = xoffset/2
        yo2 = yoffset/2

        # Compute location of where we want to send the pivot and not pivot feet.
        pivot_dest = [avgx - xo2, avgy - yo2]
        not_pivot_dest = [avgx + xo2, avgy + yo2]

        # Compute where we need to put the sidestep if we end up using it.
        sidestep_offset_v = self.transform_left_offset(sidestep_rel_offset)
        sidestep_abs_v = add_v(sidestep_offset_v, msg_v_to_v(not_pivot_start_pos))
        sidestep_abs_msg_v = v_to_msg_v(sidestep_abs_v)
        sidestep_msg_q = q_to_msg_q(quaternion_about_axis(radians(angle/2 + left_ft_angle), [0, 0, 1]))

        # Convert our locations and angles into parameters that ROS can take. Note that we are
        # applying the same rotation to both feet. Note that if align_to_angle is true
        # then angle + left_ft_angle is the same as the align_angle.
        both_feet_msg_q = q_to_msg_q(quaternion_about_axis(radians(angle + left_ft_angle), [0, 0, 1]))
        pivot_dest_msg_v = Vector3(pivot_dest[0], pivot_dest[1], pivot_start_pos.z)
        not_pivot_dest_msg_v = Vector3(not_pivot_dest[0], not_pivot_dest[1], not_pivot_start_pos.z)

        # For reporting and checking to see if we are really making much of a move.
        pivot_offset = [pivot_dest[0] - pivot_start_pos.x, pivot_dest[1] - pivot_start_pos.y]
        not_pivot_offset = [not_pivot_dest[0] - not_pivot_start_pos.x, not_pivot_dest[1] - not_pivot_start_pos.y]

        def is_small(d):
            return -0.015 < d[0] < 0.015 and -0.015 < d[1] < 0.015

        # print "pivot_offset=" + str(pivot_offset) + ",not_pivot_offset=" + str(not_pivot_offset)
        if -0.3 < angle < 0.3 and is_small(pivot_offset) and is_small(not_pivot_offset):
            log("Skipping turn, because the turn is likely to introduce as much error as "
                "would be removed by attempted move.")
            self.steps = 0
            self.planned_steps = 0
            return

        footsteps = []

        # Make the footsteps.
        cap_angle = 5  # Always allow moves of less than five degrees to be done in place.
        if current_distance > .19 and current_distance < .24:
            cap_angle = 15
        elif current_distance >= .24:
            cap_angle = 50

        # If the angle is large, we are going to assume a turn where one foot is 'planted'
        #  The other foot rotates and shifts position so that the feet remain
        #  aligned; their heels should form a line.
        # That requires that we do two things - first, make sure that the centers
        #  of our feet stay the same distance apart, and that the second foot
        #  is moved to keep them aligned
        angle_diff = not_pivot_angle - pivot_angle
        angle_check = abs(angle_diff) + abs(angle)
        large_angle = not align_to_snap and (angle_check > cap_angle or angle_check < -cap_angle)

        log('Turn angle ' + fmt(angle) + ' from angle ' + fmt(pivot_angle) +
            ' pivot on ' + foot_type + ' foot, pivot dest ' +
            fmt(pivot_dest) + ", not pivot dest " + fmt(not_pivot_dest))
        log('Doing side step=' + fmt(large_angle) + ', foot angle diff ' + fmt(angle_diff) +
            ', feet distance to desired ratio is ' + fmt(distance_ratio))
        if large_angle:
            footsteps.append(self.create_one_abs_footstep(invert_foot(pivot_foot), sidestep_abs_msg_v,
                                                          sidestep_msg_q))
            footsteps.append(self.create_one_abs_footstep(pivot_foot, pivot_dest_msg_v, both_feet_msg_q))

        footsteps.append(self.create_one_abs_footstep(invert_foot(pivot_foot),
                                                      not_pivot_dest_msg_v,
                                                      both_feet_msg_q))
        if not large_angle:
            footsteps.append(self.create_one_abs_footstep(pivot_foot, pivot_dest_msg_v, both_feet_msg_q))

        msg.footstep_data_list = footsteps

        self.steps = 0
        self.planned_steps = len(msg.footstep_data_list)
        self.walk_failure = None
        self.start_walk = rospy.get_time()
        self.footstep_list_publisher.publish(msg)
        log('Turn ' + fmt(angle) + ' FootstepDataList: uid ' + str(msg.unique_id))

    def move_foot(self, foot_type, x, y, rotate):
        """ 
        Move a foot, relative to the current left foot frame and rotate by a particular angle.
        This code is meant to be used for setting up messy test scenarios that can be fixed by the
        *turn* method.
        """
        self.lookup_feet(report = True)
        left_ft_angle = compute_rot_angle(self.lf_start_orientation)
        left_ft_rot = quaternion_about_axis(radians(left_ft_angle), [0, 0, 1])
        offset = rotate_v([x, y ,0], left_ft_rot)
        if foot_type == "left":
            ft_loc = msg_v_to_v(self.lf_start_position)
            foot_choice = LEFT
        else:
            ft_loc = msg_v_to_v(self.rf_start_position)
            foot_choice = RIGHT
        target = add_v(ft_loc, offset)
        foot_rotation = quaternion_about_axis(radians(left_ft_angle + rotate), [0, 0, 1])

        msg = FootstepDataListRosMessage()
        msg.footstep_data_list = [self.create_one_abs_footstep(foot_choice, v_to_msg_v(target), q_to_msg_q(foot_rotation))]
        msg.default_transfer_time = self.transfer_time
        msg.default_swing_time = self.swing_time
        msg.execution_mode = msg.OVERRIDE  # Override means replace others
        msg.unique_id = self.zarj.uid
        self.steps = 0
        self.planned_steps = len(msg.footstep_data_list)
        self.walk_failure = None
        self.start_walk = rospy.get_time()
        self.footstep_list_publisher.publish(msg)
        log("Moving {} foot by x={}, y={}, and rotating to {}".format(
            foot_type, fmt(x), fmt(y), fmt(left_ft_angle + rotate)))
        while not self.walk_is_done():
            rospy.sleep(0.2)
        rospy.sleep(0.2)

    def create_side_steps(self, distance, start_foot,
                          stride=SIDE_STRIDE, start=0.0):
        """ Create a series of side footsteps. """

        sign = 1.0
        if distance < 0:
            sign = -1.0
            distance = distance * sign
            start = start * sign

        total_distance = start
        foot = start_foot

        footsteps = []

        while total_distance + .001 < distance:
            if foot == start_foot:
                total_distance += stride
            if total_distance + 0.001 >= distance:
                footsteps.append(
                    self.create_one_footstep(foot, [0.0, sign * distance, 0.0]))
                break
            footsteps.append(
                self.create_one_footstep(foot,
                                         [0.0, sign * total_distance, 0.0]))
            foot = invert_foot(foot)

        foot = invert_foot(foot)
        footsteps.append(self.create_one_footstep(foot,
                                                  [0.0, sign * distance, 0.0]))

        return footsteps

    def side(self, distance):
        """
            Step to the side without changing orientation.
            Positive steps to the left, negative steps to the right
        """
        msg = FootstepDataListRosMessage()
        msg.default_transfer_time = self.DEFAULT_TRANSFER_TIME
        msg.default_swing_time = self.DEFAULT_SWING_TIME

        msg.execution_mode = msg.OVERRIDE  # Override means replace others
        msg.unique_id = self.zarj.uid

        self.lookup_feet(report=True)
        if distance < 0:
            start_foot = FootstepDataRosMessage.RIGHT
        else:
            start_foot = FootstepDataRosMessage.LEFT

        msg.footstep_data_list = self.create_side_steps(distance, start_foot,
                                                        self.SIDE_STRIDE)

        self.steps = 0
        self.planned_steps = len(msg.footstep_data_list)
        self.walk_failure = None
        self.start_walk = rospy.get_time()
        self.footstep_list_publisher.publish(msg)
        log('FootstepDataList: uid ' + str(msg.unique_id))

    def create_xy_steps(self, movement, start_foot, stride=DEFAULT_STRIDE):
        """ Create a series of xy direction footsteps. """
        footsteps = []
        current_step = Vector3(0, 0, 0)
        foot = start_foot

        while current_step.x != movement.x or current_step.y != movement.y:
            if foot == start_foot:
                full_step = False
                d_x = movement.x - current_step.x
                d_y = movement.y - current_step.y
                if abs(d_x) > stride:
                    d_x = sign(d_x) * stride
                    full_step = True
                if abs(d_y) > stride:
                    d_y = sign(d_y) * stride
                    full_step = True

                current_step.x += d_x
                current_step.y += d_y

                footsteps.append(
                    self.create_one_footstep(foot, [current_step.x,
                                                    current_step.y, 0.0],
                                             world=True))
                if not full_step:
                    break
            else:
                footsteps.append(
                    self.create_one_footstep(foot, [current_step.x,
                                                    current_step.y, 0.0],
                                             world=True))
            foot = invert_foot(foot)

        foot = invert_foot(foot)
        footsteps.append(self.create_one_footstep(foot, [current_step.x,
                                                         current_step.y, 0.0],
                                                  world=True))

        return footsteps

    def _find_first_xy_foot(self, point):
        """ Find what foot to move first based on distance to feet"""

        dist_l = (point.x - self.lf_start_position.x)**2 + \
                 (point.y - self.lf_start_position.y)**2

        dist_r = (point.x - self.rf_start_position.x)**2 + \
                 (point.y - self.rf_start_position.y)**2

        if dist_l <= dist_r:
            return FootstepDataRosMessage.LEFT
        else:
            return FootstepDataRosMessage.RIGHT


    def walk_to(self, point):
        """ Shuffle to a given point, point is given for the left foot """
        msg = FootstepDataListRosMessage()
        msg.default_transfer_time = self.DEFAULT_TRANSFER_TIME
        msg.default_swing_time = self.DEFAULT_SWING_TIME

        msg.execution_mode = msg.OVERRIDE  # Override means replace others
        msg.unique_id = self.zarj.uid

        self.lookup_feet()
        delta = Vector3(point.x - self.lf_start_position.x,
                        point.y - self.lf_start_position.y,
                        0)

        start_foot = self._find_first_xy_foot(point)
        msg.footstep_data_list = self.create_xy_steps(delta, start_foot, 0.15)

        self.steps = 0
        self.planned_steps = len(msg.footstep_data_list)
        self.walk_failure = None
        self.start_walk = rospy.get_time()
        self.footstep_list_publisher.publish(msg)
        log('FootstepDataList: uid ' + str(msg.unique_id))

    def fix_stance(self):
        """
            fix stance
        """
        self.lookup_feet(report=True)

        distance_between_feet = sqrt(
            (
                self.rf_start_position.x -
                self.lf_start_position.x
            )**2 + (
                self.rf_start_position.y -
                self.lf_start_position.y
            )**2)

        if abs(distance_between_feet - self.stance_width) > 0.01:
            distance = self.stance_width - distance_between_feet
        else:
            return

        msg = FootstepDataListRosMessage()
        msg.default_transfer_time = self.DEFAULT_TRANSFER_TIME
        msg.default_swing_time = self.DEFAULT_SWING_TIME

        msg.execution_mode = msg.OVERRIDE  # Override means replace others
        msg.unique_id = self.zarj.uid

        start_foot = FootstepDataRosMessage.LEFT

        msg.footstep_data_list = [self.create_one_footstep(start_foot,
                                                           [0.0, distance,
                                                            0.0])]

        self.steps = 0
        self.planned_steps = len(msg.footstep_data_list)
        self.walk_failure = None
        self.start_walk = rospy.get_time()
        self.footstep_list_publisher.publish(msg)
        log('FootstepDataList: uid ' + str(msg.unique_id))

    def create_up_steps(self, distance, rise, start_foot,
                        stride=DEFAULT_STRIDE, start=0.0):
        """ Create a series of up footsteps. """

        foot = start_foot
        footsteps = []
        footsteps.append(self.create_one_footstep(foot, [distance, 0.0, rise]))
        foot = invert_foot(foot)
        footsteps.append(self.create_one_footstep(foot, [distance, 0.0, rise]))
        return footsteps

    def step_up(self, distance, rise):
        """ Step up a step """
        msg = FootstepDataListRosMessage()
        msg.default_transfer_time = self.transfer_time
        msg.default_swing_time = self.swing_time

        msg.execution_mode = msg.OVERRIDE  # Override means replace others
        msg.unique_id = self.zarj.uid

        self.lookup_feet()
        start_foot = FootstepDataRosMessage.LEFT

        msg.footstep_data_list = self.create_up_steps(distance, rise,
                                                      start_foot,
                                                      self.DEFAULT_STRIDE)
        self.steps = 0
        self.planned_steps = len(msg.footstep_data_list)
        self.walk_failure = None
        self.start_walk = rospy.get_time()
        self.footstep_list_publisher.publish(msg)
        log('FootstepDataList: uid ' + str(msg.unique_id))

    def dead_walk(self, distance, offset, turn, snapto):
        if abs(offset) >= 0.01:
            self.side(offset)
            while not self.walk_is_done():
                rospy.sleep(0.1)

        if abs(distance) >= 0.01:
            self.forward(distance)
            while not self.walk_is_done():
                rospy.sleep(0.1)

        if abs(turn) >= 0.5:
            self.turn(turn, snapto)
            while not self.walk_is_done():
                rospy.sleep(0.1)

    def abort(self):
        if not self.walk_is_done():
            log("Aborting walk!")
            msg = AbortWalkingRosMessage()
            msg.unique_id = self.zarj.uid
            self.abort_publisher.publish(msg)
            self.start_walk = None
            self.last_footstep = None
            self.planned_steps = 0
            log("OPERATOR: You will need to re-square feet!")
