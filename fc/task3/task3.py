"""
    Team ZARJ task 3 code
"""
import os
import rospy

from copy import deepcopy
import numpy as np
import cv2
from math import atanh, cos, sin, radians, degrees, sqrt

from zarjtask import ZarjTask
from zarjmacro import ZarjMacro
from zarj.confused import ZarjConfused

from zarj.utils import log

from zarjcomm.limbtypes import LimbTypes
from zarjcomm.message import ZarjMovePalmCommand
from zarjcomm.message import ZarjGetPalmCommand, ZarjGetArmJointsResponse
from zarj.tf_conv import compute_distance_to_wall
from zarj.tf_conv import compute_rot_angle
from zarj.things import Things
import zarj
from geometry_msgs.msg import Vector3

from srcsim.msg import Leak

from checkpointc1 import CheckpointC1
from checkpointc2 import CheckpointC2
from checkpointc3 import CheckpointC3
from checkpointc4 import CheckpointC4
from checkpointc5 import CheckpointC5
from checkpointc6 import CheckpointC6
from checkpointc7 import CheckpointC7
from checkpointc8 import CheckpointC8

from geometry_msgs.msg import PointStamped

LAST_LEAK = None
LEAK_SIDE = False

def find_repair_button(macro):
    image, details = macro.fc.zarj.eyes.get_cloud_image_with_details(macro.fc.cloud)
    things = Things(image, details, 3)
    if things.repair_button is not None and things.repair_button.computed_center is not None:
        point = PointStamped()
        point.header = macro.fc.cloud.header
        det = things.repair_button.computed_center
        point.point.x = det[0]
        point.point.y = det[1]
        point.point.z = det[2]
        button = macro.fc.zarj.transform.tf_buffer.transform(point, macro.fc.zarj.walk.lfname)
        log("button found at: {}/{}/{}".format(button.point.x, button.point.y, button.point.z))

        return button.point

    return None

def spin_object(macro, side):
    """ Spin an object in prep for releasing it """
    log("Spinning and dropping object on {}".format(side))
    joints = deepcopy(LimbTypes.arm_styles['rake_above'])
    joints = LimbTypes.invert_arm_configuration(side, joints)
    macro.fc.zarj.hands.set_arm_configuration(side, joints)

    if side == 'right':
        macro.fc.zarj.pelvis.turn_body_to(75)

    if side == 'left':
        macro.fc.zarj.pelvis.turn_body_to(-75)

    joints = deepcopy(LimbTypes.arm_styles['spin_object_1'])
    joints = LimbTypes.invert_arm_configuration(side, joints)
    macro.fc.zarj.hands.set_arm_configuration(side, joints)

    rospy.sleep(1.0)

    joints = deepcopy(LimbTypes.arm_styles['spin_object_2'])
    joints = LimbTypes.invert_arm_configuration(side, joints)
    macro.fc.zarj.hands.set_arm_configuration(side, joints)

    joints = deepcopy(LimbTypes.hand_styles['open'])
    joints = LimbTypes.invert_hand_configuration(side, joints)
    macro.fc.zarj.hands.set_hand_configuration(side, joints)

    joints = deepcopy(LimbTypes.arm_styles['spin_object_1'])
    joints = LimbTypes.invert_arm_configuration(side, joints)
    macro.fc.zarj.hands.set_arm_configuration(side, joints)

    joints = deepcopy(LimbTypes.arm_styles['rake_above'])
    joints = LimbTypes.invert_arm_configuration(side, joints)
    macro.fc.zarj.hands.set_arm_configuration(side, joints)

    macro.fc.zarj.pelvis.turn_body_to(0)


class find_stairs(ZarjMacro):
    """ find_stairs """

    def start(self, _=None):
        """ Start the macro """
        self.fc.zarj.go_home(0, 0)
        rospy.sleep(0.2)
        self.fc.zarj.go_home(1, 0)
        rospy.sleep(0.2)
        self.fc.zarj.zps.look_forward()
        self.fc.zarj.zps.path_start()
        if self.stop:
            return
        self.fc.zarj.zps.set_flag(zarj.zps.FLAG_STAIRS)
        self.fc.zarj.zps.walk_to_next_finish_box()
        self.done = True

    @property
    def next_chain(self):
        """ returns the name of the next macro in the chain """
        return 'climb_stairs'


class climb_stairs(ZarjMacro):
    """ climb_stairs """

    def start(self, _=None):
        """ Start the macro """
        self._report_lidar()
        self.fc.zarj.zps.climb_stairs()
        if self.stop:
            return
        self.fc.zarj.walk.forward(0.4)
        zarj.utils.wait_for_walk(self.fc.zarj)
        self.done = True

    def _report_lidar(self):
        scan = self.fc.zarj.lidar.last_scan()
        if scan is None:
            rospy.sleep(0.1)
            scan = self.fc.zarj.lidar.last_scan()
            if scan is not None:
                log("Center of lidar is now {}".format(scan.ranges[len(scan.ranges) / 2]))

    @property
    def next_chain(self):
        """ returns the name of the next macro in the chain """
        return 'approach_door'

class approach_door(ZarjMacro):
    """ approach_door """

    def _find_feature(self, tag, low, high):
        """ Find a feature on the door"""
        # we only want the center of the image
        cloud = self.fc.zarj.eyes.get_stereo_cloud()
        image, details = self.fc.zarj.eyes.get_cloud_image_with_details(cloud)
        shape = image.shape
        image = image[0:shape[0], shape[1]/3:2*shape[1]/3]
        details = details[0:shape[0], shape[1]/3:2*shape[1]/3]
        colors = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        image_dump = os.environ.get("ZARJ_IMAGE_DUMP")

        mask = cv2.inRange(colors, low, high)
        if image_dump is not None:
            image_idx = rospy.get_time()
            cv2.imwrite(image_dump+'/door_{}.png'.format(image_idx), image)
            cv2.imwrite(image_dump+'/door_{}_{}.png'.format(tag, image_idx),
                        mask)
        indexes = mask.nonzero()
        points = []
        for i in range(len(indexes[1])):
            pnt = details[indexes[0][i], indexes[1][i]]
            if pnt[2] > 2.0:
                print 'Discard non {} point, too far away'.format(tag)
                continue
            points.append(details[indexes[0][i], indexes[1][i]])
        points = np.array(points)

        if len(points) < 10:
            log("Failed to find {} in the door".format(tag))
            return None

        avg = np.mean(points, axis=0)

        point = PointStamped()
        point.header = cloud.header
        point.point.x = avg[0]
        point.point.y = avg[1]
        point.point.z = avg[2]

        final = self.fc.zarj.transform.tf_buffer.transform(point, 'pelvis')
        log('{} located at about {}'.format(tag, final.point))
        return final.point.y

    def _find_window(self):
        """ Find the bluish window """
        window_low = np.array([95, 10, 30])   # gimp 190 5 11
        window_high = np.array([144, 25, 145])  # gimp 210 8 54
        return self._find_feature('window', window_low, window_high)

    def _find_wheel(self):
        """ Find the wheel on the black door """
        wheel_low = np.array([28, 10, 15])   # gimp 60 5 7
        wheel_high = np.array([46, 15, 65])  # gimp 90 5 25
        return self._find_feature('wheel', wheel_low, wheel_high)

    def _find_door(self):
        """ Find the door, The most distant point in our cloud """
        cloud = self.fc.zarj.eyes.get_stereo_cloud()
        image, details = self.fc.zarj.eyes.get_cloud_image_with_details(cloud)

        # we only want the center of the image
        shape = image.shape
        print shape
        cloud = details[0:2*shape[0]/3, shape[1]/3:2*shape[1]/3]
        cloud = np.compress([False, False, True, False], cloud, axis=2)
        cloud = cloud.flatten()
        return np.nanmax(cloud)

    def start(self, _=None):
        """ Start the macro """
        self.fc.zarj.zps.look_forward()
        self.fc.zarj.walk.fix_stance()
        self.fc.zarj.walk.turn(1.0, snap_to=45)
        zarj.utils.wait_for_walk(self.fc.zarj)
        if self.stop:
            return
        distance = self._find_door()
        log('Door is {} m away.'.format(distance))
        if distance > 2.5:
            raise ZarjConfused("The door should never be > 2.5m away. I think "
                               "it is {}m away, Guide me!".format(distance))
        while round(distance, 2) > 0.64:
            space = distance - 0.64
            log('Door is {} m away. {} to move.'.format(distance, space))
            # Want to be 0.65m away from the door
            # But approach with exactness
            if self.stop:
                return
            if space > 0.3:
                self.fc.zarj.walk.forward(3*space/4)
                zarj.utils.wait_for_walk(self.fc.zarj)
            elif space > 0.12:
                self.fc.zarj.walk.forward(0.1)
                zarj.utils.wait_for_walk(self.fc.zarj)
            elif space > 0.06:
                self.fc.zarj.walk.forward(0.05)
                zarj.utils.wait_for_walk(self.fc.zarj)
            elif space < 0.0:
                break
            else:
                self.fc.zarj.walk.turn(1.0, snap_to=45, small_forward=space)
                zarj.utils.wait_for_walk(self.fc.zarj)
            self.fc.zarj.walk.turn(1.0, snap_to=45)
            zarj.utils.wait_for_walk(self.fc.zarj)

            offset = self._find_window()
            if offset is None:
                offset = self._find_wheel()
            if offset is None:
                raise ZarjConfused("I can see no features on the door! "
                                   "Am I lined up?")
            if abs(offset) > 0.01:
                self.fc.zarj.walk.side(offset)
                zarj.utils.wait_for_walk(self.fc.zarj)

            new_distance = self._find_door()
            if new_distance > distance:
                raise ZarjConfused("I moved forward, but the door got "
                                   "farther away! {} vs {}! Help?".format(
                                       distance, new_distance))
            distance = new_distance
        self.fc.zarj.neck.head_down(False)
        self.done = True

    @property
    def next_chain(self):
        """ returns the name of the next macro in the chain """
        return 'unlock_door'

class unlock_door(ZarjMacro):
    """ unlock_door """

    def _spin_wheel(self):
        """ Spin the wheel. """
        joints = deepcopy(LimbTypes.arm_styles['door_out'])
        joints = LimbTypes.invert_arm_configuration('left', joints)
        self.fc.zarj.hands.set_arm_configuration('left', joints, 1.0)
        rospy.sleep(0.5)

        joints = deepcopy(LimbTypes.arm_styles['door_top'])
        joints = LimbTypes.invert_arm_configuration('left', joints)
        self.fc.zarj.hands.set_arm_configuration('left', joints, 1.0)
        rospy.sleep(0.5)

        joints = deepcopy(LimbTypes.arm_styles['door_bottom'])
        joints = LimbTypes.invert_arm_configuration('left', joints)
        self.fc.zarj.hands.set_arm_configuration('left', joints, 1.5)
        rospy.sleep(0.5)

    def start(self, _=None):
        """ Start the macro """
        joints = deepcopy(LimbTypes.hand_styles['claw'])
        joints = LimbTypes.invert_hand_configuration('left', joints)
        self.fc.zarj.hands.set_hand_configuration('left', joints)
        if self.stop:
            return

        self._spin_wheel()
        if self.stop:
            return
        self._spin_wheel()
        if self.stop:
            return
        self._spin_wheel()
        if self.stop:
            return
        self._spin_wheel()
        if self.stop:
            return
        self._spin_wheel()
        if self.stop:
            return

        joints = deepcopy(LimbTypes.hand_styles['open'])
        joints = LimbTypes.invert_hand_configuration('left', joints)
        self.fc.zarj.hands.set_hand_configuration('left', joints)

        joints = deepcopy(LimbTypes.arm_styles['door_out'])
        joints = LimbTypes.invert_arm_configuration('left', joints)
        self.fc.zarj.hands.set_arm_configuration('left', joints, 1.0)
        rospy.sleep(0.5)

        self.done = True

    @property
    def next_chain(self):
        """ returns the name of the next macro in the chain """
        return 'open_door'

class open_door(ZarjMacro):
    """ open_door """

    def start(self, _=None):
        """ Start the macro """
        joints = deepcopy(LimbTypes.hand_styles['fist'])
        joints = LimbTypes.invert_hand_configuration('left', joints)
        self.fc.zarj.hands.set_hand_configuration('left', joints)

        joints = deepcopy(LimbTypes.hand_styles['fist'])
        joints = LimbTypes.invert_hand_configuration('right', joints)
        self.fc.zarj.hands.set_hand_configuration('right', joints)

        joints = deepcopy(LimbTypes.arm_styles['door_push'])
        joints = LimbTypes.invert_arm_configuration('left', joints)
        self.fc.zarj.hands.set_arm_configuration('left', joints, 0.5)

        joints = deepcopy(LimbTypes.arm_styles['door_push'])
        joints = LimbTypes.invert_arm_configuration('right', joints)
        self.fc.zarj.hands.set_arm_configuration('right', joints, 1.0)

        self.fc.zarj.zps.look_forward()

        self.fc.zarj.walk.forward(0.4)
        zarj.utils.wait_for_walk(self.fc.zarj)
        if self.stop:
            return
        rospy.sleep(0.2)

        joints = deepcopy(LimbTypes.arm_styles['tuck'])
        joints = LimbTypes.invert_arm_configuration('right', joints)
        self.fc.zarj.hands.set_arm_configuration('right', joints, 1.0)

        self.fc.zarj.walk.side(-0.1)
        zarj.utils.wait_for_walk(self.fc.zarj)
        if self.stop:
            return
        rospy.sleep(0.2)

        self.fc.zarj.walk.forward(0.4)
        zarj.utils.wait_for_walk(self.fc.zarj)
        if self.stop:
            return
        rospy.sleep(0.2)

        self.fc.zarj.walk.forward(0.4)
        zarj.utils.wait_for_walk(self.fc.zarj)
        if self.stop:
            return
        rospy.sleep(0.2)

        self.fc.zarj.walk.side(-0.1)
        zarj.utils.wait_for_walk(self.fc.zarj)
        if self.stop:
            return
        rospy.sleep(0.2)

        joints = deepcopy(LimbTypes.arm_styles['tuck'])
        joints = LimbTypes.invert_arm_configuration('left', joints)
        self.fc.zarj.hands.set_arm_configuration('left', joints, 1.0)
        if self.stop:
            return

        self.fc.zarj.walk.forward(1.0)
        zarj.utils.wait_for_walk(self.fc.zarj)
        if self.stop:
            return
        rospy.sleep(0.2)

        self.fc.zarj.walk.turn(-45)
        zarj.utils.wait_for_walk(self.fc.zarj)
        if self.stop:
            return
        self.fc.zarj.walk.turn(-45)
        zarj.utils.wait_for_walk(self.fc.zarj)
        if self.stop:
            return

        self.fc.zarj.go_home(0, 0)
        rospy.sleep(0.2)
        self.fc.zarj.go_home(1, 0)
        rospy.sleep(0.2)

        self.done = True

    @property
    def next_chain(self):
        """ returns the name of the next macro in the chain """
        return 'enter_habitat'

class enter_habitat(ZarjMacro):
    """ enter_habitat """

    def start(self, _=None):
        """ Start the macro """
        self.fc.zarj.walk.forward(1.7)
        zarj.utils.wait_for_walk(self.fc.zarj)
        self.fc.zarj.zps.set_flag(zarj.zps.FLAG_HABITAT)

class walk_to_table(ZarjMacro):
    """ walk_to_table """

    def start(self, _=None):
        """ Start the macro """
        anchor = self.fc.assure_anchor("center")
        if anchor is None:
            log('Click on the 2 farthest parts of the side of the table '
                'you want to approach')
            return False

        self.fc.zarj.walk.fix_stance()
        zarj.utils.wait_for_walk(self.fc.zarj)

        r_x = anchor.adjusted[0]
        r_y = anchor.adjusted[1]
        r_a = anchor.angle

        log('Table is {},{} at angle {}'.format(r_x, r_y, r_a))

        off_a = 90 - r_a
        off_x = sin(radians(off_a))*0.75
        off_y = cos(radians(off_a))*0.75

        tgt_x = r_x - off_x
        tgt_y = r_y + off_y

        log('Approach point is {},{}'.format(tgt_x, tgt_y))

        heading = degrees(atanh(tgt_y/tgt_x))
        distance = sqrt(tgt_x**2 + tgt_y**2)

        log('Path has a heading of {} and distance {}m'.format(heading,
                                                               distance))

        self.fc.zarj.walk.turn(heading, snap_to=0.0)
        zarj.utils.wait_for_walk(self.fc.zarj)
        if self.stop:
            return

        self.fc.zarj.walk.forward(distance)
        zarj.utils.wait_for_walk(self.fc.zarj)
        if self.stop:
            return

        turn = r_a + heading

        if abs(turn) > 45:
            self.fc.zarj.walk.turn(-turn/2.0, snap_to=0.0)
            zarj.utils.wait_for_walk(self.fc.zarj)
            if self.stop:
                return
            self.fc.zarj.walk.turn(-turn/2.0, snap_to=0.0)
        else:
            self.fc.zarj.walk.turn(-turn, snap_to=0.0)
        zarj.utils.wait_for_walk(self.fc.zarj)
        if self.stop:
            return

        self.fc.zarj.walk.forward(0.2)
        zarj.utils.wait_for_walk(self.fc.zarj)


class rake_prepare(ZarjMacro):
    """ Prepare to rake an object """

    def start(self, _=None):
        """ Start the macro """
        joints = deepcopy(LimbTypes.arm_styles['rake_above'])
        joints = LimbTypes.invert_arm_configuration('right', joints)
        log("Setting right hand and arm to start raking")
        self.fc.zarj.hands.set_arm_configuration('right', joints)

        joints = deepcopy(LimbTypes.hand_styles['rake'])
        joints = LimbTypes.invert_hand_configuration('right', joints)
        self.fc.zarj.hands.set_hand_configuration('right', joints)

        joints = deepcopy(LimbTypes.arm_styles['tuck'])
        joints = LimbTypes.invert_arm_configuration('left', joints)
        log("Setting left hand out of the way we hope")
        self.fc.zarj.hands.set_arm_configuration('left', joints)

        if not self.chain_okay:
            self.fc.send_stereo_camera()
            log("Pick the object to rake; two points where hand will land.")

        self.done = True

    @property
    def next_chain(self):
        """ returns the name of the next macro in the chain """
        return "rake_move_feet"

class rake_move_feet(ZarjMacro):
    """ Prepare to rake an object """

    def start(self, _=None):
        """ Start the macro """
        anchor = self.fc.assure_anchor("rake_measure")
        if anchor is None:
            raise ZarjConfused("Need clicks on object")

        forward = anchor.adjusted[0]
        sideways = anchor.adjusted[1]

        where = [ forward, 0.0, sideways ]
        log("Rake: walking sideways {}, forward {}".format(sideways, forward))
        log("TODO - maybe this should not be a deadwalk")
        self.fc.zarj.zps.dead_walk([forward, 0.0, sideways])

        self.done = True

    @property
    def next_chain(self):
        """ returns the name of the next macro in the chain """
        return "rake_start"


class rake_start(ZarjMacro):
    def start(self, _=None):
        log("No leaning, turn 75")
        self.fc.zarj.pelvis.lean_body_to(0)
        self.fc.zarj.pelvis.turn_body_to(75)
        rospy.sleep(0.5)  # Really far turns take a little longer

        joints = deepcopy(LimbTypes.arm_styles['rake_start'])
        joints = LimbTypes.invert_arm_configuration('right', joints)
        log("Setting right arm to start raking")
        self.fc.zarj.hands.set_arm_configuration('right', joints)

        self.done = True

    @property
    def next_chain(self):
        """ returns the name of the next macro in the chain """
        return "rake_back"


class rake_back(ZarjMacro):
    def start(self, _=None):
        joints = deepcopy(LimbTypes.arm_styles['rake_start'])
        joints = LimbTypes.invert_arm_configuration('right', joints)
        self.fc.zarj.hands.set_arm_configuration('right', joints)

        endjoints = deepcopy(LimbTypes.arm_styles['rake_end'])
        joint2_increment = (endjoints[1] - joints[1]) / 10.0
        joint4_increment = (endjoints[3] - joints[3]) / 10.0

        for i in range(10):
            joints[1] += joint2_increment
            joints[3] += joint4_increment
            self.fc.zarj.hands.set_arm_configuration('right', joints)
        self.done = True

    @property
    def next_chain(self):
        """ returns the name of the next macro in the chain """
        return "rake_finish"

class rake_finish(ZarjMacro):
    def start(self, _=None):

        joints = deepcopy(LimbTypes.arm_styles['rake_above'])
        joints = LimbTypes.invert_arm_configuration('right', joints)
        log("Setting right arm clear")
        self.fc.zarj.hands.set_arm_configuration('right', joints)

        self.fc.zarj.pelvis.turn_body_to(0)
        rospy.sleep(0.5)

        self.done = True


class pickup_detector_prep(ZarjMacro):
    """ pickup_detector_prep """

    def start(self, _=None):
        """ Start the macro """
        joints = deepcopy(LimbTypes.arm_styles['pass_football'])
        joints = LimbTypes.invert_arm_configuration('right', joints)
        log("Setting right hand to pass a football")
        self.fc.zarj.hands.set_arm_configuration('right', joints)

        joints = deepcopy(LimbTypes.arm_styles['king_tut'])
        joints = LimbTypes.invert_arm_configuration('left', joints)
        log("Extending left arm, setting left hand to grab down")
        self.fc.zarj.hands.set_arm_configuration('left', joints)

        joints = deepcopy(LimbTypes.hand_styles['open_down'])
        joints = LimbTypes.invert_hand_configuration('left', joints)
        self.fc.zarj.hands.set_hand_configuration('left', joints)

        rospy.sleep(0.5)

        self.fc.send_stereo_camera()
        log("Pick two points along the _GRIP_, long way (antenna <--> base) of "
            "the detector. Generally just a little in toward the robot from "
            "the center line")


class pickup_detector_move_feet(ZarjMacro):
    """pickup_detector_move_feet"""

    def start(self, _=None):
        """ Start the macro """
        anchor = self.fc.assure_anchor("center")
        if anchor is None:
            return

        if len(self.fc.zarj.zps.waypoint_stack) == 0:
            self.fc.zarj.zps.mark_waypoint()

        log("Object is {} away, {} to the side, at angle {}".format(anchor.adjusted[0], anchor.adjusted[1], anchor.angle))

        forward = anchor.adjusted[0] - 0.75
        if anchor.angle > 0:
            sideways = anchor.adjusted[1] - (-.30)
        else:
            sideways = anchor.adjusted[1] - .25

        where = [ forward, 0.0, sideways ]
        log("Walking sideways {}, forward {}".format(sideways, forward))
        log("TODO - maybe this should not be a deadwalk")
        self.fc.zarj.zps.dead_walk([forward, 0.0, sideways])

        log("Look left")
        self.fc.zarj.neck.neck_control([ 0.5, 1.0, 0 ], True)

        log("Lean down -30, turn -45")
        self.fc.zarj.pelvis.lean_body_to(-30)
        self.fc.zarj.pelvis.turn_body_to(-45)

        rospy.sleep(0.5)

    @property
    def next_chain(self):
        """ returns the name of the next macro in the chain """
        return "pickup_detector_hand_start"


class pickup_detector_hand_start(ZarjMacro):
    """pickup_detector_hand_start"""

    def start(self, _=None):
        """ Start the macro """
        anchor = self.fc.assure_anchor("center")
        if anchor is None:
            return False

        x = anchor.adjusted[0]
        y = anchor.adjusted[1]
        z = 0.838 + 0.1

        log("Commanding hand to position just above object")
        msg = ZarjMovePalmCommand('left', False, x, y, z, -1 * anchor.angle, 25, 90, True)
        self.fc.process_palm_msg(msg)

        log("Giving it a while to settle")
        rospy.sleep(3.0)

        log("Commanding hand onto object")
        msg = ZarjMovePalmCommand('left', False, x, y, z - .08, -1 * anchor.angle, 25, 90, True)
        self.fc.process_palm_msg(msg)

        rospy.sleep(0.5)

    @property
    def next_chain(self):
        """ returns the name of the next macro in the chain """
        return 'pickup_detector_grab'

class pickup_detector_grab(ZarjMacro):
    """ pickup_detector_grab """

    def start(self, _=None):
        """ Start the macro """
        anchor = self.fc.assure_anchor("center")
        if anchor is None:
            return False

        x = anchor.adjusted[0]
        y = anchor.adjusted[1]
        z = 0.838 + 0.1

        log("Commanding hand to grab array")

        joints = deepcopy(LimbTypes.hand_styles['partial_grab'])
        joints = LimbTypes.invert_hand_configuration('left', joints)
        self.fc.zarj.hands.set_hand_configuration('left', joints)

        rospy.sleep(0.5)  # A little extra time to let us get a good grip

        joints = deepcopy(LimbTypes.hand_styles['partial_ungrab'])
        joints = LimbTypes.invert_hand_configuration('left', joints)
        self.fc.zarj.hands.set_hand_configuration('left', joints)

        log("Commanding hand down a bit more, and trying to bend around a hair (1)")
        msg = ZarjMovePalmCommand('left', False, x, y, z - .11, -1 * anchor.angle, 30, 90, True)
        self.fc.process_palm_msg(msg)

        joints = deepcopy(LimbTypes.hand_styles['partial_grab2'])
        joints = LimbTypes.invert_hand_configuration('left', joints)
        self.fc.zarj.hands.set_hand_configuration('left', joints)

        joints = deepcopy(LimbTypes.hand_styles['partial_ungrab'])
        joints = LimbTypes.invert_hand_configuration('left', joints)
        self.fc.zarj.hands.set_hand_configuration('left', joints)

        log("Commanding hand down a bit more, and trying to bend around a hair (2)")
        msg = ZarjMovePalmCommand('left', False, x, y, z - .12, -1 * anchor.angle, 30, 90, True)
        self.fc.process_palm_msg(msg)

        log("Going for final grab")
        joints = deepcopy(LimbTypes.hand_styles['grab_object'])
        joints = LimbTypes.invert_hand_configuration('left', joints)
        self.fc.zarj.hands.set_hand_configuration('left', joints)

        rospy.sleep(1.0)  # A little settle time helps

    @property
    def next_chain(self):
        """ returns the name of the next macro in the chain """
        return 'pickup_detector_all_done'


class pickup_detector_all_done(ZarjMacro):
    """ pickup_detector_all_done """

    def start(self, _=None):
        """ Start the macro """
        anchor = self.fc.assure_anchor("center")
        if anchor is None:
            return False

        x = anchor.adjusted[0]
        y = anchor.adjusted[1]
        z = 0.838 + 0.25

        log("Commanding up 25 cm")
        log("Cancel quick if you don't like it")
        msg = ZarjMovePalmCommand('left', False, x, y, z, 0, 0, 90, True)
        self.fc.process_palm_msg(msg)

        rospy.sleep(3.0)
        if self.stop:
            return

        log("Flipping hand over")
        msg = ZarjMovePalmCommand('left', False, x, y, z, -30, -30, -90, True)
        self.fc.process_palm_msg(msg)

        log("Looking up a hair")
        self.fc.zarj.neck.neck_control([ 0.3, 1.0, 0 ], True)
        log("Wait forever.  It should come into view.")
        log("If you do not like the grab, you can do partial ungrab, and grab to try again.")

class pickup_repair_tool_prep(ZarjMacro):
    """ pickup_repair_tool_prep """

    def start(self, _=None):
        """ Start the macro """
        joints = deepcopy(LimbTypes.arm_styles['tuck'])
        joints = LimbTypes.invert_arm_configuration('left', joints)
        log("Setting left hand to tuck")
        self.fc.zarj.hands.set_arm_configuration('left', joints)

        joints = deepcopy(LimbTypes.arm_styles['king_tut'])
        joints = LimbTypes.invert_arm_configuration('right', joints)
        log("Extending left arm, setting left hand to grab down")
        self.fc.zarj.hands.set_arm_configuration('right', joints)

        joints = deepcopy(LimbTypes.hand_styles['open_down'])
        joints = LimbTypes.invert_hand_configuration('right', joints)
        self.fc.zarj.hands.set_hand_configuration('right', joints)

        rospy.sleep(0.5)

        self.fc.send_stereo_camera()
        log("Pick two points along the repair tool so we know it's angle.")
        log("If you have it rolled towards you - button facing you - then you can")
        log("execute and chain pickup_repair_tool_vision_set_angle")


class pickup_repair_tool_move_feet(ZarjMacro):
    """pickup_repair_tool_move_feet"""

    def start(self, _=None):
        """ Start the macro """
        anchor = self.fc.assure_anchor("center")
        if anchor is None:
            return

        if len(self.fc.zarj.zps.waypoint_stack) == 0:
            self.fc.zarj.zps.mark_waypoint()

        log("Object is {} away, {} to the side, at angle {}".format(anchor.adjusted[0], anchor.adjusted[1], anchor.angle))

        forward = anchor.adjusted[0] - 0.75
        if anchor.angle > 0:
            sideways = (anchor.adjusted[1] + self.fc.zarj.walk.DEFAULT_STANCE_WIDTH) - (-.30)
        else:
            sideways = (anchor.adjusted[1] + self.fc.zarj.walk.DEFAULT_STANCE_WIDTH) - .30

        where = [ forward, 0.0, sideways ]
        log("Walking sideways {}, forward {}".format(sideways, forward))
        log("TODO - maybe this should not be a deadwalk")

        self.fc.zarj.zps.dead_walk([forward, 0.0, sideways])

        log("Look right")
        self.fc.zarj.neck.neck_control([ 0.5, -1.0, 0 ], True)

        log("Lean down -30, turn 45")
        self.fc.zarj.pelvis.lean_body_to(-30)
        self.fc.zarj.pelvis.turn_body_to(45)

        rospy.sleep(0.5)
        log("Next, either start the hand for pickup, or roll the tool away.")


class pickup_repair_tool_roll_prep(ZarjMacro):
    """ Prepare Roll the tool away from us """

    def start(self, _=None):
        """ Start the macro """

        log("Look right")
        self.fc.zarj.neck.neck_control([ 0.5, -1.0, 0 ], True)

        log("Lean down -30, turn 45")
        self.fc.zarj.pelvis.lean_body_to(-35)
        self.fc.zarj.pelvis.turn_body_to(45)

        log("Set right arm a little higher")
        joints = deepcopy(LimbTypes.arm_styles['king_tut_high'])
        joints = LimbTypes.invert_arm_configuration('right', joints)
        self.fc.zarj.hands.set_arm_configuration('right', joints)

        self.fc.send_stereo_camera()
        log("Pick two points along the repair tool so we know it's angle.")

class pickup_repair_tool_roll_away(ZarjMacro):
    """ Roll the tool away from us """

    def start(self, _=None):
        """ Start the macro """
        anchor = self.fc.assure_anchor("center")
        if anchor is None:
            return False

        repair_button_point = find_repair_button(self)
        if repair_button_point is None:
            raise ZarjConfused("Can't find blue button")

        joints = deepcopy(LimbTypes.hand_styles['open'])
        joints = LimbTypes.invert_hand_configuration('right', joints)
        self.fc.zarj.hands.set_hand_configuration('right', joints)

        x = repair_button_point.x - (.1 * cos(radians(anchor.angle)))
        y = repair_button_point.y + (.1 * sin(radians(anchor.angle)))
        z = repair_button_point.z + 0.1

        log("Commanding hand to short of pushing, but high")
        msg = ZarjMovePalmCommand('right', False, x, y, z, -1 * anchor.angle, 25, -90, True)
        self.fc.process_palm_msg(msg)

        rospy.sleep(1.0)

        log("Commanding hand to short of pushing")
        z = repair_button_point.z - 0.10
        msg = ZarjMovePalmCommand('right', False, x, y, z, -1 * anchor.angle, 25, -90, True)
        self.fc.process_palm_msg(msg)

        log("Giving it a while to settle")
        rospy.sleep(1.0)

        x = repair_button_point.x + (.15 * cos(radians(anchor.angle)))
        y = repair_button_point.y - (.15 * sin(radians(anchor.angle)))
        z = repair_button_point.z - 0.07

        log("Commanding hand to roll it")
        msg = ZarjMovePalmCommand('right', False, x, y, z, -1 * anchor.angle, 25, -90, True)
        self.fc.process_palm_msg(msg)

        x = repair_button_point.x - (.1 * cos(radians(anchor.angle)))
        y = repair_button_point.y + (.1 * sin(radians(anchor.angle)))
        z = repair_button_point.z + 0.1

        log("Commanding hand to back off again")
        msg = ZarjMovePalmCommand('right', False, x, y, z, -1 * anchor.angle, 25, -90, True)
        self.fc.process_palm_msg(msg)



class pickup_repair_tool_vision_set_angle(ZarjMacro):
    """ Use vision to pickup repair tool; prep stage"""

    def start(self, _=None):
        """ Start the macro """
        self.fc.repair_angle = None
        anchor = self.fc.assure_anchor("center")
        if anchor is None:
            raise ZarjConfused("Require two points to set the angle")

        self.fc.repair_angle = anchor.angle

        button = find_repair_button(self)
        if button is None:
            raise ZarjConfused("Can't find blue button")

        self.fc.points[0] = [ button.x, button.y, button.z ]
        self.fc.points[1] = None


    @property
    def next_chain(self):
        """ returns the name of the next macro in the chain """
        return 'pickup_repair_tool_one_point_move_feet'


class pickup_repair_tool_one_point_move_feet(ZarjMacro):
    """ Use vision to pickup repair tool; prep stage"""

    def start(self, _=None):
        """ Start the macro """
        anchor = self.fc.assure_anchor("vision_repair_move")
        if anchor is None:
            raise ZarjConfused("Require a point to grab")

        if len(self.fc.zarj.zps.waypoint_stack) == 0:
            self.fc.zarj.zps.mark_waypoint()

        log("Object is {} away, {} to the side, at angle {}".format(anchor.adjusted[0], anchor.adjusted[1], self.fc.repair_angle))

        forward = anchor.adjusted[0]
        if self.fc.repair_angle > 0:
            sideways = (anchor.adjusted[1] + self.fc.zarj.walk.DEFAULT_STANCE_WIDTH) - (-.20)
        else:
            sideways = (anchor.adjusted[1] + self.fc.zarj.walk.DEFAULT_STANCE_WIDTH) - .20

        where = [ forward, 0.0, sideways ]
        log("Walking sideways {}, forward {}".format(sideways, forward))
        log("TODO - maybe this should not be a deadwalk")

        self.fc.zarj.zps.dead_walk([forward, 0.0, sideways])

        log("Look right")
        self.fc.zarj.neck.neck_control([ 0.5, -1.0, 0 ], True)

        log("Lean down -30, turn 45")
        self.fc.zarj.pelvis.lean_body_to(-30)
        self.fc.zarj.pelvis.turn_body_to(45)

        joints = deepcopy(LimbTypes.arm_styles['king_tut_high'])
        joints = LimbTypes.invert_arm_configuration('right', joints)
        log("Move right arm out of the way")
        self.fc.zarj.hands.set_arm_configuration('right', joints)

        rospy.sleep(0.5)

    @property
    def next_chain(self):
        """ returns the name of the next macro in the chain """
        return 'pickup_repair_tool_one_point_start'


class pickup_repair_tool_one_point_start(ZarjMacro):
    """ Use vision to pickup relative to the button """

    def start(self, _=None):
        """ Start the macro """
        if self.fc.repair_angle is None:
            raise ZarjConfused("Repair angle must be set.")
        anchor = self.fc.assure_anchor("identity")
        if anchor is None:
            return False

        log("Commanding hand to initial shape")
        joints = deepcopy(LimbTypes.hand_styles['repair_grab1'])
        joints = LimbTypes.invert_hand_configuration('right', joints)
        self.fc.zarj.hands.set_hand_configuration('right', joints)

        # We want the hand a bit past the button
        x = anchor.adjusted[0] + (0.06 * cos(radians(self.fc.repair_angle)))
        y = anchor.adjusted[1] - (0.06 * sin(radians(self.fc.repair_angle)))

        # And down towards the center of the tool
        x = x - (.03 * sin(radians(self.fc.repair_angle)))
        y = y - (.03 * cos(radians(self.fc.repair_angle)))

        z = 0.838 + 0.1

        log("Commanding hand to above the button")
        msg = ZarjMovePalmCommand('right', False, x, y, z, -1 * self.fc.repair_angle, 35, -90, True)
        self.fc.process_palm_msg(msg)

        rospy.sleep(3.0)

        log("Commanding hand onto object")
        msg = ZarjMovePalmCommand('right', False, x, y, z - .08, -1 * self.fc.repair_angle, 35, -90, True)
        self.fc.process_palm_msg(msg)


    @property
    def next_chain(self):
        """ returns the name of the next macro in the chain """
        return 'pickup_repair_tool_one_point_grab'


class pickup_repair_tool_one_point_grab(ZarjMacro):
    """ Use vision to pickup relative to the button """

    def start(self, _=None):
        """ Start the macro """
        if self.fc.repair_angle is None:
            raise ZarjConfused("Repair angle must be set.")
        anchor = self.fc.assure_anchor("identity")
        if anchor is None:
            return False

        # We want the hand a bit past the button
        x = anchor.adjusted[0] + (0.06 * cos(radians(self.fc.repair_angle)))
        y = anchor.adjusted[1] - (0.06 * sin(radians(self.fc.repair_angle)))

        # And down towards the center of the tool
        x = x - (.03 * sin(radians(self.fc.repair_angle)))
        y = y - (.03 * cos(radians(self.fc.repair_angle)))

        z = 0.838 + 0.1

        log("Commanding hand to grab repair tool")

        joints = deepcopy(LimbTypes.hand_styles['repair_grab2'])
        joints = LimbTypes.invert_hand_configuration('right', joints)
        self.fc.zarj.hands.set_hand_configuration('right', joints)

        rospy.sleep(0.5)  # A little extra time to let us get a good grip

        joints = deepcopy(LimbTypes.hand_styles['repair_ungrab'])
        joints = LimbTypes.invert_hand_configuration('right', joints)
        self.fc.zarj.hands.set_hand_configuration('right', joints)

        log("Commanding hand down a bit more (1)")
        msg = ZarjMovePalmCommand('right', False, x, y, z - .09, -1 * self.fc.repair_angle, 35, -90, True)
        self.fc.process_palm_msg(msg)

        joints = deepcopy(LimbTypes.hand_styles['repair_grab3'])
        joints = LimbTypes.invert_hand_configuration('right', joints)
        self.fc.zarj.hands.set_hand_configuration('right', joints)

        joints = deepcopy(LimbTypes.hand_styles['repair_ungrab'])
        joints = LimbTypes.invert_hand_configuration('right', joints)
        self.fc.zarj.hands.set_hand_configuration('right', joints)

        log("Commanding hand down a bit more (2)")
        msg = ZarjMovePalmCommand('right', False, x, y, z - .11, -1 * self.fc.repair_angle, 35, -90, True)
        self.fc.process_palm_msg(msg)

        log("Going for final grab")
        joints = deepcopy(LimbTypes.hand_styles['grab'])
        joints = LimbTypes.invert_hand_configuration('right', joints)
        self.fc.zarj.hands.set_hand_configuration('right', joints)

    @property
    def next_chain(self):
        """ returns the name of the next macro in the chain """
        return 'pickup_repair_tool_one_point_standup'


class pickup_repair_tool_one_point_standup(ZarjMacro):
    """ Make sure it all seems good """

    def start(self, _=None):
        """ Start the macro """
        log("Move right arm up, turn back to normal position")
        joints = deepcopy(LimbTypes.arm_styles['king_tut'])
        joints = LimbTypes.invert_arm_configuration('right', joints)
        self.fc.zarj.hands.set_arm_configuration('right', joints)
        self.fc.zarj.neck.neck_control([ 0.5, 0.0, 0 ], True)

        self.fc.zarj.pelvis.lean_body_to(0)
        self.fc.zarj.pelvis.turn_body_to(0)



class pickup_repair_tool_hand_start(ZarjMacro):
    """pickup_repair_tool_hand_start"""

    def start(self, _=None):
        """ Start the macro """
        anchor = self.fc.assure_anchor("center")
        if anchor is None:
            return False

        x = anchor.adjusted[0]
        y = anchor.adjusted[1]
        z = anchor.adjusted[2] + .1

        log("Commanding hand to position just above object")
        msg = ZarjMovePalmCommand('right', False, x, y, z, -1 * anchor.angle, 25, -90, True)
        self.fc.process_palm_msg(msg)

        log("Giving it a while to settle")
        rospy.sleep(3.0)

        log("Commanding hand onto object")
        msg = ZarjMovePalmCommand('right', False, x, y, z - .08, -1 * anchor.angle, 25, -90, True)
        self.fc.process_palm_msg(msg)

        rospy.sleep(0.5)

    @property
    def next_chain(self):
        """ returns the name of the next macro in the chain """
        return 'pickup_repair_tool_grab'

class pickup_repair_tool_grab(ZarjMacro):
    """ pickup_repair_tool_grab """

    def start(self, _=None):
        """ Start the macro """
        anchor = self.fc.assure_anchor("center")
        if anchor is None:
            return False

        x = anchor.adjusted[0]
        y = anchor.adjusted[1]
        z = anchor.adjusted[2] + .1

        log("Commanding hand to grab array")

        joints = deepcopy(LimbTypes.hand_styles['partial_grab'])
        joints = LimbTypes.invert_hand_configuration('right', joints)
        self.fc.zarj.hands.set_hand_configuration('right', joints)

        rospy.sleep(0.5)  # A little extra time to let us get a good grip

        joints = deepcopy(LimbTypes.hand_styles['partial_ungrab'])
        joints = LimbTypes.invert_hand_configuration('right', joints)
        self.fc.zarj.hands.set_hand_configuration('right', joints)

        log("Commanding hand down a bit more, and trying to bend around a hair (1)")
        msg = ZarjMovePalmCommand('right', False, x, y, z - .11, -1 * anchor.angle, 30, -90, True)
        self.fc.process_palm_msg(msg)

        joints = deepcopy(LimbTypes.hand_styles['partial_grab2'])
        joints = LimbTypes.invert_hand_configuration('right', joints)
        self.fc.zarj.hands.set_hand_configuration('right', joints)

        joints = deepcopy(LimbTypes.hand_styles['partial_ungrab'])
        joints = LimbTypes.invert_hand_configuration('right', joints)
        self.fc.zarj.hands.set_hand_configuration('right', joints)

        log("Commanding hand down a bit more, and trying to bend around a hair (2)")
        msg = ZarjMovePalmCommand('right', False, x, y, z - .12, -1 * anchor.angle, 30, -90, True)
        self.fc.process_palm_msg(msg)

        log("Going for final grab")
        joints = deepcopy(LimbTypes.hand_styles['grab_object'])
        joints = LimbTypes.invert_hand_configuration('right', joints)
        self.fc.zarj.hands.set_hand_configuration('right', joints)

class pickup_repair_tool_test_grip(ZarjMacro):
    """ pickup_repair_tool_test_grip """

    def start(self, _=None):
        """ Start the macro """

        current = self.fc.zarj.hands.get_joint_states('right')
        joints = [current[0]['rightShoulderPitch'][0],
                  current[1]['rightShoulderRoll'][0],
                  current[2]['rightShoulderYaw'][0],
                  current[3]['rightElbowPitch'][0],
                  current[4]['rightForearmYaw'][0],
                  current[5]['rightWristRoll'][0],
                  current[6]['rightWristPitch'][0]]
        joints[0] -= 0.2
        self.fc.zarj.hands.set_arm_configuration('right', joints)


class pickup_repair_tool_all_done(ZarjMacro):
    """ pickup_repair_tool_all_done """

    def start(self, _=None):
        """ Start the macro """
        self.fc.zarj.zps.look_forward()
        self.fc.zarj.pelvis.turn_body_to(0)
        joints = deepcopy(LimbTypes.arm_styles['front_wipe_top'])
        joints = LimbTypes.invert_arm_configuration('right', joints)
        self.fc.zarj.hands.set_arm_configuration('right', joints)
        joints = deepcopy(LimbTypes.arm_styles['tuck'])
        joints = LimbTypes.invert_arm_configuration('right', joints)
        self.fc.zarj.hands.set_arm_configuration('right', joints)

        self.fc.zarj.walk.turn(1.0, snap_to=1)
        zarj.utils.wait_for_walk(self.fc.zarj)
        self.fc.zarj.zps.return_to_waypoint()

class spin_and_drop_detector(ZarjMacro):
    def start(self, _=None):
        spin_object(self, 'left')
        self.done = True

class spin_and_drop_repair_tool(ZarjMacro):
    def start(self, _=None):
        spin_object(self, 'right')
        self.done = True

def _lidar_wall(macro):
    scan = macro.fc.zarj.lidar.last_scan()
    if scan is not None:
        chunk = int(radians(15.0) / scan.angle_increment)
        center = int(abs(scan.angle_min / scan.angle_increment))
        angle = degrees(scan.angle_increment * ((chunk*2)+1))
        angle, distance = compute_distance_to_wall(
            angle, scan.ranges[center - chunk], scan.ranges[center + chunk])
        distance = min(scan.ranges[center - chunk], distance)
        distance = min(scan.ranges[center + chunk], distance)
        distance = min(scan.ranges[center], distance)
        return distance, angle
    return None, None

class walk_to_wall(ZarjMacro):
    """ walk_to_wall """

    def start(self, _=None):
        """ Start the macro """
        self.fc.zarj.zps.look_forward()
        self.fc.zarj.walk.fix_stance()
        zarj.utils.wait_for_walk(self.fc.zarj)
        if self.stop:
            return
        distance, angle = _lidar_wall(self)

        if distance is None:
            raise ZarjConfused("Cannot approach wall without lidar")

        log('wall is {} m away.'.format(distance))

        while round(distance, 2) > 0.63:
            space = distance - 0.63
            log('Wall is {} m away. {} to move.'.format(distance, space))
            # But approach with exactness
            if self.stop:
                return
            if space > 0.3:
                self.fc.zarj.walk.forward(space/2)
                zarj.utils.wait_for_walk(self.fc.zarj)
            elif space > 0.10:
                self.fc.zarj.walk.forward(0.05)
                zarj.utils.wait_for_walk(self.fc.zarj)
            elif space < 0.0:
                break
            else:
                self.fc.zarj.walk.turn(0.0, snap_to=0, small_forward=space)
                zarj.utils.wait_for_walk(self.fc.zarj)

            new_distance, angle = _lidar_wall(self)
            if new_distance is None:
                raise ZarjConfused("Cannot approach wall without lidar")
            distance = new_distance
            if angle is not None and abs(round(angle, 0)) > 0.0:
                log("adjusting angle to wall by {} degree".format(angle))
                self.fc.zarj.walk.turn(angle, snap_to=0.0)
                zarj.utils.wait_for_walk(self.fc.zarj)
        self.done = True


class move_to_sweep_start(ZarjMacro):
    """ move_to_sweep_start """

    def _approach_corner(self):
        distance, _ = _lidar_wall(self)
        log('wall is {} m away.'.format(distance))
        if distance is None:
            raise ZarjConfused("Cannot approach wall without lidar")

        while round(distance, 2) > 1.10:
            space = distance - 1.10
            log('Wall is {} m away. {} to move.'.format(distance, space))
            # But approach with exactness
            if self.stop:
                return
            if space > 0.3:
                self.fc.zarj.walk.forward(3*space/4)
                zarj.utils.wait_for_walk(self.fc.zarj)
            elif space > 0.12:
                self.fc.zarj.walk.forward(0.1)
                zarj.utils.wait_for_walk(self.fc.zarj)
            elif space > 0.06:
                self.fc.zarj.walk.forward(0.05)
                zarj.utils.wait_for_walk(self.fc.zarj)
            elif space < 0.0:
                break
            else:
                self.fc.zarj.walk.turn(0.0, snap_to=0, small_forward=space)
                zarj.utils.wait_for_walk(self.fc.zarj)

            new_distance, _ = _lidar_wall(self)
            if new_distance is None:
                raise ZarjConfused("Cannot approach wall without lidar")
            distance = new_distance

    def start(self, _=None):
        """ Start the macro """
        self.fc.zarj.walk.turn(30, snap_to=0.0)
        zarj.utils.wait_for_walk(self.fc.zarj)
        if self.stop:
            return
        self.fc.zarj.walk.turn(30, snap_to=0.0)
        zarj.utils.wait_for_walk(self.fc.zarj)
        if self.stop:
            return
        self.fc.zarj.walk.turn(30, snap_to=0.0)
        zarj.utils.wait_for_walk(self.fc.zarj)
        if self.stop:
            return
        self._approach_corner()
        if self.stop:
            return
        self.fc.zarj.walk.turn(-30, snap_to=0.0)
        zarj.utils.wait_for_walk(self.fc.zarj)
        if self.stop:
            return
        self.fc.zarj.walk.turn(-30, snap_to=0.0)
        zarj.utils.wait_for_walk(self.fc.zarj)
        if self.stop:
            return
        self.fc.zarj.walk.turn(-30, snap_to=0.0)
        zarj.utils.wait_for_walk(self.fc.zarj)
        if self.stop:
            return
        self.fc.zarj.walk.turn(-30, snap_to=0.0)
        zarj.utils.wait_for_walk(self.fc.zarj)
        if self.stop:
            return
        self.fc.zarj.walk.turn(-30, snap_to=0.0)
        zarj.utils.wait_for_walk(self.fc.zarj)
        if self.stop:
            return
        self.fc.zarj.walk.turn(-30, snap_to=0.0)
        zarj.utils.wait_for_walk(self.fc.zarj)

class perform_sweep(ZarjMacro):
    """ perform_sweep """

    def __init__(self, fc, chain_okay):
        super(perform_sweep, self).__init__(fc, chain_okay)
        self.leak = None
        self.best_leak = 0.01

    def _leak_update(self, msg):
        """ Receive updates on leak position """
        if self.leak is None or msg.value != self.leak:
            self.leak = msg.value

    def _check_for_leak(self, last_one):
        """ leaking? """
        rospy.sleep(0.2)
        if last_one and self.best_leak > 0.01:
            return True
        if self.leak > 0.01 or self.best_leak > 0.01:
            log('---> LEAK BEING DETECTED ({}) <---'.format(self.leak))
            if self.leak > self.best_leak:
                self.best_leak = self.leak
                return False
            else:
                return True
        return False

    def _front_sweep(self):
        """ _front_sweep """
        global LEAK_SIDE
        LEAK_SIDE = False
        joints = deepcopy(LimbTypes.arm_styles['front_wipe_bottom'])
        joints = LimbTypes.invert_arm_configuration('left', joints)
        self.fc.zarj.hands.set_arm_configuration('left', joints)
        if self.stop:
            return
        while joints[0] > LimbTypes.arm_styles['front_wipe_top'][0]:
            if self.stop:
                return
            if self.leak > 0.01:
                joints[0] -= 0.1
            else:
                joints[0] -= 0.2
            if self._check_for_leak(\
               joints[0] > LimbTypes.arm_styles['front_wipe_top'][0]):
                return True
            self.fc.zarj.hands.set_arm_configuration('left', joints, 0.5)
        if self.stop:
            return
        joints = deepcopy(LimbTypes.arm_styles['front_wipe_bottom'])
        joints = LimbTypes.invert_arm_configuration('left', joints)
        self.fc.zarj.hands.set_arm_configuration('left', joints)
        return False

    def _back_sweep(self):
        """ _back_sweep """
        global LEAK_SIDE
        LEAK_SIDE = True
        joints = deepcopy(LimbTypes.arm_styles['back_wipe_bottom'])
        joints = LimbTypes.invert_arm_configuration('left', joints)
        self.fc.zarj.hands.set_arm_configuration('left', joints)
        if self.stop:
            return
        done = False
        while not done:
            if self.stop:
                return
            if joints[0] < LimbTypes.arm_styles['back_wipe_top'][0]:
                if self.leak > 0.01:
                    joints[0] += 0.1
                else:
                    joints[0] += 0.2
            else:
                joints[2] -= 0.1
            if joints[0] > 2.0:
                joints[0] = 2.0
            done = joints[0] >= LimbTypes.arm_styles['back_wipe_top'][0] and\
                   joints[2] <= LimbTypes.arm_styles['back_wipe_top'][2]
            if self._check_for_leak(done):
                return True
            self.fc.zarj.hands.set_arm_configuration('left', joints, 0.5)
        joints = deepcopy(LimbTypes.arm_styles['back_wipe_bottom'])
        joints = LimbTypes.invert_arm_configuration('left', joints)
        self.fc.zarj.hands.set_arm_configuration('left', joints)
        return False

    def _step_forward(self):
        """ whee """
        step = 0.1

        joints = deepcopy(LimbTypes.arm_styles['front_wipe_bottom'])
        joints = LimbTypes.invert_arm_configuration('left', joints)
        self.fc.zarj.hands.set_arm_configuration('left', joints)
        if self.stop:
            return
        joints = deepcopy(LimbTypes.arm_styles['tuck'])
        joints = LimbTypes.invert_arm_configuration('left', joints)
        self.fc.zarj.hands.set_arm_configuration('left', joints)
        if self.stop:
            return
        self.fc.zarj.walk.turn(0.0, snap_to=0.0, small_forward=step)
        zarj.utils.wait_for_walk(self.fc.zarj)
        if self.stop:
            return
        self._check_distance()
        return step

    def _check_distance(self):
        """ Dont drift """
        # Note we with our head the way it is, and the angle
        # in the "right" position our lidar scan returns a distance of about
        # 0.75
        # the angle should be 24

        new_distance, angle = _lidar_wall(self)

        turn = round(angle, 0) - 24
        if turn != 0.0:
            log('Miss-aligned to leak wall by angle {}'.format(turn))
            self.fc.zarj.walk.turn(turn, snap_to=0.0)
            zarj.utils.wait_for_walk(self.fc.zarj)
            new_distance, angle = _lidar_wall(self)

        diff = round(new_distance, 2) - 0.75
        if diff != 0.0:
            log('Off the leak path by {}m'.format(diff))
            self.fc.zarj.walk.side(diff)
            zarj.utils.wait_for_walk(self.fc.zarj)

    def _full_cycle(self, movement):
        """ one time around """
        if not self._front_sweep():
            if self.stop:
                return
            if movement < 2.0:
                if self._back_sweep():
                    return True, 0
                if self.stop:
                    return
            movement += self._step_forward()
            return False, movement
        return True, 0

    def start(self, _=None):
        """ Start the macro """
        global LAST_LEAK
        self.leak = None
        self.best_leak = 0.01

        sub = rospy.Subscriber("/task3/checkpoint5/leak", Leak,
                               self._leak_update)
        self.fc.zarj.neck.neck_control([0.35, 1.0, 0], True)
        self._check_distance()
        found, stage = self._full_cycle(0)
        if self.stop:
            return
        while found is not None and not found:
            log('Leak not found at : {}'.format(stage))
            found, stage = self._full_cycle(stage)
            if self.stop:
                break
        if found:
            self.fc.process_get_palm_msg(ZarjGetPalmCommand('left'))
            joint_values = self.fc.zarj.hands.get_joint_values('left')
            ack = ZarjGetArmJointsResponse('left', joint_values)
            self.fc.zarj_comm.push_message(ack)
            log('leak found! stage {}'.format(stage))
            palm_transform = self.fc.zarj.hands.get_current_hand_center_transform(
                'left', 'world')
            LAST_LEAK = PointStamped()
            LAST_LEAK.header.frame_id = 'world'
            LAST_LEAK.point = palm_transform.translation
            log('Leak at {}'.format(LAST_LEAK))
            log('Arm and palm positions updated')

        sub.unregister()

class repair_position(ZarjMacro):
    """ repair_position """

    def _ensure_leak(self):
        """ If we do not have a leak detected assume our hand is in the
            right place"""

        global LAST_LEAK, LEAK_SIDE

        if LAST_LEAK is None:
            palm = self.fc.zarj.hands.get_current_hand_center_transform(
                'left', 'world')
            LAST_LEAK = PointStamped()
            LAST_LEAK.header.frame_id = 'world'
            LAST_LEAK.point = palm.translation
            log("No leak previously detected.. Assuming left hand is correct")
            log('Leak at {}'.format(LAST_LEAK))

            current = self.fc.zarj.hands.get_joint_states('left')
            LEAK_SIDE = current[0]['leftShoulderPitch'][0] > 0
            log('Leak is behind us: '.format(LEAK_SIDE))

    def _turn_into_position(self):
        """ Turn to face the right way """
        if LEAK_SIDE:
            joints = deepcopy(LimbTypes.arm_styles['front_wipe_bottom'])
            joints = LimbTypes.invert_arm_configuration('left', joints)
            self.fc.zarj.hands.set_arm_configuration('left', joints)
            if self.stop:
                return

        joints = deepcopy(LimbTypes.arm_styles['tuck'])
        joints = LimbTypes.invert_arm_configuration('left', joints)
        self.fc.zarj.hands.set_arm_configuration('left', joints)
        if self.stop:
            return

        self.fc.zarj.walk.turn(30, snap_to=0.0)
        zarj.utils.wait_for_walk(self.fc.zarj)
        if self.stop:
            return
        self.fc.zarj.walk.turn(30, snap_to=0.0)
        zarj.utils.wait_for_walk(self.fc.zarj)
        if self.stop:
            return
        self.fc.zarj.walk.turn(30, snap_to=0.0)
        zarj.utils.wait_for_walk(self.fc.zarj)
        if self.stop:
            return
        if LEAK_SIDE:
            self.fc.zarj.walk.turn(30, snap_to=0.0)
            zarj.utils.wait_for_walk(self.fc.zarj)
            if self.stop:
                return
            self.fc.zarj.walk.turn(30, snap_to=0.0)
            zarj.utils.wait_for_walk(self.fc.zarj)
            if self.stop:
                return
            self.fc.zarj.walk.turn(30, snap_to=0.0)
            zarj.utils.wait_for_walk(self.fc.zarj)
            if self.stop:
                return

    def start(self, _=None):
        """ Start the macro """
        self._ensure_leak()
        self._turn_into_position()
        leak = self.fc.zarj.transform.tf_buffer.transform(
            LAST_LEAK, self.fc.zarj.walk.lfname)
        if LEAK_SIDE:
            walk = 0.55 - leak.point.x
            offset = -0.42 - leak.point.y
        else:
            walk = 0.42 - leak.point.x
            offset = -0.91 - leak.point.y

        if abs(round(walk, 2)) > 0:
            self.fc.zarj.walk.forward(walk)
            zarj.utils.wait_for_walk(self.fc.zarj)

        if self.stop:
            return

        if abs(round(offset, 2)) > 0:
            self.fc.zarj.walk.side(offset)
            zarj.utils.wait_for_walk(self.fc.zarj)

        if self.stop:
            return

        if LEAK_SIDE:
            joints = deepcopy(LimbTypes.arm_styles['repair_start_side'])
            joints = LimbTypes.invert_arm_configuration('right', joints)
            self.fc.zarj.hands.set_arm_configuration('right', joints,
                                                     movetime=2)
        else:
            joints = deepcopy(LimbTypes.arm_styles['front_wipe_bottom'])
            joints = LimbTypes.invert_arm_configuration('right', joints)
            self.fc.zarj.hands.set_arm_configuration('right', joints,
                                                     movetime=2)
            if self.stop:
                return
            joints = deepcopy(LimbTypes.arm_styles['repair_prep'])
            joints = LimbTypes.invert_arm_configuration('right', joints)
            self.fc.zarj.hands.set_arm_configuration('right', joints,
                                                     movetime=2)
            if self.stop:
                return
            joints = deepcopy(LimbTypes.arm_styles['repair_start'])
            joints = LimbTypes.invert_arm_configuration('right', joints)
            self.fc.zarj.hands.set_arm_configuration('right', joints,
                                                     movetime=2)

        if self.stop:
            return

        if not LEAK_SIDE:
            log("Giving time for the hand to settle")
            rospy.sleep(1.0)
            if self.stop:
                return

        if LEAK_SIDE:
            self.fc.zarj.neck.neck_control([0.0, -1.0, 0], True)
        else:
            self.fc.zarj.neck.neck_control([0.0, 0.0, 0], True)

        palm = self.fc.zarj.hands.get_current_hand_center_transform('right')
        point_vector = Vector3(LAST_LEAK.point.x, LAST_LEAK.point.y,
                               LAST_LEAK.point.z)
        self.fc.zarj.hands.move_hand_center_abs('right', point_vector,
                                                palm.rotation)

        distance, angle = _lidar_wall(self)
        log("Wall is {} away at {} angle".format(distance, angle))

class repair_pepper_wall(ZarjMacro):
    """ repair_pepper_wall """

    def _do_repair(self, a, b):
        """ do repair """
        if LEAK_SIDE:
            point_vector = Vector3(a, 0, b)
        else:
            point_vector = Vector3(0, a, b)
        print 'Move by', point_vector
        self.fc.zarj.hands.move_hand_center_rel('right', point_vector,
                                                move_time=0.5)

        # TODO: What are we going to do to ensure we are touching the wall?

        rospy.sleep(2.0)
        return False

    def start(self, _=None):
        """ Start the macro """
        cycle_stop = 12
        resolution = 0.02

        delta_x = -1
        delta_z = -1
        pos_x = 0
        pos_z = 0

        cycle = 1
        self._do_repair(0, 0)
        while cycle <= cycle_stop:
            if self.stop:
                return
            for _ in range(cycle):
                if self.stop:
                    return
                pos = resolution * delta_z
                pos_z += pos
                log("trying {},{}".format(pos_x, pos_z))
                self._do_repair(0, pos)
                if self.fc.checkpoint == 8:
                    log("Leak Repaired!")
                    return
            if self.stop:
                return
            for _ in range(cycle):
                if self.stop:
                    return
                pos = resolution * delta_x
                pos_x += pos
                log("trying {},{}".format(pos_x, pos_z))
                self._do_repair(pos, 0)
                if self.fc.checkpoint == 8:
                    log("Leak Repaired!")
                    return
            cycle += 1
            delta_z = 0 - delta_z
            delta_x = 0 - delta_x

class Task3(ZarjTask):
    """ This is the class that drives task 3 """

    def __init__(self, fc):
        ZarjTask.__init__(self, fc, 3,
            [CheckpointC1(self), CheckpointC2(self), CheckpointC3(self),
             CheckpointC4(self), CheckpointC5(self), CheckpointC6(self),
             CheckpointC7(self), CheckpointC8(self)])
