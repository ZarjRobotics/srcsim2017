"""
    Team ZARJ task 1 code
"""
import os
import rospy
import cv2
import copy
import numpy as np

from zarjtask import ZarjTask
from zarjmacro import ZarjMacro
from zarj.utils import log
from zarj.confused import ZarjConfused
import zarj
from zarjcomm.limbtypes import LimbTypes
from srcsim.msg import Satellite

from checkpointa1 import CheckpointA1
from checkpointa2 import CheckpointA2
from checkpointa3 import CheckpointA3
from checkpointa4 import CheckpointA4

from geometry_msgs.msg import PointStamped

#blue wheel!
LOW_BLUE_SAT = np.array([110, 120, 50])
HIGH_BLUE_SAT = np.array([130, 255, 130])

#red wheel!
LOW_RED_SAT = np.array([0, 180, 80])
HIGH_RED_SAT = np.array([10, 255, 130])

STD_FACTOR = 10

APPROACH_DISTANCE = 0.43
OFFSET_BLUE = 0.06

# Points beyond 5m away are clearly NOT the dish, it is never that far away
DISH_MAXIMUM = 5.0

# if pitch and yaw move less than WHEEL_MOVEMENT_THRESHOLD we log a message
WHEEL_MOVEMENT_THRESHOLD = .02
THRESHOLD_FAIL_COUNT = 10

def _add_bounding_point(point, corners):
    """ expanding the bounding box with the point """
    if corners[0] is None:
        corners[0] = [point[0], point[1], point[2]]
        corners[1] = [point[0], point[1], point[2]]
        return

    if point[0] < corners[0][0]:
        corners[0][0] = point[0]
    if point[0] > corners[1][0]:
        corners[1][0] = point[0]

    if point[1] < corners[0][1]:
        corners[0][1] = point[1]
    if point[1] > corners[1][1]:
        corners[1][1] = point[1]

    if point[2] < corners[0][2]:
        corners[0][2] = point[2]
    if point[2] > corners[1][2]:
        corners[1][2] = point[2]

def _expand_box(corner1, corner2, point):
    """ expanding the bounding box with the point """
    if point.point.x < corner1.point.x:
        corner1.point.x = point.point.x
    if point.point.x > corner2.point.x:
        corner2.point.x = point.point.x

    if point.point.y < corner1.point.y:
        corner1.point.y = point.point.y
    if point.point.y > corner2.point.y:
        corner2.point.y = point.point.y

    if point.point.z < corner1.point.z:
        corner1.point.z = point.point.z
    if point.point.z > corner2.point.z:
        corner2.point.z = point.point.z

class Task1Macro(ZarjMacro):
    """ Common code for task1 macros """

    def _find_control(self, tag, mask, details, header, factor, other_one=None):
        """ Find a control """

        corners = [None, None]
        indexes = mask.nonzero()
        points = []
        indexes2 = []
        for i in range(len(indexes[1])):
            pnt = details[indexes[0][i], indexes[1][i]]
            if pnt[2] > DISH_MAXIMUM:
                print 'rejecting point, too distant ', pnt[2]
                mask[indexes[0][i], indexes[1][i]] = 0
                continue
            if other_one:
                dist = np.sqrt((other_one[0].point.y - pnt[0])**2 + \
                               (other_one[0].point.z - pnt[1])**2 +\
                               (other_one[0].point.x - pnt[2])**2)
                if dist > 1.0:
                    print 'rejecting point, too far away from other one', dist
                    mask[indexes[0][i], indexes[1][i]] = 0
                    continue
            points.append(details[indexes[0][i], indexes[1][i]])
            indexes2.append((indexes[0][i], indexes[1][i]))
        points = np.array(points)

        if len(points) < 3:
            log("Error: only {} {} points found.".format(len(points), tag))
            raise ZarjConfused('Error: only {} {} points found.'.format(
                len(points), tag))

        mean = np.mean(points, axis=0)[2]
        stdv = np.std(points, axis=0)[2]

        for i, pnt in enumerate(points):
            if abs(pnt[2] - mean) < factor * stdv:
                _add_bounding_point(pnt, corners)
            else:
                mask[indexes[0][i], indexes[1][i]] = 0

        point = PointStamped()
        point.header = header
        point.point.x = corners[0][0]
        point.point.y = corners[0][1]
        point.point.z = corners[0][2]
        cor1 = self.fc.zarj.transform.tf_buffer.transform(point, 'pelvis')

        point = PointStamped()
        point.header = header
        point.point.x = corners[1][0]
        point.point.y = corners[1][1]
        point.point.z = corners[1][2]
        cor2 = self.fc.zarj.transform.tf_buffer.transform(point, 'pelvis')

        log("{}, {} {}".format(tag, cor1, cor2))

        return (cor1, cor2)

    def find_satellite(self, factor=STD_FACTOR, check_close=False):
        """ Try to guess where the satellite dish is """
        cloud = self.fc.zarj.eyes.get_stereo_cloud()
        colors, details = self.fc.zarj.eyes.get_cloud_image_with_details(cloud)
        colors = cv2.cvtColor(colors, cv2.COLOR_BGR2HSV)

        image_dump = os.environ.get("ZARJ_IMAGE_DUMP")
        if image_dump is not None:
            image_idx = rospy.get_time()

        mask = cv2.inRange(colors, LOW_RED_SAT, HIGH_RED_SAT)

        if image_dump is not None:
            cv2.imwrite(image_dump+'/satellite_red_{}.png'.format(image_idx),
                        mask)

        red = self._find_control('red', mask, details, cloud.header, factor)

        if image_dump is not None:
            cv2.imwrite(image_dump+'/satellite_red_filtered_{}.png'.format(
                image_idx), mask)

        mask = cv2.inRange(colors, LOW_BLUE_SAT, HIGH_BLUE_SAT)

        if image_dump is not None:
            image_idx = rospy.get_time()
            cv2.imwrite(image_dump+'/satellite_blue_{}.png'.format(image_idx),
                        mask)

        if check_close:
            blue = self._find_control('blue', mask, details, cloud.header,
                                      factor, red)
        else:
            blue = self._find_control('blue', mask, details, cloud.header,
                                      factor)

        if image_dump is not None:
            image_idx = rospy.get_time()
            cv2.imwrite(image_dump+'/satellite_blue_filtered_{}.png'.format(
                image_idx), mask)


        cor1 = PointStamped()
        cor1.header = red[0].header
        cor1.point.x = red[0].point.x
        cor1.point.y = red[0].point.y
        cor1.point.z = red[0].point.z

        cor2 = PointStamped()
        cor2.header = red[0].header
        cor2.point.x = red[0].point.x
        cor2.point.y = red[0].point.y
        cor2.point.z = red[0].point.z

        _expand_box(cor1, cor2, red[1])
        _expand_box(cor1, cor2, blue[0])
        _expand_box(cor1, cor2, blue[1])

        log('I think the satellite controls are at {} {}'.format(cor1, cor2))

        return red, blue, (cor1, cor2)

    def claw_hand(self, side):
        """ claw hands """
        joints = copy.deepcopy(LimbTypes.hand_styles['claw_no_thumb'])
        joints = LimbTypes.invert_hand_configuration(side, joints)
        self.fc.zarj.hands.set_hand_configuration(side, joints)

    def claw_hand2(self, side):
        """ claw hands """
        joints = copy.deepcopy(LimbTypes.hand_styles['claw'])
        joints = LimbTypes.invert_hand_configuration(side, joints)
        self.fc.zarj.hands.set_hand_configuration(side, joints)

    def grab_hand(self, side):
        """ Grab """
        joints = copy.deepcopy(LimbTypes.hand_styles['grab'])
        joints = LimbTypes.invert_hand_configuration(side, joints)
        self.fc.zarj.hands.set_hand_configuration(side, joints)

    def arm_out(self, side):
        """ arm out """
        joints = copy.deepcopy(LimbTypes.arm_styles['control_out'])
        joints = LimbTypes.invert_arm_configuration(side, joints)
        self.fc.zarj.hands.set_arm_configuration(side, joints, 0.5)
        self.claw_hand(side)

    def side_pre(self, side):
        """ Just outside the side of the wheel """
        joints = copy.deepcopy(LimbTypes.arm_styles['control_preside_grab'])
        joints = LimbTypes.invert_arm_configuration(side, joints)
        self.fc.zarj.hands.set_arm_configuration(side, joints, 0.5)

    def side_final(self, side):
        """ Final side control position """
        joints = copy.deepcopy(LimbTypes.arm_styles['control_side_grab'])
        joints = LimbTypes.invert_arm_configuration(side, joints)
        self.fc.zarj.hands.set_arm_configuration(side, joints)

    def grab_side(self, side):
        """ Grab the wheel side """
        self.side_pre(side)
        self.side_final(side)
        self.claw_hand2(side)
        self.grab_hand(side)

    def release_side(self, side):
        """ Release the side of the control wheel """
        self.claw_hand2(side)
        self.side_pre(side)

    def top_pre(self, side):
        """ Just outside the top of the wheel """
        joints = copy.deepcopy(LimbTypes.arm_styles['control_pretop_grab'])
        joints = LimbTypes.invert_arm_configuration(side, joints)
        self.fc.zarj.hands.set_arm_configuration(side, joints, 0.5)

    def top_final(self, side):
        """ Final top control position """
        joints = copy.deepcopy(LimbTypes.arm_styles['control_top_grab'])
        joints = LimbTypes.invert_arm_configuration(side, joints)
        self.fc.zarj.hands.set_arm_configuration(side, joints)

    def grab_top(self, side):
        """ Grab the wheel top"""
        self.top_pre(side)
        self.top_final(side)
        self.claw_hand2(side)
        self.grab_hand(side)

    def arms_home(self):
        """ return arm to the home position """
        self.fc.zarj.go_home(0, 0)
        rospy.sleep(0.1)
        self.fc.zarj.go_home(1, 0)

    def release_top(self, side):
        """ Release the top of the control wheel """
        self.claw_hand2(side)
        self.top_pre(side)

    def rotate_to_top(self, side):
        """ rotate to the top """
        self.grab_side(side)
        self.top_final(side)
        self.release_top(side)
        self.arm_out(side)

    def rotate_to_side(self, side):
        """ rotate to the side"""
        self.grab_top(side)
        self.side_final(side)
        self.release_side(side)
        self.arm_out(side)


class find_satellite(Task1Macro):
    """ Find the Satellite Dish """

    def __init__(self, fc, chain_okay):
        super(find_satellite, self).__init__(fc, chain_okay)
        self.sat_location = None

    def _locate_satellite(self, retry=False):
        """ Locate the satellite dish """
        red, blue, final = self.find_satellite(2, True)

        retries = 5
        while (red is None or blue is None) and retries > 0:
            rospy.sleep(0.5)
            red, blue, final = self.find_satellite(2, True)
            retries -= 1

        if red is None or blue is None:
            raise ZarjConfused('Could not find both red and blue wheels')

        side = 0
        if final[0].point.y < 0:
            log('controls are on the right')
            side = -1
        else:
            log('controls are on the left')
            side = 1

        blue_center_y = (blue[0].point.y + blue[1].point.y) / 2
        blue_center_x = (blue[0].point.x + blue[1].point.x) / 2
        red_center_y = (red[0].point.y + red[1].point.y) / 2
        red_center_x = (red[0].point.x + red[1].point.x) / 2
        y_offset = round(abs(blue_center_y - red_center_y), 3)
        x_offset = round(abs(blue_center_x - red_center_x), 3)
        if y_offset == 0.0:
            y_offset = 0.001
        if x_offset == 0.0:
            x_offset = 0.001
        ratio = x_offset/y_offset
        log('y_offset = {}'.format(y_offset))
        log('x_offset = {}'.format(x_offset))
        log('distance = {}'.format(x_offset + y_offset))
        log('ratio = {}'.format(ratio))

        if x_offset + y_offset < 0.2:
            if not retry:
                log('Controls are not in a logical position, too close... '
                    'Retry')
                return self._locate_satellite(True)
            else:
                raise ZarjConfused(
                    'Controls are not in a logical position, too close')

        # This is not really rocket science:
        # 45 should have ratios around 1
        # 90 has ratios above above 3 even as high as 400
        # ahead would have ratios approaching 0
        if np.isclose([x_offset], [y_offset])[0] or \
           (ratio >= 0.6 and ratio <= 3.0):
            log('controls are 45 degrees')
            turn = 45
            correction = -0.7
            second = 0.3
        elif round((blue_center_y + red_center_y)/2, 2) == 0.0:
            log('controls are directly ahead')
            turn = 0
            correction = -0.5
            second = 0.0
        else:
            log('controls are 90 degrees')
            turn = 90
            correction = 0.0
            second = 0.3

        final_center_x = (final[0].point.x + final[1].point.x) / 2

        log("Satellite Plan: distance {}, turn {}, approach {}".format(
            final_center_x + correction, turn*side, second))
        return final_center_x + correction, turn*side, second

    def start(self, _=None):
        """ Start the macro """
        if self.fc.task == 0 and self.chain_okay:
            raise ZarjConfused("OPERATOR ERROR: You asked to start satellite "
                               "stuff but we are still task 0, so srcsim has "
                               "not been started! I dont think so!")
        self.fc.zarj.zps.look_forward()
        self.sat_location = self._locate_satellite(False)
        self.done = True

    @property
    def next_chain(self):
        """ returns the name of the next macro in the chain """
        return 'walk_to_satellite'

class walk_to_satellite(Task1Macro):
    """ Walk to the Satellite Dish """

    def __init__(self, fc, chain_okay):
        super(walk_to_satellite, self).__init__(fc, chain_okay)
        self.sat_location = None

    def start(self, previous=None):
        """ Start the macro """
        if previous and hasattr(previous, 'sat_location'):
            self.sat_location = previous.sat_location
        else:
            raise ZarjConfused('No satallite location known')
        self.fc.zarj.zps.set_poi((self.sat_location[0], self.sat_location[1]))
        # walk to dish
        self.fc.zarj.zps.walk_to_poi()
        if self.stop:
            return
        self.fc.zarj.zps.mark_waypoint()
        if self.stop:
            return
        # second walk if needed
        if self.sat_location[2] > 0:
            self.fc.zarj.walk.forward(self.sat_location[2])
            zarj.utils.wait_for_walk(self.fc.zarj)
        self.done = True

    @property
    def next_chain(self):
        """ returns the name of the next macro in the chain """
        return 'look_closer'

class look_closer(Task1Macro):
    """look_closer"""

    def __init__(self, fc, chain_okay):
        super(look_closer, self).__init__(fc, chain_okay)
        self.sat_location = None

    def _prepare_to_turn(self):
        """ _prepare_to_turn """
        self.fc.zarj.hands.HAND_MOVE_TIME = 0.5

        # Arms Out
        self.arm_out('right')
        self.arm_out('left')

    def start(self, _=None):
        """ Start the macro """
        self._prepare_to_turn()

        self.fc.zarj.zps.look_down()

        #have the look_down settle
        rospy.sleep(1)

        log('closer look')
        _, blue, _ = self.find_satellite()

        retries = 5
        while blue is None and retries > 0:
            rospy.sleep(0.5)
            _, blue, _ = self.find_satellite()
            retries -= 1

        if blue is None:
            raise ZarjConfused('Unable to find blue control!')

        y_center = blue[1].point.y + OFFSET_BLUE
        x_front = blue[0].point.x

        log('satellite {} over, {} ahead'.format(y_center, x_front))

        self.sat_location = (y_center, x_front)
        self.done = True

    @property
    def next_chain(self):
        """ returns the name of the next macro in the chain """
        return 'move_closer'

class move_closer(Task1Macro):
    """move_closer"""

    def __init__(self, fc, chain_okay):
        super(move_closer, self).__init__(fc, chain_okay)
        self.sat_location = None

    def _lean_in(self):
        """ Lean in """
        self.fc.zarj.neck.head_down(False)
        self.fc.zarj.pelvis.lean_body_to(-25)

    def start(self, previous=None):
        """ Start the macro """
        if previous and hasattr(previous, 'sat_location'):
            self.sat_location = previous.sat_location
        else:
            raise ZarjConfused('No satallite location known')

        log("====> Satellite Movement Plan:\n\t{}m side\n\t {}m ahead".format(
            self.sat_location[0], self.sat_location[1] - APPROACH_DISTANCE))

        if self.sat_location[1] - APPROACH_DISTANCE > 0.25:
            raise(ZarjConfused("OPERATOR! headed off the path trying to approach the satellite  Not right!"))

        if abs(self.sat_location[0]) > 0.01:
            self.fc.zarj.walk.side(self.sat_location[0])
            zarj.utils.wait_for_walk(self.fc.zarj)
        if self.stop:
            return
        if abs(self.sat_location[1] - APPROACH_DISTANCE) > 0.01:
            self.fc.zarj.walk.forward(self.sat_location[1] - APPROACH_DISTANCE)
            zarj.utils.wait_for_walk(self.fc.zarj)
        if self.stop:
            return

        self._lean_in()
        self.done = True

    @property
    def next_chain(self):
        """ returns the name of the next macro in the chain """
        return 'correct_alignment'

class correct_alignment(Task1Macro):
    """correct_alignment"""

    def __init__(self, fc, chain_okay):
        super(correct_alignment, self).__init__(fc, chain_okay)
        self.last_msg = None
        self.satellite = None
        self.last_pitch = 0
        self.last_yaw = 0
        self.pitch_fail_count = 0
        self.yaw_fail_count = 0

    def _correct_alignment(self, msg):
        """ Correct satellite alignment """
        if not msg.pitch_correct_now:
            log("Correcting pitch: target pitch {} current pitch {} ".format(msg.target_pitch, msg.current_pitch))
            if msg.current_pitch - msg.target_pitch > 0:
                self.rotate_to_top('right')
                movement = abs((msg.current_pitch - self.last_pitch))
                if movement < WHEEL_MOVEMENT_THRESHOLD and self.last_pitch != 0:
                    if movement == 0:
                        log("Wheel did not move at all!")
                        self.pitch_fail_count += 5
                    else:
                        log("Wheel only moved {}, retry {} of {}!".format(
                            movement, self.pitch_fail_count,
                            THRESHOLD_FAIL_COUNT))
                        self.pitch_fail_count += 1
                    if self.pitch_fail_count > THRESHOLD_FAIL_COUNT:
                        # Not moving wheel, fix it!
                        log("Wheel not moving; last pitch {} current pitch {} "
                            "movement {}".format(self.last_pitch,
                                                 msg.current_pitch, movement))
                        raise ZarjConfused('Wheel not moving')
                else:
                    self.pitch_fail_count = 0
            else:
                self.rotate_to_side('right')
                movement = abs((self.last_pitch - msg.current_pitch))
                if movement < WHEEL_MOVEMENT_THRESHOLD and self.last_pitch != 0:
                    if movement == 0:
                        log("Wheel did not move at all!")
                        self.pitch_fail_count += 5
                    else:
                        log("Wheel only moved {}, retry {} of {}!".format(
                            movement, self.pitch_fail_count,
                            THRESHOLD_FAIL_COUNT))
                        self.pitch_fail_count += 1
                    if self.pitch_fail_count > THRESHOLD_FAIL_COUNT:
                        # Not moving wheel, fix it!
                        log("Wheel not moving; last pitch {} current pitch {} "
                            "movement {}".format(self.last_pitch,
                                                 msg.current_pitch, movement))
                        raise ZarjConfused('Wheel not moving')
                else:
                    self.pitch_fail_count = 0
            self.last_pitch = msg.current_pitch

        if not msg.yaw_correct_now:
            log("Correcting yaw: target yaw {} current yaw {} ".format(msg.target_yaw, msg.current_yaw))
            if msg.current_yaw - msg.target_yaw < 0:
                self.rotate_to_top('left')
                movement = abs((self.last_yaw - msg.current_yaw))
                if movement < WHEEL_MOVEMENT_THRESHOLD and self.last_yaw != 0:
                    if movement == 0:
                        log("Wheel did not move at all!")
                        self.yaw_fail_count += 5
                    else:
                        log("Wheel only moved {}, retry {} of {}!".format(
                            movement, self.yaw_fail_count,
                            THRESHOLD_FAIL_COUNT))
                        self.yaw_fail_count += 1
                    if self.yaw_fail_count > THRESHOLD_FAIL_COUNT:
                        # Not moving wheel, fix it!
                        log("Wheel not moving; last yaw {} current yaw {} "
                            "movement {}".format(self.last_yaw, msg.current_yaw,
                                                 movement))
                        raise ZarjConfused('Wheel not moving')
                else:
                    self.yaw_fail_count = 0
            else:
                self.rotate_to_side('left')
                movement = abs((self.last_yaw - msg.current_yaw))
                if movement < WHEEL_MOVEMENT_THRESHOLD and self.last_yaw != 0:
                    if movement == 0:
                        log("Wheel did not move at all!")
                        self.yaw_fail_count += 5
                    else:
                        log("Wheel only moved {}, retry {} of {}!".format(
                            movement, self.yaw_fail_count,
                            THRESHOLD_FAIL_COUNT))
                        self.yaw_fail_count += 1
                    if self.yaw_fail_count > THRESHOLD_FAIL_COUNT:
                        # Not moving wheel, fix it!
                        log("Wheel not moving; last yaw {} current yaw {} "
                            "movement {}".format(self.last_yaw, msg.current_yaw,
                                                 movement))
                        raise ZarjConfused('Wheel not moving')
                else:
                    self.yaw_fail_count = 0
            self.last_yaw = msg.current_yaw

    def _satellite_update(self, msg):
        """ Receive updates on satellite position """
        self.last_msg = msg

    def start(self, _=None):
        """ Start the macro """
        sub = rospy.Subscriber("/task1/checkpoint2/satellite",
                               Satellite, self._satellite_update)

        while not self.done and not self.stop:
            if self.last_msg is not None:
                if self.last_msg.pitch_completed and \
                   self.last_msg.yaw_completed:
                    break
                self._correct_alignment(self.last_msg)
            rospy.sleep(0.01)

        sub.unregister()
        self.done = True

    @property
    def next_chain(self):
        """ returns the name of the next macro in the chain """
        if len(self.fc.zarj.zps.waypoint_stack) == 0:
            raise ZarjConfused("OPERATOR: We cannot chain")
        return 'task1_finish_box'

class task1_finish_box(Task1Macro):
    """task1_finish_box"""

    def start(self, _=None):
        """ Start the macro """
        self.fc.zarj.zps.look_forward()
        if self.stop:
            return
        if self.fc.zarj.zps.state & zarj.zps.STATE_PRESTART:
            self.fc.zarj.zps.path_start()
        else:
            self.fc.zarj.zps.return_to_waypoint()
            if self.stop:
                return
            self.arms_home()
            self.fc.zarj.zps.continue_from_poi()
        if self.stop:
            return

        self.fc.zarj.zps.walk_to_next_finish_box()
        if self.stop:
            return
        self.fc.zarj.zps.look_for_exit()
        log("I am out of the Task1 Wilderness")

        self.done = True

class Task1(ZarjTask):
    """ This is the class that drives task 1 """

    def __init__(self, fc):
        ZarjTask.__init__(self, fc, 1,
            [CheckpointA1(self), CheckpointA2(self), CheckpointA3(self),
             CheckpointA4(self)])
