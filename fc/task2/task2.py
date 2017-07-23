"""
    Team ZARJ task 2 code
"""
import math
import rospy

from copy import deepcopy

from zarjtask import ZarjTask
from zarjmacro import ZarjMacro
from zarj.confused import ZarjConfused
from zarjcomm.limbtypes import LimbTypes
from zarj.utils import log
from zarjcomm.message import ZarjMovePalmCommand
from zarj.things import Things
from zarjcomm.anchor import Anchor
import zarj
from geometry_msgs.msg import PointStamped

from checkpointb1 import CheckpointB1
from checkpointb2 import CheckpointB2
from checkpointb3 import CheckpointB3
from checkpointb4 import CheckpointB4
from checkpointb5 import CheckpointB5
from checkpointb6 import CheckpointB6


def find_choke(macro):
    macro.fc.send_stereo_camera()
    image, details = macro.fc.zarj.eyes.get_cloud_image_with_details(macro.fc.cloud)
    things = Things(image, details, 2)
    if things.choke_inner:
        point = PointStamped()
        point.header = macro.fc.cloud.header
        det = details[things.choke_inner.position[1], things.choke_inner.position[0]]
        point.point.x = det[0]
        point.point.y = det[1]
        point.point.z = det[2]
        inner = macro.fc.zarj.transform.tf_buffer.transform(point, macro.fc.zarj.walk.lfname)
        log("inner choke found at: {}/{}/{}".format(inner.point.x, inner.point.y, inner.point.z))

        if things.choke_outer:
            point = PointStamped()
            point.header = macro.fc.cloud.header
            det = details[things.choke_outer.position[1], things.choke_outer.position[0]]
            point.point.x = det[0]
            point.point.y = det[1]
            point.point.z = det[2]
            outer = macro.fc.zarj.transform.tf_buffer.transform(point, macro.fc.zarj.walk.lfname)
            log("outer choke found at: {}/{}/{}".format(outer.point.x, outer.point.y, outer.point.z))

            macro.fc.points[0] = [outer.point.x, outer.point.y, outer.point.z]
            macro.fc.points[1] = [inner.point.x, inner.point.y, inner.point.z]

            return True

    return False

def find_button(macro):
    macro.fc.send_stereo_camera()
    image, details = macro.fc.zarj.eyes.get_cloud_image_with_details(macro.fc.cloud)
    things = Things(image, details, 2)
    if things.array_button is not None:
        point = PointStamped()
        point.header = macro.fc.cloud.header
        det = things.array_button.computed_center
        point.point.x = det[0]
        point.point.y = det[1]
        point.point.z = det[2]
        button = macro.fc.zarj.transform.tf_buffer.transform(point, macro.fc.zarj.walk.lfname)
        log("button found at: {}/{}/{}".format(button.point.x, button.point.y, button.point.z))

        macro.fc.points[0] = [ button.point.x, button.point.y, button.point.z ]
        macro.fc.points[1] = None

        return True

    return False

def find_plug(macro):
    macro.fc.send_stereo_camera()
    image, details = macro.fc.zarj.eyes.get_cloud_image_with_details(macro.fc.cloud)
    things = Things(image, details, 2)
    if things.power_plug:
        point = PointStamped()
        point.header = macro.fc.cloud.header
        det = details[things.power_plug.position[1], things.power_plug.position[0]]
        point.point.x = det[0]
        point.point.y = det[1]
        point.point.z = det[2]
        plug = macro.fc.zarj.transform.tf_buffer.transform(point, macro.fc.zarj.walk.lfname)
        log("plug at: {}/{}/{}".format(plug.point.x, plug.point.y, plug.point.z))
        macro.fc.points[0] = [plug.point.x, plug.point.y, plug.point.z]
        macro.fc.points[1] = None

        return True

    return False

class get_out_of_startbox(ZarjMacro):
    """ pickup_array_prep """

    def start(self, _=None):
        """ Start the macro """
        log("Starting the path - should get us to the exit")
        self.fc.zarj.zps.path_start()

        log("Walking out of the finish box")
        self.fc.zarj.zps.dead_walk([0.6, 0.0, 0.0])


class pickup_array_prep(ZarjMacro):
    """ pickup_array_prep """

    def start(self, _=None):
        """ Start the macro """
        joints = deepcopy(LimbTypes.arm_styles['pass_football'])
        joints = LimbTypes.invert_arm_configuration('right', joints)
        log("Setting right hand to pass a football")
        self.fc.zarj.hands.set_arm_configuration('right', joints)

        joints = deepcopy(LimbTypes.arm_styles['tuck'])
        joints = LimbTypes.invert_arm_configuration('left', joints)
        log("Tucking left arm, setting left hand to opposition")
        self.fc.zarj.hands.set_arm_configuration('left', joints)

        joints = deepcopy(LimbTypes.hand_styles['oppose'])
        joints = LimbTypes.invert_hand_configuration('left', joints)
        self.fc.zarj.hands.set_hand_configuration('left', joints)

        log("Looking down")
        self.fc.zarj.neck.neck_control([0.5, 0, 0], True)

        rospy.sleep(0.5)

        self.fc.send_stereo_camera()
        log("Pick the two top corners of the array handle.")
        self.done = True


class pickup_array_move_feet(ZarjMacro):
    """pickup_array_move_feet"""

    def start(self, _=None):
        """ Start the macro """
        anchor = self.fc.assure_anchor("center")
        if anchor is None:
            raise ZarjConfused("Cannot find the array")

        log("Array is {} away, {} to the side, at angle {}".format(anchor.adjusted[0], anchor.adjusted[1], anchor.angle))

        forward = anchor.adjusted[0] - 0.71
        if anchor.angle > 0:
            self.fc.last_trailer_move = .17
        else:
            self.fc.last_trailer_move = -.17
        sideways = anchor.adjusted[1] + self.fc.last_trailer_move

        log("Walking sideways {}, forward {}".format(sideways, forward))
        log("TODO - maybe this should not be a deadwalk")
        self.fc.zarj.zps.dead_walk([forward, 0.0, sideways])

        log("Lean down -30, turn -40, look left")
        self.fc.zarj.pelvis.lean_body_to(-30)
        self.fc.zarj.neck.neck_control([ 0.5, 0.75, 0 ], True)
        self.fc.zarj.pelvis.turn_body_to(-40)

        if anchor.angle > 0:
            joints = deepcopy(LimbTypes.arm_styles['tuck_out'])
            joints = LimbTypes.invert_arm_configuration('left', joints)
            log("Swinging left arm out, to make the move easier.")
            self.fc.zarj.hands.set_arm_configuration('left', joints)

        rospy.sleep(0.5)

        if not self.chain_okay:
            self.fc.send_stereo_camera()
            log("Pick the two top corners of the array handle.")

        self.done = True

    @property
    def next_chain(self):
        """ returns the name of the next macro in the chain """
        return 'pickup_array_move_hand_start'


class pickup_array_move_hand_start(ZarjMacro):
    """pickup_array_move_hand_start"""

    def start(self, _=None):
        """ Start the macro """
        anchor = self.fc.assure_anchor("center")
        if anchor is None:
            raise ZarjConfused("Cannot find the array")

        x = anchor.adjusted[0] - (.1 * math.cos(math.radians(anchor.angle)))
        y = anchor.adjusted[1] + (.1 * math.sin(math.radians(anchor.angle)))
        z = anchor.adjusted[2] - 0.045

        log("Commanding hand to position just shy of handle")
        msg = ZarjMovePalmCommand('left', False, x, y, z, -1 * anchor.angle, 0, -90, True)
        self.fc.process_palm_msg(msg)
        self.done = True

    @property
    def next_chain(self):
        """ returns the name of the next macro in the chain """
        return 'pickup_array_move_hand_under'


class pickup_array_move_hand_under(ZarjMacro):
    """pickup_array_move_hand_under"""

    def start(self, _=None):
        """ Start the macro """
        anchor = self.fc.assure_anchor("center")
        if anchor is None:
            raise ZarjConfused("Cannot find the array")

        x = anchor.adjusted[0] + (.06 * math.cos(math.radians(anchor.angle)))
        y = anchor.adjusted[1] - (.06 * math.sin(math.radians(anchor.angle)))
        z = anchor.adjusted[2] - 0.045

        log("Commanding hand to position under handle")
        msg = ZarjMovePalmCommand('left', False, x, y, z, -1 * anchor.angle, 0, -90, True)
        self.fc.process_palm_msg(msg)

    @property
    def next_chain(self):
        """ returns the name of the next macro in the chain """
        return 'pickup_array_grab'


class pickup_array_grab(ZarjMacro):
    """pickup_array_grab"""

    def start(self, _=None):
        """ Start the macro """
        log("Commanding hand to grab array")

        joints = deepcopy(LimbTypes.hand_styles['grab'])
        joints = LimbTypes.invert_hand_configuration('left', joints)
        self.fc.zarj.hands.set_hand_configuration('left', joints)

        rospy.sleep(0.5)  # A little extra time to let us get a good grip
        self.done = True

    @property
    def next_chain(self):
        """ returns the name of the next macro in the chain """
        return 'pickup_array_lift'

class pickup_array_lift(ZarjMacro):
    """pickup_array_lift"""

    def start(self, _=None):
        """ Start the macro """
        log("Lifting sequence; unlean, unturn, move left arm")

        self.fc.zarj.pelvis.lean_body_to(0)
        self.fc.zarj.pelvis.turn_body_to(0)
        self.fc.zarj.neck.neck_control([ 0.5, 0.0, 0 ], True)

        joints = deepcopy(LimbTypes.arm_styles['carry_array'])
        joints = LimbTypes.invert_arm_configuration('left', joints)
        self.fc.zarj.hands.set_arm_configuration('left', joints)

        joints = deepcopy(LimbTypes.arm_styles['carry_array_middle'])
        joints = LimbTypes.invert_arm_configuration('left', joints)
        self.fc.zarj.hands.set_arm_configuration('left', joints)
        self.done = True

        #TODO - before you next this one, throw a ZarjConfused if the
        #       checkpoint did not increment (and it's not after a spin_array)

class spin_array(ZarjMacro):
    """ Prepare to spin the array; assumes we are standing at the trailer, holding it """

    def start(self, _=None):
        """ Start the macro """

        joints = deepcopy(LimbTypes.arm_styles['carry_array_higher'])
        joints = LimbTypes.invert_arm_configuration('left', joints)
        self.fc.zarj.hands.set_arm_configuration('left', joints)
        rospy.sleep(0.5)

        if self.fc.last_trailer_move is not None:
            log("Walking back to trailer center by {}; back up a bit".format(self.fc.last_trailer_move * -1))
            log("TODO - maybe this should not be a deadwalk")
            self.fc.zarj.zps.dead_walk([-0.06, 0.0, self.fc.last_trailer_move * -1])
            self.fc.last_trailer_move = None
            self.fc.clear_points()

        self.fc.zarj.pelvis.turn_body_to(-45)

        joints = deepcopy(LimbTypes.arm_styles['spin_array_1'])
        joints = LimbTypes.invert_arm_configuration('left', joints)
        self.fc.zarj.hands.set_arm_configuration('left', joints)
        rospy.sleep(0.5)

        self.fc.zarj.pelvis.turn_body_to(-75)

        joints = deepcopy(LimbTypes.arm_styles['spin_array_2'])
        joints = LimbTypes.invert_arm_configuration('left', joints)
        self.fc.zarj.hands.set_arm_configuration('left', joints)
        rospy.sleep(0.5)

        joints = deepcopy(LimbTypes.arm_styles['spin_array_3'])
        joints = LimbTypes.invert_arm_configuration('left', joints)
        self.fc.zarj.hands.set_arm_configuration('left', joints)
        rospy.sleep(0.5)

        joints = deepcopy(LimbTypes.hand_styles['oppose'])
        joints = LimbTypes.invert_hand_configuration('left', joints)
        self.fc.zarj.hands.set_hand_configuration('left', joints)
        rospy.sleep(0.5)

        joints = deepcopy(LimbTypes.arm_styles['spin_array_clear'])
        joints = LimbTypes.invert_arm_configuration('left', joints)
        self.fc.zarj.hands.set_arm_configuration('left', joints)
        rospy.sleep(0.5)

        self.fc.zarj.pelvis.turn_body_to(-60)

        joints = deepcopy(LimbTypes.arm_styles['king_tut'])
        joints = LimbTypes.invert_arm_configuration('left', joints)
        self.fc.zarj.hands.set_arm_configuration('left', joints)
        rospy.sleep(0.5)

        self.fc.zarj.pelvis.turn_body_to(0)

        self.done = True

    @property
    def next_chain(self):
        """ returns the name of the next macro in the chain """
        return 'pickup_array_prep'

class drop_array_look_for_choke(ZarjMacro):
    """ Use vision to try to find the choke in order to drop it """
    def start(self, _=None):
        """ Start the macro """
        if not find_choke(self):
            raise ZarjConfused("Cannot find the choke")
        else:
            self.done = True

    @property
    def next_chain(self):
        """ returns the name of the next macro in the chain """
        return 'drop_array_get_ready'

class drop_array_get_ready(ZarjMacro):
    """ Make sure the body is in a good state for this move """

    def start(self, _=None):
        """ Start the macro """
        anchor = self.fc.assure_anchor("drop_array")
        if anchor is None:
            raise ZarjConfused("Cannot find the choke")

        log("Set left arm to carry higher, right to football")
        joints = deepcopy(LimbTypes.arm_styles['carry_array_higher'])
        joints = LimbTypes.invert_arm_configuration('left', joints)
        self.fc.zarj.hands.set_arm_configuration('left', joints)

        joints = deepcopy(LimbTypes.arm_styles['pass_football'])
        joints = LimbTypes.invert_arm_configuration('right', joints)
        self.fc.zarj.hands.set_arm_configuration('right', joints)

        sideways = anchor.adjusted[1]
        if sideways > 0:
            log("Stepping left {} before swinging array".format(sideways))
            log("TODO - maybe this should not be a deadwalk")
            self.fc.zarj.zps.dead_walk([0.0, 0.0, sideways])

        joints = deepcopy(LimbTypes.arm_styles['release_array_1'])
        joints = LimbTypes.invert_arm_configuration('left', joints)
        log("Setting left arm to hold it out")
        self.fc.zarj.hands.set_arm_configuration('left', joints)
        rospy.sleep(0.5)

        log("Lean down, move neck so we can see choke")
        self.fc.zarj.pelvis.lean_body_to(-15)
        self.fc.zarj.pelvis.turn_body_to(-45)
        self.fc.zarj.neck.neck_control([ 0.5, 0.0, 0 ], True)

        if not self.chain_okay:
            self.fc.send_stereo_camera()
            log("Pick the inner choke, followed by the outer choke")

        self.done = True

    @property
    def next_chain(self):
        """ returns the name of the next macro in the chain """
        return 'drop_array_look_for_choke_2'

class drop_array_look_for_choke_2(ZarjMacro):
    """ Use vision to try to find the choke in order to drop it """
    def start(self, _=None):
        """ Start the macro """
        if not find_choke(self):
            raise ZarjConfused("Cannot find the choke")
        else:
            self.done = True

    @property
    def next_chain(self):
        """ returns the name of the next macro in the chain """
        return 'drop_array_move_feet'


class drop_array_move_feet(ZarjMacro):
    """ Move the feet into position suitable for dropping the array """

    def start(self, _=None):
        """ Start the macro """
        anchor = self.fc.assure_anchor("drop_array")
        if anchor is None:
            raise ZarjConfused("Cannot find the choke")

        log("Array is {} away, {} to the side, at angle {}".format(anchor.adjusted[0], anchor.adjusted[1], anchor.angle))

        forward = anchor.adjusted[0]
        sideways = anchor.adjusted[1]

        log("Walking sideways {}, forward {}".format(sideways, forward))
        log("TODO - maybe this should not be a deadwalk")
        self.fc.zarj.zps.dead_walk([forward, 0.0, sideways])
        self.fc.clear_points()

        if not self.chain_okay:
            self.fc.send_stereo_camera()
            log("Pick the inner choke, followed by the outer choke")

        self.done = True

    @property
    def next_chain(self):
        """ returns the name of the next macro in the chain """
        return 'drop_array_lookat_choke_again'

class drop_array_lookat_choke_again(ZarjMacro):
    """ Use vision to try to find the choke in order to drop it """
    def start(self, _=None):
        """ Start the macro """
        if not find_choke(self):
            raise ZarjConfused("Cannot find the choke")
        else:
            self.done = True

    @property
    def next_chain(self):
        """ returns the name of the next macro in the chain """
        return 'drop_array_perform_drop'

class drop_array_perform_drop(ZarjMacro):
    """ Make sure the body is in a good state for this move """

    def start(self, _=None):
        """ Start the macro """
        anchor = self.fc.assure_anchor("drop_array")
        if anchor is None:
            raise ZarjConfused("Cannot find the choke")

        joints = deepcopy(LimbTypes.arm_styles['release_array_1'])
        joints = LimbTypes.invert_arm_configuration('left', joints)
        log("Setting left arm to hold it out")
        self.fc.zarj.hands.set_arm_configuration('left', joints)
        rospy.sleep(0.5)

        log("Lowering hand")
        msg = ZarjMovePalmCommand('left', False, 0.81, 0.15, 1.16, 0.0, 0.0, -90.0, True)
        self.fc.process_palm_msg(msg)

        joints = deepcopy(LimbTypes.hand_styles['oppose'])
        joints = LimbTypes.invert_hand_configuration('left', joints)
        log("Opening hand")
        self.fc.zarj.hands.set_hand_configuration('left', joints)
        rospy.sleep(0.25)

        log("Retracting hand")
        msg = ZarjMovePalmCommand('left', False, 0.61, 0.15, 1.16, 0.0, 0.0, -90.0, True)
        self.fc.process_palm_msg(msg)

        if self.fc.task != 2 or self.fc.checkpoint < 3:
            raise ZarjConfused("Checkpoint did not progress")

        self.done = True


    @property
    def next_chain(self):
        """ returns the name of the next macro in the chain """
        return 'open_array_prep'


class open_array_prep(ZarjMacro):
    """ Get ready to open the array"""

    def start(self, _=None):
        """ Start the macro """

        joints = deepcopy(LimbTypes.hand_styles['open'])
        joints = LimbTypes.invert_hand_configuration('left', joints)
        self.fc.zarj.hands.set_hand_configuration('left', joints)

        log("Looking down and left")
        self.fc.zarj.neck.neck_control([0.5, 1.0, 0], True)

        rospy.sleep(0.5)

        self.fc.clear_points()
        if not self.chain_okay:
            self.fc.send_stereo_camera()
            log("Click the button.")

        self.done = True

    @property
    def next_chain(self):
        """ returns the name of the next macro in the chain """
        return 'open_array_look_for_button'

class open_array_look_for_button(ZarjMacro):
    """ Use vision to try to find the choke in order to drop it """
    def start(self, _=None):
        """ Start the macro """
        if not find_button(self):
            raise ZarjConfused("Cannot find the button")
        else:
            self.done = True

    @property
    def next_chain(self):
        """ returns the name of the next macro in the chain """
        return 'open_array_reach_above'

class open_array_reach_above(ZarjMacro):
    """ Reach above the button """

    def start(self, _=None):
        """ Start the macro """
        anchor = self.fc.assure_anchor("task2_prep_button")
        if anchor is None:
            raise ZarjConfused("Cannot find the button")

        x = anchor.adjusted[0]
        y = anchor.adjusted[1]
        z = anchor.adjusted[2]

        log("Commanding hand above button")
        msg = ZarjMovePalmCommand('left', False, x, y, z, 0, 0, 0, True)
        self.fc.process_palm_msg(msg)

        self.done = True

    @property
    def next_chain(self):
        """ returns the name of the next macro in the chain """
        return 'open_array_press_button'

class open_array_press_button(ZarjMacro):
    """ Reach above the button """

    def start(self, _=None):
        """ Start the macro """
        anchor = self.fc.assure_anchor("task2_prep_button")
        if anchor is None:
            raise ZarjConfused("Cannot find the button")

        x = anchor.adjusted[0]
        y = anchor.adjusted[1]
        z = anchor.adjusted[2] - 0.15

        log("Commanding hand onto button")
        msg = ZarjMovePalmCommand('left', False, x, y, z, 0, 0, 0, True)
        self.fc.process_palm_msg(msg)

        self.done = True

    @property
    def next_chain(self):
        """ returns the name of the next macro in the chain """
        return 'open_array_lift_hand'

class open_array_lift_hand(ZarjMacro):
    """ Reach above the button """

    def start(self, _=None):
        """ Start the macro """
        anchor = self.fc.assure_anchor("task2_prep_button")
        if anchor is None:
            raise ZarjConfused("Cannot find the button")

        x = anchor.adjusted[0]
        y = anchor.adjusted[1]
        z = anchor.adjusted[2] + 0.1

        log("Commanding hand to go back up over release array")
        msg = ZarjMovePalmCommand('left', False, x, y, z, 0, 0, 0, True)
        self.fc.process_palm_msg(msg)

        if self.fc.task != 2 or self.fc.checkpoint < 4:
            raise ZarjConfused("Checkpoint did not progress")

        self.done = True

    @property
    def next_chain(self):
        """ returns the name of the next macro in the chain """
        return 'pickup_cable_prep'

class pickup_cable_prep(ZarjMacro):
    """ pickup_cable_prep """

    def start(self, _=None):
        """ Start the macro """

        joints = deepcopy(LimbTypes.arm_styles['pass_football'])
        joints = LimbTypes.invert_arm_configuration('left', joints)
        log("Setting left arm to pass a football")
        self.fc.zarj.hands.set_arm_configuration('left', joints)

        joints = deepcopy(LimbTypes.arm_styles['king_tut'])
        joints = LimbTypes.invert_arm_configuration('right', joints)
        log("king tut right arm, set to the bird")
        self.fc.zarj.hands.set_arm_configuration('right', joints)

        joints = deepcopy(LimbTypes.hand_styles['dabird'])
        joints = LimbTypes.invert_hand_configuration('right', joints)
        self.fc.zarj.hands.set_hand_configuration('right', joints)

        log("Looking down and right; lean over")
        self.fc.zarj.neck.neck_control([0.5, 0.0, 0], True)
        self.fc.zarj.pelvis.lean_body_to(-30)

        rospy.sleep(0.5)

        self.fc.clear_points()
        if not self.chain_okay:
            self.fc.send_stereo_camera()
            log("Click the inside cable chocke, and then the outer choke, in that order.")

        self.done = True

    @property
    def next_chain(self):
        """ returns the name of the next macro in the chain """
        return 'pickup_cable_look_for_choke'

class pickup_cable_look_for_choke(ZarjMacro):
    """ Use vision to try to find the choke """
    def start(self, _=None):
        """ Start the macro """
        if not find_choke(self):
            raise ZarjConfused("Cannot find the choke")
        else:
            self.done = True

    @property
    def next_chain(self):
        """ returns the name of the next macro in the chain """
        return 'pickup_cable_move_feet'

class pickup_cable_move_feet(ZarjMacro):
    """pickup_cable_move_feet"""

    def start(self, _=None):
        """ Start the macro """
        anchor = self.fc.assure_anchor("pickup_cable")
        if anchor is None:
            raise ZarjConfused("Cannot find the choke")

        log("Array is {} away, {} to the side, at angle {}".format(anchor.adjusted[0], anchor.adjusted[1], anchor.angle))

        forward = anchor.adjusted[0]
        sideways = anchor.adjusted[1]

        log("Walking sideways {}, forward {}".format(sideways, forward))
        log("TODO - maybe this should not be a deadwalk")
        self.fc.zarj.zps.dead_walk([forward, 0.0, sideways])

        log("Look right")
        self.fc.zarj.neck.neck_control([ 0.5, -1.0, 0 ], True)

        log("Lean and turn")
        self.fc.zarj.pelvis.lean_body_to(-30)
        self.fc.zarj.pelvis.turn_body_to(30)

        if not self.chain_okay:
            self.fc.send_stereo_camera()
            log("Click the inside cable choke, and then the outer choke, in that order.")

        self.done = True

    @property
    def next_chain(self):
        """ returns the name of the next macro in the chain """
        return 'pickup_cable_move_hand_start'

class pickup_cable_move_hand_start(ZarjMacro):
    """pickup_cable_move_hand_start"""

    def start(self, _=None):
        """ Start the macro """
        anchor = self.fc.assure_anchor("center")
        if anchor is None:
            raise ZarjConfused("Cannot find the choke")

        x = anchor.adjusted[0]
        y = anchor.adjusted[1] + (.05 * math.sin(math.radians(anchor.angle)))
        z = anchor.adjusted[2] + 0.14

        log("Commanding hand above cable end")
        msg = ZarjMovePalmCommand('right', False, x, y, z, -1 * anchor.angle, 20, -90, True)
        self.fc.process_palm_msg(msg)
        self.done = True

        log("Giving the hand a long time to settle down.")
        rospy.sleep(3.0)

    @property
    def next_chain(self):
        """ returns the name of the next macro in the chain """
        return 'pickup_cable_move_hand_finish'

class pickup_cable_move_hand_finish(ZarjMacro):
    """pickup_cable_move_hand_start"""

    def start(self, _=None):
        """ Start the macro """
        anchor = self.fc.assure_anchor("center")
        if anchor is None:
            raise ZarjConfused("Cannot find the choke")

        x = anchor.adjusted[0]
        y = anchor.adjusted[1] + (.05 * math.sin(math.radians(anchor.angle)))
        z = anchor.adjusted[2] + 0.04

        log("Commanding hand onto cable end")
        msg = ZarjMovePalmCommand('right', False, x, y, z, -1 * anchor.angle, 20, -90, True)
        self.fc.process_palm_msg(msg)
        self.done = True

    @property
    def next_chain(self):
        """ returns the name of the next macro in the chain """
        return 'pickup_cable_grab'

class pickup_cable_grab(ZarjMacro):
    """pickup_cable_grab"""

    def start(self, _=None):
        """ Start the macro """
        log("Commanding hand to pinch cable")

        joints = deepcopy(LimbTypes.hand_styles['grab_cable'])
        joints = LimbTypes.invert_hand_configuration('right', joints)
        self.fc.zarj.hands.set_hand_configuration('right', joints)

        rospy.sleep(0.5)  # A little extra time to let us get a good grip
        self.done = True

    @property
    def next_chain(self):
        """ returns the name of the next macro in the chain """
        return 'pickup_cable_lift'

class pickup_cable_lift(ZarjMacro):
    """pickup_cable_lift"""

    def start(self, _=None):
        """ Start the macro """
        anchor = self.fc.assure_anchor("center")
        if anchor is None:
            raise ZarjConfused("Cannot find the choke")
        log("Lifting cable; unlean, unturn, move hands")

        x = anchor.adjusted[0] + .1
        y = anchor.adjusted[1] + .1
        z = anchor.adjusted[2] + 0.3

        log("Commanding hand into the air")
        msg = ZarjMovePalmCommand('right', False, x, y, z, 0, 0, 45, True)
        self.fc.process_palm_msg(msg)

        self.fc.zarj.neck.neck_control([0.5, 0, 0], True)

        self.fc.clear_points()
        if not self.chain_okay:
            self.fc.send_stereo_camera()
            log("Click the center of the power plug.")

        if self.fc.task != 2 or self.fc.checkpoint < 5:
            raise ZarjConfused("Checkpoint did not progress")

        self.done = True

    @property
    def next_chain(self):
        """ returns the name of the next macro in the chain """
        return 'plugin_look_for_plug'


class plugin_look_for_plug(ZarjMacro):
    """ Use vision to try to find the plug"""
    def start(self, _=None):
        """ Start the macro """
        if not find_plug(self):
            raise ZarjConfused("Cannot find the plug")
        else:
            self.done = True

    @property
    def next_chain(self):
        """ returns the name of the next macro in the chain """
        return 'plugin_move_feet'

class plugin_move_feet(ZarjMacro):
    """ Move the feet so we can plug in """

    def start(self, _=None):
        """ Start the macro """
        anchor = self.fc.assure_anchor("connect_cable")
        if anchor is None:
            raise ZarjConfused("Cannot find the plug")
        log("Moving feet to plug in anchor")

        forward = anchor.adjusted[0]
        sideways = anchor.adjusted[1]

        # This is generally a move bacwkards; let's do that before
        #  the step to the side, to keep the cord from tangling
        log("Walking forward {}".format(forward))
        self.fc.zarj.zps.dead_walk([forward, 0.0, 0.0])

        log("Walking sideways {}".format(sideways))
        self.fc.zarj.zps.dead_walk([0.0, 0.0, sideways])

        log("TODO - maybe this should not be a deadwalk")

        log("Look down")
        self.fc.zarj.neck.neck_control([ 0.5, .0, 0 ], True)

        log("Lean and turn forward")
        self.fc.zarj.pelvis.lean_body_to(-30)
        self.fc.zarj.pelvis.turn_body_to(0)

        if not self.chain_okay:
            self.fc.send_stereo_camera()
            log("Click the power plug")

        self.done = True

    @property
    def next_chain(self):
        """ returns the name of the next macro in the chain """
        return 'plugin_move_hand_pre_start'

class plugin_move_hand_pre_start(ZarjMacro):
    """ Move the cable way up and right in the hopes that it will drape better """

    def start(self, _=None):
        """ Start the macro """
        anchor = self.fc.assure_anchor("plugin_pre_start")
        if anchor is None:
            raise ZarjConfused("Cannot find the plug")

        x = anchor.adjusted[0]
        y = anchor.adjusted[1]
        z = anchor.adjusted[2]

        log("Commanding hand above and left of plug; x {}, y {}, z {}".format(x, y, z))
        msg = ZarjMovePalmCommand('right', False, x, y, z, 0, 0, 45, True)
        self.fc.process_palm_msg(msg)
        self.done = True

    @property
    def next_chain(self):
        """ returns the name of the next macro in the chain """
        return 'plugin_move_hand_start'

class plugin_move_hand_start(ZarjMacro):
    """pickup_cable_move_hand_start"""

    def start(self, _=None):
        """ Start the macro """
        anchor = self.fc.assure_anchor("plugin_start")
        if anchor is None:
            raise ZarjConfused("Cannot find the plug")

        x = anchor.adjusted[0]
        y = anchor.adjusted[1] + 0.05
        z = anchor.adjusted[2]

        log("Commanding hand above and left of plug; x {}, y {}, z {}".format(x, y, z))
        msg = ZarjMovePalmCommand('right', False, x, y, z, 0, 0, 45, True)
        self.fc.process_palm_msg(msg)
        self.done = True

        log("Giving the hand a long time to settle down.")
        rospy.sleep(3.0)

    @property
    def next_chain(self):
        """ returns the name of the next macro in the chain """
        return 'plugin_move_hand_finish'

class plugin_move_hand_finish(ZarjMacro):
    """ Finish plugging it in """

    def start(self, _=None):
        """ Start the macro """
        anchor = self.fc.assure_anchor("plugin_finish")
        if anchor is None:
            raise ZarjConfused("Cannot find the plug")

        x = anchor.adjusted[0]
        y = anchor.adjusted[1]
        z = anchor.adjusted[2]

        log("Commanding hand to plug er in; x {}, y {}, z {}".format(x, y, z))
        msg = ZarjMovePalmCommand('right', False, x, y, z, 0, 0, 45, True)
        self.fc.process_palm_msg(msg)
        self.done = True

        log("Give it a while.")
        rospy.sleep(3.0)

        if self.fc.task != 2 or self.fc.checkpoint < 6:
            raise ZarjConfused("Checkpoint did not progress")

        self.done = True

    @property
    def next_chain(self):
        """ returns the name of the next macro in the chain """
        return 'plugin_letitgo'


class plugin_letitgo(ZarjMacro):
    """ Let go of the cable """

    def start(self, _=None):
        """ Start the macro """
        anchor = self.fc.assure_anchor("plugin_finish")
        if anchor is None:
            raise ZarjConfused("Cannot find the plug")

        x = anchor.adjusted[0]
        y = anchor.adjusted[1]
        z = anchor.adjusted[2] + .2

        log("Flipping bird, commanding hand to let go; x {}, y {}, z {}".format(x, y, z))
        joints = deepcopy(LimbTypes.hand_styles['purebird'])
        joints = LimbTypes.invert_hand_configuration('right', joints)
        self.fc.zarj.hands.set_hand_configuration('right', joints)

        msg = ZarjMovePalmCommand('right', False, x, y, z, 0, 0, -90, True)
        self.fc.process_palm_msg(msg)

        self.done = True
        self.fc.clear_points()


class task2_finish_box(ZarjMacro):
    """task2_finish_box"""

    def start(self, _=None):
        """ Start the macro """
        self.fc.zarj.zps.look_forward()
        if self.fc.zarj.zps.state & zarj.zps.STATE_PRESTART:
            self.fc.zarj.zps.path_start()
        self.fc.zarj.zps.walk_to_next_finish_box()
        self.fc.zarj.zps.look_for_exit()
        self.done = True

class Task2(ZarjTask):
    """ This is the class that drives task 2 """

    def __init__(self, fc):
        ZarjTask.__init__(self, fc, 2,
            [CheckpointB1(self), CheckpointB2(self), CheckpointB3(self),
             CheckpointB4(self), CheckpointB5(self), CheckpointB6(self)])
