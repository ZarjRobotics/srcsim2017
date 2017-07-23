"""
    This is the internal GPS / position tracking / map logic

    This should be the core of Zarj movement it trieds to keep track of
    the robots location in the world and also where it can safely move next.

    Start the robot using
    zps.path_start() - this will determin if it is in a finish box or not
                       and if so move to the forward exit, otherwise is a no-op

    Core robot navigation is done using:

    zps.next_path_point() - Move to the next turn or point where the zps needs
                            to make another decision

    zps.walk_to_next_finish_box() - walks a continues path until it reaches a
                                    basically repeated calls to next_path_point

    zps.exit_finish_box() - Looks for an exit and exits a finish box

    If you want to go 'off the path' there are 2 mechanisims for that

    Waypoints:  This basically tags Zarj's current location with the intention
                of moving back to that exact location later. Generally used if
                you are at a path point and need to do task specific moving
                about.  It is a stack so you can mark repeated waypoints if
                needed

    zps.mark_waypoint() - marks the present location as a waypoint
    zps.return_to_waypoint() - return to the last marked waypoint


    Points of Interest (poi):  These are points along the path that you want
                               Zarj to be able to walk to while continuing to
                               navigate.  Unlike waypoints there is no return
                               point, a POI will be along the path and can be
                               continued from. only 1 poi can be active at a
                               time

                                IMPORTANT notes about poi :
                                * They have to be directly ahead of the robot.
                                * Then cannot be after another turn or
                                  something.
                                * They have to be outside of finish boxes

    zps.set_poi() - set the point of interest
    zps.walk_to_poi() - navigate to that poi
    zps.continue_from_poi() - continue from the current poi to the next path
                              point


   Dead Walking: The idea here is to walk to a point, keeping it in our path
                 stack, but do not do any fancy pathing detection or poi style
                 tracking. That means the operator can tell the robot 'GO HERE'
                 and it will no brains, no pathing, just walking where you say
                 and then you can resume more upper level path navigation from
                 there.  They look like poi in format (distance, turn, offset)

    zps.dead_walk() - walk to a point

    This is the upper level overview...

    For the core of the pathing decision making look at zps.determine_path() and
    zps.analyze_path()

    Note that zps.look_for_next_path() is the higher level way to look at an
    image and get the decision from it without actually controling the robot.
    zps.determine_path() can be used if you already have an image.
"""
import os
import rospy
from geometry_msgs.msg import PoseStamped, Vector3
from tf.transformations import quaternion_matrix
from geometry_msgs.msg import PointStamped

import cv2
import numpy as np

import zarj.navigation as navigation
from .utils import wait_for_walk, log, debug_sequence
from .confused import ZarjConfused
from .tf_conv import fmt

STATE_FINISH_BOX = 1<<0
STATE_PRESTART = 1<<1
STATE_POI = 1<<2

FLAG_STAIRS = 1<<0
FLAG_HABITAT = 1<<1
FLAG_ALLOW_HALF = 1<<2

PATH_FORWARD_WIGGLE = 10 # ratio of area to width of the end points of a path
                         # that is still considered to end in a flat end
                         # This is important becasue sometime we are not 100%
                         # dead on streight on the path so we get a little
                         # offset

DEFAULT_BACKUP = 0.3
SMALL_BACKUP = 0.05

MAXIMUM_BACKUP = 0.5 # meters

YELLOW_LOW = np.array([25, 50, 50])
YELLOW_HIGH = np.array([35, 255, 255])

class ZPS(object):
    """ The ZPS (Zarg Positioning System) is used to track Zarj's movement
        and the path ahead and behind """

    class PathDecision(object):
        """ a pathing decision """

        def __init__(self, distance=0, turn=None, offset=0.0, state=0) :
            self.origin = None
            self.target = None
            self.target_orientation = 0
            self.distance = distance
            self.turn = turn
            self.state = state
            self.offset = offset

        def figure_target(self, zarj, previous):
            """ Set the origin """
            self.origin = previous.target
            if abs(self.distance) > 0.001:
                pelvis = zarj.transform.lookup_transform('world', 'pelvis',
                                                         rospy.Time()).transform
                quaternion = [pelvis.rotation.w, pelvis.rotation.x,
                              pelvis.rotation.y, pelvis.rotation.z]
                matrix = quaternion_matrix(quaternion)
                point = np.matrix([0, self.distance, 0, 1], dtype='float32')
                point.resize((4, 1))
                rotated = matrix*point
                self.target = Vector3(self.origin.x - rotated.item(1),
                                      self.origin.y + rotated.item(2),
                                      self.origin.z)
            else:
                self.target = self.origin

            if self.turn is not None:
                self.target_orientation = self.turn + \
                    previous.target_orientation

        def set_state(self, state):
            """ Set a state """
            self.state |= state

        def clear_state(self, state):
            """ clear a state """
            self.state &= ~state

    def __init__(self, zarj):
        self.stop = False
        self.path_stack = []
        self.waypoint_stack = []
        self.point_of_interest = None
        self.after_poi = None
        self.zarj = zarj
        self.state = STATE_PRESTART
        self.features = 0
        self.present_offset = None
        self.flags = 0
        self.backup = DEFAULT_BACKUP
        self.backup_distance =  0
        self.crop = False

        self.rail = None

        self.max_distance = self.zarj.navigation.distance_to_row(544)

        start = self.PathDecision()
        start.origin = Vector3(0, 0, 0)
        start.target = Vector3(0, 0, 0)
        self.path_stack.append(start)

    def set_flag(self, flag):
        """ Set a flag """
        self.flags |= flag

    def clear_flag(self, flag):
        """ clear a flag"""
        self.flags &= ~flag

    def get_position(self):
        """ Where are we... """
        world = self.zarj.transform.lookup_transform('world',
                                                     self.zarj.walk.lfname,
                                                     rospy.Time())
        chest_position = self.zarj.transform.lookup_transform(
            'pelvis', 'torso', rospy.Time()).transform

        pose = PoseStamped()
        pose.header.frame_id = 'pelvis'
        pose.header.stamp = rospy.Time()
        pose.pose.position = world.transform.translation
        pose.pose.orientation = chest_position.rotation

        return pose

    def mark_waypoint(self):
        """ Mark a position, likly for later return """
        pos = self.get_position()
        self.waypoint_stack.append(pos)

    def pop_waypoint(self):
        """ Pop a waypoint but do not return there """
        if len(self.waypoint_stack):
            return self.waypoint_stack.pop()
        return None

    def return_to_waypoint(self):
        """ Return to the top waypoint """
        pos = self.pop_waypoint()
        if pos is not None:
            self.zarj.pelvis.turn_body_to_pose(pos)
            self.zarj.walk.walk_to(pos.pose.position)
            wait_for_walk(self.zarj)

    def clear_waypoints(self):
        """ clear all waypoints """
        self.waypoint_stack = []

    def set_expected_feature(self, feature):
        """ set a feature, means more error processing """
        self.features |= feature

    def clear_expected_feature(self, feature):
        """ set a feature, means more error processing """
        self.features &= ~feature

    def set_state(self, state):
        """ Set a state """
        self.state |= state

    def clear_state(self, state):
        """ clear a state """
        self.state &= ~state

    def look_for_next_path(self, debug=False):
        """ look for the next path point """
        if self.state & STATE_FINISH_BOX:
            return self.exit_finish_box()
        if self.flags & FLAG_STAIRS:
            path = self.determine_path(None, debug)
        else:
            image = self.zarj.navigation.get_navigation_image(self.crop)
            path = self.determine_path(image.copy(), debug)
            self.present_offset = self.find_path_offset(image,
                                                        navigation.BASE_ROW)

        return path

    def next_path_point(self, debug=False, path=None, deadwalk=False):
        """ move to the next path point """
        if self.flags & FLAG_HABITAT and not deadwalk:
            log('ZPS pathing is disabled in the habitat')
            return
        if self.stop and not deadwalk:
            raise ZarjConfused("Asked to stop! ZPS halted."
                               "Use path_start to restart")
        if path is None:
            path = self.look_for_next_path(debug)
        if self.backup_distance >= MAXIMUM_BACKUP:
            raise ZarjConfused('Backing up too much! Tell me where to go!')
        if self.rail is not None:
            path.offset = self.rail['offset']
        log("*Path Decision*\n\toffset to: {}m\n\twalk: {}m\n\tturn: {}".format(
            path.offset, path.distance, path.turn))
        self.center_on_offset(path.offset, deadwalk)
        self.zarj.walk.fix_stance()
        wait_for_walk(self.zarj)
        if self.stop and not deadwalk:
            raise ZarjConfused("Asked to stop! ZPS halted."
                               "Use path_start to restart")

        if path.distance is not 0:
            path.figure_target(self.zarj, self.path_stack[-1])
            self.path_stack.append(path)
            if abs(path.distance) > 0.001:
                log("=> Walking: {0}".format(path.distance))
                self.zarj.walk.forward(path.distance, square_up=True)
                wait_for_walk(self.zarj)
            else:
                log("Short walk ignored {0}".format(path.distance))

            if self.stop and not deadwalk:
                raise ZarjConfused("Asked to stop! ZPS halted."
                                   "Use path_start to restart")

            if self.rail is not None:
                self.rail['range'] -= path.distance
                log('RAIL AT {}'.format(self.rail))
                if self.rail['range'] <= -0.5:
                    log('Past the rail, resetting to path')
                    # Overcorrect a little bit
                    self.center_on_offset(-self.rail['offset']/2, deadwalk)
                    self.rail = None
                    # adjust our backup
                    self.backup = SMALL_BACKUP

            if self.stop and not deadwalk:
                raise ZarjConfused("Asked to stop! ZPS halted."
                                   "Use path_start to restart")

            if path.turn is not None and path.turn != 0:
                log("=> Turning : {0}".format(path.turn))
                if self.present_offset is not None and abs(self.present_offset) >= 0.1 and path.turn > 45:
                    self.zarj.walk.turn(path.turn/2.0)
                    wait_for_walk(self.zarj)
                    self.zarj.walk.turn(path.turn/2.0)
                    wait_for_walk(self.zarj)
                else:
                    self.zarj.walk.turn(path.turn)
                    wait_for_walk(self.zarj)

            if self.stop and not deadwalk:
                raise ZarjConfused("Asked to stop! ZPS halted."
                                   "Use path_start to restart")

            log("Reached Path point {0}".format(len(self.path_stack)))
        else:
            log("FAILURE: Could not find path...")
            raise ZarjConfused("Could not find path...")


    def exit_finish_box(self, debug=False, image=None):
        """ Exit the finish box """
        if self.flags & FLAG_HABITAT:
            log('ZPS pathing is disabled in the habitat')
            return
        if not self.state & STATE_FINISH_BOX:
            log("FAILURE: Not in a finish box")
            raise ZarjConfused("Not in a finish box, but asked to leave")
        self.clear_state(STATE_FINISH_BOX)
        distance, turn = self.look_for_exit(debug, image)
        if distance is None:
            log("FAILURE: Could not find exit from finish box")
            raise ZarjConfused("Could not find exit from finish box")
        else:
            distance -= self.zarj.navigation.perspective.m_row_zero - 0.2
            _exit = self.PathDecision(distance, turn)
            _exit.figure_target(self.zarj, self.path_stack[-1])
            log("=>exit at {0} : {1}".format(_exit.target,
                                           _exit.target_orientation))
            self.path_stack.append(_exit)
            self.zarj.walk.forward(distance, square_up=True)
            wait_for_walk(self.zarj)

    def walk_to_next_finish_box(self, debug=False):
        """ Walk to the next finish box """
        if self.flags & FLAG_HABITAT:
            log('ZPS pathing is disabled in the habitat')
            return
        while not self.state & STATE_FINISH_BOX:
            self.next_path_point(debug)
        if self.state & STATE_FINISH_BOX:
            log("Reached finish box")

    def look_for_exit(self, debug=False, image=None):
        """ Look for an exit path """
        if image is None:
            image = self.zarj.navigation.get_navigation_image(self.crop)
        path = self.zarj.navigation.look_for_exit(image, debug)
        if path is None:
            self.zarj.pelvis.turn_body_to(-75)
            self.zarj.neck.head_down()
            image, _ = self.zarj.eyes.get_images()
            if self.crop:
                image = image[0:image.shape[0], 65:image.shape[1]-65]
            path = self.zarj.navigation.look_for_exit(image, debug)
            self.zarj.pelvis.turn_body_to(0)
            self.zarj.neck.head_forward()
            if path is None:
                self.zarj.pelvis.turn_body_to(75)
                self.zarj.neck.head_down()
                image, _ = self.zarj.eyes.get_images()
                if self.crop:
                    image = image[0:image.shape[0], 65:image.shape[1]-65]
                path = self.zarj.navigation.look_for_exit(image, debug)
                self.zarj.pelvis.turn_body_to(0)
                self.zarj.neck.head_forward()
                if path is None:
                    return None, None
                else:
                    self.zarj.walk.turn(90)
                    while not self.zarj.walk.walk_is_done():
                        rospy.sleep(0.2)
                    image = self.zarj.navigation.get_navigation_image(self.crop)
                    path = self.zarj.navigation.look_for_exit(image, debug)
                    return path, 90
            else:
                self.zarj.walk.turn(-90)
                while not self.zarj.walk.walk_is_done():
                    rospy.sleep(0.2)
                image = self.zarj.navigation.get_navigation_image(self.crop)
                path = self.zarj.navigation.look_for_exit(image, debug)
                return path, -90
        else:
            return path, 0

    def look_down(self):
        """ look downward! """
        self.zarj.neck.head_down(False)
        self.zarj.pelvis.lean_body_to(-15)

    def look_forward(self):
        """ look forward! """
        self.zarj.neck.head_forward(False)
        self.zarj.pelvis.lean_body_to(0)

    def find_path_offset(self, image, base_row, unwarped=True):
        """ Find how far off from center we are"""

        output = self.zarj.navigation.figure_path(image, base_row, False)
        if output is not None and output['offset'] is not None:
            if unwarped:
                return -output['offset'] * \
                       self.zarj.navigation.perspective.m_per_col
            else:
                return output['offset']
        return None

    def center_on_offset(self, target_offset=0.0, deadwalk=False):
        """ Center ourselves on the path """
        if deadwalk or self.present_offset is not None:
            if deadwalk:
                offset = target_offset
            else:
                offset = target_offset - self.present_offset
            log("=> Adjusting to offset:{0} current {1} correction {2}".format(
                target_offset, self.present_offset, offset))
            limit = 0.06
            if deadwalk:
                limit = 0.01
            if abs(offset) > limit:
                self.zarj.walk.side(offset)
                wait_for_walk(self.zarj)
                if not deadwalk:
                    self.present_offset = target_offset

    def _find_stair_base(self, cloud, distance, center, width):
        """ Try to find the base of the stairs really exactly """
        _, details = self.zarj.eyes.get_cloud_image_with_details(cloud)
        occ = 0
        pnt = None
        while pnt is None:
            if not np.isnan(details[distance][center-occ][0]):
                pnt = details[distance][center-occ]
                break
            if not np.isnan(details[distance][center+occ][0]):
                pnt = details[distance][center+occ]
                break
            occ += 1
            if occ >= width/6:
                occ = 0
                distance -= 1
        if pnt is not None:
            point = PointStamped()
            point.header = cloud.header
            point.point.x = pnt[0]
            point.point.y = pnt[1]
            point.point.z = pnt[2]
            pnt = self.zarj.transform.tf_buffer.transform(point,
                                                          self.zarj.walk.lfname)
            return pnt.point.x
        log("ERROR: Could not find stair base")
        return None

    def _look_for_rails(self, path, yellow, image_dump, cloud):
        """ Code to look for and handle the rails in Task 3 """
        # We have to watch out for those stupid stupid 'guard' rails
        hsv = cv2.cvtColor(yellow, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, YELLOW_LOW, YELLOW_HIGH)
        if image_dump is not None:
            cv2.imwrite(image_dump + '/yellow_path_{0}.png'.format(
                debug_sequence()), mask)
        if cv2.countNonZero(mask) > 0:
            cimage, details = self.zarj.eyes.get_cloud_image_with_details(
                cloud)
            hsv = cv2.cvtColor(cimage, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, YELLOW_LOW, YELLOW_HIGH)
        if cv2.countNonZero(mask) > 0:
            indexes = mask.nonzero()
            points = []
            for i in range(len(indexes[1])):
                points.append(details[indexes[0][i], indexes[1][i]])
            points = np.array(points)
            max_points = np.amax(points, axis=0)
            min_points = np.amin(points, axis=0)

            # Thing to remember:
            # *) the points are in x, y, z
            #    x is side to side positive to the LEFT
            #    y is up/down
            #    z is distance

            log('Rail {}m away vs my distance of {}m'.format(min_points[2],
                                                             path.distance))

            if min_points[2] <= path.distance + 0.2:
                # Avoidance maybe needed
                side = None
                jut = 0

                # This is not the cleanest.. but it is working.
                if min_points[0] < -0.6 and max_points[0] > 0.6:
                    log('Rail fully blocks path.. must be 2 rails')
                elif max_points[0] > -0.6 and min_points[0] < 0:
                    # rails will be opposite turns
                    if path.turn is not None:
                        if path.turn > 0:
                            print 'path turns left, rail on right'
                            side = 'right'
                            jut = abs(-0.6 - max_points[0])
                        elif path.turn <= 0:
                            print 'path turns right, rail on left'
                            side = 'left'
                            jut = abs(min_points[0] - 0.6)
                    else:
                        print 'On the right:', min_points[0]
                        side = 'right'
                        jut = abs(-0.6 - max_points[0])
                elif min_points[0] < 0.6 and max_points[0] > 0:
                    # rails will be opposite turns
                    if path.turn is not None:
                        if path.turn >= 0:
                            print 'path turns left, rail on right'
                            side = 'right'
                            jut = abs(-0.6 - max_points[0])
                        elif path.turn < 0:
                            print 'path turns right, rail on left'
                            side = 'left'
                            jut = abs(min_points[0] - 0.6)
                    else:
                        print 'On the left:', max_points[0]
                        side = 'left'
                        jut = abs(min_points[0] - 0.6)
                else:
                    print 'Not in the way'

                if side == 'left':
                    log('RAIL IN THE WAY ON THE LEFT: {}m'.format(jut))
                    path.offset = -0.35
                    self.rail = {'offset':path.offset,
                                 'range':min_points[2]}
                elif side == 'right':
                    log('RAIL IN THE WAY ON THE RIGHT: {}m'.format(jut))
                    path.offset = 0.35
                    self.rail = {'offset':path.offset,
                                 'range':min_points[2]}
            else:
                log('Rail too far away to care. Ignoring')

    def determine_path(self, image, debug=False, unwarped=True):
        """
            This is the upper level turn image into path function.
            The lower level core decisions are made by zps.analyze_path

            Here we take that decision, unwarp the pixels into meters and tweak
            things based on the turns.
        """
        if image is None:
            self.look_down()
            image, _ = self.zarj.eyes.get_images()
            if self.crop:
                image = image[0:image.shape[0], 65:image.shape[1]-65]
            if self.flags & FLAG_STAIRS:
                detection = image.copy()
                cloud = self.zarj.eyes.get_stereo_cloud()
                self.present_offset = self.find_path_offset(image,
                                                            navigation.BASE_ROW)
            self.look_forward()
        image_dump = os.environ.get("ZARJ_IMAGE_DUMP")
        if image_dump is not None:
            cv2.imwrite(image_dump + '/raw_path_{0}.png'.format(
                debug_sequence()), image)
        markup = debug or image_dump is not None
        output = self.zarj.navigation.figure_path(image, navigation.BASE_ROW,
                                                  markup)
        path = self.analyze_path(output)
        pixel_distance = path.distance

        if unwarped and path.distance != 0:
            path.distance = self.zarj.navigation.unwarp_perspective(
                (image.shape[1]/2, path.distance))

        if self.flags & FLAG_STAIRS:
            self._look_for_rails(path, detection, image_dump, cloud)

        if abs(self.max_distance - path.distance) < 0.1 or path.turn is None:
            if path.distance > 0:
                log("no path.turn in sight, cutting distance short")
                path.distance -= (self.zarj.navigation.perspective.m_row_zero +\
                                  0.2)
                path.turn = 0
            else:
                log('Could not find expected path... back up a bit')
                path.distance = -self.backup
                path.turn = 0
                self.backup_distance += self.backup
        elif path.turn == 0:
            # It is also important to note that the base of the stairs is going
            # to look JUST like this.  So we may want  a flag or state that can
            # be set to know we are no longer looking for finish boxes but
            # instead having a turn of 0 means the base of the stairs.
            if self.rail is not None:
                log("Try to get past the rail")
            elif self.flags & FLAG_STAIRS:
                log("===> Looks like the base of the stairs")
                distance = self._find_stair_base(cloud, pixel_distance,
                                                 image.shape[1]/2,
                                                 output['width'])
                if distance is not None:
                    path.distance = distance - 0.18 # Foot frame to Toe
                self.set_state(STATE_FINISH_BOX)
            else:
                log("===> Likely finish box in sight")
                path.distance += 1.5024
                self.set_state(STATE_FINISH_BOX)

        if path.distance > 0:
            self.backup_distance = 0

        if markup:
            txt = "({0:.2f}, {1})".format(path.distance, path.turn)
            cv2.putText(image, txt, (0, image.shape[0] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255))
        if debug:
            name = "_path_path_"
            cv2.namedWindow(name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(name, 500, 250)
            cv2.imshow(name, image)
            cv2.waitKey(1)
        if image_dump is not None:
            cv2.imwrite(image_dump + '/path_{0}.png'.format(debug_sequence()),
                        image)

        return path

    def analyze_path(self, data, silent=False):
        """
            This function needs more documentation because it is really the
            brains of the path decision making.

            This is called by zps.determine_path() which is what turns an image
            into raw pathing data then from there we need to figure out what the
            robot should do next.

            Note that at this point all values in the data are in pixels
        """
        path = self.PathDecision()

        if data is None:
            return path

        print data

        # Here is the beginning of the logic. First we see if we have both sides
        # shown. with situations like the trailer sometimes one half of the path
        # is obsucred.
        if data['right'] is not None and data['left'] is not None:
            # Finish box decision making! This is way more fragle than I had
            # hoped. Right now we look to see if the two edge end at ALMOST the
            # exact same place. 'distance' is in pixles here.

            # Fun trick,  turn the 4 points of the second segment into a contour
            # and check the area.  If that areas is 0 or near 0 then they are
            # coliner! of course we will never be near 0 so instead lets get
            # a ratio of area to width, then we can say a factor of X or under
            # is likely to be a path end.

            end = np.array([[data['left']['segments'][1][0]],
                            [data['left']['segments'][1][1]],
                            [data['right']['segments'][1][1]],
                            [data['right']['segments'][1][0]]])
            area = cv2.contourArea(end)
            width = abs(data['left']['segments'][1][0][0] - \
                        data['right']['segments'][1][0][0])

            if area/width < PATH_FORWARD_WIGGLE and \
               abs(data['left']['distance'] - data['right']['distance']) < 100:
                path.distance = data['right']['distance']
                path.turn = 0
                return path

            # We have not detected the end of a path. However the second
            # segments of each edge do not go in a common direction. This can
            # mean 2 things
            # 1) We are catching just the very edge of a turn so
            #    only half of the turn is in our image.
            # 2) Half of our path is being obscured by an obstical
            if np.sign(data['left']['segments'][1][0][0] - \
                       data['left']['segments'][1][1][0]) != \
               np.sign(data['right']['segments'][1][0][0] - \
                       data['right']['segments'][1][1][0]):

                if not silent:
                    log('Unknown turn: one side goes left the other '
                        'right')
                # CASE 1) report the distance as the shortest side we can
                # see and by having turn as None we signal the turn is
                # unknown
                path.distance = min(data['right']['distance'],
                                    data['left']['distance'])
                path.turn = None
                return path

            # The trick here is that paths are resolved in this order:
            # First) Adjust offest
            # Second) Walk distance
            # Third) turn feet

            # So if we have gotten here we have determined we can see both sides
            # of the path and it shows us a rational turn. We need to determin
            # what that turn is.  All of this code is written based on
            # examination of LEFT camera images only that means that 45 degree
            # turns to not look quite 45 degrees, also averaging left and right
            # sides of the path give a better picture
            theta = (data['left']['angle'] + data['right']['angle'])/2
            right = data['left']['segments'][1][0][0] > \
                    data['left']['segments'][1][1][0]
            ave_distance = (data['right']['distance'] + \
                            data['left']['distance'])/2
            min_distance = min(data['right']['distance'],
                               data['left']['distance'])
            # Ouch so these elif blocks are the result of if we only have one
            # side of the path (likely because of the trailer) and we have to
            # still make turn decisions. That is because SOMETIMES the trailer
            # actaully overlaps a turn so we cannot just march past is and look
            # again.
            #
            # FIXME: Becaue of angle calcuations are based on an averate of left
            # and right sides, just having one side really screws up turn
            # decisions. That needs to be figured out and fixed
        elif data['right'] is not None:
            if not self.flags & FLAG_STAIRS:
                if not self.flags & FLAG_ALLOW_HALF:
                    log("FAILED(): No path found")
                    return path
                theta = data['right']['angle']
                right = data['right']['segments'][1][0][0] > \
                        data['right']['segments'][1][1][0]
                ave_distance = data['right']['distance']
                min_distance = ave_distance
            else:
                path.distance = 0
                path.turn = None
                return path
        elif data['left'] is not None:
            if not self.flags & FLAG_STAIRS:
                if not self.flags & FLAG_ALLOW_HALF:
                    log("FAILED(): No path found")
                    return path
                theta = data['left']['angle']
                right = data['left']['segments'][1][0][0] > \
                        data['left']['segments'][1][1][0]
                ave_distance = data['left']['distance']
                min_distance = ave_distance
            else:
                path.distance = 0
                path.turn = None
                return path
        else:
            log("FAILED(): No path found")
            return path

        # Here is the mentioned turn decisions
        if theta > 25 and theta < 65:
            if right:
                path.turn = -45
            else:
                path.turn = 45
        elif theta > 70 and theta < 110:
            if right:
                path.turn = -90
            else:
                path.turn = 90

        if path.turn is None:
            raise ZarjConfused('FAILED(): Unknown angle ({0})'.format(theta))
        else:
            path.distance = ave_distance

        return path

    def path_start(self, debug=False, _=None, image=None):
        """
            First called when we dont know if we are in a finish box or
            on a path
        """
        self.stop = False
        self.zarj.walk.fix_stance()
        wait_for_walk(self.zarj)

        if self.flags & FLAG_HABITAT:
            self.clear_state(STATE_PRESTART)
            return

        self.zarj.walk.turn(1.0, 45) # Snap to multiple of 45 degrees.
        wait_for_walk(self.zarj)
        if image is None:
            image = self.zarj.navigation.get_navigation_image(self.crop)
        distance = self.zarj.navigation.look_for_exit(image.copy(), debug=debug,
                                                      unwarped=False)
        if distance is None:
            raise ZarjConfused("No starting path found! panic now?")
        if distance > 20:
            self.set_state(STATE_FINISH_BOX)
            self.exit_finish_box(debug, image)
        self.clear_state(STATE_PRESTART)

    def previous_path_point(self):
        """
            This is about un-walking a path point. So going backwards.
            This does not do anything if you are already at the beginning of the
            path
        """
        if len(self.path_stack) <= 1:
            log("Cannot pop an empty path")
            return

        path = self.path_stack.pop()
        path.distance *= -1
        path.turn *= -1

        if path.turn is not None and path.turn != 0:
            log("Un-Turning: {0}".format(path.turn))
            self.zarj.walk.turn(path.turn)
            wait_for_walk(self.zarj)

        if path.distance != 0:
            log("Un-Walking: {0}".format(path.distance))
            self.zarj.walk.forward(path.distance, square_up=True)
            wait_for_walk(self.zarj)

        self.center_on_offset(-path.offset)

        log("Returned to Path point {0}".format(len(self.path_stack)))

    def set_poi(self, point):
        """
            Set the point of interest
            a point of interest is a tuple of (distance, angle) or
            (distance, angle, offset)
            they are reletive to the robot at the moment they are set.

            It will be assumed that a poi is directly ahead of the robot
            and no fancy path finding is required to get there.
        """
        distance = point[0]
        if distance == 0.0:
            distance = 0.001
        if len(point) < 3:
            self.point_of_interest = self.PathDecision(distance, point[1])
        else:
            self.point_of_interest = self.PathDecision(distance, point[1],
                                                       point[2])

    def walk_to_poi(self, debug=False):
        """  Walk, keeping track of path, to the poi """
        if self.flags & FLAG_HABITAT:
            log('ZPS pathing is disabled in the habitat')
            return
        if self.stop:
            raise ZarjConfused("Asked to stop! ZPS halted."
                               "Use path_start to restart")
        if self.state & STATE_PRESTART:
            self.path_start(debug)
            self.point_of_interest.distance -= self.path_stack[-1].distance
        log('poi distance: {0}'.format(self.point_of_interest.distance))
        log('poi turn: {0}'.format(self.point_of_interest.turn))
        after_poi = self.look_for_next_path(debug)
        log('after poi distance: {0}'.format(after_poi.distance))
        log('afterpoi turn: {0}'.format(after_poi.turn))
        if after_poi.distance > self.point_of_interest.distance:
            log('poi first then next step')
            after_poi.distance -= self.point_of_interest.distance
            self.next_path_point(debug, self.point_of_interest)
            self.after_poi = [after_poi]
        else:
            log('figure next step first then poi')
            self.point_of_interest.distance -= after_poi.distance
            if self.point_of_interest.distance == 0.0:
                self.point_of_interest.distance = 0.001

            self.next_path_point(debug, after_poi)
            # unturn the next path
            #
            #  Yes, this is a lot of unecessary turning. Optimizations welcome
            #  just dont break the path stack
            #
            if after_poi.turn:
                unturn = self.PathDecision(0.001, -after_poi.turn)
                self.next_path_point(debug, unturn)
            # walk to poi
            self.next_path_point(debug, self.point_of_interest)
            # build return path
            self.after_poi = []
            unwalk = self.PathDecision(-self.point_of_interest.distance, 0)
            self.after_poi.append(unwalk)
            if after_poi.turn:
                turnagain = self.PathDecision(0.001, after_poi.turn)
                self.after_poi.append(turnagain)
        log('Reached point of interest')
        self.set_state(STATE_POI)

    def continue_from_poi(self, debug=False):
        """ Continue walking from a poi """
        if self.flags & FLAG_HABITAT:
            log('ZPS pathing is disabled in the habitat')
            return
        if self.stop:
            raise ZarjConfused("Asked to stop! ZPS halted."
                               "Use path_start to restart.")
        if not self.state & STATE_POI or self.point_of_interest is None:
            return
        if self.point_of_interest.turn:
            unturn = self.PathDecision(0.001, -self.point_of_interest.turn)
            self.next_path_point(debug, unturn)
        if self.after_poi:
            for path in self.after_poi:
                self.next_path_point(debug, path)
        self.after_poi = None
        self.point_of_interest = None
        self.clear_state(STATE_POI)

    def dead_walk(self, point, debug=False):
        """
            Dead walk to a point, also resets the state and poi.
            the point is (distance, turn, offset)
            Rember that walking is done in the order of offset, then distance,
            then turn.

            To Resume pathing you should be able to just call
            zps.next_path_point() to continue on the path.
        """
        self.state = 0
        self.point_of_interest = None
        self.after_poi = None
        distance = point[0]
        if distance == 0:
            distance = 0.001
        if len(point) < 3:
            path = self.PathDecision(distance, point[1])
        else:
            path = self.PathDecision(distance, point[1], point[2])
        self.next_path_point(debug, path, True)

    def one_stair(self, run, rise):
        """ Climb 1 stair """
        cur_lf_x = self.zarj.walk.get_oriented_lf_forward_distance()
        # self.zarj.walk.turn(0.0, snap_to = 45, small_forward = small_forward)
        # wait_for_walk(self.zarj)
        self.zarj.walk.step_up(run, rise)

        wait_for_walk(self.zarj)

        # Putting the revert to "home" pose here before the "orientation, small move" realignment
        # seems to do two things. One if there is foot slippage, then waiting a little bit
        # of time allows us to get an accurate reading on the current position of the left foot,
        # otherwise the realignment may believe the left foot is more forward than it actually is.
        # Second, by doing the "step-step" motion before taking the next step up seems to reduce
        # the chance for right foot drag when going to the next step.
        rospy.sleep(0.2)
        self.zarj.go_home(0, 2)
        rospy.sleep(0.2)

        # Make sure we actually went as far forward as desired and our feet
        # are properly fixed up after taking the step.
        new_lf_x = self.zarj.walk.get_oriented_lf_forward_distance()
        actual_run = new_lf_x - cur_lf_x
        diff = run - actual_run
        if diff < -0.002:
            log("Strangely overstepped by {}, this should not be possible, sleeping for rospy 1 second".format(
                fmt(diff)
            ))
            rospy.sleep(1)
            new_lf_x2 = self.zarj.walk.get_oriented_lf_forward_distance()
            actual_run = new_lf_x2 - cur_lf_x
            diff = run - actual_run
            if abs(new_lf_x2 - new_lf_x) > 0.002:
                log("Sleeping materially changed result to new missed by value of {}.".format(fmt(diff)))

        if diff > 0.003:
            log("Desired step forward missed by {} on its *run*.".format(diff))
        # If we managed to do a full step, that probably indicates that we are not fully up against the stair
        # and need to move forward more. This is a change from a prior view where we assumed a small diff
        # indicated a good thing.
        small_forward = 0.02 if diff < 0.002 else 0.008
        self.zarj.walk.turn(0.0, snap_to = 45, small_forward = small_forward, report = False, align_to_snap = True)
        wait_for_walk(self.zarj)

        new_lf_x2 = self.zarj.walk.get_oriented_lf_forward_distance()
        small_forward2 = run - (new_lf_x2 - cur_lf_x)
        log("New missed by is {}.".format(fmt(small_forward2)))
        if small_forward2 > diff + 0.005:
            log("Applying adjustment again.")
            self.zarj.walk.turn(0.0, snap_to = 45, small_forward = small_forward2, report = False,
                                align_to_snap = True)
            wait_for_walk(self.zarj)

        if self.stop:
            raise ZarjConfused("Asked to stop! ZPS halted."
                               "Use path_start to restart")



    def climb_stairs(self):
        """
            This is base code to climb the stairs to the top...
        """
        if self.stop:
            raise ZarjConfused("Asked to stop! ZPS halted."
                               "Use path_start to restart")
        self.zarj.walk.set_swing_time(1.0)
        self.zarj.walk.set_transfer_time(0.5)
        self.zarj.walk.set_stance_width(0.25)
        # This I am positive of from the model
        step = 0.2031
        first_step = 0.1129
        last_step = 0.1804
        depth = 0.2388 + 0.012 # yeah step a little longer than the footbed
        # end being positive
        self.zarj.walk.fix_stance()
        # self.zarj.walk.turn(0)
        wait_for_walk(self.zarj)
        #self.zarj.pelvis.lean_body_to(-20)
        self.zarj.pelvis.lean_body_to(-25)
        self.zarj.pelvis.change_height(0.15)
        rospy.sleep(0.2)

        self.one_stair(depth + 0.05, first_step)
        log('*** Step 1')

        self.one_stair(depth + 0.02, step)
        log('*** Step 2')
        self.one_stair(depth, step)
        log('*** Step 3')
        self.one_stair(depth, step)
        log('*** Step 4')
        self.one_stair(depth, step)
        log('*** Step 5')
        self.one_stair(depth, step)
        log('*** Step 6')
        self.one_stair(depth, step)
        log('*** Step 7')
        self.one_stair(depth, step)
        log('*** Step 8')

        self.one_stair(depth + 0.05, last_step)
        log('*** Stairs done!')
        self.zarj.pelvis.lean_body_to(0)
        self.zarj.pelvis.change_height(-0.15)
        rospy.sleep(0.2)

    def all_stop(self):
        """ The all stop. try to stop everything you are doing """
        self.stop = True
