"""
    Classes for picking up things from the world
"""
import sys
import cv2
from zarj.utils import is_cv2
from math import pi, sin, cos, isnan, sqrt
import numpy as np
from copy import deepcopy

__all__ = ['Thing', 'Things']

class Thing(object):
    """ This is one 'thing' in the world
        The position, big_detector_screen, and button fields provide a (x,y)
        index into details where there is a valid x/y/z coordinate.
        You can transform them as you will.
        Length and width should be in meters.
        Angle is 0 for facing right, -90 for upright, and so on. """

    UNKNOWN = 0
    LEAK_DETECTOR = 1
    REPAIR_TOOL = 2
    CHOKE_INNER = 3
    CHOKE_OUTER = 4
    POWER_PLUG = 5
    ARRAY_BUTTON = 6
    REPAIR_TOOL_BUTTON = 7

    def __init__(self, length, width, position, angle):
        self.type = self.UNKNOWN
        self.length = length
        self.width = width
        self.position = position
        self.computed_center = None
        self.angle = angle

    def set_detector(self, center, up):
        self.type = self.LEAK_DETECTOR
        self.big_detector_screen = center
        if not up:
            self.angle -= 180

    def set_repair_tool(self, button, up):
        self.type = self.REPAIR_TOOL
        self.button = button
        if not up:
            self.angle -= 180

    def set_choke_inner(self):
        self.type = self.CHOKE_INNER

    def set_choke_outer(self):
        self.type = self.CHOKE_OUTER

    def set_power_plug(self):
        self.type = self.POWER_PLUG

    def set_button_fragment(self):
        self.type = self.BUTTON_FRAGMENT

    def set_array_button(self):
        self.type = self.ARRAY_BUTTON

    def set_repair_button(self):
        self.type = self.REPAIR_TOOL_BUTTON


class Things(object):
    """ This finds multiple things in the world, and remembers them """

    MINIMUM_TOOL_WIDTH = 0.05
    MAXIMUM_TOOL_WIDTH = 0.12

    MINIMUM_TOOL_LENGTH = 0.15
    MAXIMUM_TOOL_LENGTH = 0.45

    TOOL_MUST_BE_WITHIN = 2.0

    def __init__(self, img, details, task=3, debug=False):
        self.img = img
        self.details = details
        self.things = []
        self.leak_detector = None
        self.repair_tool = None
        self.choke_inner = None
        self.choke_outer = None
        self.power_plug = None
        self.array_button = None
        self.repair_button = None
        self.debug = debug
        if task == 3:
            self._find_habitat_things()
        if task == 2:
            self._find_solar_box_things()

    def is_detector(self, img):
        """ This uses color to determine if we have a detector, and if so, returns where
            the big screen and smaller screen is in the subimage """
        lower = np.array([190, 190, 0], dtype = "uint8")
        upper = np.array([255, 255, 100], dtype = "uint8")
        mask = cv2.inRange(img, lower, upper)

        contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = contours[0] if is_cv2() else contours[1]
        if len(contours) == 0:
            return None, False

        sorted_contours = sorted(contours, cmp=lambda a,b: int(cv2.contourArea(b)) - int(cv2.contourArea(a)))
        center, radius = cv2.minEnclosingCircle(sorted_contours[0])
        up = True
        if len(contours) > 1:
            center2, radius = cv2.minEnclosingCircle(sorted_contours[1])
            if center2[1] < center[1]:
                up = False

        if self.debug:
            debug_img = img.copy()
            cv2.drawContours(debug_img, [sorted_contours[0]], -1, (0, 255, 0), 2)
            cv2.imshow("cont", debug_img)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

        return center, up

    def is_repair_tool(self, img):
        """ This uses color to determine if we have a repair tool, and if so, returns where
            the button is located within the provided subimage """
        lower = np.array([190, 0, 0], dtype = "uint8")
        upper = np.array([255, 125, 100], dtype = "uint8")
        mask = cv2.inRange(img, lower, upper)

        contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = contours[0] if is_cv2() else contours[1]
        if len(contours) == 0:
            return None, False

        sorted_contours = sorted(contours, cmp=lambda a,b: int(cv2.contourArea(b)) - int(cv2.contourArea(a)))
        center, radius = cv2.minEnclosingCircle(sorted_contours[0])
        up = True
        if center[1] > (img.shape[0] / 2):
            up = False

        if self.debug:
            debug_img = img.copy()
            cv2.drawContours(debug_img, [sorted_contours[0]], -1, (0, 255, 0), 2)
            cv2.imshow("cont", debug_img)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

        return center, up


    def find_valid_central_point(self, startx, starty):
        """ Given the center of an object, hunt around until we find a place
            that gives us a 'real world' coordinate from the stereo details """
        for i in range(25):
            if (starty + i) < self.details.shape[0] and (startx + i < self.details.shape[1]):
                if not (isnan(self.details[starty + i, startx + i][0]) or
                    isnan(self.details[starty + i, startx + i][1]) or
                    isnan(self.details[starty + i, startx + i][2])):
                   return (startx + i, starty + i)

            if (starty - i) >= 0 and (startx - i) >= 0:
                if not (isnan(self.details[starty - i, startx - i][0]) or
                    isnan(self.details[starty - i, startx - i][1]) or
                    isnan(self.details[starty - i, startx - i][2])):
                   return (startx - i, starty - i)
        return None


    def find_valid_point(self, startx, starty, stopx, stopy, vertical):
        """ Given a line that bisects an object, travel along that line until
            we find a place where we get non nan coordinates from the stereo image """
        maxy = self.details.shape[0]
        maxx = self.details.shape[1]

        startx = max(startx, 0)
        startx = min(startx, maxx - 1)
        stopx = max(stopx, 0)
        stopx = min(stopx, maxx - 1)

        starty = max(starty, 0)
        starty = min(starty, maxy - 1)
        stopy = max(stopy, 0)
        stopy = min(stopy, maxy - 1)

        h = stopy - starty
        w = stopx - startx

        x = startx
        y = starty

        horizrange = range(startx, stopx + 1) if w >= 0 else range(stopx, startx + 1)
        vertrange = range(starty, stopy + 1) if h >= 0 else range(stopy, starty + 1)

        while x in horizrange and y in vertrange:
            if not (isnan(self.details[y, x][0]) or
                isnan(self.details[y, x][1]) or
                isnan(self.details[y, x][2])):
               return (x, y)

            if vertical:
                if h == 0:
                    return None
                y += 1 if h > 0 else -1
                x = startx + int((y - starty) * w / h)
            else:
                if w == 0:
                    return None
                x += 1 if w > 0 else -1
                y = starty + int((x - startx) * h / w)

        return None


    def find_distance(self, pointa, pointb):
        """ Find the distance between two objects, or how far one object is away """
        if pointb is None:
            deltax = self.details[pointa[1], pointa[0]][0]
            deltay = self.details[pointa[1], pointa[0]][1]
            deltaz = self.details[pointa[1], pointa[0]][2]
        else:
            deltax = self.details[pointa[1], pointa[0]][0] - self.details[pointb[1], pointb[0]][0]
            deltay = self.details[pointa[1], pointa[0]][1] - self.details[pointb[1], pointb[0]][1]
            deltaz = self.details[pointa[1], pointa[0]][2] - self.details[pointb[1], pointb[0]][2]

        return sqrt((deltax * deltax) + (deltay * deltay) + (deltaz * deltaz))

    def compute_center(self, left, right, top, bottom):
        """ If we have a contour with good edges but a bad center, use the edges to compute a center """
        minx = min(self.details[left[1], left[0]][0],
                   self.details[right[1], right[0]][0],
                   self.details[top[1], top[0]][0],
                   self.details[bottom[1], bottom[0]][0])
        maxx = max(self.details[left[1], left[0]][0],
                   self.details[right[1], right[0]][0],
                   self.details[top[1], top[0]][0],
                   self.details[bottom[1], bottom[0]][0])
        miny = min(self.details[left[1], left[0]][1],
                   self.details[right[1], right[0]][1],
                   self.details[top[1], top[0]][1],
                   self.details[bottom[1], bottom[0]][1])
        maxy = max(self.details[left[1], left[0]][1],
                   self.details[right[1], right[0]][1],
                   self.details[top[1], top[0]][1],
                   self.details[bottom[1], bottom[0]][1])
        avg_z = (self.details[left[1], left[0]][2] +
                   self.details[right[1], right[0]][2] +
                   self.details[top[1], top[0]][2] +
                   self.details[bottom[1], bottom[0]][2]) / 4.0

        return [ (minx + maxx) / 2, (miny + maxy) / 2, avg_z ]

    def find_dimensions(self, points):
        """ Find the top, bottom, sides, and center of a interesting objects """
        box = sorted(points, cmp=lambda a,b: cmp(a[1], b[1]) or cmp(a[0], b[0]))
        topx = (box[0][0] + box[1][0]) / 2
        topy = (box[0][1] + box[1][1]) / 2

        bottomx = (box[2][0] + box[3][0]) / 2
        bottomy = (box[2][1] + box[3][1]) / 2

        rightx = (box[1][0] + box[3][0]) / 2
        righty = (box[1][1] + box[3][1]) / 2

        leftx = (box[0][0] + box[2][0]) / 2
        lefty = (box[0][1] + box[2][1]) / 2

        centerx = (topx + bottomx + rightx + leftx) / 4
        centery = (topy + bottomy + righty + lefty) / 4

        top = self.find_valid_point(topx, topy, bottomx, bottomy, True)
        bottom = self.find_valid_point(bottomx, bottomy, topx, topy, True)
        left = self.find_valid_point(leftx, lefty, rightx, righty, False)
        right = self.find_valid_point(rightx, righty, leftx, lefty, False)
        center = self.find_valid_central_point(centerx, centery)

        return (top, bottom, left, right, center)

    def _union_box(self, a, b):
        x = min(a[0], b[0])
        y = min(a[1], b[1])
        w = max(a[0]+a[2], b[0]+b[2]) - x
        h = max(a[1]+a[3], b[1]+b[3]) - y
        return (x, y, w, h)

    def _bound_to_boxpoints(self, box):
        x = box[0]
        y = box[1]
        w = box[2]
        h = box[3]
        return ( (x, y), (x + w, y), (x + w, y + h), (x, y + h))

    def handle_mouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONUP:
            print x
            print y
            print self.img[y, x]

    def _find_array_button_thing(self):
        """ Find the array button on the solar array box """

        """ This uses color to determine if we have a choke """
        lower = np.array([0, 0, 60], dtype = "uint8")
        upper = np.array([20, 20, 255], dtype = "uint8")
        mask = cv2.inRange(self.img, lower, upper)

        blurred = cv2.GaussianBlur(mask, (5, 5), 0)
        thresh = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY)[1]

        contours = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = contours[0] if is_cv2() else contours[1]

        debug_img = None
        if self.debug:
            debug_img = self.img.copy()

        button_box = None
        for c in contours:
            box = cv2.boundingRect(c)

            if button_box is None:
                button_box = box
            else:
                button_box = self._union_box(deepcopy(button_box), box)

        if button_box is None:
            return

        top,bottom,left,right,center = self.find_dimensions(np.int0(np.array(self._bound_to_boxpoints(button_box))))
        if top is None or left is None or center is None:
            return None

        height = self.find_distance(top, bottom)
        width = self.find_distance(left, right)

        if self.debug:
            for c in contours:
                cv2.drawContours(debug_img, [c], -1, (0, 255, 0), 2)

            cv2.circle(debug_img, top, 5, (255, 255, 0))
            cv2.circle(debug_img, bottom, 5, (255, 255, 0))
            cv2.circle(debug_img, left, 5, (255, 255, 0))
            cv2.circle(debug_img, right, 5, (255, 255, 0))
            cv2.rectangle(debug_img, (button_box[0], button_box[1]),
                    (button_box[0] + button_box[2], button_box[1] + button_box[3]), (128, 0, 128), 2)
            #cv2.circle(debug_img, center, 5, (255, 255, 0))

            cv2.imshow("button picture", debug_img)
            cv2.setMouseCallback("button picture", self.handle_mouse)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

        self.array_button = Thing(height, width, center, None)
        self.array_button.set_array_button()
        self.array_button.computed_center = self.compute_center(left, right, top, bottom)
        self.things.append(self.array_button)

    def _find_power_plug_thing(self):
        """ Find the power plug at the solar array box """

        """ This uses color to determine if we have a choke """
        lower = np.array([100, 40, 0], dtype = "uint8")
        upper = np.array([255, 255, 20], dtype = "uint8")
        mask = cv2.inRange(self.img, lower, upper)

        blurred = cv2.GaussianBlur(mask, (5, 5), 0)
        thresh = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY)[1]

        contours = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = contours[0] if is_cv2() else contours[1]

        sorted_contours = sorted(contours, cmp=lambda a,b: int(cv2.contourArea(b)) - int(cv2.contourArea(a)))

        if len(sorted_contours) > 0:
            plug = self._find_a_thing(sorted_contours[0], 0, 0.06, 0, 0.06, 99.0)

            if plug is not None:
                plug.set_power_plug()
                self.things.append(plug)
                self.power_plug = plug

                if self.debug:
                    debug_img = self.img.copy()
                    for c in sorted_contours:
                        cv2.drawContours(debug_img, [c], -1, (0, 255, 0), 2)
                    cv2.imshow("plug picture", debug_img)
                    cv2.setMouseCallback("plug picture", self.handle_mouse)
                    cv2.waitKey(0)
                    cv2.destroyAllWindows()

    def _find_choke_thing(self):
        """ Find the choke at the solar array box """

        """ This uses color to determine if we have a choke """
        lower = np.array([128, 0, 0], dtype = "uint8")
        upper = np.array([255, 10, 20], dtype = "uint8")
        mask = cv2.inRange(self.img, lower, upper)

        blurred = cv2.GaussianBlur(mask, (5, 5), 0)
        thresh = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY)[1]

        contours = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = contours[0] if is_cv2() else contours[1]

        # Note: you can get your view of the choke cut; so you see an end as 2 contours.
        #  In a dream world, we would combine two small controus close together.
        #  But for now, we just operate with the two biggest contours
        sorted_contours = sorted(contours, cmp=lambda a,b: int(cv2.contourArea(b)) - int(cv2.contourArea(a)))

        if len(sorted_contours) >= 2:
            thing1 = self._find_a_thing(sorted_contours[0], 0, 0.1, 0, 0.1, 99.0)
            thing2 = self._find_a_thing(sorted_contours[1], 0, 0.1, 0, 0.1, 99.0)

            if thing1 is not None and thing2 is not None:
                print "We have a choke!"
                box1 = cv2.boundingRect(sorted_contours[0])
                box2 = cv2.boundingRect(sorted_contours[1])
                if box1[0] > box2[0]:
                    inner = thing1
                    outer = thing2
                else:
                    inner = thing2
                    outer = thing1

                inner.set_choke_inner()
                self.things.append(inner)
                self.choke_inner = inner
                outer.set_choke_outer()
                self.things.append(outer)
                self.choke_outer = outer

                if self.debug:
                    debug_img = self.img.copy()
                    for c in sorted_contours:
                        cv2.drawContours(debug_img, [c], -1, (0, 255, 0), 2)
                    cv2.imshow("choke picture", debug_img)
                    cv2.setMouseCallback("choke picture", self.handle_mouse)
                    cv2.waitKey(0)
                    cv2.destroyAllWindows()

    def _find_repair_button_thing(self):
        """ Find the repairt button in front of us """

        """ This uses color to determine if we have a repair button"""
        lower = np.array([190, 0, 0], dtype = "uint8")
        upper = np.array([255, 150, 20], dtype = "uint8")
        mask = cv2.inRange(self.img, lower, upper)

        blurred = cv2.GaussianBlur(mask, (5, 5), 0)
        thresh = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY)[1]

        contours = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = contours[0] if is_cv2() else contours[1]

        debug_img = None
        if self.debug:
            debug_img = self.img.copy()
            cv2.imshow("button mask", thresh)
            cv2.setMouseCallback("button mask", self.handle_mouse)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

        button_box = None
        for c in contours:
            box = cv2.boundingRect(c)

            if button_box is None:
                button_box = box
            else:
                button_box = self._union_box(deepcopy(button_box), box)

        if button_box is None:
            print "No button box"
            return

        top,bottom,left,right,center = self.find_dimensions(np.int0(np.array(self._bound_to_boxpoints(button_box))))
        if top is None or left is None or center is None:
            print "Missing one of bounds.  {}/{}/{}".format(top, left, center)
            return None

        height = self.find_distance(top, bottom)
        width = self.find_distance(left, right)

        if self.debug:
            for c in contours:
                cv2.drawContours(debug_img, [c], -1, (0, 255, 0), 2)

            cv2.circle(debug_img, top, 5, (255, 255, 0))
            cv2.circle(debug_img, bottom, 5, (255, 255, 0))
            cv2.circle(debug_img, left, 5, (255, 255, 0))
            cv2.circle(debug_img, right, 5, (255, 255, 0))
            cv2.rectangle(debug_img, (button_box[0], button_box[1]),
                    (button_box[0] + button_box[2], button_box[1] + button_box[3]), (128, 0, 128), 2)
            #cv2.circle(debug_img, center, 5, (255, 255, 0))

            cv2.imshow("repair button picture", debug_img)
            cv2.setMouseCallback("repair button picture", self.handle_mouse)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

        self.repair_button = Thing(height, width, center, None)
        self.repair_button.set_repair_button()
        self.repair_button.computed_center = self.compute_center(left, right, top, bottom)
        self.things.append(self.repair_button)

    def _find_solar_box_things(self):
        self._find_array_button_thing()
        self._find_choke_thing()
        self._find_power_plug_thing()

    def _find_a_thing(self, c, min_height, max_height, min_width, max_width, max_distance, debug_img=None):
        rect = cv2.minAreaRect(c)
        box = cv2.cv.BoxPoints(rect) if is_cv2() else cv2.boxPoints(rect)

        top,bottom,left,right,center = self.find_dimensions(np.int0(np.array(box)))

        if top is None or left is None or center is None:
            return None

        vertical = self.find_distance(top, bottom)
        horizontal = self.find_distance(left, right)
        away = self.find_distance(center, None)

        if vertical > horizontal:
            height = vertical
            width = horizontal
            flipped = False
        else:
            height = horizontal
            width = vertical
            flipped = True

        if height < min_height or height > max_height:
            return None

        if width < min_width or width > max_height:
            return None

        if away > max_distance:
            return None

        # This page was helpful in understanding angle
        # https://namkeenman.wordpress.com/2015/12/18/open-cv-determine-angle-of-rotatedrect-minarearect/
        angle = rect[2]
        if rect[1][0] < rect[1][1]:
            angle -= 90.0

        if debug_img is not None:
            x,y,w,h = cv2.boundingRect(c)
            cv2.drawContours(debug_img, [c], -1, (0, 255, 0), 2)
            cv2.drawContours(debug_img, [np.int0(np.array(box))], -1, (0, 0, 255), 2)
            cv2.rectangle(debug_img,(x,y),(x+w,y+h),(255,0,0),2)

            cv2.circle(debug_img, top, 5, (255, 255, 0))
            cv2.circle(debug_img, bottom, 5, (255, 255, 0))
            cv2.circle(debug_img, left, 5, (255, 255, 0))
            cv2.circle(debug_img, right, 5, (255, 255, 0))
            cv2.circle(debug_img, center, 5, (255, 255, 0))


        return Thing(height, width, center, angle)


    def _find_habitat_things(self):
        """ Find interesting objects in the habitat world """
        self._find_repair_button_thing()

        # Start by grey scale, blur, and thresholding
        gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        thresh = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY)[1]

        # find contours in the thresholded image
        contours = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = contours[0] if is_cv2() else contours[1]

        debug_img = self.img.copy()

        for c in contours:
            thing = self._find_a_thing(c, self.MINIMUM_TOOL_LENGTH, self.MAXIMUM_TOOL_LENGTH,
                                     self.MINIMUM_TOOL_WIDTH, self.MAXIMUM_TOOL_WIDTH,
                                     self.TOOL_MUST_BE_WITHIN, debug_img)
            if thing is None:
                continue

            x,y,w,h = cv2.boundingRect(c)
            detector_center, up = self.is_detector(self.img[y:y+h,x:x+w].copy())

            if detector_center is not None:
                adjusted_detector = (int(detector_center[0] + x), int(detector_center[1] + y))
                thing.set_detector(adjusted_detector, up)
                self.things.append(thing)
                self.leak_detector = thing

            repair_center, up = self.is_repair_tool(self.img[y:y+h,x:x+w].copy())
            if repair_center is not None:
                adjusted_repair = (int(repair_center[0] + x), int(repair_center[1] + y))
                thing.set_repair_tool(adjusted_repair, up)
                self.things.append(thing)
                self.repair_tool = thing



        if self.debug:
            for c in contours:
                cv2.drawContours(debug_img, [c], -1, (0, 255, 0), 2)
            cv2.imshow("habitat things", debug_img)
            cv2.setMouseCallback("habitat things", self.handle_mouse)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
