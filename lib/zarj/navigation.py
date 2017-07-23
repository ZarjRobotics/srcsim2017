"""
    Process for trying to determine path direction

    Some notes with default head position
        row 5 is 2.84 m  away
        row 10 is 2.86 m away

    Looking down (0.5 upper neck pitch *NOT HEAD*)
        row 10 is about 1m away
        bottom of image is about 5m away

    Note that each path square is about 1.525m square
    Looking down (0.5) the end of the tile we are standing on is at row 92

    The 90 degree turn tile will show its turn at about row 140


    More random notes.
        Head looking down (0.5)
        Torso Lean 0 degrees
            furthest row :  ???
            closest row : 1.05 m

     M = [[  1.73853990e+00  -6.35973235e-01  -4.05079796e+02]
          [ -4.51028104e-16   2.88624788e-01  -3.90798505e-14]
          [  8.67361738e-19  -1.30767502e-03   1.00000000e+00]]

        Head looking down (0.5)
        Torso Lean 15 degrees
            furthest row : 3.0 m
            closest row : 0.762 m

      M = [[  1.00000000e+00  -4.49195400e-01   1.23238225e-13]
           [ -1.47320163e-15   5.00736377e-01   2.09845813e-13]
           [  8.46596196e-19  -9.17764013e-04   1.00000000e+00]]

    More Notes:
        A single path tile is 1.524m square
        A single path tile has 12 hexs on it
        FINISH_BOX_SIZE = 3.048m
"""
import cv2
import os
import numpy as np
from .utils import is_cv2, log, debug_sequence

__all__ = ['Navigation']

_LEFT_IS_90 = lambda x: x < 75 and x > 65
_RIGHT_IS_90 = lambda x: x < 110 and x > 100

INTERSECT_LEFT = 0
INTERSECT_RIGHT = 1

INTERSECT_CONTOUR_INDEX = 0
INTERSECT_COLUMN = 1

BASE_ROW = 10

# determined by examing the model
METERS_PER_HEX = 0.127

R5_WIDTH = 1.5 # NEED TO LOOK THIS UP AND VERIFY IT

class NavPerspective(object):
    """ Point of Persective for Navigation Images """

    def __init__(self, mat, rows, zero, col):
        self.map_matrix = mat
        self.rows_per_hex = rows
        self.m_row_zero = zero
        self.m_per_col = col
        self.m_per_row = METERS_PER_HEX / self.rows_per_hex

    def do_unwarp(self, where):
        """ do the opencv2 unwarp """
        point = np.array([[where[0], where[1]]], dtype='float32')
        apoint = np.array([point])
        turn = cv2.perspectiveTransform(apoint, self.map_matrix)
        return self.m_row_zero + (turn[0][0][1] * self.m_per_row)

    def build_rangefinding_image(self, image, leader_line=40, increment=10,
                                 h_rule=False):
        """ Build a rangefinding image """
        img = cv2.flip(image, -1)
        cv2.line(img, (img.shape[1]/2, 0), (img.shape[1]/2, img.shape[0]),
                 (0, 255, 0), 3)
        horizon = 0
        for row in range(1, image.shape[0]/increment):
            base_row = (row * increment)
            dist = self.do_unwarp((image.shape[1]/2, base_row))
            if dist < 0:
                cv2.line(img, (img.shape[1]/2-leader_line,
                               image.shape[0] - base_row),
                         (img.shape[1]/2+leader_line,
                          image.shape[0] - base_row), (0, 0, 255), 5)
                horizon = base_row - increment
                break
            txt = "{0:.2f}m".format(dist)
            size = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0]
            if row % 2:
                cv2.line(img, (img.shape[1]/2, image.shape[0] - base_row),
                         (img.shape[1]/2+leader_line,
                          image.shape[0] - base_row), (0, 255, 0), 1)
                cv2.putText(img, txt, (img.shape[1]/2 + leader_line + 5,
                                       (image.shape[0] - base_row)+size[1]/2),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0))
            else:
                cv2.line(img, (img.shape[1]/2, image.shape[0] - base_row),
                         (img.shape[1]/2-leader_line, image.shape[0] - base_row), (0, 255, 0), 1)
                cv2.putText(img, txt, (img.shape[1]/2 - \
                                      (leader_line + 5 + size[0]),
                                       (image.shape[0] - base_row)+size[1]/2),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0))

        if horizon == 0:
            horizon = image.shape[0]

        center = image.shape[1]/2 + int(0.035 / self.m_per_col)
        width = int(R5_WIDTH / self.m_per_col)/2
        point = np.array([[center+width, horizon], [center-width, horizon]],
                         dtype='float32')
        apoint = np.array([point])
        turn = cv2.perspectiveTransform(apoint, np.linalg.inv(self.map_matrix))
        far_width = int(abs(turn[0][0][0] - turn[0][1][0])/2)
        cv2.line(img, (center+width, image.shape[0]),
                 (center+far_width, image.shape[0] - horizon),
                 (0, 0, 255), 1)
        cv2.line(img, (center-width, image.shape[0]),
                 (center-far_width, image.shape[0] - horizon),
                 (0, 0, 255), 1)

        if h_rule:
            img = cv2.copyMakeBorder(img, 0, 40, 0, 0,
                                     cv2.BORDER_CONSTANT, value=(0, 0, 0))

            for col in range(1, image.shape[1]/80):
                base_col = image.shape[1]/2 + (col * 40)
                cv2.line(img, (base_col, img.shape[0] - 20), (base_col,
                                                              img.shape[0] - 40),
                         (0, 255, 0), 1)
                txt = "{0:.2f}m".format(self.m_per_col*col*40)
                size = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, 0.3, 1)[0]
                cv2.putText(img, txt, (base_col - (size[0]/2), img.shape[0] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 255, 0))

                base_col = image.shape[1]/2 - (col * 40)
                cv2.line(img, (base_col, img.shape[0] - 20), (base_col,
                                                              img.shape[0] - 40),
                         (0, 255, 0), 1)
                txt = "{0:.2f}m".format(self.m_per_col*col*-40)
                size = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, 0.3, 1)[0]
                cv2.putText(img, txt, (base_col - (size[0]/2), img.shape[0] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 255, 0))

        return img

## values for a 15 degree torso lean along with the 0.5 neck pitch
PERSPECTIVE_FULL_DOWN = NavPerspective(
    np.array([[1.00000000e+00, -4.61397059e-01, -7.90085228e-14],
              [-2.50209721e-16, 5.09765625e-01, -4.66471479e-14],
              [9.72230042e-19, -9.01166131e-04, 1.00000000e+00]],
             dtype="float32"), 30.0, 0.762, 1.524 / 678.0)

# values represent 0.5 neck pitch no body lean
PERSPECTIVE_HEAD_DOWN = NavPerspective(
    np.array([[1.00000000e+00, -7.00367647e-01, -1.89796638e-13],
              [-6.64512594e-16, 2.55859375e-01, 1.68287259e-13],
              [-2.59011591e-18, -1.36790556e-03, 1.00000000e+00]],
             dtype='float32'), 12.0, 1.15, 1.524 / 570.0)

PERSPECTIVE_AHEAD = NavPerspective(
    np.array([[1.00000000e+00, -4.45402299e-01, -1.66446718e-15],
              [7.05002463e-17, 6.06060606e-03, -3.84848403e-15],
              [-1.99461515e-16, -2.85614768e-03, 1.00000000e+00]],
             dtype='float32'), 0.095, 2.67, 1.524 / 330.0)

STD_UPPER_GRAY = np.array([0, 0, 95])
ALT_UPPER_GRAY = np.array([0, 0, 155])

class Navigation(object):
    """ Navigation unit """

    lower_gray = np.array([0, 0, 60])

    perspective = PERSPECTIVE_FULL_DOWN

    def __init__(self, zarj=None):
        super(Navigation, self).__init__()
        self.zarj = zarj
        self.upper_gray = STD_UPPER_GRAY

    @staticmethod
    def find_contours(mask, smooth_factor=0.005):
        """ Find the contours in a given mask """
        border = 5
        # Canny detection breaks down with the edge of the image
        my_mask = cv2.copyMakeBorder(mask, border, border, border, border,
                                     cv2.BORDER_CONSTANT, value=(0, 0, 0))

        my_mask = cv2.cvtColor(my_mask, cv2.COLOR_BGR2GRAY)

        if is_cv2():
            contours, _ = cv2.findContours(my_mask, cv2.RETR_EXTERNAL,
                                           cv2.CHAIN_APPROX_SIMPLE)
        else:
            _, contours, _ = cv2.findContours(my_mask, cv2.RETR_EXTERNAL,
                                              cv2.CHAIN_APPROX_SIMPLE)

        # shift the contours back down
        for contour in contours:
            for pnt in contour:
                if pnt[0][1] > border:
                    pnt[0][1] = pnt[0][1] - border
                else:
                    pnt[0][1] = 0
                if pnt[0][0] > border:
                    pnt[0][0] = pnt[0][0] - border
                else:
                    pnt[0][0] = 0

        closed_contours = []
        for contour in contours:
            epsilon = smooth_factor*cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            area = cv2.contourArea(approx)
            # if they are too small they are not edges
            if area < 200:
                continue
            closed_contours.append(approx)

        return closed_contours

    @staticmethod
    def find_intersect(image, contours, row, direction, center_col=None):
        """
            Find the intersection from a given centerline to the first
            contours to the left and right
        """
        if center_col is not None:
            col = center_col
        else:
            col = image.shape[1] / 2
        intersect = None
        i_contour = None
        while intersect is None:
            for i, contour in enumerate(contours):
                if cv2.pointPolygonTest(contour, (col, row), False) >= 0:
                    intersect = col
                    i_contour = i
                    break
            col = col + direction
            if col < 0 or col > image.shape[1]:
                break

        return i_contour, intersect

    @staticmethod
    def forward_intersect(image, contours, center_col=None):
        """
            Find if there is a contour intersect forward
        """
        if center_col is not None:
            col = center_col
        else:
            col = image.shape[1] / 2
        intersect = None
        i_contour = None
        row = 0
        while intersect is None:
            for i, contour in enumerate(contours):
                if cv2.pointPolygonTest(contour, (col, row), False) >= 0:
                    intersect = row
                    i_contour = i
                    break
            row += 1
            if row > image.shape[0]:
                break

        if intersect is None:
            intersect = image.shape[0]
        return {'contour': i_contour, 'distance': intersect}


    def build_mask(self, image):
        """ Build the mask to find the path edges """
        kernel = np.ones((3, 3), np.uint8)
        img = cv2.bilateralFilter(image, 9, 75, 75)
        img = cv2.erode(img, kernel, iterations=1)

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_gray, self.upper_gray)

        mask2 = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        mask2 = cv2.erode(mask2, kernel)
        mask2 = cv2.dilate(mask2, kernel, iterations=1)

        return mask2

    def side_intersect(self, image, contours, row, markup=True):
        """ Find intersections to both sides along a row """
        if markup:
            cv2.line(image, (0, row), (image.shape[1], row), (0, 0, 255), 1)

        cnt_l, col_l = self.find_intersect(image, contours, row, -1)
        if markup and cnt_l is not None:
            cv2.drawContours(image, [contours[cnt_l]], -1, (0, 255, 255), -1)
            cv2.circle(image, (col_l, row), 4, (0, 255, 0), 2)

        cnt_r, col_r = self.find_intersect(image, contours, row, 1)
        if markup and cnt_r is not None:
            cv2.drawContours(image, [contours[cnt_r]], -1, (255, 255, 0), -1)
            cv2.circle(image, (col_r, row), 4, (0, 255, 0), 2)

        return (cnt_l, col_l), (cnt_r, col_r)

    @staticmethod
    def perpendicular(vector):
        """ get perpendicular vector """
        perp = np.empty_like(vector)
        perp[0] = -vector[1]
        perp[1] = vector[0]
        return perp

    # line segment a given by endpoints seg_a1, seg_a2
    # line segment b given by endpoints seg_b1, seg_b2
    def seg_intersect(self, seg_a1, seg_a2, seg_b1, seg_b2):
        """ Get intersection point between 2 segments """
        delta_a = seg_a2-seg_a1
        delta_b = seg_b2-seg_b1
        delta_p = seg_a1-seg_b1
        dap = self.perpendicular(delta_a)
        denom = np.dot(delta_b, dap)
        if denom.astype(float) == 0:
            return None
        h_value = np.dot(delta_p, dap) / denom
        if not (h_value.astype(float) >= 0 and h_value.astype(float) <= 1):
            return None
        return h_value * delta_b + seg_b1

    def find_left_segment(self, image, contours, sides, base_row):
        """ Find the segment on the left intersecting contour """
        contour = contours[sides[INTERSECT_LEFT][INTERSECT_CONTOUR_INDEX]]
        column = sides[INTERSECT_LEFT][INTERSECT_COLUMN]

        left_most = 0
        segment = None
        point = None

        for i, end in enumerate(contour):
            if i == 0:
                start = contour[-1][0]
            pnt = self.seg_intersect(
                np.array([float(image.shape[1])/2.0, float(base_row)],
                         dtype=float),
                np.array([float(column), float(base_row)], dtype=float),
                start,
                np.array(end[0], dtype=float))
            if pnt is not None and pnt[0] > left_most:
                left_most = pnt[0]
                segment = np.array([start, end[0]], dtype=int)
                point = pnt
            start = np.array(end[0], dtype=float)
        segment = np.array(sorted(segment, key=lambda segment: segment[1]))
        return segment, point

    def find_right_segment(self, image, contours, sides, base_row):
        """ Find the segment on the right intersecting contour """
        contour = contours[sides[INTERSECT_RIGHT][INTERSECT_CONTOUR_INDEX]]
        column = sides[INTERSECT_RIGHT][INTERSECT_COLUMN]

        right_most = image.shape[1]
        segment = None
        point = None

        for i, end in enumerate(contour):
            if i == 0:
                start = contour[-1][0]
            pnt = self.seg_intersect(
                np.array([float(image.shape[1])/2.0, float(base_row)],
                         dtype=float),
                np.array([float(column), float(base_row)], dtype=float),
                start,
                np.array(end[0], dtype=float))
            if pnt is not None and pnt[0] < right_most:
                right_most = pnt[0]
                segment = np.array([start, end[0]], dtype=int)
                point = pnt
            start = np.array(end[0], dtype=float)
        segment = np.array(sorted(segment, key=lambda segment: segment[1]))
        return segment, point

    @staticmethod
    def find_next_right_segment(contours, sides, segment):
        """ Find the next right segment on a contour """
        contour = contours[sides[INTERSECT_RIGHT][INTERSECT_CONTOUR_INDEX]]

        seg_1 = None
        seg_2 = None

        for i, point in enumerate(contour):
            if np.array_equal(point[0], segment[0]):
                seg_1 = i
            elif np.array_equal(point[0], segment[1]):
                seg_2 = i
            if seg_1 is not None and seg_2 is not None:
                break

        seg_1 += 1
        seg_2 += 1

        if seg_1 >= len(contour):
            seg_1 = 0
        if seg_2 >= len(contour):
            seg_2 = 0

        return np.array([contour[seg_1][0], contour[seg_2][0]])

    @staticmethod
    def find_next_left_segment(contours, sides, segment):
        """ Find the next left segment on a contour """
        contour = contours[sides[INTERSECT_LEFT][INTERSECT_CONTOUR_INDEX]]

        seg_1 = None
        seg_2 = None

        for i, point in enumerate(contour):
            if np.array_equal(point[0], segment[0]):
                seg_1 = i
            elif np.array_equal(point[0], segment[1]):
                seg_2 = i
            if seg_1 is not None and seg_2 is not None:
                break

        seg_1 -= 1
        seg_2 -= 1

        if seg_1 < 0:
            seg_1 = len(contour)-1
        if seg_2 < 0:
            seg_2 = len(contour)-1

        return np.array([contour[seg_1][0], contour[seg_2][0]])

    @staticmethod
    def segment_angle(line1, line2):
        """ Angle between two segments """
        vector_a = np.array([line1[0][0]-line1[1][0],
                             line1[0][1]-line1[1][1]])
        vector_b = np.array([line2[0][0]-line2[1][0],
                             line2[0][1]-line2[1][1]])
        # Get dot prod
        dot_prod = np.dot(vector_a, vector_b)
        # Get magnitudes
        magnitude_a = np.dot(vector_a, vector_a)**0.5
        magnitude_b = np.dot(vector_b, vector_b)**0.5
        # Get angle in radians and then convert to degrees
        angle = np.arccos(dot_prod/magnitude_b/magnitude_a)
        # Basically doing angle <- angle mod 360
        ang_deg = np.rad2deg(angle)%360

        if ang_deg-180 >= 0:
            # As in if statement
            return 360 - ang_deg
        else:
            return ang_deg

    def figure_path(self, image, base_row, markup=True, silent=False):
        """ Figure out the path direction from an image """
        contours = self.find_contours(self.build_mask(image), 0.01)

        if markup:
            cv2.drawContours(image, contours, -1, (255, 255, 255))

        forward = self.forward_intersect(image, contours)
        if markup:
            if forward['contour'] is not None:
                cv2.circle(image, (image.shape[1]/2, forward['distance']), 4,
                           (0, 255, 0), 2)
            cv2.line(image, (image.shape[1]/2, 0),
                     (image.shape[1]/2, image.shape[0]),
                     (0, 0, 255), 1)

        if forward['distance'] is not None and forward['distance'] < base_row:
            if not silent:
                log("FAILED({0}): Beyond path".format(base_row))
            return None

        left = None
        right = None
        sides = self.side_intersect(image, contours, base_row, markup)
        if sides[INTERSECT_LEFT][INTERSECT_CONTOUR_INDEX] is not None:
            left = {}
            left['start'] = base_row
            left['segments'] = []
            segment1, point = self.find_left_segment(image, contours, sides,
                                                     base_row)
            left['segments'].append(segment1)
            if markup:
                cv2.circle(image, (point[0].astype(int),
                                   point[1].astype(int)), 8,
                           (255, 100, 255), 2)
                cv2.line(image, tuple(segment1[1]), tuple(segment1[0]),
                         (255, 100, 255), 2)

            segment2 = self.find_next_left_segment(contours, sides, segment1)
            left['segments'].append(segment2)

            if markup:
                cv2.line(image, tuple(segment2[1]), tuple(segment2[0]),
                         (255, 0, 0), 2)

            left['angle'] = self.segment_angle(segment1, segment2)
            left['distance'] = max(segment1[0][1], segment1[1][1])

        if sides[INTERSECT_RIGHT][INTERSECT_CONTOUR_INDEX] is not None:
            right = {}
            right['start'] = base_row
            right['segments'] = []
            segment1, point = self.find_right_segment(image, contours, sides,
                                                      base_row)
            right['segments'].append(segment1)
            if markup:
                cv2.circle(image, (point[0].astype(int),
                                   point[1].astype(int)), 8,
                           (255, 100, 255), 2)
                cv2.line(image, tuple(segment1[1]), tuple(segment1[0]),
                         (255, 100, 255), 2)

            segment2 = self.find_next_right_segment(contours, sides, segment1)
            right['segments'].append(segment2)
            if markup:
                cv2.line(image, tuple(segment2[1]), tuple(segment2[0]),
                         (255, 0, 0), 2)

            right['angle'] = self.segment_angle(segment1, segment2)
            right['distance'] = max(segment1[0][1], segment1[1][1])

        if right is not None and left is not None:
            if markup:
                cv2.circle(image, (image.shape[1]/2,
                                   int((right['distance'] + \
                                        left['distance'])/2)),
                           4, (0, 255, 0), 2)

            width = abs(sides[INTERSECT_LEFT][INTERSECT_COLUMN] - \
                        sides[INTERSECT_RIGHT][INTERSECT_COLUMN])
            offset = (sides[INTERSECT_LEFT][INTERSECT_COLUMN] + width/2) -\
                            (image.shape[1]/2.0)
        else:
            width = None
            offset = None

        return {'width': width, 'forward': forward, 'offset':offset,
                'left': left, 'right': right}

    def unwarp_perspective(self, where):
        """ Take a pixel row and turn it into an unwarped distance """
        return self.perspective.do_unwarp(where)

    def distance_to_row(self, row):
        """ return the furthest that can be seen, None is infinitiy """
        return self.unwarp_perspective((0, row))

    def look_for_exit(self, image, debug=False, unwarped=True):
        """ Look for an exit from our position """
        for row in range(1, image.shape[0]/10):
            img = image.copy()
            base_row = (row * 10) + 5
            image_dump = os.environ.get("ZARJ_IMAGE_DUMP")
            markup = debug or (image_dump is not None)
            output = self.figure_path(img, base_row, markup, True)
            if output is not None and output['right'] is not None and\
               output['left'] is not None:
                real_distance = self.unwarp_perspective((image.shape[1]/2,
                                                         base_row))
                if unwarped:
                    base_row = real_distance
                if markup:
                    txt = "({0:.2f})".format(real_distance)
                    cv2.putText(img, txt, (0, img.shape[0]-10),
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255))
                if debug:
                    name = "_exit_decision_"
                    cv2.namedWindow(name, cv2.WINDOW_NORMAL)
                    cv2.resizeWindow(name, 500, 250)
                    cv2.imshow(name, img)
                    cv2.waitKey(1)
                if image_dump is not None:
                    cv2.imwrite(image_dump + '/exit_{0}.png'.format(
                        debug_sequence()), img)
                if real_distance > 1.8:
                    log('Wait a second! An exit {} away is too far away'.format(
                        real_distance))
                    return None
                return base_row
        return None

    def get_navigation_image(self, crop=False):
        """ Get a navigation ready image """
        self.zarj.zps.look_down()
        image, _ = self.zarj.eyes.get_images()
        if crop:
            image = image[0:image.shape[0], 65:image.shape[1]-65]
        self.zarj.zps.look_forward()
        image_dump = os.environ.get("ZARJ_IMAGE_DUMP")
        if image_dump is not None:
            cv2.imwrite(image_dump + '/navigation_{0}.png'.format(
                debug_sequence()), image)
        return image
