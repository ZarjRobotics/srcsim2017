#!/usr/bin/env python
"""
    Eyeballs for the robot
"""
import rospy
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

__all__ = ['ZarjTrainingEyes']

class ZarjTrainingEyes(object):
    """ This is the multisense package """

    def __init__(self, prefix=None, color_range=None):
        rospy.loginfo("Zarj eyes initialization begin")
        self.bridge = CvBridge()
        self.prefix = prefix
        self.color_range = None
        if color_range:
            self.color_range = (
                np.array(color_range[0]),
                np.array(color_range[1])
            )

        self.image_sub_left = None
        self.image_sub_right = None

        rospy.loginfo("Zarj eyes initialization finished")

    def __del__(self):
        if self.image_sub_left is not None:
            self.image_sub_left.unregister()
        if self.image_sub_right is not None:
            self.image_sub_right.unregister()

    def start_processing(self):
        """ Start processing data """
        rospy.loginfo("Processing started")
        self.image_sub_left = rospy.Subscriber(
            "/multisense/camera/left/image_rect_color", Image,
            self.left_color_detection)
        self.image_sub_right = rospy.Subscriber(
            "/multisense/camera/right/image_rect_color", Image,
            self.right_color_detection)

    def ready(self):
        """ Are the eyes ready for use """
        if self.image_sub_left is None:
            return False
        return True

    def process_image(self, cv_image, header, tag):
        """ process the image """
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # mask for color range
        if self.color_range:
            mask = cv2.inRange(hsv, self.color_range[0], self.color_range[1])
            count = cv2.countNonZero(mask)
            if count:
                kernel = np.ones((5, 5), np.uint8)
                mask = cv2.dilate(mask, kernel, iterations=2)
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                               cv2.CHAIN_APPROX_NONE)

                for i, c in enumerate(contours):
                    x, y, w, h = cv2.boundingRect(c)
                    if self.prefix is not None:
                        name = '{0}{1}_{2}_{3}.png'.format(self.prefix,
                                                           tag,
                                                           header.seq, i)
                        print name
                        roi = cv_image[y:y+h, x:x+w]
                        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
                        gray = cv2.equalizeHist(gray)
                        cv2.imwrite(name, gray)

                for c in contours:
                    x, y, w, h = cv2.boundingRect(c)
                    cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0))
            elif self.prefix is not None:
                name = '{0}Negative_{1}_{2}.png'.format(self.prefix, tag,
                                                        header.seq, )
                cv2.imwrite(name, cv_image)

        cv2.namedWindow(tag, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(tag, 600, 600)
        cv2.imshow(tag, cv_image)
        cv2.waitKey(1)

    def left_color_detection(self, data):
        """ Callback to detect r, g, and b values in the image """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print e

        self.process_image(cv_image, data.header, 'left')

    def right_color_detection(self, data):
        """ Callback to detect r, g, and b values in the image """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print e

        self.process_image(cv_image, data.header, 'right')
