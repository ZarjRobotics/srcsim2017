#!/usr/bin/env python

# Experiment with moving the hand to a point

import sys
import cv2
import numpy as np
sys.path.append('../lib/')
import zarj
import rospy
from geometry_msgs.msg import PointStamped

rospy.init_node("army")

class Main(object):
    """ Class to perform all main logic. """

    def __init__(self, name):
        rospy.init_node(name)

        if not rospy.has_param('/ihmc_ros/robot_name'):
            rospy.logerr("Cannot run team_zarj_main.py, missing parameters!")
            rospy.logerr("Missing parameter '/ihmc_ros/robot_name'")
        else:
            self.zarj = zarj.ZarjOS()

    def handle_mouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONUP:
            point = PointStamped()
            point.header = self.cloud.header
            point.point.x = self.details[y, x][0]
            point.point.y = self.details[y, x][1]
            point.point.z = self.details[y, x][2]

            print x, y, self.details[y,x]
            result = self.zarj.transform.tf_buffer.transform(point, 'rightIndexFingerPitch1Link')
            print result

    def process_cloud(self):
        self.zarj.transform.init_transforms()
        self.cloud = self.zarj.eyes.get_stereo_cloud()
        self.data = self.zarj.eyes.get_cloud_data(self.cloud)
        self.raw_img, self.details = self.zarj.eyes.get_cloud_image_with_details(self.cloud)
        print "<<<< FOCUS IMAGE WINDOW AND PRESS ANY KEY TO CONTINUE >>>>"
        cv2.imshow("cloud view", raw_img)
        cv2.setMouseCallback("cloud view", handle_mouse)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def run(self, argv):
        i = 1
        while i + 1 <= len(argv):
            hand = sys.argv[i]
            style = sys.argv[i + 1]
            i = i + 2
            self.zarj.hands.named_arm_configuration(hand, style)

if __name__ == '__main__':
        try:
            Main('army').run(sys.argv)
        except rospy.ROSInterruptException:
            pass

