#!/usr/bin/env python

# Experiment with finding the button 

import sys
import cv2
import numpy as np
sys.path.append('../lib/')
import zarj
import rospy
from zarj.things import Things
from geometry_msgs.msg import PointStamped

def _button_box_union(a,b):
    x = min(a[0], b[0])
    y = min(a[1], b[1])
    w = max(a[0]+a[2], b[0]+b[2]) - x
    h = max(a[1]+a[3], b[1]+b[3]) - y
    return (x, y, w, h)


class Main(object):
    """ Class to perform all main logic. """

    def __init__(self, name):
        rospy.init_node(name)

        if not rospy.has_param('/ihmc_ros/robot_name'):
            rospy.logerr("Cannot run team_zarj_main.py, missing parameters!")
            rospy.logerr("Missing parameter '/ihmc_ros/robot_name'")
        else:
            self.zarj = zarj.ZarjOS()

    def find_button(self):
        cloud = self.zarj.eyes.get_stereo_cloud()
        image, details = self.zarj.eyes.get_cloud_image_with_details(cloud)
        things = Things(image, details, 2, True)


        if things.array_button is not None and things.array_button.computed_center is not None:
            button = PointStamped()
            button.header = cloud.header
            button.point.x = things.array_button.computed_center[0]
            button.point.y = things.array_button.computed_center[1]
            button.point.z = things.array_button.computed_center[2]
            button_in_foot = self.zarj.transform.tf_buffer.transform(button, self.zarj.walk.lfname)
            p = button_in_foot.point
            print "JPW sez button is {}/{}/{}".format(p.x, p.y, p.z)


    def run(self, argv):
        print "find_button returns {}".format(self.find_button())

if __name__ == '__main__':
        try:
            Main('army').run(sys.argv)
        except rospy.ROSInterruptException:
            pass

