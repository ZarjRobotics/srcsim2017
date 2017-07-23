#!/usr/bin/env python

# Experiment with  finding a point from the cloud

import sys
import cv2
import numpy as np
sys.path.append('../lib/')
import zarj
import rospy
from geometry_msgs.msg import PointStamped

#red button?
LOW_RED_SAT = np.array([0, 180, 50])
HIGH_RED_SAT = np.array([10, 255, 130])

rospy.init_node("rngfindlive")

ME = zarj.ZarjOS()
ME.transform.init_transforms()

def handle_mouse(event, x, y, flags, param):
    global details
    global cloud
    global ME
    if event == cv2.EVENT_LBUTTONUP:
        point = PointStamped()
        point.header = cloud.header
        point.point.x = details[y, x][0]
        point.point.y = details[y, x][1]
        point.point.z = details[y, x][2]

        print x, y, details[y,x]
        result = ME.transform.tf_buffer.transform(point, 'rightIndexFingerPitch1Link')
        print result

#leftimg,_ = ME.eyes.get_images()
#img = zarj.navigation.PERSPECTIVE_AHEAD.build_rangefinding_image(leftimg)
#cv2.imshow("leftimg", img)

#img = zarj.navigation.PERSPECTIVE_AHEAD.build_rangefinding_image(raw_img)

cloud = ME.eyes.get_stereo_cloud()
data = ME.eyes.get_cloud_data(cloud)
raw_img, details = ME.eyes.get_cloud_image_with_details(cloud)
pictures = ME.eyes.get_images()

print "cloud type {}, picture type {}".format(type(raw_img), type(pictures[0]))
print "cloud shape {}, picture shape{}".format(raw_img.shape, pictures[0].shape)
#colors = ME.eyes.get_cloud_colors(cloud)
#data = ME.eyes.get_cloud_data(cloud)
#hsv = cv2.cvtColor(colors, cv2.COLOR_BGR2HSV)
#mask = cv2.inRange(hsv, LOW_RED_SAT, HIGH_RED_SAT)
#print "size {}, ndim {} ".format(hsv.size, hsv.ndim)
#print "size {}, ndim {} ".format(hsv[0].size, hsv[0].ndim)
#print "size {}, ndim {} ".format(hsv[0][0].size, hsv[0][0].ndim)
#print "-----------"
#print hsv
#print "-----------"
#print "size {}, ndim {} ".format(mask.size, mask.ndim)
#print "size {}, ndim {} ".format(mask[0].size, mask[0].ndim)
#print mask
#i = 0
#for x in mask[0]:
#    if x > 0:
#        print "{}: {} {} {}".format(i, x, data[i], hsv[0][i])
#    i = i + 1

#hsv = cv2.cvtColor(raw_img, cv2.COLOR_BGR2HSV)

#mask = cv2.inRange(hsv, LOW_RED_SAT, HIGH_RED_SAT)


print "<<<< FOCUS IMAGE WINDOW AND PRESS ANY KEY TO CONTINUE >>>>"
cv2.imshow("cloud view", raw_img)
cv2.setMouseCallback("cloud view", handle_mouse)

#cv2.imshow("mask", mask)
#masked = cv2.bitwise_and(raw_img, raw_img, mask=mask)
#cv2.imshow("mask", masked)
cv2.waitKey(0)


cv2.destroyAllWindows()
