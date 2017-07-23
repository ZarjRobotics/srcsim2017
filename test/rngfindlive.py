#!/usr/bin/env python

# Quick range finder test app

import sys
import cv2
sys.path.append('../lib/')
import zarj
import rospy

rospy.init_node("rngfindlive")

ME = zarj.ZarjOS()

leftimg,_ = ME.eyes.get_images()
img = zarj.navigation.PERSPECTIVE_AHEAD.build_rangefinding_image(leftimg)
cv2.imshow("leftimg", img)

raw_img = ME.eyes.get_cloud_image(ME.eyes.get_stereo_cloud())
img = zarj.navigation.PERSPECTIVE_AHEAD.build_rangefinding_image(raw_img)

print "<<<< FOCUS IMAGE WINDOW AND PRESS ANY KEY TO CONTINUE >>>>"
cv2.imshow("cloud view", img)
cv2.waitKey(0)


cv2.destroyAllWindows()
