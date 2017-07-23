#!/usr/bin/env python
"""
    Zarj solution for qual 1.

    This requires stereo_image_proc to be started, with a
    correlation_window_size set to 31
"""
import sys
import rospy
import os

sys.path.append('../lib')
sys.path.append('./lib')
import zarj.training_eyes

low_blue = [110, 50, 50]
high_blue = [130, 255, 255]

low_green = [50, 40, 40]
high_green = [90, 255, 255]

if __name__ == '__main__':

    prefix = None
    if 'EYES_PREFIX' in os.environ:
        prefix = os.environ['EYES_PREFIX']

    rospy.init_node('q1', anonymous=True)

    # make sure our correlation_window_size is correct
    rospy.set_param('/multisense/camera/stereo_proc/correlation_window_size',
                    31)
    rospy.set_param('/multisense/camera/left/imager_rate', 1)
    rospy.set_param('/multisense/camera/right/imager_rate', 1)

    il = zarj.training_eyes.ZarjTrainingEyes(prefix, (low_blue, high_blue))
    rospy.sleep(0.5)
    il.start_processing()

    while True:
        rospy.sleep(0.1)
