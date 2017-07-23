#!/usr/bin/env python
"""
    Zarj solution for qual 1.
"""
import sys
import rospy
import os

sys.path.append('../lib')
sys.path.append('./lib')
import zarj.eyes
import zarj.detection_processor

if __name__ == '__main__':
    rospy.init_node('q1', anonymous=True)
    rospy.set_param('/multisense/camera/left/imager_rate', 1)
    rospy.set_param('/multisense/camera/right/imager_rate', 1)

    EYES = zarj.eyes.Eyes()
    PROCESSOR = zarj.detection_processor.DetectionProcessor(
        '../cascade/satellite.xml')
    PROCESSOR.side = zarj.eyes.Eyes.BOTH
    rospy.sleep(0.5)
    EYES.register_processor(PROCESSOR)

    while True:
        rospy.sleep(0.1)
