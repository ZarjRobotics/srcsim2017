#!/usr/bin/env python
"""
    Just watching processor for the robot
"""
import cv2
from .vision_processor import VisionProcessor

__all__ = ['WatchingProcessor']

class WatchingProcessor(VisionProcessor):
    """ Cascade object detection """

    def __init__(self):
        super(WatchingProcessor, self).__init__()

    def process_image(self, cv_image, header, tag):
        """ process the image """
        cv2.namedWindow("watching:"+tag, cv2.WINDOW_NORMAL)
        cv2.resizeWindow("watching:"+tag, 500, 250)
        cv2.imshow("watching:"+tag, cv_image)
        cv2.waitKey(1)
