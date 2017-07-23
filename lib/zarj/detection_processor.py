#!/usr/bin/env python
"""
    Object detection processor for the robot
"""
import cv2
from .vision_processor import VisionProcessor

__all__ = ['DetectionProcessor']

class DetectionProcessor(VisionProcessor):
    """ Cascade object detection """

    def __init__(self, detector):
        super(DetectionProcessor, self).__init__()
        self.cascade = cv2.CascadeClassifier(detector)

    def process_image(self, cv_image, header, tag):
        """ process the image """
        objects = self.cascade.detectMultiScale(cv_image)
        for obj in objects:
            cv2.rectangle(
                cv_image, (obj[0], obj[1]),
                (obj[0] + obj[2], obj[1] + obj[3]), (0, 255, 0))

        cv2.namedWindow(tag, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(tag, 600, 600)
        cv2.imshow(tag, cv_image)
        cv2.waitKey(1)
