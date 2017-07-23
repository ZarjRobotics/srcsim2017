"""
    Global Macros
"""
import numpy as np
import cv2
import rospy
from zarjmacro import ZarjMacro
from zarj.utils import log

from zarjcomm.message import ZarjPicture

class test1(ZarjMacro):
    """ Testing macro 1 """

    def start(self, _=None):
        """ Start the macro """
        log("Testing macro 1!")
        rospy.sleep(1.0)
        self.done = True

    @property
    def next_chain(self):
        """ returns the name of the next macro in the chain """
        return 'test2'

class test2(ZarjMacro):
    """ Testing macro 2! """

    def start(self, _=None):
        """ Start the macro """
        log("Testing macro 2!")
        self.done = True
