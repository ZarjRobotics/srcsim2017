#!/usr/bin/env python
"""
    Eyeballs for the robot
"""
import sys
import struct
import ctypes
import rospy
import cv2

import tf2_geometry_msgs
from . import tf

from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from geometry_msgs.msg import PointStamped

#import laser_geometry
#from sensor_msgs.msg import LaserScan
#from std_msgs.msg import String, Empty, Float64
#from matplotlib import pyplot as plt

__all__ = ['DetectionRequest', 'ZarjEyes']

_PIXEL_COUNT_ON = 50 # if color pixels exceed this value, the color is detected
_PIXEL_COUNT_OFF = 10 # if color pixels are less that this value, color is gone

class _CloudPlane(object):
    """ A class turning a bunch of cloud points on a plane and trying
        to solve that plane """

    def __init__(self):
        self.top_x = None
        self.top_y = None
        self.top_z = None
        self.bottom_x = None
        self.bottom_y = None
        self.bottom_z = None

    def reset(self):
        """ Reset the plane """
        self.top_x = None
        self.top_y = None
        self.top_z = None
        self.bottom_x = None
        self.bottom_y = None
        self.bottom_z = None

    def add_point(self, point):
        """ Add a point to the plane """
        if self.top_x is None or point.x < self.top_x:
            self.top_x = point.x
        elif self.bottom_x is None or point.x > self.bottom_x:
            self.bottom_x = point.x
        if self.top_y is None or point.y < self.top_y:
            self.top_y = point.y
        elif self.bottom_y is None or point.y > self.bottom_y:
            self.bottom_y = point.y
        if self.top_z is None or point.z < self.top_z:
            self.top_z = point.z
        elif self.bottom_z is None or point.z > self.bottom_z:
            self.bottom_z = point.z

    def valid(self):
        """ Is the plane valid """
        return self.bottom_x is not None and self.top_x is not None

    def find_center(self):
        """ Return the center of the plane as x,y,z """
        if not self.valid():
            return None
        return ((self.bottom_x - self.top_x) / 2) + self.top_x, \
               ((self.bottom_y - self.top_y) / 2) + self.top_y, \
               ((self.bottom_z - self.top_z) / 2) + self.top_z

class DetectionRequest(object):
    """ A detection request for the eyes """

    def __init__(self, name, hsv_low, hsv_high, min_reports,
                 stablize_time, callback, debug=False):
        self.name = name
        self.color_low = hsv_low
        self.color_high = hsv_high
        if type(callback) is not list:
            self.callbacks = [None, None, callback]
        else:
            self.callbacks = callback
        self.active = None
        self.report_count = 0
        if self.callbacks[2]:
            self.published = False
        else:
            self.published = True
        self.last_seq = None
        self.cloud = _CloudPlane()
        self.stablize_time = stablize_time
        self.min_reports = min_reports
        self.frame_id = None
        self.debug = debug
        self.baseline = None
        self.baseline_cnt = 0
        self.low_count = _PIXEL_COUNT_OFF
        self.high_count = _PIXEL_COUNT_ON
        self.baseline_fuzzy = 0

    def record_baseline(self, fuzzyness=10, frames=5):
        """ Record a baseline mask """
        self.baseline = None
        self.baseline_cnt = frames
        self.baseline_fuzzy = fuzzyness
        self.low_count = _PIXEL_COUNT_OFF
        self.high_count = _PIXEL_COUNT_ON

    def ready(self):
        """ Is the detection ready for use """
        return self.baseline_cnt is 0

    def detection(self, hsv_image):
        """Check for detection in the image """
        mask = cv2.inRange(hsv_image, self.color_low, self.color_high)
        if self.baseline_cnt > 0:
            nmask = cv2.bitwise_not(mask)
            if self.baseline is None:
                rospy.loginfo("getting baseline for {}".format(self.name))
                self.baseline = nmask
            else:
                self.baseline = cv2.bitwise_or(nmask, self.baseline)
                mask = cv2.bitwise_and(mask, self.baseline)
                count = cv2.countNonZero(mask) + self.baseline_fuzzy
                self.low_count = max(self.low_count, count)
                self.high_count = self.low_count + self.baseline_fuzzy
            self.baseline_cnt -= 1
            return
        elif self.baseline is not None:
            mask = cv2.bitwise_and(mask, self.baseline)
        count = cv2.countNonZero(mask)
        if count > self.low_count and self.active is None:
            self.active = rospy.get_rostime()
            rospy.loginfo("{} ACTIVE ({})".format(self.name, count))
            self.cloud.reset()
            if self.callbacks[0] is not None:
                self.callbacks[0](self.name)
        elif self.active is not None and count < self.high_count:
            rospy.loginfo("{} GONE ({})".format(self.name, count))
            self.cloud.reset()
            self.active = None
            if self.callbacks[2] is not None:
                self.published = False
            self.report_count = 0
            if self.callbacks[1] is not None:
                self.callbacks[1](self.name)

        # DEBUGGING to see what the masked image for the request is
        if self.debug:
            cv2.namedWindow(self.name, cv2.WINDOW_NORMAL)
            if self.baseline is not None:
                cv2.namedWindow(self.name+'_baseline', cv2.WINDOW_NORMAL)
                cv2.imshow(self.name+'_baseline', self.baseline)
            cv2.imshow(self.name, mask)
            cv2.waitKey(1)

        # to to see if we notify the location callback
        if self.is_active() and self.report_count > self.min_reports:
            now = rospy.get_rostime()
            if (self.active + self.stablize_time) < now:
                self.published = True
                point = PointStamped()
                center = self.cloud.find_center()
                point.header.seq = 1
                point.header.stamp = now
                point.header.frame_id = self.frame_id
                point.point.x = center[0]
                point.point.y = center[1]
                point.point.z = center[2]
                if self.callbacks[2] is not None:
                    self.callbacks[2](self.name, point)

    def is_active(self):
        """ Is detection active """
        return self.active and not self.published

    def hsv_in_range(self, hsv_color):
        """ Is a given color in the request range """
        if self.is_active():
            return cv2.inRange(hsv_color, self.color_low, self.color_high)
        return False

    def add_point(self, point):
        """ Add the point to the plane """
        if self.last_seq != point.header.seq:
            self.report_count += 1
            self.last_seq = point.header.seq
            self.frame_id = point.header.frame_id
        self.cloud.add_point(point.point)

class ZarjEyes(object):
    """ This is the multisense package """

    def __init__(self, frame, transform_module=None):
        rospy.loginfo("Zarj eyes initialization begin")
        self.bridge = CvBridge()

        if transform_module is not None:
            self.tf = transform_module
        else:
            self.tf = tfzarj.TfZarj(rospy.get_param('/ihmc_ros/robot_name'))
            self.tf.init_transforms()

        self.detection_requests = []
        self.frame_id = frame

        self.image_sub_left = None
        self.image_sub_cloud = None

        rospy.loginfo("Zarj eyes initialization finished")

    def __del__(self):
        if self.image_sub_left is not None:
            self.image_sub_left.unregister()
        if self.image_sub_cloud is not None:
            self.image_sub_cloud.unregister()

    def start_processing(self):
        """ Start processing data """
        rospy.loginfo("Processing started")
        self.image_sub_left = rospy.Subscriber(
            "/multisense/camera/left/image_raw", Image, self.color_detection)
        self.image_sub_cloud = rospy.Subscriber(
            "/multisense/image_points2_color", PointCloud2, self.cloud)

    def ready(self):
        """ Are the eyes ready for use """
        if self.image_sub_left is None:
            return False
        for request in self.detection_requests:
            if not request.ready():
                return False
        return True

    def add_detection_request(self, request):
        """ Add a detection request """
        self.detection_requests.append(request)

    def remove_detection_request(self, request_name):
        """ remove a named detection request """
        for request in self.detection_requests:
            if request.name == request_name:
                self.detection_requests.remove(request)
                return

    def color_detection(self, data):
        """ Callback to detect r, g, and b values in the image """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print e

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        for request in self.detection_requests:
            request.detection(hsv)

    #############################################
    # Stereo image point cloud processing members
    #############################################

    @staticmethod
    def extract_rgb(color):
        """ Extract RGB values from the float32 color """
        # cast float32 to int so that bitwise operations are possible
        s = struct.pack('>f', color)
        i = struct.unpack('>l', s)[0]
        pack = ctypes.c_uint32(i).value
        r = (pack & 0x00FF0000)>> 16
        g = (pack & 0x0000FF00)>> 8
        b = (pack & 0x000000FF)
        return r, g, b

    @staticmethod
    def convertRGBtoHSV(color):
        """ Convert an RGB to HSV """
        clr = np.uint8([[color]])
        return cv2.cvtColor(clr, cv2.COLOR_RGB2HSV)

    def build_point(self, x, y, z, header):
        """ Take an X,Y,Z and convert it into a head point """
        point = PointStamped()
        point.header = header
        point.point.x = x
        point.point.y = y
        point.point.z = z
        head_pnt = self.tf.tf_buffer.transform(point, self.frame_id)
        head_pnt.header.seq = header.seq
        return head_pnt

    def cloud(self, data):
        """ Callback to process the point cloud """
        active = False
        for request in self.detection_requests:
            if request.is_active():
                active = True
                break
        if not active:
            return

        for x, y, z, c in pc2.read_points(data,
                                          field_names=("x", "y", "z", "rgb"),
                                          skip_nans=True):

            #request and point adding here!
            point = None
            rgb_color = self.extract_rgb(c)
            hsv_color = self.convertRGBtoHSV(rgb_color)
            for request in self.detection_requests:
                if request.hsv_in_range(hsv_color):
                    if point is None:
                        point = self.build_point(x, y, z, data.header)
                    request.add_point(point)

#class lidar_listener(object):
#
#    def __init__(self):
#        self.bridge = CvBridge()
#        self.image_sub = rospy.Subscriber("/multisense/lidar_scan",
#                                          LaserScan, self.callback)
#
#        ROBOT_NAME = rospy.get_param('/ihmc_ros/robot_name')
#        publisher = rospy.Publisher(
#            "/multisense/set_spindle_speed".format(ROBOT_NAME), Float64,
#            queue_size=1)
#        msg = Float64(3.0)
#        publisher.publish(msg)
#
#        self.count = 0
#        self.laser_projector = laser_geometry.LaserProjection()
#        print "Lidar Init done"
#
#    def callback(self, data):
#        print "In callback, Lidar"
#        cloud = self.laser_projector.projectLaser(data)
#        print cloud
#        for p in pc2.read_points(cloud,
#                                 field_names=("x", "y", "z", "intensity"),
#                                 skip_nans=True):
#            print p
