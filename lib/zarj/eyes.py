"""
    Eyeballs for the robot
"""
import numpy as np

import rospy
import pickle
import os
import cv2
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError

__all__ = ['Eyes']

class Eyes(object):
    """ This is the multisense package """

    LEFT = 1
    RIGHT = 2
    BOTH = 3

    def __init__(self):
        rospy.loginfo("Zarj eyes initialization begin")
        self.bridge = CvBridge()
        self.sub_left = None
        self.sub_right = None
        self.sub_cloud = None
        self.processors = []
        self.disabled = False
        self.left_image = None
        self.right_image = None
        self.cloud = None
        self.p_left = None
        self.cloud_image_count = 0
        self.info_sub_l = rospy.Subscriber(
            "/multisense/camera/left/camera_info", CameraInfo, self.info_left)

        rospy.loginfo("Zarj eyes initialization finished")

    def __del__(self):
        if self.sub_left is not None:
            self.sub_left.unregister()
        if self.sub_right is not None:
            self.sub_right.unregister()
        if self.sub_cloud is not None:
            self.sub_cloud.unregister()

    def info_left(self, data):
        """ Camera Info """
        self.p_left = np.matrix(data.P, dtype='float64')
        self.p_left.resize((3, 4))
        self.info_sub_l.unregister()

    def start_processing(self):
        """ Start processing data """
        if self.disabled:
            rospy.loginfo("Processing started")
        self.disabled = False

        if self.sub_left is None:
            self.sub_left = rospy.Subscriber(
                "/multisense/camera/left/image_color", Image,
                self.left_color_detection)
            rospy.sleep(0.1)
        if self.sub_right is None:
            self.sub_right = rospy.Subscriber(
                "/multisense/camera/right/image_color", Image,
                self.right_color_detection)
            rospy.sleep(0.1)
        if self.sub_cloud is None:
            self.sub_cloud = rospy.Subscriber(
                "/multisense/image_points2_color", PointCloud2,
                self.stereo_cloud)

    def stop_processing(self):
        """ Stop eye processing """
        if not self.disabled:
            rospy.loginfo("Processing stopped")
        self.disabled = True
        if self.sub_left:
            self.sub_left.unregister()
            self.sub_left = None
        if self.sub_right:
            self.sub_right.unregister()
            self.sub_right = None
        if self.sub_cloud:
            self.sub_cloud.unregister()
            self.sub_cloud = None

    def ready(self):
        """ Are the eyes ready for use """
        return self.p_left is not None

    def register_processor(self, processor):
        """ Register a processor """
        self.processors.append(processor)
        if self.disabled:
            return
        self.start_processing()

    def left_color_detection(self, data):
        """ Callback to detect r, g, and b values in the image """
        try:
            self.left_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as excep:
            print excep

        for processor in self.processors:
            if processor.side&self.LEFT == self.LEFT:
                processor.process_image(self.left_image, data.header, 'left')

    def right_color_detection(self, data):
        """ Callback to detect r, g, and b values in the image """
        try:
            self.right_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as excep:
            print excep

        for processor in self.processors:
            if processor.side&self.RIGHT == self.RIGHT:
                processor.process_image(self.right_image, data.header, 'right')

    def stereo_cloud(self, data):
        """ Callback to process the point cloud """
        self.cloud = data

    def get_images(self):
        """ Get a single image """
        self.left_image = None
        self.right_image = None

        self.start_processing()

        while self.left_image is None or self.right_image is None:
            rospy.sleep(0.3)

        return self.left_image, self.right_image

    def get_stereo_cloud(self):
        """ Get the stereo cloud """
        self.start_processing()
        while self.cloud is None:
            rospy.sleep(0.3)
        return self.cloud

    @staticmethod
    def get_cloud_colors(data):
        """ Get colors from the cloud """
        dtype = np.dtype('float32')
        dtype = dtype.newbyteorder('<')
        buf = np.frombuffer(data.data, dtype)
        buf = np.resize(buf, (data.width * data.height, 8))
        buf = np.compress([True, True, True, False, True, False, False,
                           False], buf, axis=1)
        cond = np.isnan(buf).any(1)
        buf[cond] = [0.0, 0.0, 0.0, 0.0]
        buf = np.compress([False, False, False, True], buf, axis=1)
        nstr = buf.tostring()
        rgb = np.fromstring(nstr, dtype='uint8')
        rgb.resize((data.height * data.width), 4)
        rgb = np.compress([True, True, True, False], rgb, axis=1)
        return np.array([rgb])

    @staticmethod
    def get_cloud_data(data):
        """ Get the data out of a cloud as a numpy array """
        dtype = np.dtype('float32')
        dtype = dtype.newbyteorder('<')
        buf = np.frombuffer(data.data, dtype)
        buf = np.resize(buf, (data.width * data.height, 8))
        return np.compress([True, True, True, False, True, False, False,
                            False], buf, axis=1)

    def get_cloud_image(self, data):
        """ Get an image from the cloud """
        dta = np.zeros((data.height, data.width), dtype="float32")

        dtype = np.dtype('float32')
        dtype = dtype.newbyteorder('<')
        buf = np.frombuffer(data.data, dtype)
        buf = np.resize(buf, (data.width * data.height, 8))
        buf = np.compress([True, True, True, True, True, False, False, False],
                          buf, axis=1)
        buf = buf[~np.isnan(buf).any(1)]

        for point in buf:
            point[3] = 1.0
            src = np.asmatrix(point[:4])
            src = np.reshape(src, (4, 1))
            dst = np.dot(self.p_left, src)
            pnt_w = dst[2, 0]
            if pnt_w != 0:
                img_x = dst[0, 0] / pnt_w
                img_y = dst[1, 0] / pnt_w
                dta[img_y, img_x] = point[4]

        nstr = dta.tostring()
        img = np.fromstring(nstr, dtype='uint8')
        img.resize(data.height, data.width, 4)
        img = np.compress([True, True, True, False], img, axis=2)
        return img

    def get_cloud_image_with_details(self, data):
        """ Get an image from the cloud """
        # Get the cloud data into 'buf' at the right size
        dtype = np.dtype('float32')
        dtype = dtype.newbyteorder('<')
        buf = np.frombuffer(data.data, dtype)
        buf = np.resize(buf, (data.height, data.width, 8))

        # Extract the details by removing the extraneous floats.
        #  (The cloud sends 8 floats, but only 4 have data; see 'fields'
        #  in the cloud if you want more details)
        details = np.compress([True, True, True, False, True, False, False, False],
                          buf, axis=2)

        # We want an image, but only where we have details.  So first,
        #   make an image by grabbing just the rgb.  Then screen out
        #   pixels with any nan's.  (That is, the stereo image processing
        #   will send valid RGB for every pixle, but not valid distances
        #   for every pixel).
        dta = np.compress([False, False, False, True], details, axis=2)

        cond = np.isnan(details).any(2)
        dta[cond] = [0.0]

        # Finally, convert the float32 into uint8, and discard the
        #   4th byte as we don't use it
        nstr = dta.tostring()
        img = np.fromstring(nstr, dtype='uint8')
        img.resize(data.height, data.width, 4)
        img = np.compress([True, True, True, False], img, axis=2)

        image_dump = os.environ.get("ZARJ_IMAGE_DUMP")
        if image_dump is not None:
            cloud_image_and_details = { 'img': img, 'details': details }
            filename = image_dump + "/cloud_image_and_details.{}.pickle".format(self.cloud_image_count)
            pickle.dump(cloud_image_and_details, open(filename, "wb" ), protocol=pickle.HIGHEST_PROTOCOL)
            cv2.imwrite(image_dump+'/cloud_image_{}.png'.format(
                self.cloud_image_count), img)


        self.cloud_image_count += 1
        return img, details
