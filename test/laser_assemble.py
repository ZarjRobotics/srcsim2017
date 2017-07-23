#!/usr/bin/env python
""" Team ZARJ competition mainline """

import rospy
import sys
import cv2
import numpy

sys.path.append('../lib')
sys.path.append('./lib')

from zarj.zarjos import ZarjOS
from srcsim.msg import Task
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import PointCloud2
from laser_geometry import LaserProjection
from laser_assembler.srv import *


class Main(object):
    """ Class to perform all main logic. """

    def __init__(self, name):
        rospy.init_node(name)

        if not rospy.has_param('/ihmc_ros/robot_name'):
            rospy.logerr("Cannot run team_zarj_main.py, missing parameters!")
            rospy.logerr("Missing parameter '/ihmc_ros/robot_name'")

        else:
            self.zarj_os = ZarjOS()

    def scan(self, msg):
        """ Handle a laser scan """
        rospy.loginfo("scan came in")
        rospy.loginfo("frame " + msg.header.frame_id + "; angle_min " + str(msg.angle_min) +
            "; angle_max " + str(msg.angle_max) + "; angle_increment " + str(msg.angle_increment))
        #img = self.projection.projectLaser(msg)
        #rospy.loginfo("h: " + str(img.height) + " w: " + str(img.width) + " step: " + str(img.point_step))

    def create_image_from_cloud(self, points):
        size = 100, 100, 1
        img = numpy.zeros(size, numpy.uint8)

        #tf = self.zarj_os.transform.lookup_transform('world', 'head', rospy.get_rostime())
        print points[0]
        #print self.zarj_os.transform.transform_from_world(points[0])
        for p in points:
            #tp = self.zarj_os.transform.transform_from_world(p)
            ipx = int((p.x * 100.0) + 50.0)
            ipy = int((p.y * 100.0) + 50.0)
            if ipx >= 0 and ipx < 100 and ipy >= 0 and ipy < 100:
                ipz = int(p.z * 100.0)
                if ipz > 255:
                    ipz = 255
                img[ipx, ipy] = ipz
        return img


    def run(self):
        """ Run our code """
        rospy.loginfo("Start laser test code")
        rospy.wait_for_service("assemble_scans2")

        # Todo - publish the spin logic...
        self.laser_publisher = rospy.Publisher("/multisense/lidar_cloud", PointCloud, queue_size=10)
        self.scan_service = rospy.ServiceProxy('assemble_scans', AssembleScans)
        #self.bridge = CvBridge()

        #self.zarj_os.zps.look_down()

        while True:
            begin = rospy.get_rostime()
            rospy.sleep(3.0)
            resp = self.scan_service(begin, rospy.get_rostime())
            self.laser_publisher.publish(resp.cloud)
            #print "Got cloud {}x{}".format(resp.cloud.height, resp.cloud.width)
            img = self.create_image_from_cloud(resp.cloud.points)
            cv2.destroyAllWindows()
            print "New image"
            cv2.imshow("My image", img)
            cv2.waitKey(1)
            #print resp
            #cv_image = self.bridge.imgmsg_to_cv2(resp.cloud, "bgr8")
            #cv2.imshow("Point cloud", cv_image)
    

if __name__ == '__main__':
    try:
        Main('team_zarj_laser').run()
    except rospy.ROSInterruptException:
        pass
