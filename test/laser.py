#!/usr/bin/env python
""" Team ZARJ competition mainline """

# TODO - THIS IS INCOMPLETE

import rospy
import sys
import cv2
import numpy

sys.path.append('../lib')
sys.path.append('./lib')

from zarj.zarjos import ZarjOS
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from math import pi, sin, cos
from copy import deepcopy


class Main(object):
    """ Class to perform all main logic. """

    def __init__(self, name):
        rospy.init_node(name)

        self.start_rotation = None

        self.WIDTH = 320
        self.HEIGHT = 240

        self.set_x_range(-30.0, 30.0)
        self.set_y_range(-30.0, 30.0)
        self.set_z_range(-30.0, 30.0)

        self.completed_image = None

        if not rospy.has_param('/ihmc_ros/robot_name'):
            rospy.logerr("Cannot run team_zarj_main.py, missing parameters!")
            rospy.logerr("Missing parameter '/ihmc_ros/robot_name'")

        else:
            self.zarj = ZarjOS()

    def blank_image(self):
        size = self.HEIGHT, self.WIDTH, 1
        self.img = numpy.zeros(size, numpy.uint8)

    def set_x_range(self, minx, maxx):
        self.minx = minx
        self.maxx = maxx
        self.xscale = float(self.WIDTH) / (self.maxx - self.minx)

    def set_y_range(self, miny, maxy):
        self.miny = miny
        self.maxy = maxy
        self.yscale = float(self.HEIGHT) / (self.maxy - self.miny)

    def set_z_range(self, minz, maxz):
        self.minz = minz
        self.maxz = maxz
        self.zscale = 255.0 / (self.maxz - self.minz)

    def scale_point(self, x, y, z):
        ipx = int((x - self.minx) * self.xscale)
        ipy = int((y - self.miny) * self.yscale)
        ipz = int((z - self.minz)  * self.zscale)

        return ipx, ipy, ipz

    def rotation_complete(self):
        rospy.loginfo("Hey a rotation is done.")
        self.completed_image = deepcopy(self.img)
        cv2.imwrite("lidar.png", self.completed_image)
        cv2.imshow("My image", self.completed_image)
        pictures = self.zarj.eyes.get_images()
        rgb = cv2.flip(pictures[0], -1)
        cv2.imwrite("lefteye.png", rgb)
        cv2.imshow("Left eye", rgb)
        cv2.waitKey(0)
        #for y in range(0, 99):
        #    sys.stdout.write(str(y) + ": ")
        #    for x in range(0, 99):
        #        sys.stdout.write(str(self.img[x, y]) + ' ')
        #    sys.stdout.write(str(y) + "\n")

        self.blank_image()
        self.start_rotation = self.rotation

    def add_point(self, x, y, z):
        if x < self.minx or x > self.maxx or y < self.miny or y > self.maxy or z < self.minz or z > self.maxz:
            return

        ipx,ipy,ipz = self.scale_point(x, y, z)
        if ipz > 255:
            ipz = 255
        #print(str(ipx) + "," + str(ipy) + "," + str(ipz))
        if ipx >= 0 and ipx < self.WIDTH and ipy >= 0 and ipy < self.HEIGHT and ipz >= 0 and ipz < 256:
            self.img[ipy, ipx] = ipz

    def recv_joint_state(self, msg):
        """ Track the state of the Hokuyo joint """
        if msg.name[0] == "hokuyo_joint":
            if self.start_rotation is None:
                self.start_rotation = msg.position[0]
                self.blank_image()

            self.rotation = msg.position[0]
            if self.rotation >= (self.start_rotation + (2 * pi)):
                self.rotation_complete()

    def recv_scan(self, msg):
        """ Handle a laser scan """
        if not self.start_rotation is None:
            r = self.rotation % (2 * pi)
            a = msg.angle_min
            for range in msg.ranges:
                if not range == float('inf'):
                    x = cos(r) * sin(a) * range
                    y = sin(r) * sin(a) * range
                    z = cos(a) * range
                    self.add_point(x, y, z)

                    #print (str(self.rotation) + "," + str(r) + "," +
                    #    str(a) + "," + str(range) + "," +
                    #    str(x) + "," + str(y) + "," + str(z))
                a = a + msg.angle_increment
                #print (str(xlow) + "," + str(xhigh) + "," + str(ylow) + "," + str(yhigh))

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

        #self.blank_image()
        #for x in range(0, 99):
        #    self.add_point(x * 1.0, x * 1.0, 10)

        #cv2.imshow("Hackme", self.img)
        #cv2.waitKey(0)

        self.joint_subscriber = rospy.Subscriber("/multisense/joint_states", JointState, self.recv_joint_state)
        self.scan_subscriber = rospy.Subscriber("/multisense/lidar_scan", LaserScan, self.recv_scan)
        self.spindle = rospy.Publisher("/multisense/set_spindle_speed", Float64, queue_size=10)

        self.set_x_range(-1.0, 1.0)
        self.set_y_range(-1.0, 1.0)
        self.set_z_range(0.0, 2.0)

        rospy.sleep(0.1)
        rospy.loginfo("Setting spindle going")
        self.spindle.publish(Float64(1.0))

        #self.zarj.zps.look_down()

        rospy.loginfo("Spin forever, hopefully receiving data...")
        while True:
            rospy.sleep(1.0)
            #print "Got cloud {}x{}".format(resp.cloud.height, resp.cloud.width)
            #img = self.create_image_from_cloud(resp.cloud.points)
            #cv2.destroyAllWindows()
            #print "New image"
            #cv2.imshow("My image", img)
            #cv2.waitKey(1)
            #print resp
            #cv_image = self.bridge.imgmsg_to_cv2(resp.cloud, "bgr8")
            #cv2.imshow("Point cloud", cv_image)
    

if __name__ == '__main__':
    try:
        Main('team_zarj_laser').run()
    except rospy.ROSInterruptException:
        pass
