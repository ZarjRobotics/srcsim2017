""" Zarj transforms """

import sys

import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3

class TfZarj(object):
    """ Transforms for zarj """

    def __init__(self, name):
        self.tf_buffer = None
        self.tf_listener = None
        self.robot_name = name
        self.rate = rospy.Rate(10.0)
        self.base_pose = None
        self.pose_subscriber = None
        self.world_transform = Vector3(0, 0, 0)

        self.gazebo_subscriber = None
        self.gazebo_model = None

    def __del__(self):
        if self.pose_subscriber is not None:
            self.pose_subscriber.unregister()
        if self.gazebo_subscriber is not None:
            self.gazebo_subscriber.unregister()

    def received_gazebo_calibration(self, data):
        """ Receive a gazebo message for calibration """
        if self.gazebo_subscriber is None:
            return
        index = data.name.index('valkyrie')
        self.gazebo_model = data.pose[index].position
        self.gazebo_subscriber.unregister()
        self.gazebo_subscriber = None

    def received_pose_calibration(self, data):
        """ Receive a pose message for calibration """
        if self.pose_subscriber is None:
            return
        self.base_pose = data
        self.pose_subscriber.unregister()
        self.pose_subscriber = None
        self.world_transform = Vector3(
            data.pose.pose.position.x - self.gazebo_model.x,
            data.pose.pose.position.y - self.gazebo_model.y,
            data.pose.pose.position.z - self.gazebo_model.z)

    def init_transforms(self):
        """ Initialize the tf2 transforms """
        rospy.loginfo("Start transform calibration.")
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(300.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        while True:
            try:
                now = rospy.get_rostime()
                self.tf_buffer.lookup_transform('head',
                                                'left_camera_optical_frame',
                                                now - rospy.Duration(0.1))
            except:
                self.rate.sleep()
                continue
            break
        rospy.loginfo('transform calibration finished.')

    def set_world_transform(self, vector):
        """ Set the world transform for our frames """
        self.world_transform = vector

    def transform_to_world(self, point):
        """ Transform a point by the world offset """
        point.x -= self.world_transform.x
        point.y -= self.world_transform.y
        point.z -= self.world_transform.z

    def transform_from_world(self, point):
        """ Transform a world point by the robot frame"""
        point.x += self.world_transform.x
        point.y += self.world_transform.y
        point.z += self.world_transform.z

    def lookup_frame(self, frame):
        """ helper to lookup frames"""
        source = "/ihmc_ros/{0}/{1}".format(self.robot_name, frame)
        if rospy.has_param(source):
            return rospy.get_param(source)
        return None

    def lookup_transform(self, target_frame, source_frame, time):
        """ Lookup a transform """
        if time > rospy.Time(0.1):
            time - rospy.Duration(0.1)
        return self.tf_buffer.lookup_transform(target_frame, source_frame, time)

    def lookup_frame_in_world(self, source_frame, time):
        """ Lookup a frame in the world """
        return self.lookup_transform('world', source_frame, time)

    def lookup_frame_in_world(self, source_frame, time):
        """ Lookup a frame in the world """
        value = self.lookup_transform('world', source_frame, time)
        self.transform_to_world(value.transform.translation)
        return value

    def received_pose_checkin(self, data):
        """ Receive a pose message for a checkin """
        self.pose_subscriber.unregister()
        self.pose_subscriber = None

        self.transform_to_world(data.pose.pose.position)
        rospy.loginfo("New Robot Pose:" + str(data))
        head = self.lookup_frame_in_world('head',
                                           data.header.stamp)
        rospy.loginfo("New Robot Head:" + str(head))
        head = self.lookup_frame_in_world('upperNeckPitchLink',
                                           data.header.stamp)
        rospy.loginfo("New Robot Upper Neck:" + str(head))
        foot = self.lookup_frame_in_world('leftFoot',
                                           data.header.stamp)
        rospy.loginfo("New Robot Left Foot:" + str(foot))
        foot = self.lookup_frame_in_world('rightFoot',
                                           data.header.stamp)
        rospy.loginfo("New Robot Right Foot:" + str(foot))
        hand = self.lookup_frame_in_world('leftPalm',
                                           data.header.stamp)
        rospy.loginfo("New Robot Left Hand:" + str(hand))
        hand = self.lookup_frame_in_world('rightPalm',
                                           data.header.stamp)
        rospy.loginfo("New Robot Right Hand:" + str(hand))

    def robot_checkin(self):
        """ Check in with the robot """
        output = "/ihmc_ros/{0}/output/".format(self.robot_name)
        self.pose_subscriber = rospy.Subscriber(output + "robot_pose",
                                                Odometry,
                                                self.received_pose_checkin)
        while self.pose_subscriber is not None:
            self.rate.sleep()
