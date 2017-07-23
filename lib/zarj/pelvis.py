"""
    Logic to control the Pelvis
"""
from math import radians

from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_about_axis, \
                               euler_from_quaternion, \
                               quaternion_from_euler
import tf2_geometry_msgs
from ihmc_msgs.msg import PelvisOrientationTrajectoryRosMessage
from ihmc_msgs.msg import SO3TrajectoryPointRosMessage
from ihmc_msgs.msg import ChestTrajectoryRosMessage
from ihmc_msgs.msg import PelvisHeightTrajectoryRosMessage
from ihmc_msgs.msg import TrajectoryPoint1DRosMessage
import rospy
from .utils import log

class Pelvis(object):
    """ Control of the torso """

    def __init__(self, zarj_os):
        log("Zarj pelvis initialization begin")

        self.zarj = zarj_os
        self.publishers = []
        self.orientation_publisher = rospy.Publisher(
            "/ihmc_ros/{0}/control/pelvis_orientation_trajectory".format(self.zarj.robot_name),
           PelvisOrientationTrajectoryRosMessage, queue_size=10)
        self.publishers.append(self.orientation_publisher)

        self.height_publisher = rospy.Publisher(
            "/ihmc_ros/{0}/control/pelvis_height_trajectory".format(self.zarj.robot_name),
           PelvisHeightTrajectoryRosMessage, queue_size=10)
        self.publishers.append(self.orientation_publisher)

        self.chest_publisher = rospy.Publisher(
            "/ihmc_ros/{0}/control/chest_trajectory".format(
                self.zarj.robot_name),
            ChestTrajectoryRosMessage, queue_size=10)
        self.publishers.append(self.chest_publisher)
        self.zarj.wait_for_publishers(self.publishers)

        log("Zarj initialization completed")

    def __del__(self):
        log("TODO - implement __del__ method.")
        self.zarj.close_publishers(self.publishers)

    def get_lean_points(self, angle):
        """ Return a set of trajectory points to accomplish a lean """
        point = SO3TrajectoryPointRosMessage()

        point.time = 1.0  # Just give it a second to get there
        rotate = quaternion_about_axis(radians(angle), [0, 0, -1])
        point.orientation = Quaternion(rotate[1], rotate[2], rotate[3], rotate[0])
        point.angular_velocity = Vector3(0, 0, 0)
        log('Lean to: {}'.format(point))

        points = []
        points.append(point)
        return points


    def lean(self, angle):
        """ Lean forward or back to a given angle """

        msg = PelvisOrientationTrajectoryRosMessage()
        msg.taskspace_trajectory_points = self.get_lean_points(angle)

        msg.execution_mode = msg.OVERRIDE
        msg.unique_id = self.zarj.uid

        self.orientation_publisher.publish(msg)

    def lean_body(self, angle):
        """ Lean forward or back a given angle """
        chest_position = self.zarj.transform.lookup_transform(
            'world', 'torso', rospy.Time()).transform

        log("Lean body {} degrees".format(angle))
        msg = ChestTrajectoryRosMessage()
        msg.unique_id = self.zarj.uid

        msg.execution_mode = msg.OVERRIDE

        euler = euler_from_quaternion((chest_position.rotation.w,
                                       chest_position.rotation.x,
                                       chest_position.rotation.y,
                                       chest_position.rotation.z))

        point = SO3TrajectoryPointRosMessage()
        point.time = 1.0
        point.orientation = chest_position.rotation

        qua = quaternion_from_euler(euler[0], euler[1] + radians(angle),
                                    euler[2])
        point.orientation = Quaternion(qua[1], qua[2], qua[3], qua[0])
        point.angular_velocity = Vector3(0.0, 0.0, 0.0)

        msg.taskspace_trajectory_points = [point]

        self.chest_publisher.publish(msg)

    def lean_body_to(self, angle, wait=True):
        """ Lean forward or back to a given angle """
        chest_position = self.zarj.transform.lookup_transform(
            'world', 'torso', rospy.Time()).transform

        log("Lean body to {} degrees".format(angle))
        msg = ChestTrajectoryRosMessage()
        msg.unique_id = self.zarj.uid

        msg.execution_mode = msg.OVERRIDE

        euler = euler_from_quaternion((chest_position.rotation.w,
                                       chest_position.rotation.x,
                                       chest_position.rotation.y,
                                       chest_position.rotation.z))

        point = SO3TrajectoryPointRosMessage()
        point.time = 1.0
        point.orientation = chest_position.rotation

        qua = quaternion_from_euler(euler[0], radians(angle), euler[2])
        point.orientation = Quaternion(qua[1], qua[2], qua[3], qua[0])
        point.angular_velocity = Vector3(0.0, 0.0, 0.0)

        msg.taskspace_trajectory_points = [point]

        self.chest_publisher.publish(msg)
        if wait:
            rospy.sleep(point.time + 0.2)


    def turn_body(self, angle):
        """
            Turn the torso by a given angle

            Angles smaller than 110 or larger than 250 are unstable
        """
        log("Turn body {} degrees".format(angle))
        chest_position = self.zarj.transform.lookup_transform(
            'world', 'torso', rospy.Time()).transform

        msg = ChestTrajectoryRosMessage()
        msg.unique_id = self.zarj.uid

        msg.execution_mode = msg.OVERRIDE

        euler = euler_from_quaternion((chest_position.rotation.w,
                                       chest_position.rotation.x,
                                       chest_position.rotation.y,
                                       chest_position.rotation.z))

        point = SO3TrajectoryPointRosMessage()
        point.time = 1.0

        qua = quaternion_from_euler(euler[0] + radians(angle), euler[1],
                                    euler[2])
        point.orientation = Quaternion(qua[1], qua[2], qua[3], qua[0])
        point.angular_velocity = Vector3(0.0, 0.0, 0.0)

        msg.taskspace_trajectory_points = [point]

        self.chest_publisher.publish(msg)

    def turn_body_to(self, angle, wait=True):
        """
            Turn the torso to a given angle as related to the pelvis

            Angles smaller than 110 or larger than 250 are unstable
        """
        log("Turn body to {} degrees".format(angle))
        chest_position = self.zarj.transform.lookup_transform(
            'pelvis', 'torso', rospy.Time()).transform

        msg = ChestTrajectoryRosMessage()
        msg.unique_id = self.zarj.uid

        msg.execution_mode = msg.OVERRIDE

        euler = euler_from_quaternion((chest_position.rotation.w,
                                       chest_position.rotation.x,
                                       chest_position.rotation.y,
                                       chest_position.rotation.z))

        qua = quaternion_from_euler(radians(-angle-180), euler[1], euler[2])

        pose = PoseStamped()
        pose.pose.orientation = Quaternion(qua[1], qua[2], qua[3], qua[0])
        pose.header.frame_id = 'pelvis'
        pose.header.stamp = rospy.Time()
        result = self.zarj.transform.tf_buffer.transform(pose, 'world')


        point = SO3TrajectoryPointRosMessage()
        point.time = 1.0
        point.orientation = result.pose.orientation
        point.angular_velocity = Vector3(0.0, 0.0, 0.0)

        msg.taskspace_trajectory_points = [point]

        self.chest_publisher.publish(msg)

        if wait:
            rospy.sleep(point.time + 0.1)

    def turn_body_to_pose(self, pose):
        """
            Turn the torso to a given orientation

            Angles smaller than 110 or larger than 250 are unstable
        """
        log("Turn body to match pose {}".format(pose))
        msg = ChestTrajectoryRosMessage()
        msg.unique_id = self.zarj.uid
        msg.execution_mode = msg.OVERRIDE

        result = self.zarj.transform.tf_buffer.transform(pose, 'world')

        point = SO3TrajectoryPointRosMessage()
        point.time = 1.0
        point.orientation = result.pose.orientation
        point.angular_velocity = Vector3(0.0, 0.0, 0.0)

        msg.taskspace_trajectory_points = [point]

        self.chest_publisher.publish(msg)

    def get_height_points(self, delta):
        """ Return a set of trajectory points to accomplish a pelvis raise """
        point = TrajectoryPoint1DRosMessage()
        point.time = 0.2
        pelvis_position = self.zarj.transform.lookup_transform(
            'world', 'pelvis', rospy.Time()).transform
        height = pelvis_position.translation.z

        height += delta
        point.position = height
        points = []
        points.append(point)
        return points

    def change_height(self, delta):
        """ Change the pelvis height """
        log('Change pelvis height by {}'.format(delta))
        msg = PelvisHeightTrajectoryRosMessage()
        msg.trajectory_points = self.get_height_points(delta)

        msg.execution_mode = msg.OVERRIDE
        msg.unique_id = self.zarj.uid

        self.height_publisher.publish(msg)
