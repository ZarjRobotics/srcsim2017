#!/usr/bin/env python

""" Exploring transforms """

import sys
import rospy

sys.path.append('../lib')
import zarj.tf

if __name__ == '__main__':
    try:
        if not rospy.has_param('/ihmc_ros/robot_name'):
            rospy.loginfo("Cannot run zarj_tf.py, missing parameters!")
            rospy.loginfo("Missing parameter '/ihmc_ros/robot_name'")

        else:
            ROBOT_NAME = rospy.get_param('/ihmc_ros/robot_name')

            rospy.init_node('tf_zarj')

            TF = zarj.tf.TfZarj(ROBOT_NAME)
            TF.init_transforms()
            TF.robot_checkin()

    except rospy.ROSInterruptException:
        pass
