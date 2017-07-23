""" Internal Utility Functions """

__ALL__ = ['debug_sequence', 'log', 'is_cv2', 'is_cv3']

try:
    import rospy
except ImportError:
    import time

    START_TIME = time.time()

    def log(msg):
        """ log messages """
        print msg
    def debug_sequence():
        """ debug sequence """
        return time.time() - START_TIME
    def wait_for_walk(_):
        """ wait for a walk to finish """
        pass
else:
    def log(msg):
        """ log messages """
        rospy.loginfo(msg)
    def debug_sequence():
        """ debug sequence """
        return rospy.get_time()
    def wait_for_walk(zarj):
        """ wait for a walk to finish """
        while not zarj.walk.walk_is_done():
            rospy.sleep(0.2)
        rospy.sleep(0.2)


def is_cv2():
    """ Check if openCV 2 """
    return check_opencv_version("2.")

def is_cv3():
    """ Check if openCV 3 """
    return check_opencv_version("3.")

def check_opencv_version(major, lib=None):
    """ Check openCV Version """
    try:
        if lib is None:
            import cv2 as lib
        return lib.__version__.startswith(major)
    except ImportError:
        return False
