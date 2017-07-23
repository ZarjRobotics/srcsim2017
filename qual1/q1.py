#!/usr/bin/env python
"""
    Zarj solution for qual 1.

    This requires stereo_image_proc to be started, with a
    correlation_window_size set to 31
"""
import sys
import rospy
import os

import tf2_geometry_msgs
sys.path.append('../lib')
sys.path.append('./lib')
import zarj.eyes

from srcsim.msg import Console
from std_msgs.msg import Empty
import numpy as np

STABLIZE_TIME = rospy.Duration.from_sec(0.1)  # minimum time to publish
if 'ZARJ_STABLIZE_TIME' in os.environ:
    STABLIZE_TIME = rospy.Duration.from_sec(float(os.environ['ZARJ_STABLIZE_TIME']))

MIN_REPORT_COUNT = 1 # minimum reports before publishing
if 'ZARJ_REPORT_COUNT' in os.environ:
    MIN_REPORT_COUNT = int(os.environ['ZARJ_REPORT_COUNT'])

FRAME = 'head'
if 'ZARJ_FRAME' in os.environ:
    FRAME = os.environ['ZARJ_FRAME']

QUAL_GOING = True

def qual1_publish(name, center):
    report = Console()
    report.r = 0
    report.g = 0
    report.b = 0

    if name == 'red':
        report.r = 1
    elif name == 'green':
        report.g = 1
    elif name == 'blue':
        report.b = 1

    report.x = center.point.x
    report.y = center.point.y
    report.z = center.point.z
    rospy.loginfo('Publishing at {}'.format(center.header.stamp.secs))
    #print report

    results.publish(report)

def let_batch_know(signal):
    if 'BATCH_SIGNAL_DIR' in os.environ:
        open(os.environ['BATCH_SIGNAL_DIR'] + "/"+signal+".signal", "w")

def qual_start(name):
    rospy.loginfo('Qual 1 start!')

def qual_end(name):
    global QUAL_GOING
    rospy.loginfo('Qual 2 end!')
    let_batch_know("done")
    QUAL_GOING = False

def build_qual1_detections(eyes):
    lower_blue = np.array([110, 100, 100])
    upper_blue = np.array([130, 255, 255])
    lower_green = np.array([50, 100, 100])
    upper_green = np.array([70, 255, 255])
    lower_red = np.array([0, 100, 100])
    upper_red = np.array([10, 255, 255])

    red = zarj.eyes.DetectionRequest('red', lower_red, upper_red, MIN_REPORT_COUNT,
                           STABLIZE_TIME, qual1_publish)
    blue = zarj.eyes.DetectionRequest('blue', lower_blue, upper_blue, MIN_REPORT_COUNT,
                           STABLIZE_TIME, qual1_publish)
    green = zarj.eyes.DetectionRequest('green', lower_green, upper_green,
                             MIN_REPORT_COUNT, STABLIZE_TIME, qual1_publish)

    eyes.add_detection_request(red)
    eyes.add_detection_request(green)
    eyes.add_detection_request(blue)

    w1 = np.array([0, 0, 200])
    w2 = np.array([0, 0, 255])

    w = zarj.eyes.DetectionRequest('white', w1, w2,
                             1, STABLIZE_TIME, [qual_start, qual_end, None])
    w.record_baseline(1000)

    eyes.add_detection_request(w)

if __name__ == '__main__':

    rospy.init_node('q1', anonymous=True)

    # make sure our correlation_window_size is correct
    rospy.set_param('/multisense/camera/stereo_proc/correlation_window_size',
                    31)
    rospy.set_param('/multisense/camera/left/imager_rate', 49)
    rospy.set_param('/multisense/camera/right/imager_rate', 49)

    results = rospy.Publisher("/srcsim/qual1/light", Console, queue_size=10)

    il = zarj.eyes.ZarjEyes(FRAME)
    build_qual1_detections(il)
    il.start_processing()

    while not il.ready():
        rospy.sleep(0.1)

    rospy.loginfo("Ready for qual 1")
    # start qual1
    start = rospy.Publisher("/srcsim/qual1/start", Empty, latch=True,
                             queue_size=10)
    try:
        start.publish(Empty())
    except:
        print "Publish failed"

    try:
        while QUAL_GOING:
            rospy.sleep(0.1)
    except KeyboardInterrupt:
        print "Exit"
