#!/usr/bin/env python
"""
    Zarj Pathfinding experiment
"""
import sys
import rospy

sys.path.append('../lib')
sys.path.append('./lib')
import zarj
import zarj.navigation
import zarj.watching_processor
from zarj.utils import wait_for_walk

if __name__ == '__main__':
    rospy.init_node('eyes', anonymous=True)

    rospy.set_param('/multisense/camera/left/imager_rate', 1)
    rospy.set_param('/multisense/camera/right/imager_rate', 1)

    ME = zarj.ZarjOS()
    PROCESSOR = zarj.watching_processor.WatchingProcessor()
    PROCESSOR.side = ME.eyes.LEFT
    ME.eyes.register_processor(PROCESSOR)

    DEBUG = True

    ME.walk.set_stride(0.4)
    ME.walk.set_swing_time(0.8)
    ME.walk.set_transfer_time(0.3)

    ME.zps.path_start(DEBUG)
    ME.zps.walk_to_next_finish_box(DEBUG)
    ME.zps.exit_finish_box(DEBUG)
    ME.zps.walk_to_next_finish_box(DEBUG)
    ME.zps.exit_finish_box(DEBUG)
