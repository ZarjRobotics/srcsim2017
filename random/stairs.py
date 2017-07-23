#!/usr/bin/env python
"""
    Zarj stair climbing experiment
"""
import sys
import rospy

sys.path.append('../lib')
sys.path.append('./lib')
import zarj
from zarj.utils import wait_for_walk

def up_stair(run, rise, cnt):
    ME.walk.step_up(run, rise)
    wait_for_walk(ME)
    print 'ARIC:', cnt, ' Stablize... Hold it...'
    rospy.sleep(0.2)
    ME.go_home(0, 2)
    rospy.sleep(0.2)


if __name__ == '__main__':
    rospy.init_node('eyes', anonymous=True)
    ME = zarj.ZarjOS()

    ME.walk.set_swing_time(1.0)
    ME.walk.set_transfer_time(0.5)
    ME.walk.set_stance_width(0.25)

# This I am positive of
    step = 0.2031
    first_step = 0.1129
    last_step = 0.1804
    depth = 0.2388 + 0.01 # yeah step a little longer than the footbed
# end being positive

    ME.walk.fix_stance()
    wait_for_walk(ME)
    ME.pelvis.lean_body_to(-20)

    ME.pelvis.change_height(0.1)
    rospy.sleep(0.5)

    up_stair(depth, first_step, 1)

    up_stair(depth, step, 2)
    up_stair(depth, step, 3)
    up_stair(depth, step, 4)
    up_stair(depth, step, 5)
    up_stair(depth, step, 6)
    up_stair(depth, step, 7)
    up_stair(depth, step, 8)

    up_stair(depth + 0.05, last_step, 9)
    print 'ARIC: DONE'
    ME.pelvis.lean_body_to(0)
    ME.pelvis.change_height(-0.1)
