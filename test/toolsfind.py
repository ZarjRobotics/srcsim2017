#!/usr/bin/env python

import sys
import pickle
sys.path.append('../lib/')
from zarj.things import Things


if __name__ == '__main__':
    try:
        for img in sys.argv[1:]:
            pickled_stereo_images = pickle.load(open(img, "rb" ))
            things = Things(pickled_stereo_images['img'], pickled_stereo_images['details'], True)

    except rospy.ROSInterruptException:
        pass
