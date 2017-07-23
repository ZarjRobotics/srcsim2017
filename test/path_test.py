#!/opt/local/bin/python

import sys
import cv2
sys.path.append('../lib/')
import zarj

IMAGES = ['test_images/navigation_126.565.png',
          'test_images/raw_path_54.743.png',
          'test_images/exit.png',
          'test_images/forward.png',
          'test_images/l45.png',
          'test_images/r45.png',
          'test_images/_img_0.png',
          'test_images/image6.png']

ME = zarj.ZarjOS()
for image in IMAGES:
    Image = cv2.imread(image)
    print ME.zps.determine_path(Image, True)
    print "<<<< FOCUS IMAGE WINDOW AND PRESS ANY KEY TO CONTINUE >>>>"
    cv2.waitKey(0)
