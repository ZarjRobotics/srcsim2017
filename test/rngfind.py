#!/opt/local/bin/python

import sys
import cv2
sys.path.append('../lib/')
import zarj

IMAGES = [('test_images/ahead.png', zarj.navigation.PERSPECTIVE_AHEAD),
          ('test_images/head_down.png', zarj.navigation.PERSPECTIVE_HEAD_DOWN),
          ('test_images/full_down.png', zarj.navigation.PERSPECTIVE_FULL_DOWN)]

for image, persp in IMAGES:
    Image = cv2.imread(image)
    ME = zarj.ZarjOS()
    img = persp.build_rangefinding_image(Image)
    print "<<<< FOCUS IMAGE WINDOW AND PRESS ANY KEY TO CONTINUE >>>>"
    cv2.imshow("ranges", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
