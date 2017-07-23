#!/opt/local/bin/python

import sys
import cv2
sys.path.append('../lib/')
import zarj.detection_processor

IMAGES = ['test_images/image1.png',
          'test_images/image2.png',
          'test_images/image3.png',
          'test_images/image4.png',
          'test_images/image5.png',
          'test_images/image6.png']

def showimage(img, name):
    """ Show an image waiting for a keystroke """
    cv2.imshow(name, img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

for image in IMAGES:
    Image = cv2.imread(image)
    pp = zarj.detection_processor.DetectionProcessor('../cascade/satellite.xml')
    pp.process_image(Image, None, 'test')
    print "<<<< FOCUS IMAGE WINDOW AND PRESS ANY KEY TO CONTINUE >>>>"
    showimage(Image, 'test')
