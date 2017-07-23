#!/opt/local/bin/python
''' This is the tool to help generate map_matrix values for perspectives... '''

import cv2
import numpy as np

# Here is the image that is being used.
# Ideally you want a streight ahead path for as far as you can get. One corner
# of the Path at 0,0 (is ideal)
IMAGE = cv2.imread('output2.png')

# This is the trapazoid of the path:
# Top left corner [x,y],  Bottom left corner, top right corner, bottom right
# corner
SRC = np.array([[0, 0], [155, 348], [330, 0], [157, 348]], dtype="float32")

# This is the calculated unWARPED rectangle
DST = np.array([SRC[0], [SRC[0][0], SRC[1][1]], SRC[2], [SRC[2][0], SRC[3][1]]],
               dtype="float32")

START = IMAGE[SRC[0][1].astype(int):SRC[1][1].astype(int),
              SRC[0][0].astype(int):SRC[2][0].astype(int)]

# Just show our image with just the path shown
cv2.imshow('path', START)

# Here is where we calculated the transform
M = cv2.getPerspectiveTransform(SRC, DST)

print repr(M)

# NOTE: If your path is NOT cornered at 0,0 in the source image then the
#       unwarped image will unwarp the left part of the image

WARPED = cv2.warpPerspective(START, M, (START.shape[1], START.shape[0]))
cv2.imwrite('warped.png', WARPED)
cv2.imshow('warped', WARPED)
cv2.waitKey(0)
cv2.destroyAllWindows()
