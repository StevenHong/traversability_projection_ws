# This package converts the stereo images to a depth image
#
# Written by Steven Hong, 01/27/2022
#

import numpy as np
import cv2 as cv
from matplotlib import pyplot as plt

for x in range(1200):
	number = '%06d' % x
	path1 = '/home/steven/kitti/odometry/sequences/10/image_2/' + number + '.png'
	path2 = '/home/steven/kitti/odometry/sequences/10/image_3/' + number + '.png'
	print(path1)
	imgL = cv.imread(path1,0)
	imgR = cv.imread(path2,0)

	stereo = cv.StereoBM_create(numDisparities=16, blockSize=19)
	disparity = stereo.compute(imgL,imgR, disparity=cv.CV_16UC1)
	#plt.imshow(disparity,'gray')
	#plt.show()
	exportp = '/home/steven/kitti/10/depth_img/' + number + '.png'
	cv.imwrite(exportp,image_message)


