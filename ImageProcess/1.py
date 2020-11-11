
import math
import cv2
import numpy as np
import os
import matplotlib.pyplot as plt

import copy


def cv_imread(file_path):
    cv_img = cv2.imdecode(np.fromfile(file_path,dtype=np.uint8),-1)
    return cv_img


# read the image and convert to gray
img = cv_imread('straight.jpg') # straight left right
img = img[:150,:,:] #150,640
img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# use a circle kernel to mask the obstacle
cv2.circle(img,(240,-80),150,255,-1)
cv2.imwrite('straight_gray.jpg'  , img)

# convert to binary
_, img = cv2.threshold(img, 100, 255, cv2.THRESH_BINARY)

# dilate operation
kernel = np.ones((3, 3), np.uint8)
img = cv2.dilate(img,kernel,iterations=3)

# use hough transformation to detect lines
mask = img.copy()
mask[mask<100]=1
mask[mask>=100]=0
lines = cv2.HoughLines(mask,1,np.pi/180,100) 


# count
lines = lines[:,0,1]

left_cnt = copy.deepcopy(lines)
right_cnt = copy.deepcopy(lines)
left_cnt[left_cnt<1.5]=0
left_cnt[left_cnt>1.5]=1
left_cnt = left_cnt.sum()

right_cnt[right_cnt<1.5]=1
right_cnt[right_cnt>1.5]=0
right_cnt = right_cnt.sum()


print(left_cnt,right_cnt)
cv2.imwrite('straight_binary.jpg'  , img)

       
