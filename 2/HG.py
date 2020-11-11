# -*- coding: utf-8 -*-
"""
Created on Thu Jun 21 22:07:07 2018

@author: Dynasting
"""
from driver import driver
import cv2
import numpy as np
import math
import time

dri=driver()
flag1=True
flag2=flag3=flag4=False
dri.setStatus(motor=0,servo=0,mode="speed")
cap = cv2.VideoCapture(1)
ret, frame = cap.read()
for i in range(15):
    ret, frame = cap.read()
    cv2.waitKey(150)
while(cap.isOpened()):
    # read image
    ret, frame = cap.read()
    rows,cols,dimension=frame.shape
    M=cv2.getRotationMatrix2D((cols/2,rows/2),180,1)
    img=cv2.warpAffine(frame,M,(cols,rows))
    # get hand data from the rectangle sub window on the screen
    #cv2.rectangle(img, (50,50), (400,500), (0,255,0),0)
    crop_img = img[50:400, 50:500]

    # convert to grayscale
    grey = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY)

    # applying gaussian blur
    value = (35, 35)
    blurred = cv2.GaussianBlur(grey, value, 0)

    # thresholdin: Otsu's Binarization method
    _, thresh1 = cv2.threshold(blurred, 127, 255,
                               cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)

    # show thresholded image
    #cv2.imshow('Thresholded', thresh1)

    # check OpenCV version to avoid unpacking error
    (version, _, _) = cv2.__version__.split('.')

    if version == '3':
        image, contours, hierarchy = cv2.findContours(thresh1.copy(), \
               cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    elif version == '2':
        contours, hierarchy = cv2.findContours(thresh1.copy(),cv2.RETR_TREE, \
               cv2.CHAIN_APPROX_NONE)

    # find contour with max area
    cnt = max(contours, key = lambda x: cv2.contourArea(x))

    # create bounding rectangle around the contour (can skip below two lines)
    x, y, w, h = cv2.boundingRect(cnt)
    #cv2.rectangle(crop_img, (x, y), (x+w, y+h), (0, 0, 255), 0)

    # finding convex hull
    hull = cv2.convexHull(cnt)

    # drawing contours
    drawing = np.zeros(crop_img.shape,np.uint8)
    #cv2.drawContours(drawing, [cnt], 0, (0, 255, 0), 0)
    #cv2.drawContours(drawing, [hull], 0,(0, 0, 255), 0)

    # finding convex hull
    hull = cv2.convexHull(cnt, returnPoints=False)

    # finding convexity defects
    defects = cv2.convexityDefects(cnt, hull)
    count_defects = 0
    #cv2.drawContours(thresh1, contours, -1, (0, 255, 0), 3)

    # applying Cosine Rule to find angle for all defects (between fingers)
    # with angle > 90 degrees and ignore defects
    for i in range(defects.shape[0]):
        s,e,f,d = defects[i,0]

        start = tuple(cnt[s][0])
        end = tuple(cnt[e][0])
        far = tuple(cnt[f][0])

        # find length of all sides of triangle
        a = math.sqrt((end[0] - start[0])**2 + (end[1] - start[1])**2)
        b = math.sqrt((far[0] - start[0])**2 + (far[1] - start[1])**2)
        c = math.sqrt((end[0] - far[0])**2 + (end[1] - far[1])**2)

        # apply cosine rule here
        angle = math.acos((b**2 + c**2 - a**2)/(2*b*c)) * 57

        # ignore angles > 90 and highlight rest with red dots
        if angle <= 90:
            count_defects += 1
            cv2.circle(crop_img, far, 1, [0,0,255], -1)
        #dist = cv2.pointPolygonTest(cnt,far,True)

        # draw a line from start to end i.e. the convex points (finger tips)
        # (can skip this part)
        #cv2.line(crop_img,start, end, [0,255,0], 2)
        #cv2.circle(crop_img,far,5,[0,0,255],-1)

    # define actions required
    if count_defects == 1 and flag1:
        #cv2.putText(img, "Message 1", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 2, 2)
        print "message 1"
        flag1=False
        flag2=True
        for i in range (1):
            dri.setStatus(motor = -0.05, servo=0,mode="speed")
            time.sleep(0.15)
    if count_defects == 2 and flag2:
        #cv2.putText(img, "Message 2", (5, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, 2)
        for i in range(1):
            dri.setStatus(motor = 0.05,servo=0, mode="speed")
            time.sleep(0.15)
        flag2=False
        flag3=True
        print "message 2"
    if count_defects == 3 and flag3:
        #cv2.putText(img, "Message 3", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 2, 2)
        st=1.0
        for i in range (6):
            dri.setStatus(motor = 0, servo=st, mode="speed")
            st=abs(1-st)
            time.sleep(0.3)
        flag3=False
        flag4=True
        print "message 3"
    if count_defects == 4 and flag4:
        #cv2.putText(img, "Message 4", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 2, 2)
        dri.setStatus(motor = 0, servo=-1.0, mode="speed")
        st=-1.0
        for i in range (6):
            dri.setStatus(motor = 0, servo=st, mode="speed")
            st=-1*abs(1+st)
            time.sleep(0.3)
        print "message 4"
        flag4=False
    else:
        #cv2.putText(img,"Default Message", (50, 50),cv2.FONT_HERSHEY_SIMPLEX, 2, 2)
        #dri.setStatus(motor = 0, servo=0, mode="speed")
        print "message default"
    time.sleep(0.2)
    # show appropriate images in windows
    #cv2.imshow('Basic Hand Gestures', img)
    all_img = np.hstack((drawing, crop_img))
    #cv2.imshow('Contours', all_img)
    #if count_defects>=1:
    #   break
    #k = cv2.waitKey(10)
    #if k == 27:
    #   break
d.close()
cap.release()
