# -*- coding: utf-8 -*-
"""
Created on Sun May 06 16:52:48 2018

@author: Yunpeng
"""
#from __future__ import print_function
from __future__ import division
import cv2
import numpy as np

'''
Read Whether There Is a Stop Sign And Return the Value
Input     frame - Image in numpy.array
Output    dict  -  {'x':1,'y':1,'size':10,'detect':True,'error':False}
'''
def readStopSign(frame):
    try:
        # MIN THRESHOLD FOR DETECTION
        MIN_THRESHOLD = 15

        # set blue thresh
        lower_blue = np.array([100,120,120])
        upper_blue = np.array([135,255,255])
        # blur
        hsv = cv2.GaussianBlur(frame,(3,3),0)
        
        # RGB to HSV
        hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    
        # inRange
        blue = cv2.inRange(hsv, lower_blue, upper_blue)
        #cv2.imshow("blue", blue)
        
        # Erosion
        kernel = np.ones((2,2),np.uint8)
        '''
        for _ in range(3):
            erosion = cv2.erode(blue,kernel,iterations = 1)
            erosion = cv2.dilate(blue,kernel,iterations = 1)
        '''
        blue=cv2.morphologyEx(blue,cv2.MORPH_CLOSE,kernel)
        #Laplacian
        canny = cv2.Canny(blue,120,180)
        
        #circle
        circles = cv2.HoughCircles(canny,cv2.HOUGH_GRADIENT,1,20,
                                param1=50,param2=60,minRadius=0,maxRadius=0)
    
        max_r = 0
        max_xy = (0,0)
        try:
            circles = np.uint16(np.around(circles))
        except AttributeError:
            return {'detect':False,'error':False}
        for i in circles[0,:]:
            # draw the outer circle
            #cv2.circle(frame,(i[0],i[1]),i[2],(0,255,0),2)
            # draw the center of the circle
            #cv2.circle(frame,(i[0],i[1]),2,(0,0,255),3)
            # 19  26
            if i[2] > max_r:
                max_xy = (i[0],i[1])
                max_r = i[2]
                circle = i
                
        #cv2.imshow("c",frame)
        #cv2.waitKey(0)
   
        #print(max_r)
        #print(max_xy)
        # Return Output
        if max_r > MIN_THRESHOLD:
            return {'x':circle[0],'y':circle[1],'size':circle[2],'detect':True,'error':False}
        else:
            return {'detect':False,'error':False}
        
    except:
        return {'detect':False,'error':True}
