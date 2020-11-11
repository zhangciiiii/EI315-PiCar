# -*- coding: utf-8 -*-
"""
Created on Sun May 13 21:36:20 2018

@author: cxs
"""

import cv2
import numpy as np
import math

class Getlen:
    def __init__(self,p1,p2):
        self.x=p1[0]-p2[0]
        self.y=p1[1]-p2[1]
        #用math.sqrt（）求平方根
        self.len= math.sqrt((self.x**2)+(self.y**2))
    #定义得到直线长度的函数
    def getlen(self):
        return self.len

def detectGarage(img):
  try:
    frame = cv2.blur(img,(7,7))

    lower_blue = np.array([70,0,0])
    upper_blue = np.array([135,255,160])
    hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    lot = cv2.inRange(hsv, lower_blue, upper_blue)
    kernel = np.ones((10,10),np.uint8)

    for _ in range(3):
        lot = cv2.erode(lot,kernel,iterations = 1)
        lot = cv2.dilate(lot,kernel,iterations = 1)
    
    for _ in range(3):
        lot = cv2.dilate(lot,kernel,iterations = 1)
        lot = cv2.erode(lot,kernel,iterations = 1)
    
    out_binary, contours, hierarchy = cv2.findContours(lot,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)  
    max_cnt = 0
    max_area = 0
    for cnt in contours:
        if cv2.contourArea(cnt) < 640*480*0.0025 or cv2.contourArea(cnt) > 640*480*0.95:
            contours.remove(cnt)
    
        if cv2.contourArea(cnt) > max_area:
            max_area = cv2.contourArea(cnt)
            max_cnt = cnt
  
    #hull=cv2.convexHull(max_cnt)
    epsilon=0.05*cv2.arcLength(max_cnt,True)
    approx=cv2.approxPolyDP(max_cnt,epsilon,True)
    
    center_line,center=pointsProcess1(approx)
    front_center=pointsProcess2(approx)
    
    if (abs(center[1]-center_line[1])>=1.0):
        #The angle between the center line of the parking space and the middle line of the picture
        err_angle = math.atan ((-1.0*(center[0]-center_line[0])/(center[1]-center_line[1]))) 
    else:
        err_angle=0
   
    sp = lot.shape
    height = sp[0]  # height(rows) of image
    width = sp[1]  # width(colums) of image

    if center[1]<240:
        err_position =(front_center[0] -width/2)/320.0#The vertical distance to the center of the car
        status=1
    else:
        err_position =(center[0] -width/2)/320.0
        status=2
    stopFlagDist = Getlen((width/2 , height),center)
    if(stopFlagDist.getlen() < 70 or center[1]>440):
        stop = True
    else:
        stop = False

    return {'error':False,'stop':stop,'status':status,'err_position':err_position,'err_angle':err_angle}
  except:
    return {'error':True,'stop':True} 

#get center of further line and center of the garage
def pointsProcess1(approx):
    lt_x,lt_y=480,480
    rt_x,rt_y=480,480
    sum_x,sum_y,num_p=0,0,0
    for _ in approx:
        p=_[0]
        if p[1]<lt_y:
            lt_x,lt_y=p[0],p[1]
        sum_x+=p[0]
        sum_y+=p[1]
        num_p+=1
    for _ in approx:
        p=_[0]
        if p[1]<rt_y and p[1]>=lt_y and p[0] != lt_x:
            rt_x,rt_y=p[0],p[1]
    if (lt_x>rt_x):
        lt_x,rt_x=rt_x,lt_x
        lt_y,rt_y=rt_y,lt_y
    cx,cy=float(sum_x)/num_p,float(sum_y)/num_p
    return ((lt_x+rt_x)/2,(lt_y+rt_y)/2),(int(cx),int(cy))

 #get center of closer line   
def pointsProcess2(approx):
    lt_x,lt_y=0,0
    rt_x,rt_y=0,0
    for _ in approx:
        p=_[0]
        if p[1]>lt_y:
            lt_x,lt_y=p[0],p[1]
    for _ in approx:
        p=_[0]
        if p[1]>rt_y and p[1]<=lt_y and p[0] != lt_x:
            rt_x,rt_y=p[0],p[1]
    if (lt_x>rt_x):
        lt_x,rt_x=rt_x,lt_x
        lt_y,rt_y=rt_y,lt_y
    return ((lt_x+rt_x)/2,(lt_y+rt_y)/2)
