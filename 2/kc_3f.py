# -*- coding: utf-8 -*-
"""
Created on Sun May 06 16:48:20 2018

@author: Yunpeng
"""

import cv2
from driver import driver
import time
import numpy as np

from line import *
from stopsign import readStopSign
from park import detectGarage

k_p_servo=1.3  #when following the black line, control the servo in P control and set motor as a constant value  
thresh_y=400
thresh_r=40

def kc_3f():
    d=driver()
    d.setStatus(motor=0,servo=0,mode="speed")

    cap = cv2.VideoCapture(1) #open forward camera
    for i in range(15): #mask pictures with bad light condition
        ret,frame=cap.read()
        cv2.waitKey(100)
    
    #first stage: following black line
    sm,st=0.0,0.0
    last_st=st
    sharpturn_count=0
    while True:
        #get real-time image
        ret, frame = cap.read()
        
        #reverse image to get the right perspective
        rows,cols,dimension=frame.shape           
        M=cv2.getRotationMatrix2D((cols/2,rows/2),180,1)
        dst=cv2.warpAffine(frame,M,(cols,rows))
        #if the car find the stop sign, then quit "following black line" mode
        d_sign=readStopSign(dst)
        if d_sign['detect']:
            d.setStatus(motor=0.0, servo=0.0, dist=0x00, mode="stop")
            break
        
        #cut out a roi of image around aim point and handle it to a binary image 
        thresh1=img_process(dst[425:435,:])# thresh value is 90, need to adjust accordingly
        thresh2=img_process(dst[400:410,:])# thresh value is 90, need to adjust accordingly
     
        #cv2.imwrite("thresh.png",thresh)   
        #find center of black line around aim point and control the car accordingly 
        flag1,target_y1=find_blackline(thresh1,640,50)
        flag2,target_y2=find_blackline(thresh2,640,50)
        if (flag1==0 and flag2==0): #formal mode
            st=standardize(k_p_servo*(320.0-0.3*target_y1-0.7*target_y2)/320.0)
            sm=0.05
        elif (flag1==1 or flag2==1):#crossroad appears
            thresh=img_process(dst[370:380,:])
            flag3,target_y3=find_blackline(thresh,640,50)
            if (flag3==0):
                st=standardize(k_p_servo*(1-target_y3/320.0))
                sm=0.1
            else:
                st=last_st
                sm=0.03
        elif (flag2==2 or flag1==2):       #black points is too little,black line is lost
            st=last_st
            sm=0.05
        
        if abs(st)==1:
            sharpturn_count +=1
        else:
            sharpturn_count=0

        if sharpturn_count==7:
            d.setStatus(motor = -0.05, servo = 0)
        elif sharpturn_count==8:
            d.setStatus(motor = 0, servo = 0)
            sharpturn_count=0
        else:
            d.setStatus(motor = sm, servo = st)
        print "flag1",flag1,"target_y1",target_y1,"flag2",flag2,"target_y2",target_y2
        print("Motor: %0.2f, Servo: %0.2f" % (sm,st))
        last_st=st
        time.sleep(0.2)
    
    #second stage: move towards the stop sign until the distance between car and sign is small enough
    while True:
        ret, frame = cap.read()
        #reverse image to get the right perspective
        rows,cols,dimension=frame.shape           
        M=cv2.getRotationMatrix2D((cols/2,rows/2),180,1)
        dst=cv2.warpAffine(frame,M,(cols,rows))
        d_sign=readStopSign(dst)
        if not d_sign['error'] and d_sign['detect']:
            x,y,r=d_sign['x'],d_sign['y'],d_sign['size']
            if y>=thresh_y or r>=thresh_r:
                d.setStatus(motor=0.0, servo=0.0, dist=0x00, mode="stop")
                break
            else:
                st,sm=standsrdize(1.0-x/320.0),0.05
                d.setStatus(motor = sm, servo = st, mode="speed")
                print("Motor: %0.2f, Servo: %0.2f" % (sm,st))
                time.sleep(0.5)
        else:
            d.setStatus(motor=0,servo=0,mode="stop")
            #print "error! cann't find the stop sign."
    cap.release()
    
    #third stage: open back camera and park the car to specific garage
    cap = cv2.VideoCapture(0) #open back camera
    for i in range(15): #mask pictures with bad light condition
        ret,frame=cap.read()
        cv2.waitKey(100)
    a,b=1.2,0.2
    check_flag=False
    last_st=0
    while True:
        ret, frame = cap.read()
        result=detectGarage(frame)
        if not result['error']:
            if result['stop']:
                d.setStatus(motor=0.0, servo=0.0, dist=0x00, mode="stop")
                if not check_flag:
                    check_flag=True
                    time.sleep(0.5)
                    continue
                else:
                    break
            if result['status']==1:
                a,b=1.0,0.22
            else:
                a,b=1.3,0.27
            err_p,err_a=result['err_position'],result['err_angle']
            #print("err_position: %0.2f, err_angle: %0.2f" % (err_p,err_a))
            st=standardize(a*err_p+b*err_a)
            sm=-0.05
            if abs(st-last_st)>=0.25:
                sm=0
            d.setStatus(motor = sm, servo = st, mode="speed")
            print("Motor: %0.2f, Servo: %0.2f" % (sm,st))
            last_st=st
            time.sleep(0.4)
        else:
            d.setStatus(motor=0,servo=0,mode="stop")
            #print "error! cann't find the garage."
    
    #finally, stop the car and quit the program
    d.setStatus(motor=0.0, servo=0.0, dist=0x00, mode="stop")
    cap.release()
    cv2.destroyAllWindows()
    d.close()
    del d
    return 0

#avoid servo go beyond [-1,1]
def standardize(value):  
    if (value>1.0):
        return 1.0
    elif (value<-1.0):
        return -1.0
    else:
        return value

if __name__ == '__main__':
    kc_3f()
