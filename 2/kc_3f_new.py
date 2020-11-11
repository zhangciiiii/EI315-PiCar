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

k_p_servo=-1.05  #when following the black line, control the servo in P control and set motor as a constant value  
thresh_r=35

def kc_3f_new():
    cap = cv2.VideoCapture(1) #open forward camera
    for i in range(15): #mask pictures with bad light condition
        ret,frame=cap.read()
        cv2.waitKey(100)
    d=driver()
    d.setStatus(motor=0,servo=0,mode="speed")
    
    #first stage: following black line
    sm,st=0.0,0.0
    last_st=st
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
        
        thresh=imgPreProcess(dst,435,475)
        #cv2.imshow("thresh",thresh)
        #cv2.waitKey(2)
        #choose three aim points and claculate three errors between [-1,1]  
        err1=getLineError(thresh[0:10,:])# aim point1 that is most further
        err2=getLineError(thresh[10:25,:])# aim point2 that is further
        err3=getLineError(thresh[25:40,:])# aim point3 that is most close
        #control the car
        if (isinstance(err1,float) and isinstance(err2,float) and isinstance(err3,float)):
            st=standardize(k_p_servo*(0.35*err1 + 0.35*err2 +0.3 *err3))
            sm=0.09
        elif (isinstance(err1,str) and isinstance(err2,float) and isinstance(err3,float)):
            st=standardize(k_p_servo*(0.55*err2 +0.45 *err3))
            sm=0.08
        elif (isinstance(err1,str) and isinstance(err2,str) and isinstance(err3,float)):
            st=standardize(k_p_servo*err3)
            sm=0.07
        else:
            st=last_st
            sm=0.06
        #filtering
        if  abs(st-last_st)>0.5:
            sm=0
        d.setStatus(motor = sm, servo = st)
        print("Motor: %0.2f, Servo: %0.2f" % (sm,st))
        last_st=st
        time.sleep(0.15)
    
    #second stage: move towards the stop sign until the distance between car and sign is small enough
    while True:
        ret, frame = cap.read()
        #reverse image to get the right perspective
        rows,cols,dimension=frame.shape           
        M=cv2.getRotationMatrix2D((cols/2,rows/2),180,1)
        dst=cv2.warpAffine(frame,M,(cols,rows))
        d_sign=readStopSign(dst)
        if not d_sign['error'] and d_sign['detect']:#if detect is succeed and error is not occurred
            r,x=d_sign['size'],d_sign['x']
            if r>=thresh_r:       
                d.setStatus(motor=0.0, servo=0.0, dist=0x00, mode="stop")
                break
            else:
                st,sm=standardize(1.0-x/320.0),0.03  #moving toward the sign
                d.setStatus(motor = sm, servo = st, mode="speed")
                print("Motor: %0.2f, Servo: %0.2f" % (sm,st))
                time.sleep(0.4)
        else:
            d.setStatus(motor=0,servo=0,mode="stop")
            #print "error! cann't find the stop sign."
    cap.release()
    
    #third stage: open back camera and park the car to specific garage
    cap = cv2.VideoCapture(0) #open back camera
    for i in range(15): #mask pictures with bad light condition
        ret,frame=cap.read()
        cv2.waitKey(100)
	
    a,b=1.0,0.25
    last_st=0
    index=4

    #adjust the car's orientation for parking, cycle times need to be adjusted.
    
    while True:
        ret, frame = cap.read()
        result=detectGarage(frame,index)
        if not result['error']:
            if result['stop']:
                d.setStatus(motor=0.0, servo=0.0, dist=0x00, mode="stop")
                break
            err_p,err_a=result['err_position'],result['err_angle']
			#segmented weighting parameters
            if result['status']==1:
                a,b=1.0,0.25
            else:
                a,b=1.4,0.27
            st=standardize(a*err_p+b*err_a)
            sm=-0.05
			#filtering
            if abs(st-last_st)>=0.3:
                sm=0
            d.setStatus(motor = sm, servo = st, mode="speed")
            print("Motor: %0.2f, Servo: %0.2f" % (sm,st))
            last_st=st
            time.sleep(0.4)
        else:
            d.setStatus(motor=0,servo=0,mode="stop")
            print "error! cann't find the garage."
    
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
    kc_3f_new()
