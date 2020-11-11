# -*- coding: utf-8 -*-
"""
Created on Sun May 06 16:50:15 2018

@author: Yunpeng
"""
import cv2
import numpy as np

#old line function
 #handle input image to a binary image
def img_process(roi):
    kernel=np.ones((5,5),np.uint8)
    blur_img=cv2.medianBlur(roi,5)
    gray=cv2.cvtColor(blur_img,cv2.COLOR_BGR2GRAY)
    ret,thresh = cv2.threshold(gray,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    for i in range(3):
	thresh=cv2.morphologyEx(thresh,cv2.MORPH_CLOSE,kernel)
    return thresh

 #find center of black line around aim point 
def find_blackline(img,cols,black_num_threshold):
    special_flag=0
    blackline_center=range(10)
    blackline_center_sum=0
    for i in range(10):
        finish_flag=False
        deal_flag=0
        white=0   #white points that have been counted
        black=0   #black points that have been counted
        black_start=0
        for j in range(cols-2):
            if (finish_flag):  
                break
            tmp=img[i,j]
            if (j==0):
                if (tmp==1):#if the first point is a black point,then center of black line is black/2.  
                   deal_flag=1
                   white += 1
                else:
                    deal_flag=2
                    black += 1
            else:         #if the first point is a white point, then center of black line is white+black/2.
                if(deal_flag==1):
                    if (tmp==255):
                        white +=1
                    else:
                        black_start=j
                        black += 1
                        deal_flag=2
                elif (deal_flag==2):
                    if (j-black_start ==10 and black>8):#mask noisy points in part
                        deal_flag=3
                    if (tmp==255):
                        deal_flag=1
                        black=0
                    else:
                        black += 1
                else:
                    if (tmp==255):
                        finish_flag=True
                    else:
                        black += 1
        if (black>black_num_threshold):    #if black points are so many, then it is a special line
            special_flag=1
        if (black==0):
            special_flag=2
        blackline_center[i]=white+black/2
        #print "black_center",i,blackline_center[i]
        blackline_center_sum += blackline_center[i]
    return special_flag,blackline_center_sum/10   #weighted average value

#new line function
def imgPreProcess(frame,row_begin,row_end):
    #image pre-processing
    blur_img=cv2.medianBlur(frame,5)
    gray=cv2.cvtColor(blur_img,cv2.COLOR_BGR2GRAY)
    ret,thresh = cv2.threshold(gray[row_begin:row_end,:],0,255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU) 
   
    # do opening operation to mask narrow gap
    kernel=np.ones((5,5),np.uint8)
    for i in range(4):
        thresh=cv2.morphologyEx(thresh,cv2.MORPH_OPEN,kernel)
    thresh=cv2.erode(thresh,kernel,iterations=1)
    return thresh

def getLineError(roi):
    try:
        #calculate central point of black part(mainly black line)
        M = cv2.moments(roi)
        #print(int(M['m10']/M['m00']))
        #return a float number between [-1,1]
        return float((M['m10']/M['m00'])/320.0 - 1)
        #return int(M['m10']/M['m00'])
    except:
        return "INVALID"  