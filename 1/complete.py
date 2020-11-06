#!/usr/bin/python
# -*- coding: utf-8 -*-

from driver import driver
import numpy as np
import cv2
import math
import matplotlib.pyplot as plt
import time
import io
from time import sleep
import picamera

#定义全局变量
pattern=1
mm=0
ss=0
d=driver()
latest_mm=0.1
latest_ss=0 
e=0
ed=0
e1=0
ei=0
u=0
flag_stop=0
ss1=0
ss2=0

pixel_rgb = np.array([[85,60,50],[75,35,130],[30,125,120],[124,107,40]])
pixel_l = np.array([[108,100,46],[161,180,46],[31,180,46],[95,140,46]])
pixel_u = np.array([[112,130,255],[169,255,255],[35,255,255],[98,190,255]])

class point:
    def __init__(self,x=0,y=0):
        self.x = x
        self.y = y


def motion_process(image_size,motion_angle):  #运动仿真
    PSF = np.zeros(image_size)  
    #print(image_size)  
    center_position=(image_size[0]-1)/2  
    #print(center_position)  
  
    slope_tan=math.tan(motion_angle*math.pi/180)  
    slope_cot=1/slope_tan  
    if slope_tan<=1:  
        for i in range(15):  
            offset=round(i*slope_tan)    #((center_position-i)*slope_tan)  
            PSF[int(center_position+offset),int(center_position-offset)]=1  
        return PSF / PSF.sum()  #对点扩散函数进行归一化亮度  
    else:  
        for i in range(15):  
            offset=round(i*slope_cot)  
            PSF[int(center_position-offset),int(center_position+offset)]=1  
        return PSF / PSF.sum()  

def wiener(input,PSF,eps,K=0.01):        #维纳滤波，K=0.01，防止运动模糊 
    input_fft=fft.fft2(input)  
    PSF_fft=fft.fft2(PSF) +eps  
    PSF_fft_1=np.conj(PSF_fft) /(np.abs(PSF_fft)**2 + K)  
    result=fft.ifft2(input_fft * PSF_fft_1)  
    result=np.abs(fft.fftshift(result))  
    return result  

def crossroad_process(img):
    height=img.shape[0]
    width=img.shape[1]
    # roi is the road condition
    roi=img[int(0.7*height):int(height),int(0.2*width):int(0.8*width)]
    gray = cv2.cvtColor(roi,cv2.COLOR_BGR2GRAY)
    # find Harris corners
    gray = np.float32(gray)
    dst = cv2.cornerHarris(gray,2,3,0.04)
    dst = cv2.dilate(dst,None)
    ret, dst = cv2.threshold(dst,0.01*dst.max(),255,0)
    dst = np.uint8(dst)

    # find centroids
    ret, labels, stats, centroids = cv2.connectedComponentsWithStats(dst)

    # define the criteria to stop and refine the corners
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.001)
    corners = cv2.cornerSubPix(gray,np.float32(centroids),(5,5),(-1,-1),criteria)

    # Now draw them
    res = np.int0(corners)
    point_num = res.shape[0]

    roi_height=roi.shape[0]
    roi_width=roi.shape[1]

    sum_y=0
    sum_x=0
    cnt=0

    for i in range(point_num):
        if ( res[i,1]<int(0.6*roi_height) and res[i,1]>int(0.4*roi_height)
         and res[i,0]<int(0.7*roi_width) and res[i,0]>int(0.3*roi_width) ):
            roi[res[i,1],res[i,0]]=[0,0,255] # red point
            sum_y=sum_y+res[i,1]
            sum_x=sum_x+res[i,0]
            cnt=cnt+1
    if cnt<=2:
        return (0,0)
    else:      #检测到交叉路口
        print "crossroad!"
        point_target_y=int(sum_y/cnt)  #单点控制
        point_target_x=int(sum_x/cnt)
        return (point_target_x-int(0.5*roi_width),roi_height-point_target_y) 
    

def calculate(width, row, img): #计算前方黑线中心
    sum_column = 0
    sum_black = 0
    for i in range(0, width):
        if img[row][i] == 0:
            sum_column += 1
            sum_black += i
    if sum_column==0:
        return point(0,0)
    else:
        average = sum_black / sum_column
        center = point()
        center.x=average
        center.y=row
        return center

def control( x1, y1, x2, y2, x3, y3):  #控制算法
    global latest_mm,latest_ss
    global ss1,ss2
    a = x1 - x2
    b = y1 - y2
    c = x1 - x3
    d1 = y1 - y3
    a1 = ((x1 * x1 - x2 * x2) + (y1 * y1 - y2 * y2)) / 2.0
    a2 = ((x1 * x1 - x3 * x3) + (y1 * y1 - y3 * y3)) / 2.0
    theta = b * c - a * d1;
    if abs(theta) < 1e-7:
        cur=0
    else:
        x0 = (b * a2 - d1 * a1) / theta;
        y0 = (c * a1 - a * a2) / theta;
        r = np.sqrt(pow((x1 - x0), 2)+pow((y1 - y0), 2))
        cur=1/r
    if x3==-320 and y3==480: 
        d.setStatus(motor=-latest_mm,servo=ss,dist=0x2ff,mode="distance")
        print back
    elif abs(x1-x3)<5:
        d.setStatus(motor=0.04,servo=0,dist=0x00,mode="speed")
    else:
        global e,ed,ek,e1,u,ei
        e=math.atan((-x3/y3)/1.57)/2.0+math.atan((-x2/y2)/1.57)/3.0+math.atan((-x1/y1)/1.57)/6.0

        ss=constrain(-0.92,0.92,e/1.4)
        if abs(ss)>0.8:
            mm=constrain(0.02,0.06,0.06)
        else:
            mm=constrain(0.02,0.06,0.04)
        latest_mm=mm
        latest_ss=ss
        print ss
        d.setStatus(motor=mm,servo=0.7*ss+0.2*ss1+0.1*ss2,dist=0x00,mode="speed")#舵角加权控制
        ss2=ss1
        ss1=ss
    time.sleep(1.5)

#上下限函数
def constrain( low, high, value ):
    return min(high, max(low, value))

def circle(img): #原图
    #灰度化
    gray=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    circles= cv2.HoughCircles(gray,cv2.HOUGH_GRADIENT,1,100,param1=100,param2=30,minRadius=25,maxRadius=30)#检测停止符标识
    if circles is None or len(circles[0])!=1:
        return 0
    else:
        return 1
    
def stop_continue(img):  #判断是否有停止符
    flag1=circle(img)
    if flag1==1:
        #停止
        d.setStatus(mode="stop")
        #进入倒车模式
        global pattern
        pattern=2
    else:
        #正常巡线
        cruise(img)

def cruise(img):#原图，巡线算法
    cross=crossroad_process(img)
    if cross[0]==0 and cross[1]==0:
        #三点巡线
       normal_cruise(img)
    else:
        control_single(cross[0],cross[1])

    
def normal_cruise(img):#原图，常规的三点巡线
    img=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(img, 100, 255, cv2.THRESH_BINARY)#二值化
    height = thresh.shape[0]
    width = thresh.shape[1]
    thresh=thresh[int(0.6*height):int(height),int(0*width):int(1*width)]
    height = thresh.shape[0]
    width = thresh.shape[1]
    kernel = np.ones((11, 11), dtype=np.uint8)
    thresh = cv2.erode(cv2.dilate(thresh, kernel), kernel)
    
    
    point1 = calculate(width, 120, thresh)
    point1.x -= (width / 2)
    point1.y = height - point1.y

    point2 = calculate(width, 150, thresh)
    point2.x -= (width / 2)
    point2.y = height - point2.y

    point3 = calculate(width, 180, thresh)
    point3.x -= (width / 2)
    point3.y = height - point3.y

    control( point1.x , point1.y , point2.x , point2.y , point3.x , point3.y)  #利用前方三点进行控制

def control_single( x1, y1):  #单点控制
    d.setStatus(motor=0.08,servo=0.0,dist=0x00,mode="speed")

def hisEqulColor(img):
    ycrcb = cv2.cvtColor(img, cv2.COLOR_BGR2YCR_CB)
    channels = cv2.split(ycrcb)
    cv2.equalizeHist(channels[0], channels[0])
    cv2.merge(channels, ycrcb)
    cv2.cvtColor(ycrcb, cv2.COLOR_YCR_CB2BGR, img)
    return img

def grey_world(nimg):  #白平衡
   nimg = nimg.transpose(2, 0, 1).astype(np.uint32)  
   avgB = np.average(nimg[0])  
   avgG = np.average(nimg[1])  
   avgR = np.average(nimg[2])  
 
   avg = (avgB + avgG + avgR) / 3  
 
   nimg[0] = np.minimum(nimg[0] * (avg / avgB), 255)  
   nimg[1] = np.minimum(nimg[1] * (avg / avgG), 255)  
   nimg[2] = np.minimum(nimg[2] * (avg / avgR), 255)  
   return  nimg.transpose(1, 2, 0).astype(np.uint8)

def ComputeCenter(img): #遍历像素点，计算车库中心
    pixel_x=0
    pixel_y=0
    pixel_num=0
    for i in range(0,img.shape[0]):
        for j in range(0,img.shape[1]):
            if img[i,j]==0:
                pixel_y=pixel_y+i
                pixel_x=pixel_x+j
                pixel_num=pixel_num+1
    center_x=int(pixel_x/pixel_num)
    center_y=int(pixel_y/pixel_num)
    return center_x,center_y

def recog(num,img,pixel_l,pixel_u):  #识别车库
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, pixel_l[num-1]-35,pixel_u[num-1]+35)
    res = cv2.bitwise_not(mask)   #颜色过滤
    img1 = res

    blur = cv2.medianBlur(res,5) #中值滤波
    kernel = np.ones((5,5),np.uint8)  
    img_close = cv2.erode(cv2.dilate(blur, kernel), kernel)
    ret, img_close= cv2.threshold(img_close, 127, 255, cv2.THRESH_BINARY)
    img_close1 = img_close
    _, contours, hierarchy = cv2.findContours(img_close, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)  #轮廓检测
    c_max = []

    if len(contours)>1:
        for j in range(len(contours)):  
           cnt = contours[j]  
           area = cv2.contourArea(cnt)
           if(area < (img.shape[0]*img.shape[1]/100)):
               c_max.append(cnt)
        cv2.drawContours(img_close,c_max, -1, (0,0,0),25)  

        roiSize = cv2.contourArea(contours[1])
        if roiSize > 0.6*img.shape[0]*img.shape[1]:
            center_x,center_y=ComputeCenter(img_close)
        else:
            rect = cv2.minAreaRect(contours[1])  #矩形拟合
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            cv2.drawContours(res, [box], 0, (0, 0, 255), 2)

            center_x = (box[0][0]+box[2][0])/2
            center_y = (box[0][1]+box[2][1])/2

    if len(contours)==1:
        center_x,center_y=ComputeCenter(img_close)
         
    edges = cv2.Canny(img_close, 50, 150, apertureSize = 3)
    lines = cv2.HoughLines(edges,7,np.pi/180,118)  #霍夫变换检测直线
    
    k=0
    num=0
    total_k=0
    
    if type(lines)!=np.ndarray:
        k=10000
    elif len(lines)>0:  #斜率判断分类
        for i in range(0,len(lines)):
            for r,theta in lines[i]:
              if theta==0:
                  theta=0.01
              k=-np.cos(theta)/np.sin(theta)
              if abs(k)<2 :
                  num=num+1
                  total_k=total_k+k
        k=total_k/num
     
    center_x=center_x-img.shape[1]/2  
    center_y=img.shape[0]-center_y
    return (center_x,center_y,k)

def backward(x1, y1, k):  #倒车入库算法
    if k==10000 or y1<=200:
        e=x1/250.0
        mm=constrain(-0.005,-0.08,-0.01)
        ss=constrain(-0.9,0.9,e)
        d.setStatus(motor=mm,servo=ss,dist=0x0ff,mode="distance")
    elif k!=10000 and y1>150:
        if abs(k)<0.05:
            e=-k/0.5+x1/250.0
        elif abs(k)<0.1 and abs(k)>=0.5:
            e=-k/0.6+x1/250.0
        else:
            e=-k/0.8+x1/250.0
        mm=constrain(-0.005,-0.08,-0.01)
        ss=constrain(-0.9,0.9,e)
        d.setStatus(motor=mm,servo=ss,dist=0x0ff,mode="distance")
    return 0

#判断是否停止
def judge(x1,y1,k):
    if y1<40:
        d.setStatus(motor=0.0, servo=0.0, dist=0x00, mode="stop")
        global flag_stop
        flag_stop=1
    else:
        backward(x1,y1,k)
    return 0


cap=cv2.VideoCapture(1)  #开前置摄像头巡线
while (1):
    for i in range(40):
        ret, frame = cap.read()

    (h, w) = frame.shape[:2]
    center = (w // 2, h // 2)
    M = cv2.getRotationMatrix2D(center, 180, 1.0)
    initial_img = cv2.warpAffine(frame, M, (w, h))
    #转化为灰度图

    image = cv2.cvtColor(initial_img, cv2.COLOR_BGR2GRAY)
    image = cv2.blur(image,(5,5)) #均值滤波

    stop_continue(initial_img)
    if pattern==2:
      break
cap.release()

while(1):  #开后置摄像头倒车
    cap = cv2.VideoCapture(0)
    ret, frame = cap.read()
    cap.release()
    initial_img=frame
    initial_img=grey_world(initial_img)
    x1,y1,k=recog(2,initial_img,pixel_l,pixel_u)
    cap=cv2.VideoCapture(1)
    judge(x1,y1,k)
    if flag_stop==1:
        break
  
d.close()
del d
cv2.destroyAllWindows()
