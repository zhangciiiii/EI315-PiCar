import cv2
from driver import driver
import numpy as np
import time

d=driver()

class point:
    def __init__(self,x=0,y=0):
        self.x = x
        self.y = y


def circle(img): #原图
    #灰度化
    gray=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    #霍夫变换圆检测
    circles= cv2.HoughCircles(gray,cv2.HOUGH_GRADIENT,1,100,param1=100,param2=30,minRadius=100,maxRadius=120)
    x=0
    y=0
    #判断是否检测到圆和圆的个数
    if circles is None or len(circles[0])!=1:  #当圆个数不为1，则认为检测错误，flag=0
        return 0,0,0,0
    elif len(circles[0])==1:  #当检测到圆为1时，计算圆心半径和坐标，并置flag=1
        circle = circles[0][0]
        x=int(circle[0])
        y=int(circle[1])
        r=int(circle[2])
        return 1,x,y,r

def cross(img):  
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(img, 50, 150, apertureSize = 3)
    lines = cv2.HoughLinesP(edges,1,np.pi/180,50,10,10)  #霍夫变换检测直线
    x_1=0
    x_2=0
    y_1=0
    y_2=0
    
    x_3=0
    x_4=0
    y_3=0
    y_4=0
    flag1=0
    flag2=0
    flag3=0
    cross_x=0
    cross_y=0
    for i in range(0,len(lines)):
        for x1, y1, x2, y2 in lines[i]:
            if abs(x2-x1)<5:  #检测到垂直直线
               x_1=x1
               x_2=x2
               y_1=y1
               y_2=y2
               flag1=1
               break
            elif abs(y2-y1)<5:  #检测到水平直线
               x_3=x1
               x_4=x2
               y_3=y1
               y_4=y2
               flag2=1
               break
            
            if x2!=x1 and y2!=y1:
               k=(y1-y2)/(x1-x2)
               if abs(k-1)<0.5 or abs(k+1)<0.5:  #检测到停止符中的斜线
                 flag=3
                 
        if flag3==1:
            break
        if flag1==1 and flag2==1:
            break
        
    if flag3==1:
        cross_x=0
        cross_y=0
    #计算垂直的和水平的线的交点坐标
    if flag1==1 and flag2==1:
        if x_1==x_2 and y_3==y_4:
            cross_x=x_1
            cross_y=y_3
        elif x_1==x_2 and y_3!=y_4:
            k=(y_4-y_3)/(x_4-x_3)
            cross_x=x_1
            cross_y=k*(cross_x-x_3)+y_3
        elif x_1!=x_2 and y_3==y_4:
            cross_y=y_3
            k=(y_2-y_1)/(x_2-x_1)
            cross_x=(cross_y-y_1)/k+x_1
        elif x_1!=x_2 and y_3!=y_4:
            k1=(y_2-y_1)/(x_2-x_1)
            k2=(y_4-y_3)/(x_4-x_3)
            cross_x=(y_1-y_3+k2*x_3-k1*x_1)/(k2-k1)
            cross_y=k2*(cross_x-x_3)+y_3
    return cross_x,cross_y

while(1):
    cap = cv2.VideoCapture(1)
    ret, frame = cap.read()
    cap.release()
    
    (h, w) = frame.shape[:2]
    center = (w // 2, h // 2)
    M = cv2.getRotationMatrix2D(center, 180, 1.0)
    frame = cv2.warpAffine(frame, M, (w, h))
    
    flag,center_x,center_y,radius=circle(frame)
    
    height,width,depth = frame.shape
    circle_img = np.zeros((height,width), np.uint8)
    cv2.circle(circle_img,(center_x,center_y),radius+10,1,thickness=-1)
    masked_frame = cv2.bitwise_and(frame, frame, mask=circle_img)   #提取圆形ROI区域
    
    crossx=0
    crossy=0
    if flag==0:
        d.setStatus(motor=0.05, servo=0.0, dist=0x00, mode="speed")  #没检测到标识前行
    elif flag==1:
        crossx,crossy=cross(masked_frame)
        if crossx==0 and crossy==0:
            d.setStatus(motor=0.0, servo=0.0, dist=0x00, mode="stop")  #停止
            break
        elif (crossx-center_x)<-10:
            d.setStatus(motor=0.3, servo=-0.8, dist=0x00, mode="speed")#根据直线交点和圆形坐标比较判断右转
            time.sleep(1)
        elif (crossx-center_x)>10:
            d.setStatus(motor=0.3, servo=0.8, dist=0x00, mode="speed")#根据直线交点和圆形坐标比较判断左转
            time.sleep(1)                
d.close()
del d
cv2.destroyAllWindows()
