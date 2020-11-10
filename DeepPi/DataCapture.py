# 使用方法：运行后按C开始控制，Z停止控制，WSAD为前后左右增量式加速，QE为舵机向左或右打死，X为电机置零，RF切换后置/前置摄像头Esc退出

import argparse
import datetime
import math
import time

import cv2
import imutils
import numpy as np

from driver import driver
from picamera import PiCamera
from picamera.array import PiRGBArray

# crusier parameters
binary_thr = 144 # cruise threshold
kernel_size = 3 # close operator kernel size
black_lower = np.array([  0,  0,  0], dtype = np.uint8)
black_upper = np.array([180,255, 30], dtype = np.uint8)

# signal parameters
blue_lower = np.array([100,80,140], dtype = np.uint8)
blue_upper = np.array([115,255,255], dtype = np.uint8)
min_Radius = 20
max_Radius = 80

# parking parameters
index = 2 # parking number
park_lower = np.zeros([5,3],int)
park_upper = np.zeros([5,3],int)
# 0 white [not sure]
park_lower[0] = np.array([  0,  0,200], dtype = np.uint8)
park_upper[0] = np.array([180, 50,255], dtype = np.uint8)
# 1 deep blue
park_lower[1] = np.array([89, 76, 34], dtype = np.uint8)
park_upper[1] = np.array([140, 255, 200], dtype = np.uint8)
# 2 pink
park_lower[2] = np.array([122, 80, 46], dtype = np.uint8)
park_upper[2] = np.array([180, 255, 250], dtype = np.uint8)
# 3 yellow
park_lower[3] = np.array([0, 200, 102], dtype = np.uint8)
park_upper[3] = np.array([58, 255, 200], dtype = np.uint8)
# 4 green blue
park_lower[4] = np.array([80, 190, 50], dtype = np.uint8)
park_upper[4] = np.array([110, 255, 255], dtype = np.uint8)


# control variable
err = 0.0
err_last = 0.0
kp_motor = 0.01 
kp_servo = 0.008 
kd_servo = 0.001 
min_speed = 0.05
max_speed = 0.06
control_delay = 18
loss_line = 0 # count for times of the loss of line
stop_cnt = 0 # count for times of the recognization of stop sign
stop_cnt_thr = 10 # continus times of the recognization of stop that can make sure 

camera_selection = 1 # 1 = front, 2 = rear 

# main control function: camera + control
def run_picar():
    print("==========piCar Client Start=========")
    # ==============initialization for parameters==============
    # run_picar
    d = driver()
    d.setStatus(motor=0.0, servo=0.0, dist=0x00, mode="stop")
    # state related parameters
    control_flag = 0 # car move = 1, else = 0
    motor_set = 0
    servo_set = 0
    camera_selection = 1
    while True:
        if camera_selection == 1:
            # construct the argument parser and parse the arguments
            ap = argparse.ArgumentParser()
            ap.add_argument("-v", "--video", help="path to the video file")
            args = vars(ap.parse_args())
            # if the video argument is None, then we are reading from webcam
            if args.get("video", None) is None:
                camera = cv2.VideoCapture(0)
                time.sleep(0.01)
            # otherwise, we are reading from a video file
            else:
                camera = cv2.VideoCapture(args["video"])
            print 'initialization done!\n'
            count = 0
            while True:
                t0 = time.time()
                (grabbed, frame) = camera.read()
                if not grabbed:
                    'camera bug!!!!!!!!!!!!!!'
                    break
                
                # frame = imutils.resize(frame, width=500) # to be tested................
                # rotate the image
                (h, w) = frame.shape[:2]
                # set rotation center
                center = (w/2, h/2)
                # rotate
                M = cv2.getRotationMatrix2D(center, 180, 1.0)
                img = cv2.warpAffine(frame, M, (w,h))

                cv2.imshow('img',img)

                # press a key
                key = cv2.waitKey(1) & 0xFF
                #print 'key = ',key

                # press esc to exit 
                if key == 27: 
                    d.setStatus(motor=0.0, servo=0.0, dist=0x00, mode="stop")
                    print("forced exit!!")
                    camera_selection = 0
                    break
                # change camera
                if key == ord('r'):
                    camera_selection = 2
                    break
                # control
                if key == ord('w') and motor_set<=0.95:
                    motor_set += 0.05
                if key == ord('s') and motor_set>=-0.95:
                    motor_set -= 0.05
                if key == ord('a') and servo_set<=0.95:
                    servo_set += 0.05
                if key == ord('d') and servo_set>=-0.95:
                    servo_set -= 0.05
                # special control
                if key == ord('q'):
                    servo_set = 1
                if key == ord('e'):
                    servo_set = -1
                if key == ord('x'):
                    motor_set = 0
                    servo_set = 0
                # press 'o' to save the image in the current directory
                if key == ord('o'):
                    filename1 = datetime.datetime.now().strftime("%A %d %B %Y %I-%M-%S%p")+' - origin.png'
                    cv2.imwrite(filename1,frame)
                    print ('\'',filename1,'\' saved!')

                # press 'c' or 'C' to begin the control
                if key == 67 or key == 99:
                    d.setStatus(mode="speed")
                    print("begin to control!")
                    control_flag = 1

                # press 'z' or 'Z' to stop the car
                if key == 90 or key == 122:
                    d.setStatus(mode="stop")
                    print("stop control!")
                    control_flag = 0

                # control switch
                if control_flag:
                    count = count + 1
                    if count<=control_delay:
                        # print 'count = ', count
                        continue
                    else:
                        count = 0
                        d.setStatus(motor = motor_set, servo = servo_set)
                        d.getStatus(sensor = 0, mode = 0)
                    print("Motor: %0.2f, Servo: %0.2f" % (motor_set, servo_set))
                else:
                    count = count + 1
                    if count<=control_delay:
                        continue
                    else:
                        count = 0
                        motor_set = 0
                        servo_set = 0
                        d.setStatus(motor=0.0, servo=0.0, dist=0x00, mode="stop")
                        d.getStatus(sensor = 0, mode = 0)
                #time.sleep(1)
                t1 = time.time()
                print 'time consumption = ', t1-t0

            # exit cruise and open camera
            print('cruise stop!')
            # cleanup the camera and close any open windows
            camera.release()
            cv2.destroyAllWindows()
            print('Front camera closed!')
        elif camera_selection == 2:
            # initialize the camera and grab a reference to the raw camera capture
            camera = PiCamera()
            camera.resolution = (640, 480)
            camera.framerate = 32
            camera.hflip = True
            camera.vflip = True
            camera.rotation = 180
            rawCapture = PiRGBArray(camera, size=(640, 480))
            # allow the camera to warmup
            time.sleep(0.1)
            count = 0
            # capture frames from the camera
            for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
                t0 = time.time()
                # grab the raw NumPy array representing the image, then initialize the timestamp
                # and occupied/unoccupied text
                img = frame.array
                rawCapture.truncate(0)
                cv2.imshow('img',img)

                # press a key
                key = cv2.waitKey(1) & 0xFF
                #print 'key = ',key

                # press esc to exit 
                if key == 27: 
                    d.setStatus(motor=0.0, servo=0.0, dist=0x00, mode="stop")
                    print("forced exit!!")
                    camera_selection = 0
                    break
                # change camera
                if key == ord('f'):
                    camera_selection = 1
                    break
                # control
                if key == ord('w') and motor_set<=0.95:
                    motor_set += 0.05
                if key == ord('s') and motor_set>=-0.95:
                    motor_set -= 0.05
                if key == ord('a') and servo_set<=0.95:
                    servo_set += 0.05
                if key == ord('d') and servo_set>=-0.95:
                    servo_set -= 0.05
                # special control
                if key == ord('q'):
                    servo_set = 1
                if key == ord('e'):
                    servo_set = -1
                if key == ord('x'):
                    motor_set = 0
                    servo_set = 0
                # press 'o' to save the image in the current directory
                if key == ord('o'):
                    filename1 = datetime.datetime.now().strftime("%A %d %B %Y %I-%M-%S%p")+' - origin.png'
                    cv2.imwrite(filename1,frame)
                    print ('\'',filename1,'\' saved!')

                # press 'c' or 'C' to begin the control
                if key == 67 or key == 99:
                    d.setStatus(mode="speed")
                    print("begin to control!")
                    control_flag = 1

                # press 'z' or 'Z' to stop the car
                if key == 90 or key == 122:
                    d.setStatus(mode="stop")
                    print("stop control!")
                    control_flag = 0

                # control switch
                if control_flag:
                    count = count + 1
                    if count<=control_delay:
                        # print 'count = ', count
                        continue
                    else:
                        count = 0
                        d.setStatus(motor = motor_set, servo = servo_set)
                        d.getStatus(sensor = 0, mode = 0)
                    print("Motor: %0.2f, Servo: %0.2f" % (motor_set, servo_set))
                else:
                    count = count + 1
                    if count<=control_delay:
                        continue
                    else:
                        count = 0
                        motor_set = 0
                        servo_set = 0
                        d.setStatus(motor=0.0, servo=0.0, dist=0x00, mode="stop")
                        d.getStatus(sensor = 0, mode = 0)
                #time.sleep(1)
                t1 = time.time()
                print 'time consumption = ', t1-t0
        else:
            cv2.destroyAllWindows()
            break

    # task over
    print("Task finished!!!!!")
    d.setStatus(motor=0.0, servo=0.0, dist=0x00, mode="stop")
    d.close()
    del d
    print("==========piCar Client Fin==========")
    return 0
               
    
if __name__ == '__main__':
    run_picar()
