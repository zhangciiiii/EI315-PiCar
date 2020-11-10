#!/usr/bin/python
# -*- coding: utf-8 -*-

from driver import driver
import time


def run_picar():
    print("==========piCar Client Start==========")
    my_driver = driver()
    my_driver.setStatus(motor=0.0, servo=0.0, dist=0x00, mode="stop")
    tmp_speed = 1
    Speed_Control_Flag = True
  

    try:
        print("Test 2/2: distance mode")
        my_driver.setStatus(motor = 0.1, dist = 0x1ff00, mode="distance")
        while True:
            cap2 = cv2.VideoCapture(1)
            _, frame2 = cap2.read()
            cv2.imshow("image2", frame2)
            speed_param = 0
            my_driver.setStatus(servo = speed_param)
            print("Servo: %0.2f" % speed_param)

            my_driver.getStatus(sensor = 0, mode = 0)

    except KeyboardInterrupt:
        pass
    my_driver.setStatus(motor=0.0, servo=0.0, dist=0x00, mode="stop")
    my_driver.close()
    del my_driver
    print("==========piCar Client Fin==========")
    return 0
               
    
if __name__ == '__main__':
    run_picar()

