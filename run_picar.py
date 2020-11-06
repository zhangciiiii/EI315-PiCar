#!/usr/bin/python
# -*- coding: utf-8 -*-

from driver import driver
import time


def run_picar():
    print("==========piCar Client Start==========")
    my_driver = driver()
    my_driver.setStatus(motor=0.0, servo=0.0, dist=0x00, mode="stop")
    tmp_speed = 10
    Speed_Control_Flag = True

    # my_driver.setStatus(...)
    # Motor control:
    # motor = ...
    # forward  0 -> 1.0
    # backward 0 -> -1.0
                
    # Servo control:
    # servo = ...
    # left = mid = right
    # 1.0  = 0.0 =  -1.0

    # Distance:
    # dist = ...
    # 0 -> 0xffffffff
    
    # Mode control:
    # 1:speed
    # 2:distance
    # 3:stop

    # my_driver.getStatus(...)
    # query mode:
    # mode = any
    # sensor = any   
    ''' 
    try:
        print("Test 1/2: speed mode")
        my_driver.setStatus(mode="speed")
        while True:
            st = tmp_speed * 0.1 -1
            sm = st
            if abs(st) < 0.4:
                sm = 0.4 if st > 0 else -0.4
                if st == 0:
                    sm = 0
            my_driver.setStatus(motor = sm, servo = st)
            print("Motor: %0.2f, Servo: %0.2f" % (sm, st))
            if Speed_Control_Flag:
                tmp_speed += 1
            else:
                tmp_speed -= 1
            if tmp_speed > 20:
                Speed_Control_Flag = False
                tmp_speed -= 2
            if tmp_speed < 0:
                Speed_Control_Flag = True
                tmp_speed += 2
            time.sleep(1)
            # my_driver.heartBeat()
            my_driver.getStatus(sensor = 0, mode = 0)
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    '''
    try:
        print("Test 2/2: distance mode")
        my_driver.setStatus(motor = 0.5, dist = 0x1ff00, mode="distance")
        while True:
            speed_param = tmp_speed * 0.1 - 1
            my_driver.setStatus(servo = speed_param)
            print("Servo: %0.2f" % speed_param)
            '''
            if Speed_Control_Flag:
                tmp_speed += 1
            else:
                tmp_speed -= 1
            if tmp_speed > 20:
                Speed_Control_Flag = False
                tmp_speed -= 2
            if tmp_speed < 0:
                Speed_Control_Flag = True
                tmp_speed += 2
            '''
            time.sleep(1)
            my_driver.getStatus(sensor = 0, mode = 0)
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    my_driver.setStatus(motor=0.0, servo=0.0, dist=0x00, mode="stop")
    my_driver.close()
    del my_driver
    print("==========piCar Client Fin==========")
    return 0
               
    
if __name__ == '__main__':
    run_picar()

