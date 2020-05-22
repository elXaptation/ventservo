#!/usr/bin/python

import time

def drivemotor(GPIO, steps, interval, pinsi, motorposition):
    (pulsePos, directionPos, enableofflinePos) = pins
    #global motorposition
    print motorposition
    if steps>0:
        i=steps
        GPIO.output(directionPos,GPIO.HIGH)
        for y in range(steps,0,-1):
            GPIO.output(pulsePos,GPIO.HIGH)
            GPIO.output(pulsePos,GPIO.LOW)
            #print i
            time.sleep(interval)
            if i==0:
               return
            i=i-1
            ++motorposition

    elif steps<0:
        i=steps
        GPIO.output(directionPos,GPIO.LOW)
        for y in range(abs(steps),0,-1):
            GPIO.output(pulsePos,GPIO.HIGH)
            GPIO.output(pulsePos,GPIO.LOW)
            # print i
            time.sleep(interval)
            if i==0:
                return
            i=i+1

def statemotor(GPIO, stateRequest, pins):
    (pulsePos, directionPos, enableofflinePos) = pins
    if stateRequest == "enable":
        GPIO.output(enableofflinePos,GPIO.LOW)
        return "True"
    elif stateRequest == "disable":
        GPIO.output(enableofflinePos,GPIO.HIGH)
        return "False"
    else:
        currState = GPIO.input(enableofflinePos)
        if currState:
            # When currState is 1, enableofflinePos is high, meaning motor is disabled.
            return "False"
        else:
            return "True"

