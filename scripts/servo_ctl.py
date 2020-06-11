#!/usr/bin/env python

import RPi.GPIO as GPIO
import time
import threading 
import os 
import sys
from ventservo.srv import servoconfig,servoconfigResponse
from ventservo.srv import servostate,servostateResponse
from ventservo.srv import servostatus,servostatusResponse
from std_msgs.msg import Float32
import rospy

#steps_per_revolution = 1600
#servo_angle = 90
#inspiratory_rate = .001
#expiratory_rate = .005
#inspiratory_hold = 2
#expiratory_hold = 3
motorposition = 0

# Need to fix this for Python3.x
## Config-file way of doing things
execfile(sys.argv[1])

pins = [pulsePos, directionPos, enableofflinePos]

# Set up GPIO Pins
GPIO.setmode(GPIO.BOARD)
GPIO.setup(pulsePos,GPIO.OUT)
GPIO.setup(directionPos,GPIO.OUT)
GPIO.setup(enableofflinePos,GPIO.OUT)


# Initial Pin state
GPIO.output(pulsePos,GPIO.LOW)
GPIO.output(directionPos,GPIO.HIGH)
# Setting Servo Offline/Disabled at begining. Must enable.
GPIO.output(enableofflinePos,GPIO.HIGH)

def pub_position():
    global motorposition
    global stop_threads
    pub = rospy.Publisher('servoPosition', Float32, queue_size=10)
    rate = rospy.Rate(10)
    while not True:
        pub.publish(motorposition)
        rate.sleep()
        if stop_threads:
            break


def handle_servoctl(req):
    motorenable = GPIO.input(enableofflinePos) 
    global motorposition
    global servo_state
    global steps_per_revolution
    global servo_angle
    global inspiratory_rate
    global expiratory_rate
    global inspiratory_hold
    global expiratory_hold
    currentTime = rospy.get_rostime()
    try:
        if req.type == "status":
            # Send current servo control configuration
            rospy.loginfo("SERVOCTL: Request Rx: %s",req.type)
            return servostatusResponse(servo_state,steps_per_revolution, servo_angle, inspiratory_rate, expiratory_rate,inspiratory_hold, expiratory_hold, currentTime)
        elif req.type == "configuration":
            # Apply configuration
            rospy.loginfo("SERVOCTL: Request Rx: %s",req.type)
            steps_per_revolution = req.steps_per_revolution
            servo_angle = req.servo_angle
            inspiratory_rate = req.inspiratory_rate
            req.expiratory_rate = req.expiratory_rate
            inspiratory_hold = req.inspiratory_hold
            expiratory_hold = req.expiratory_hold
            return servoconfigResponse(servo_state,steps_per_revolution, servo_angle, inspiratory_rate, expiratory_rate,inspiratory_hold, expiratory_hold, currentTime)
        elif req.type == "disable":
            # Disable Servo
            rospy.loginfo("SERVOCTL: Request Rx: %s",req.type)
            #GPIO.output(enableofflinePos,GPIO.HIGH)
            servo_state = 'disable' 
            return servostateResponse(servo_state,steps_per_revolution, servo_angle, inspiratory_rate, expiratory_rate,inspiratory_hold, expiratory_hold, currentTime)
        elif req.type == "enable":
            #Enable Servo
            rospy.loginfo("SERVOCTL: Request Rx: %s",req.type)
            #GPIO.output(enableofflinePos,GPIO.LOW)
            servo_state = 'enable'
            return servostateResponse(servo_state,steps_per_revolution, servo_angle, inspiratory_rate, expiratory_rate,inspiratory_hold, expiratory_hold, currentTime)
    except:
        rospy.loginfo("SERVOCTL: WARNING: UNKNOWN request type: %s",req.type)
        rospy.loginfo("SERVOCTL: Ignoring last request. Type: %s",req.type)

def vent_inspiratory():
    global motorposition
    global steps_per_revolution
    global servo_angle
    global inspiratory_rate
    global expiratory_rate
    global inspiratory_hold
    global expiratory_hold

    servo_steps = calcsteps(servo_angle,steps_per_revolution)

    currentTime = rospy.get_rostime()

    try:
        if motorposition != 0:
            rospy.loginfo("VENT_INSPIRATORY: Servo not in acceptable position, skipping inspiratory cycle")
            if GPIO.input(enableofflinePos):
                motorposition = 0 

            return True
        
        elif not GPIO.input(enableofflinePos):
            rospy.loginfo("VENT_INSPIRATORY: Cycle Start, servo_angle: %s, servo_steps: %s, inspiratory_rate: %s", servo_angle, servo_steps, inspiratory_rate)

            ctlstart = rospy.get_rostime()

            # Set servo direction: Foward/CW
            GPIO.output(directionPos,GPIO.HIGH)

            while motorposition < servo_steps:
                GPIO.output(pulsePos,GPIO.HIGH)
                GPIO.output(pulsePos,GPIO.LOW)
                time.sleep(inspiratory_rate)
                motorposition += 1

            rospy.loginfo("VENT_INSPIRATORY: Current servo position: %s", motorposition)

            ctlend = rospy.get_rostime()
            ctlduration = calcctlduration(ctlstart,ctlend)
            rospy.loginfo("VENT_INSPIRATORY: Cycle Complete in: %s.%s", ctlduration.secs, ctlduration.nsecs)
            return True 

        elif GPIO.input(enableofflinePos):
            rospy.loginfo("VENT_INSPIRATORY: WARNING, servo disabled, skipping inspiratory cycle.")
            time.sleep(5)
            return True 
   
        else:
            rospy.loginfo("VENT_INSPIRATORY: ERROR, servo state UNKNOWN. This is a problem, can not proceed")
            time.sleep(5)
            return False

    except:
        rospy.loginfo("VENT_INSPIRATORY: ERROR, Servo drive cycle failed.")
        time.sleep(5)
        return False

def vent_expiratory():
    global motorposition
    global steps_per_revolution
    global servo_angle
    global inspiratory_rate
    global expiratory_rate
    global inspiratory_hold
    global expiratory_hold

    servo_steps = calcsteps(servo_angle,steps_per_revolution)

    currentTime = rospy.get_rostime()

    try:
        if motorposition != servo_steps:
            rospy.loginfo("VENT_EXPIRATORY: Servo not in acceptable position, skipping expiratory cycle")
            if GPIO.input(enableofflinePos):
                motorposition = 0

            return True

        elif not GPIO.input(enableofflinePos):
            rospy.loginfo("VENT_EXPIRATORY: Cycle Start, servo_angle: %s, servo_steps: %s, expiratory_rate: %s", servo_angle, servo_steps, expiratory_rate)

            ctlstart = rospy.get_rostime()

            # Set servo direction: Reverse/CCW
            GPIO.output(directionPos,GPIO.LOW)

            while motorposition > 0:
                GPIO.output(pulsePos,GPIO.HIGH)
                GPIO.output(pulsePos,GPIO.LOW)
                time.sleep(expiratory_rate)
                motorposition -= 1

            rospy.loginfo("VENT_EXPIRATORY: Current servo position: %s", motorposition)

            ctlend = rospy.get_rostime()
            ctlduration = calcctlduration(ctlstart,ctlend)
            ctlnow = rospy.get_rostime()
            rospy.loginfo("VENT_EXPIRATORY: Cycle Complete in: %s.%s", ctlduration.secs, ctlduration.nsecs)
            return True

        elif GPIO.input(enableofflinePos):
            rospy.loginfo("VENT_EXPIRATORY: WARNING, servo disabled, skipping expiratory cycle.")
            time.sleep(5)
            return True

        else:
            rospy.loginfo("VENT_EXPIRATORY: ERROR, servo state UNKNOWN. This is a problem, can not proceed")
            time.sleep(5)
            return False

    except:
        rospy.loginfo("VENT_EXPIRATORY: ERROR, Servo drive cycle failed.")
        time.sleep(5)
        return False


def vent_inspiratoryHold():
    global inspiratory_hold

    currentTime = rospy.get_rostime()
    rospy.loginfo("VENT_INSPIRATORYHOLD: waiting before starting expiratory cycle, %s", inspiratory_hold)
    time.sleep(inspiratory_hold)
    rospy.loginfo("VENT_INSPIRATORYHOLD: hold complete.")

    return True

def vent_expiratoryHold():
    global expiratory_hold

    currentTime = rospy.get_rostime()
    rospy.loginfo("VENT_EXPIRATORYHOLD: waiting before starting inspiratory cycle, %s", expiratory_hold)
    time.sleep(expiratory_hold)
    rospy.loginfo("VENT_EXPIRATORYHOLD: hold complete.")

    return True

def servo_ctl_server():
    rospy.init_node('ventservo_server')
    rospy.Service('ventservo_srv_status', servostatus, handle_servoctl)
    rospy.Service('ventservo_srv_config', servoconfig, handle_servoctl)
    rospy.Service('ventservo_srv_state', servostate, handle_servoctl)
    rospy.loginfo("Ready to receive servo control requests.")
    

def cleanup():
    global t1
    global t2
    global stop_threads

    rospy.loginfo("Node shutdown and clean-up...")
    
    # Stop Threads
    rospy.loginfo("Stopping threads...")
    stop_threads = True
    t1.join()
    t2.join()

    # Setting Servo Offline/Disabled at begining. Must enable.
    rospy.loginfo("Releasing GPIO resources")
    GPIO.output(enableofflinePos,GPIO.HIGH)
    GPIO.cleanup()
    
    rospy.loginfo("... Complete")

def calcctlduration(ctlstart,ctlend):
    ctlduration = type('', (), {})()
    ctlstart.secs = str(ctlstart.secs)
    ctlstart.nsecs = str(ctlstart.nsecs)
    startcombined = ".".join([str(ctlstart.secs),str(ctlstart.nsecs)])
    endcombined = ".".join([str(ctlend.secs),str(ctlend.nsecs)])
    ctlduration.combined = float(endcombined) - float(startcombined)
    (ctlduration.secs,ctlduration.nsecs) = str(ctlduration.combined).split('.')
    return ctlduration

def calcsteps(angle,spr):
    steps_per_degree = spr / 360.0
    return int(steps_per_degree * angle)

def driveservo():
    global vent_cycles 
    global stop_threads

    vent_cycles = 0
    while not rospy.is_shutdown():
        vent_inspiratory()
        vent_inspiratoryHold()
        vent_expiratory()
        vent_expiratoryHold()
        if not GPIO.input(enableofflinePos):
            vent_cycles += 1
        
        if servo_state == 'disable' and not GPIO.input(enableofflinePos):
            GPIO.output(enableofflinePos,GPIO.HIGH) 

        elif servo_state == 'enable'and GPIO.input(enableofflinePos):
            GPIO.output(enableofflinePos,GPIO.LOW)


        rospy.loginfo("___ DRIVESERVO: Cycles completed: %s ___", vent_cycles)

        if stop_threads:
            break

    return
    

if __name__ == "__main__":
    global t1
    global t2
    global stop_threads

    nicevalue = os.nice(0)
    rospy.loginfo("Setting Process NICE %s", nicevalue)
    rospy.on_shutdown(cleanup)
    servo_ctl_server()
    t1 = threading.Thread(target=driveservo,name='ventservo-drive')
    t2 = threading.Thread(target=pub_position,name='ventservo-pubposition')
    stop_threads = False
    t2.start()
    t1.start()
    rospy.spin()


