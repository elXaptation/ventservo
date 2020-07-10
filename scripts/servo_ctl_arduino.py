#!/usr/bin/env python

import time
import threading
import os
import sys
from ventservo.srv import servoconfig,servoconfigResponse
from ventservo.srv import servostate,servostateResponse
from ventservo.srv import servostatus,servostatusResponse
from ventservo.srv import servort,servortResponse
from ventservo.msg import servoruntime
from std_msgs.msg import Float32
import rospy

stop_threads = False
rt_motorState = "disable"
rt_servo_state = False
rt_steps_per_revolution = 0
rt_servo_angle = 0
rt_inspiratory_rate = 0
rt_expiratory_rate = 0
rt_inspiratory_hold = 0
rt_expiratory_hold = 0
rt_motorposition = 0
rt_cycles_complete = 0
rt_publish_interval = 0
rt_telemDataTime = None
ventservo_rtsrv_config = None

# Need to fix this for Python3.x
## Config-file way of doing things
execfile(sys.argv[1])

def get_telemetry():
    global stop_threads
    #rospy.init_node('ventservo_ctl', anonymous=True)
    rospy.Subscriber("ventservo_status", servoruntime, handle_telemetry)
    rospy.loginfo("Monitoring telemetry from topic /ventservo_status.")
    while not (stop_threads):
        time.sleep(1)

    return

def handle_telemetry(tdata):
    global rt_motorState
    global rt_servo_state
    global rt_steps_per_revolution
    global rt_servo_angle
    global rt_inspiratory_rate
    global rt_expiratory_rate
    global rt_inspiratory_hold
    global rt_expiratory_hold
    global rt_motorposition
    global rt_cycles_complete
    global rt_telemDataTime
    global rt_publish_interval
    #currentTime = rospy.get_rostime()
    #rospy.loginfo(tdata)
    rt_servo_state = tdata.servo_state
    rt_steps_per_revolution = tdata.steps_per_revolution
    rt_servo_angle = tdata.servo_angle
    rt_inspiratory_rate = tdata.inspiratory_rate
    rt_expiratory_rate = tdata.expiratory_rate
    rt_inspiratory_hold = tdata.inspiratory_hold
    rt_expiratory_hold = tdata.expiratory_hold
    rt_motorposition = tdata.motor_position_steps
    rt_cycles_complete = tdata.cycles_complete
    rt_telemDataTime = tdata.currentTime
    #rospy.loginfo("%i secs, %i nsecs", tdata.currentTime.secs, tdata.currentTime.nsecs)
    rt_publish_interval = tdata.publish_interval
    if rt_servo_state:
        rt_motorState = "enable"
    else:
        rt_motorState = "disable"


def push_config(cfg_servo_state,cfg_steps_per_revolution,cfg_servo_angle,cfg_inspiratory_rate,cfg_expiratory_rate,cfg_inspiratory_hold,cfg_expiratory_hold,cfg_publish_interval):
    global ventservo_rtsrv_config
    rospy.wait_for_service('ventservo_rtsrv_config')
    ventservo_rtsrv_config(cfg_servo_state,cfg_steps_per_revolution,cfg_servo_angle,cfg_inspiratory_rate,cfg_expiratory_rate,cfg_inspiratory_hold,cfg_expiratory_hold,cfg_publish_interval)

def handle_servoctl(req):
    global rt_servo_state
    global rt_motorState
    global rt_motorposition
    global rt_servo_state
    global rt_steps_per_revolution
    global rt_servo_angle
    global rt_inspiratory_rate
    global rt_expiratory_rate
    global rt_inspiratory_hold
    global rt_expiratory_hold
    global rt_telemDataTime
    global rt_publish_interval
    currentTime = rospy.get_rostime()
    cfg_servo_state = None
    cfg_motorState = None
    cfg_steps_per_revolution = None
    cfg_servo_angle = None
    cfg_inspiratory_rate = None
    cfg_expiratory_rate = None
    cfg_inspiratory_hold = None
    cfg_expiratory_hold = None

    try:
        if req.type == "status":
            # Send current servo control configuration
            rospy.loginfo("SERVOCTL: Request Rx: %s",req.type)
            return servostatusResponse(rt_motorState,rt_steps_per_revolution, rt_servo_angle, rt_inspiratory_rate, rt_expiratory_rate, rt_inspiratory_hold, rt_expiratory_hold, currentTime)
        elif req.type == "configuration":
            # Apply configuration
            rospy.loginfo("SERVOCTL: Request Rx: %s",req.type)
            cfg_steps_per_revolution = req.steps_per_revolution
            cfg_servo_angle = req.servo_angle
            cfg_inspiratory_rate = req.inspiratory_rate
            cfg_expiratory_rate = req.expiratory_rate
            cfg_inspiratory_hold = req.inspiratory_hold
            cfg_expiratory_hold = req.expiratory_hold
            push_config(rt_servo_state,cfg_steps_per_revolution,cfg_servo_angle,cfg_inspiratory_rate,cfg_expiratory_rate,cfg_inspiratory_hold,cfg_expiratory_hold,rt_publish_interval)
            time.sleep(.5)
            return servoconfigResponse(rt_motorState,rt_steps_per_revolution, rt_servo_angle, rt_inspiratory_rate, rt_expiratory_rate,rt_inspiratory_hold, rt_expiratory_hold, rt_telemDataTime)
        elif req.type == "disable":
            # Disable Servo
            rospy.loginfo("SERVOCTL: Request Rx: %s",req.type)
            push_config(False,rt_steps_per_revolution,rt_servo_angle,rt_inspiratory_rate,rt_expiratory_rate,rt_inspiratory_hold,rt_expiratory_hold,rt_publish_interval)
            time.sleep(.5)
            return servostateResponse(rt_motorState,rt_steps_per_revolution, rt_servo_angle, rt_inspiratory_rate, rt_expiratory_rate,rt_inspiratory_hold, rt_expiratory_hold, rt_telemDataTime)
        elif req.type == "enable":
            #Enable Servo
            rospy.loginfo("SERVOCTL: Request Rx: %s",req.type)
            push_config(True,rt_steps_per_revolution,rt_servo_angle,rt_inspiratory_rate,rt_expiratory_rate,rt_inspiratory_hold,rt_expiratory_hold,rt_publish_interval)
            time.sleep(.5)
            return servostateResponse(rt_motorState,rt_steps_per_revolution, rt_servo_angle, rt_inspiratory_rate, rt_expiratory_rate,rt_inspiratory_hold, rt_expiratory_hold, rt_telemDataTime)
    except:
        rospy.loginfo("SERVOCTL: EXCEPTION: request type: %s",req.type)
        rospy.loginfo("SERVOCTL: EXCEPTION: Ignoring last request")

def servo_ctl_server():
    global ventservo_rtsrv_config
    rospy.Service('ventservo_srv_status', servostatus, handle_servoctl)
    rospy.Service('ventservo_srv_config', servoconfig, handle_servoctl)
    rospy.Service('ventservo_srv_state', servostate, handle_servoctl)
    ventservo_rtsrv_config = rospy.ServiceProxy('ventservo_rtsrv_config', servort)
    rospy.loginfo("Ready to receive servo control requests.")
    while not (stop_threads):
        time.sleep(1)

    return

def cleanup():
    global t_monitor_ventservort
    global t_service_servoctl
    global stop_threads

    rospy.loginfo("Node shutdown and clean-up...")

    # Stop Threads
    rospy.loginfo("Stopping threads...")
    stop_threads = True
    t_monitor_ventservort.join()
    t_service_servoctl.join()
    t_report_cycles.join()
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

def report_cycles():
    current_cycles = rt_cycles_complete
    rospy.loginfo("___ Cycles completed: %s ___", current_cycles)
    while (True):
        #do something
        if (rt_cycles_complete > current_cycles):
            current_cycles = rt_cycles_complete
            rospy.loginfo("___ Cycles completed: %s ___", current_cycles)
        elif (stop_threads):
            break
        else:
            time.sleep(.5)

if __name__ == "__main__":
    global t_monitor_ventservort
    global t_service_servoctl
    global t_report_cycles
    nicevalue = os.nice(0)
    rospy.loginfo("Setting Process NICE %s", nicevalue)
    rospy.on_shutdown(cleanup)
    rospy.init_node('ventservo_server')
    t_monitor_ventservort = threading.Thread(target=get_telemetry,name='ventservo_telemetry-monitor')
    t_service_servoctl = threading.Thread(target=servo_ctl_server,name='ventservo_servoctl')
    t_report_cycles = threading.Thread(target=report_cycles,name='ventservo_report-cycles')
    t_monitor_ventservort.start()
    t_service_servoctl.start()
    t_report_cycles.start()
    rospy.spin()
