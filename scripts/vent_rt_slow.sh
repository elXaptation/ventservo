#!/bin/bash
echo "###### Modifying Arduino configuration"
rosservice call /ventservo_rtsrv_config "{servo_state: false, steps_per_revolution: 1600, servo_angle: 60.0, inspiratory_rate: 3.0, expiratory_rate: 6.0, inspiratory_hold: 250.0, expiratory_hold: 350.0, publish_interval: 50}"

