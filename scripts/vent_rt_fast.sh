#!/bin/bash
echo "###### Modifying Arduino configuration"
rosservice call /ventservo_rtsrv_config "{servo_state: false, steps_per_revolution: 1600, servo_angle: 50.0, inspiratory_rate: 2.0, expiratory_rate: 5.0, inspiratory_hold: 200.0, expiratory_hold: 300.0, publish_interval: 50}"

