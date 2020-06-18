#!/bin/bash
echo "###### Modifying Arduino configuration"
rosservice call /ventservo_rtsrv_config "{servo_state: true, steps_per_revolution: 1600, servo_angle: 50.0, inspiratory_rate: 1.0, expiratory_rate: 3.0, inspiratory_hold: 200.0, expiratory_hold: 300.0, publish_interval: 50}"

