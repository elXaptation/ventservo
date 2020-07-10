#!/bin/bash
echo "###### Modifying vent configuration"
rosservice call /ventservo_srv_config "{type: 'configuration', steps_per_revolution: 1600, servo_angle: 55.0, inspiratory_rate: 2.0, expiratory_rate: 5.0, inspiratory_hold: 250.0, expiratory_hold: 350}"

