#!/bin/bash
echo "###### Disableing Vent Motor"
rosservice call /ventservo_srv_state "type: 'disable'"
sleep 1
echo ""
echo "###### Modifying vent configuration"
rosservice call /ventservo_srv_config "{type: 'configuration', steps_per_revolution: 1600, servo_angle: 55.0, inspiratory_rate: 0.001, expiratory_rate: 0.002, inspiratory_hold: 0.2, expiratory_hold: 0.3}"
sleep 1
echo ""
echo "###### Enableing Vent Motor"
rosservice call /ventservo_srv_state "type: 'enable'"

