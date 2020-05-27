# ventservo
Ventservo is Emergency Ventilator motor contol package. It's created as a Robot Operating System (ROS) package. Its focused on controlling a reversible hybrid-stepper/servo motor mechanisim. I've used this package to control an Ambu-bag based emergency ventilator. In theory, different types of ventilator mechanisims can be contolled if they use a reversible motor mechanisim.

## Description
Ventservos intention is to provide motor control as a service. Whereby more sophisticated robotics softwares can handle more elaborate/sophisticated aspects of running a ventilator. This is a base functionality to enable better ventilator applications.

Ventservo was built to run on RaspberryPi and control a hybrid closed-loop servo ,[like this one](https://www.amazon.com/gp/product/B07VK2GLFY/ref=ppx_yo_dt_b_search_asin_title?ie=UTF8&psc=1), using GPIO and the rospy libs. Ventservo provides 3 services to configure, control and check status of the motor operation. It publishes motor-position in step-units to a ROS topic.

Its recommended to run the ventservo/servo-ctl node on a dedicated RaspberryPi. ROS core and execution of ventservo service-calls and monitor servovent motorposition topic on a seperate ROS machines, not the RPI running servovent.

Motor specifics are configurable. Inspiration and expiration rates are configurable, As well as the time interval between inspiration and expiration cycles is configurable. Servovent reads a config-file at start up, you can modify the configuration using the service-calls type:configuration (examples later).

This current version of ventservo is all software based and hence suffers from some pit-falls of sharing a processor between direct motor control and other functions the operating system must do. Future versions (in development now) will integrate ROS-Serial-Arduino to handle the motor specific and avoid the problems of a shared CPU. 


## Disclaimer
This software is experimental. The associared hardware is experimental. I do not offer any guarantees on the worthiness or readiness of this software and the associated hardware to be used in healthcare or by people in any application. Use at your own risk. 

## Background
I'm a independent researcher in the 3D printing, electronics and software infrastrcuture areas. Ventilators are not my buisness, I'm just interested in them and all the skills needed to bring one together. 

I needed to write ventservo because I needed to control the motor on [my 3D printable version](https://www.thingiverse.com/thing/xxxxxx) of the [MIT E-Vent project v2.0 T-Slot](https://e-vent.mit.edu). I was so impressed with the MIT design. I wanted a 3D printable version. Once I had that, I needed to test the mechanics of my version. I decided to use a closed-loop hyprid-stepper motor. The physical aspects of a 12NM hold stepper did not plug right in to the MIT design. Hence the first version being all software based versus the better mcu based. A ROS-Serial-Arduino version in the works now.

I'm releasing this software in hopes that other can beneifit from my work in their opensource projects, that those that find this usefull contribute enhancements and improvements back to this repo, and to get feedback from other more experienced coders and ventilator designers.

## How to use

### Hardware and wiring
[Ventservo example wiring diagram](docs/wiring_diagram.png)

[Printable Emergency Ventilator](https://www.thingiverse.com/thing/xxxxxx)

### Get the ventservo ROS Pkg
1. Navigate to catkin work space root

```
$ cd /your/path/to/catkin_ws
```

2. Change directory to your source-files directory

```
$ cd src
```

3. Create the ventservo and change directory 

```
$ mkdir ventservo
$ cd ventservo
```

4. Clone the Ventservo repo.

```
$ git clone git@github.com:elXaptation/ventservo.git
$ ls
CMakeLists.txt  LICENSE  package.xml  scripts  srv
```

### Build the ventservo ROS Package
1. Change directories to your catkin workspace root

```
$ cd /path/to/your/catkin_ws
```

2. Make the ventservo package

```
$ catkin_make 
```

### Ventservo motor configuration file for startup settings.

1. Change directories to the ventservo scripts directory

```
$ pwd
/path/to/your/catkin_ws

$ cd src/ventservo/scripts
$ ls
motor_config_A.py  vent_disable.sh  vent_fast.sh
servo_ctl.py       vent_enable.sh   vent_slow.sh

$ cat motor_config_A.py
#!/usr/bin/python
## PINS
pulsePos = 37
directionPos = 35
enableofflinePos = 33
## Motion Control
servo_state = "disable"
steps_per_revolution = 1600
servo_angle = 55
inspiratory_rate = .002
expiratory_rate = .005
inspiratory_hold = .25
expiratory_hold = .5
```
motor_config_A.py is passed as a commandline argument to rosrun when the ventservo node is started. This file configures the ventservo/servo_ctl.py node at start up. By default the servo is disabled, this is recommended. A seperate step will start it up, you can change the default behavior if you'd like and you know  it's safe. 

Three GPIO pins are used:
- **pulsePos** is where step pulses are sent to the stepper/servo driver. GPIO.HIGH_GPIO-LOW cycle will make the stepper move one step.

- **directionPos** indicates CW or CCW for motor direction. "Forward" is CW by default (GPIO.LOW). But can be modified on most stepper drivers. 

- **enableofflinePos** is the enable/disable pin. GPIO.LOW is motor enabled. GPIO.HIGH is disabled. 

Motion Control is the configuration used to manipulate the stepper/servo. Some config parameters are obvious. Some need some explination:

- **servo_state** enable | disable are the two options. This tells servo_control to proceed with moving the motor or not. Disable, turns off the stepper/server hold voltage/current, so you can manually manipulate the drive system. Toggleing the servo_state bascially toggles the enableofflinePos pin. Important to note, every time you toggle enable/disable, motor position is reset to 0 or the begining/start. When the configuration is changed during operation between enable to disable, then servo_control will complete the current inspiration/expiration cycle and actuial disable will occur when then motor position returns to 0.  

- **steps_per_revolution** Integer number of steps for a complete revolution of the stepper/servo. Default in my repo is 1600 because that's what I have the micro-stepping dip switches set to on the hybrid-stepper driver. Make sure this matches you hardware setting. This integer is used in calculations to determine the steps to acheive the angle of motion used to create the inspiration/expiration cycle.

- **servo_angle** The angle of motion the motor moves to create the inspiration/expiration cycle. It's important that you set this properly, you can break your machine if you set the angle too large and moving parts crash. For the Ambu-bag I developed with 50 to 55 degrees was a complete compression. 

- **inspiratory_rate** Unsigned float number in seconds representing the inter-step interval for a given angle of motion. With this parameter you can determine the speed at which the motor mechanisim compresses the Ambu-bag.

- **expiratory_rate** Unsigned float number in seconds representing the inter-step interval for a given angle of motion. With this parameter you can determine the speed at which the motor mechanisim releases the Ambu-bag.

- **inspiratory_hold** Unsigned float number in seconds representing the interval between the end of the inspiratory (compression) cycle and the expriatory (release) cycle.

- **expiratory_hold** Unsigned float number in seconds representing the interval between the end of the expiratory (release) cycle and the inspriatory (compression) cycle.

### Start/stop ventservo

### Check status of ventservo

### monitor motor position

### Modify ventservo configuration


##  TODOs
  - Migrate to Python3 and test it.
  - Test using ROS catkin build instead of catkin_make
  - Extend servo_ctl to use ROS-Serial for motor-drive functionality/thread (drive motor with arduino, not RPi CPU)
  - Enhance inspiration/expiration logic for non-linear stepping rates(curves).
