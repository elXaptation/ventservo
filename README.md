# ventservo
Ventservo is Emergency Ventilator motor contol package. It's created as a Robot Operating System (ROS) package. Its capable of controlling a reversible hybrid-stepper/servo motor mechanisim. I use this package to control an Ambu-bag based emergency ventilator. In theory, different types of ventilator mechanisims can be contolled if they use a reversible motor mechanisim.

## Description
Ventservos intention is to provide ventilator motor control as a service. Whereby more sophisticated robotics softwares can handle more elaborate/sophisticated aspects of running a ventilator. This is a base functionality to enable better ventilator applications. The ventservo project is writen in python and is composed of two parts.

### - servo_ctl.py
A self contained python script that implements the status, state and configuration servers that drive the motor movements via GPIO on the first thread and a motorposition publisher on a second thread.

### - motor_config_A.py
A start-up configuration file that is specified as a command line argument at rosnode start up (rosrun). The actual name doesn't matter, only that its executible by servo_ctl.py at start up. The execution of this file sets a collection of variables at run time, allowing you to switch hardware environments fairly quickly and get up and running with out editing the main code in servo_ctl.py.

Ventservo was built to run on RaspberryPi and control a hybrid closed-loop stepper/servo ,[like this one](https://www.amazon.com/gp/product/B07VK2GLFY/ref=ppx_yo_dt_b_search_asin_title?ie=UTF8&psc=1), using RPi GPIO and the rospy libs. Ventservo provides 3 services to configure, control and check status of the motor operation. It publishes motor-position in step-units to a ROS topic.

Its recommended to run the ventservo/servo_ctl.py node on a dedicated RaspberryPi. ROS core and execution of ventservo service-calls and monitor ventservo motorposition topic on a seperate ROS machines, not the RPi running ventservo.

Motor specifics are configurable. Inspiration and expiration rates are configurable, As well as the time interval between inspiration and expiration cycles is configurable. ventsero reads the config-file, motor_config_A.py, at start up, you can modify the configuration using the service-calls after servo_ctl.py node is running.

This current version of ventservo is all software based and hence suffers from some pit-falls of sharing a processor between direct motor control (pulsing the stepper motor driver) and other functions the operating system must do (respond to service request, etc..). This is why it's recommended that only servo_ctl.py node run on the dedicated RPi with GPIO connected to Hybrid-stepper/servo driver. Future versions (in development now) will integrate ROS-Serial-Arduino to handle the motor specific thread and avoid the problems of a shared CPU. 

## Disclaimer
This software is experimental. The associated hardware I refer to in this README is experimental. I do not offer any guarantees on the worthiness or readiness of this software and the associated hardware to be used in healthcare or by people in any application. Use at your own risk. 

## Background
I'm a independent researcher in the 3D printing, electronics and infrastructure software areas. Ventilators are not my buisness, I'm just interested in them and all the skills needed to bring one together. 

I needed to write ventservo because I needed to control the motor on [my 3D printable version](https://www.thingiverse.com/thing:4390796) of the [MIT E-Vent project v2.0 T-Slot](https://e-vent.mit.edu). I was so impressed with the MIT design. I wanted a 3D printable version. Once I had that, I needed to test the mechanics of my version. I decided to use a closed-loop hyprid-stepper motor. The physical aspects of a 12N.m hold stepper did not plug right in to the MIT design. So some significatn changes were made. A python motor control path was fast and easy for me. Hence the first version being all software based versus the better mcu based. A ROS-Serial-Arduino version is gonna be fun to learn and in the works now.

I'm releasing this software in hopes that others can beneifit from my work in their opensource projects, that those that find this usefull contribute enhancements and improvements back to this repo, and to get feedback from other more experienced coders and ventilator designers.

## How to use

### Hardware and wiring
[Ventservo example wiring diagram](docs/ventservo_example_wiring.png)

[Printable Emergency Ventilator](https://www.thingiverse.com/thing:4390796)

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
motor_config_A.py is passed as a commandline argument to rosrun when the ventservo/servo_ctl.py node is started. This file configures the ventservo/servo_ctl.py node at start up. By default the stepper/servo is disabled, this is recommended. A seperate step will enable the motor to move, you can change the default behavior if you'd like and you know  it's safe. 

Three GPIO pins are used:

These pins are 5VDC logic.

- **pulsePos** is where step pulses are sent to the stepper/servo driver. GPIO.HIGH_GPIO-LOW cycle will make the stepper move one step.

- **directionPos** indicates CW or CCW for motor direction. "Forward" is CW by default (GPIO.LOW). But can be modified on most stepper drivers. 

- **enableofflinePos** is the enable/disable pin. GPIO.LOW is motor enabled. GPIO.HIGH is disabled. 

Change the pins in the config file as you need. They are set up at the rosnode start and released at rosnode termination. 

Motion Control is the configuration used to manipulate the stepper/servo. Some config parameters are obvious. Some need some explination:

- **servo_state** enable | disable are the two options. This tells servo_control to proceed with moving the motor or not. Disable, turns off the stepper/server hold voltage/current, so you can manually manipulate the drive system. Toggleing the servo_state bascially toggles the enableofflinePos pin. Important to note, every time you toggle enable/disable, motor position is reset to 0 or the begining/start. When the configuration is changed during operation between enable to disable, then servo_control will complete the current inspiration/expiration cycle and actuial disable will occur when then motor position returns to 0.  

- **steps_per_revolution** Integer number of steps for a complete revolution of the stepper/servo. Default in my repo is 1600 because that's what I have the micro-stepping dip switches set to on the hybrid-stepper driver. Make sure this matches you hardware setting. This integer is used in calculations to determine the steps to acheive the angle of motion used to create the inspiration/expiration cycle.

- **servo_angle** The angle of motion the motor moves to create the inspiration/expiration cycle. It's important that you set this properly, you can break your machine if you set the angle too large and moving parts crash. For the Ambu-bag I developed with 50 to 55 degrees was a complete compression. 

- **inspiratory_rate** Float number in seconds representing the inter-step interval for a given angle of motion. With this parameter you can determine the speed at which the motor mechanisim compresses the Ambu-bag.

- **expiratory_rate** Float number in seconds representing the inter-step interval for a given angle of motion. With this parameter you can determine the speed at which the motor mechanisim releases the Ambu-bag.

- **inspiratory_hold** Float number in seconds representing the interval between the end of the inspiratory (compression) cycle and the expriatory (release) cycle.

- **expiratory_hold** Float number in seconds representing the interval between the end of the expiratory (release) cycle and the inspriatory (compression) cycle.

### Start the ventservo ROSnode
1. Start your ROScore. Ideally, ROScore is not running on the RPi you're running ventservo.
2. Start Ventservo servo_control on RPi with GPIO wired:
```
$ rosrun ventservo servo_ctl.py /path/to/your/catkin_ws/src/ventservo/scripts/motor_config_A.py
```
3. To stop Ventservo servo_ctl: `[ctrl]-c` ; servo_ctl will clean up threads and release GPIO resources.

### Start / Stop ventservo

When ventservo rosnode starts the included motor_config_A.py file disables the motor. This allows you to manually manipulate the ambu-bag compression fingers (assuming you're using an ambu-bag). You should should manually position the compression fingers to the position you want to use as 0-steps/home-position, the starting point. From this point all other step moves are counted. Maybe later a homeing function will be added. 

Additionally and importantly, make sure your servo_angle is "SAFE". Meaning you're not gonna instruct your motor to crush the compression fingers. Start with a servo_angle you know will not cause a problem, then fine tune it from there. Once you feel good with your starting point and your servo_angle you're ready to enable the motor. Ventservos servo_ctl.py provides a dedicated service to enable and disable the motor, ventservo_srv_state.

From the RPi or any ROS machine with the built ventservo package you can enable/disable the motor using rosservice.

- Enable motor, using rosservice:

```
$ rosservice call /ventservo_srv_state "type: 'enable'"
```
- Disable motor, using rosservice:

```
$ rosservice call /ventservo_srv_state "type: 'disable'"
```

I've included some basic bash scripts that accomplish the same thing.

- Enable motor using bash script:

```
$ cd /path/to/your/catkin_ws/src/ventservo/scripts
$ ./vent_enable.sh
```

- Disable motor using bash script:

```
$ cd /path/to/your/catkin_ws/src/ventservo/scripts
$ ./vent_disable.sh
```

### Check status of ventservo

Ventservos servo_ctl.py provides a dedicated service to collect configuration state, ventservo_srv_status.

```
$ rosservice call /ventservo_srv_status "type: 'status'"
```

### monitor motor position
Ventservo servo_ctl has a dedicated thread to publish the current motor position in steps from 0 (starting point when motor was enabled) on the ROS topic /servoPosition. The publisher is currently hard coded to 10hz.

```
$ rostopic echo /servoPosition
```

### Modify ventservo configuration
Besides being configured at start up, the venservo servo_ctl.py node will accept configurations changes via a dedicated service. Configuration/reconfiguration is intended to be totally on the fly, change any config parameter at any time. But, it's not quite there yet. Best/recommended practice, today, is to disable the motor, make the change and re-enable the motor to get the most predictable result. I plan to make on the fly really work in the future.

1. Disable the  motor:
```
$ rosservice call /ventservo_srv_state "type: 'disable'"
```

2. Modify the configuration:
```
$ rosservice call /ventservo_srv_config "{type: 'configuration', steps_per_revolution: 1600, servo_angle: 55.0, inspiratory_rate: 0.001, expiratory_rate: 0.002, inspiratory_hold: 0.2, expiratory_hold: 0.3}"
```
The configuration message is only valid if it contains all the configuration parameters.

3. Enable the motor:
```
$ rosservice call /ventservo_srv_state "type: 'enable'"
```

##  TODOs
  - Migrate to Python3 and test it.
  - Test using ROS catkin build instead of catkin_make
  - Extend servo_ctl to use ROS-Serial for motor-drive functionality/thread (drive motor with arduino, not RPi CPU)
  - Enhance inspiration/expiration logic for non-linear stepping rates(curves).
  - Make motor position publisher configurable.
  - Make on the fly config param changes work predictably.
  - Make config parameter float numbers unsigned. For now don't use negative values, don't know what it'll do.
  
