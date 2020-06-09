# ventservo
Ventservo is an Emergency Ventilator motor contol package. It's created as a Robot Operating System (ROS) package. Its capable of controlling a reversible hybrid-stepper/servo motor mechanisim. I use this package to control an Ambu-bag based emergency ventilator. In theory, different types of ventilator mechanisims can be contolled if they use a reversible motor mechanisim.

## Description
Ventservos intention is to provide ventilator motor control as a service. Whereby more sophisticated robotics softwares can handle more elaborate/sophisticated aspects of running a ventilator. This is a base functionality to enable better ventilator applications. The ventservo project is writen in python and is composed of two parts.

### 1. servo_ctl.py
A self contained python script that implements the status, state and configuration servers that drive the motor movements via GPIO on the first thread and a motorposition publisher on a second thread.

### 2. motor_config_A.py
A start-up configuration file that is specified as a command line argument at rosnode start up (rosrun). The actual name doesn't matter, only that its executible by servo_ctl.py at start up. The execution of this file sets a collection of variables at run time, allowing you to switch hardware environments fairly quickly and get up and running with out editing the main code in servo_ctl.py.

Ventservo was built to run on RaspberryPi and control a hybrid closed-loop stepper/servo ,[like this one](https://www.amazon.com/gp/product/B07VK2GLFY/ref=ppx_yo_dt_b_search_asin_title?ie=UTF8&psc=1), using RPi GPIO and the rospy libs. Ventservo provides 3 services to configure, control and check status of the motor operation. It publishes motor-position in step-units to a ROS topic.

Its recommended to run the ventservo/servo_ctl.py node on a dedicated RaspberryPi. ROS core and execution of ventservo service-calls and monitor ventservo motorposition topic on a seperate ROS machines highly recommended, not on the RPi running ventservo.

Motor specifics are configurable. Inspiration and expiration rates are configurable, As well as the time interval between inspiration and expiration cycles is configurable. ventsero reads the config-file, motor_config_A.py, at start up, you can modify the configuration using the service-calls after servo_ctl.py node is running.

This current version of ventservo is all software based and hence suffers from some pit-falls of sharing a processor between direct motor control (pulsing the stepper motor driver) and other functions the operating system must do (respond to service request, etc..). Since I'm using a closed loop hybrid-stepper, the integrated encoder helps assure the motors position matches what the software thinks the motor has acchomplished. This doesn't solve the problem that the software may miss sending a pulse, however. This is why it's recommended that only servo_ctl.py node run on the dedicated RPi with GPIO connected to Hybrid-stepper/servo driver. Not a real solution, just a best practice for research purposes. Future versions (in development now) will integrate ROS-Serial-Arduino to handle the motor specific thread and avoid the problems of a shared CPU. 

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

### Assure GPIO access for non-root user
The servo_ctl.py node needs access to GPIO. I run ubuntu 18.04 on my RPi. I needed to allow a non-root user to access the /dev/gpiomem device to drive the GPIO pins. This is how I acchomplished that.

1. Create a new group called GPIO

```
$ sudo groupadd gpio
```

2. Add your chosen user to the GPIO group

```
usermod -a -G gpio [username]
```

3. Change the group ownwership of /dev/gpiomem to group GPIO

```
sudo chgrp gpio /dev/gpiomem
```

4. Modify group permissions of /dev/gpiomem to include read/write.

```
sudo chmod g+rw /dev/gpiomem
```

I've included some basic bash script to accomplish this process.

```
$ roscd ventservo/scripts
$ sudo ./setup_grp_user.sh [username]
$ sudo ./ros-fix-gpio-perms.sh 
```

To make the /dev/gpiomem settings persist after a reboot of the RPi:

```
$ sudo cp ./ros-fix-gpio-perms.sh  /etc/rc.local
```

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

- **servo_state** enable | disable are the two options. This tells servo_control to proceed with moving the motor or not. Disable, turns off the stepper/server hold voltage/current, so you can manually manipulate the drive system. Toggleing the servo_state bascially toggles the enableofflinePos pin. Important to note, every time you toggle enable/disable, motor position is reset to 0 or the begining/start. When the configuration is changed during operation between enable to disable, then servo_control will complete the current inspiration/expiration cycle and actual disable will occur when then motor position returns to 0.  

- **steps_per_revolution** Integer number of steps for a complete revolution of the stepper/servo. Default in this repo is 1600 because that's what I have the micro-stepping dip switches set to on the hybrid-stepper driver. Make sure this matches your hardware setting. This integer is used in calculations to determine the steps to acheive the angle of motion used to create the inspiration/expiration cycle.

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
Sample Output:
```
[INFO] [1590696985.741178]: Ready to receive servo control requests.
[INFO] [1590696985.755544]: VENT_INSPIRATORY: WARNING, servo disabled, skipping inspiratory cycle.
[INFO] [1590696990.769738]: VENT_INSPIRATORYHOLD: waiting before starting expiratory cycle, 0.25
[INFO] [1590696991.033979]: VENT_INSPIRATORYHOLD: hold complete.
[INFO] [1590696991.043672]: VENT_EXPIRATORY: Servo not in acceptable position, skipping expiratory cycle
[INFO] [1590696991.051887]: VENT_EXPIRATORYHOLD: waiting before starting inspiratory cycle, 0.5
[INFO] [1590696991.563512]: VENT_EXPIRATORYHOLD: hold complete.
[INFO] [1590696991.575923]: ___ DRIVESERVO: Cycles completed: 0 ___
[INFO] [1590696991.588232]: VENT_INSPIRATORY: WARNING, servo disabled, skipping inspiratory cycle.
[INFO] [1590696996.602321]: VENT_INSPIRATORYHOLD: waiting before starting expiratory cycle, 0.25
[INFO] [1590696996.862662]: VENT_INSPIRATORYHOLD: hold complete.
[INFO] [1590696996.875088]: VENT_EXPIRATORY: Servo not in acceptable position, skipping expiratory cycle
[INFO] [1590696996.883506]: VENT_EXPIRATORYHOLD: waiting before starting inspiratory cycle, 0.5
[INFO] [1590696997.394610]: VENT_EXPIRATORYHOLD: hold complete.
[INFO] [1590696997.404429]: ___ DRIVESERVO: Cycles completed: 0 ___
```

3. To stop Ventservo servo_ctl: `[ctrl]-c` ; servo_ctl will clean up threads and release GPIO resources.

```
[INFO] [1590696997.471809]: Node shutdown and clean-up...
[INFO] [1590696997.482909]: Stopping threads...
[INFO] [1590697003.240471]: Releasing GPIO resources
[INFO] [1590697003.251971]: ... Complete
```

### Start / Stop ventservo

When ventservo rosnode starts the included motor_config_A.py file disables the motor. This allows you to manually manipulate the ambu-bag compression fingers (assuming you're using an ambu-bag). You should should manually position the compression fingers to the position you want to use as 0-steps/home-position, the starting point. From this point all other step moves are counted. Maybe later a homeing function will be added. 

Additionally and importantly, make sure your servo_angle is "SAFE". Meaning you're not gonna instruct your motor to crush the compression fingers. Start with a servo_angle you know will not cause a problem, then fine tune it from there. Once you feel good with your starting point and your servo_angle you're ready to enable the motor. Ventservos servo_ctl.py provides a dedicated service to enable and disable the motor, ventservo_srv_state.

From the RPi or any ROS machine with the built ventservo package you can enable/disable the motor using rosservice.

- Enable motor, using rosservice:

```
$ rosservice call /ventservo_srv_state "type: 'enable'"
servo_state: "enable"
steps_per_revolution: 1600
servo_angle: 55.0
inspiratory_rate: 0.00200000009499
expiratory_rate: 0.00499999988824
inspiratory_hold: 0.25
expiratory_hold: 0.5
currentTime: 
  secs: 1590697391
  nsecs: 628700017
```
servo_ctl.py log output example:

```
[INFO] [1590697391.629809]: SERVOCTL: Request Rx: enable
[INFO] [1590697393.106465]: VENT_INSPIRATORYHOLD: waiting before starting expiratory cycle, 0.25
[INFO] [1590697393.370422]: VENT_INSPIRATORYHOLD: hold complete.
[INFO] [1590697393.381193]: VENT_EXPIRATORY: Servo not in acceptable position, skipping expiratory cycle
[INFO] [1590697393.390477]: VENT_EXPIRATORYHOLD: waiting before starting inspiratory cycle, 0.5
[INFO] [1590697393.903311]: VENT_EXPIRATORYHOLD: hold complete.
[INFO] [1590697393.913208]: ___ DRIVESERVO: Cycles completed: 0 ___
[INFO] [1590697393.921268]: VENT_INSPIRATORY: Cycle Start, servo_angle: 55, servo_steps: 244, inspiratory_rate: 0.002
[INFO] [1590697394.449861]: VENT_INSPIRATORY: Current servo position: 244
[INFO] [1590697394.461490]: VENT_INSPIRATORY: Cycle Complete in: 0.526379823685
[INFO] [1590697394.473154]: VENT_INSPIRATORYHOLD: waiting before starting expiratory cycle, 0.25
[INFO] [1590697394.743381]: VENT_INSPIRATORYHOLD: hold complete.
[INFO] [1590697394.754359]: VENT_EXPIRATORY: Cycle Start, servo_angle: 55, servo_steps: 244, expiratory_rate: 0.005
[INFO] [1590697396.015296]: VENT_EXPIRATORY: Current servo position: 0
[INFO] [1590697396.026372]: VENT_EXPIRATORY: Cycle Complete in: 1.48544287682
[INFO] [1590697396.033992]: VENT_EXPIRATORYHOLD: waiting before starting inspiratory cycle, 0.5
[INFO] [1590697396.542348]: VENT_EXPIRATORYHOLD: hold complete.
[INFO] [1590697396.552532]: ___ DRIVESERVO: Cycles completed: 1 ___
[INFO] [1590697396.561824]: VENT_INSPIRATORY: Cycle Start, servo_angle: 55, servo_steps: 244, inspiratory_rate: 0.002
[INFO] [1590697397.086722]: VENT_INSPIRATORY: Current servo position: 244
[INFO] [1590697397.097825]: VENT_INSPIRATORY: Cycle Complete in: 1.3933031559
[INFO] [1590697397.110746]: VENT_INSPIRATORYHOLD: waiting before starting expiratory cycle, 0.25
[INFO] [1590697397.370610]: VENT_INSPIRATORYHOLD: hold complete.
[INFO] [1590697397.381216]: VENT_EXPIRATORY: Cycle Start, servo_angle: 55, servo_steps: 244, expiratory_rate: 0.005
[INFO] [1590697398.644216]: VENT_EXPIRATORY: Current servo position: 0
[INFO] [1590697398.656183]: VENT_EXPIRATORY: Cycle Complete in: 1.26270413399
[INFO] [1590697398.666389]: VENT_EXPIRATORYHOLD: waiting before starting inspiratory cycle, 0.5
[INFO] [1590697399.177757]: VENT_EXPIRATORYHOLD: hold complete.
[INFO] [1590697399.191261]: ___ DRIVESERVO: Cycles completed: 2 ___
```

- Disable motor, using rosservice:

```
$ rosservice call /ventservo_srv_state "type: 'disable'"
servo_state: "disable"
steps_per_revolution: 1600
servo_angle: 55.0
inspiratory_rate: 0.00200000009499
expiratory_rate: 0.00499999988824
inspiratory_hold: 0.25
expiratory_hold: 0.5
currentTime: 
  secs: 1590697404
  nsecs: 107548952
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
servo_state: "enable"
steps_per_revolution: 1600
servo_angle: 55.0
inspiratory_rate: 0.00200000009499
expiratory_rate: 0.00499999988824
inspiratory_hold: 0.25
expiratory_hold: 0.5
currentTime: 
  secs: 1590697538
  nsecs: 954833030
```

### monitor motor position
Ventservo servo_ctl has a dedicated thread to publish the current motor position in steps from 0 (starting point when motor was enabled) on the ROS topic /servoPosition. The publisher is currently hard coded to 10hz.

```
$ rostopic echo /servoPosition
data: 143.0
---
data: 123.0
---
data: 104.0
---
data: 84.0
---
data: 65.0
---
data: 45.0
---
data: 26.0
---
data: 6.0
---
data: 0.0
---
data: 0.0
---
data: 0.0
---
data: 0.0
---
data: 0.0
---
data: 5.0
---
data: 52.0
---
data: 99.0
---
data: 147.0
---
data: 194.0
---
data: 241.0
---
data: 244.0
---
data: 244.0
---
data: 244.0
---
data: 227.0
---
data: 208.0
---
data: 188.0
---
data: 169.0
---
data: 149.0
---
data: 130.0
---
data: 110.0
---
data: 91.0
---
data: 71.0
---
data: 52.0
---
data: 32.0
---
data: 13.0
---
data: 0.0
---
data: 0.0
---
data: 0.0
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
  - Make on the fly config param changes work predictably without need to disable/enable.
  - Make config parameter float numbers unsigned. For now don't use negative values, don't know what it'll do.
  
