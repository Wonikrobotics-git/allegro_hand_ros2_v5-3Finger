# Allegro_Hand_ROS2_v5-3Finger

This is the official release to control Allegro Hand V5-3Finger with ROS2(**Only 3Finger supported). Mostly, it is based on the release of Allegro Hand V5 4finger ROS2 package.
You can find our latest Allegro Hand V5 4finger ROS2 package :(https://github.com/Wonikrobotics-git/allegro_hand_ros2_v5)
And the interfaces and controllers have been improved and rewritten by Soohoon Yang(Hibo) from Wonik Robotics.

**We support three additional actions just like the Allegro Hand V5 4finger does.**
- Visualize Allegro Hand V5 changing in real time using Rviz2.
- Save customize pose using MOVEIT2 and move to the saved pose.
- Simply control hand with GUI tool instead of using keyboard.

These packages are tested on ROS2 Humble(Ubuntu 22.04). It will likely not work with newer or older versions(Rolling, Foxy ...).

## Useful Links
- Official Allegro Hand Website : https://www.allegrohand.com/
- Community Forum :  https://www.allegrohand.com/forum

## Packages

**From Allegro Hand V5, the hand is fully based on torque controller.** 

- allegro_hand_controllers : Contain two main nodes for control the hand and urdf descriptions,meshes.
	- node : Receive encoder data and compute torque using `computeDesiredTorque`.
	- grasp : Apply various pre-defined grasps or customized poses based on CAN communication.
   	- 485 : Apply various pre-defined grasps or customized poses based on RS-485 communication.
- allegro_hand_driver : Main driver for sending and receiving data with the Allegro Hand.
  	- CANAPI : Drivers for CAN communication.
  	- rs485 : Drivers for RS-485 communication.
- allegro_hand_keyboard : Node that sends the command to control Allegro Hand. All commands need to be pre-defined.
- allegro_hand_moveit : Provide MOVEIT2 package for Allegro Hand V5-3Finger.
- allegro_hand_gui : Node that control the allegro hand with gui program.
- bhand : Library files for the predefined grasps and actions., available on 64 bit versions.

## Install the PCAN driver

**With ROS2, you don't need to install PCAN driver anymore!**

If you have already installed PCAN driver, please follow instructions below.
- [Optional] Disable previously installed PEAK-CAN driver.
~~~bash
# Temporarily
sudo rmmod pcan
sudo modprobe peak_usb

# Permanent
sudo rm /etc/modprobe.d/pcan.conf
sudo rm /etc/modprobe.d/blacklist-peak.conf
~~~

## Run main controller nodes

1. Make new workspace.
~~~bash
mkdir allegro_ws
~~~

2. Install necessart package.
~~~bash
sudo apt-get update
sudo apt-get install ros-<distro>-xacro
~~~

3. Download ROS2 package for Allegro Hand V5 using below command.
~~~bash
cd ~/allegro_ws
git clone https://github.com/Wonikrobotics-git/allegro_hand_ros2_v5-3Finger.git
~~~

4. Build.
~~~bash
cd ~/allegro_ws
source /opt/ros/<distro>/setup.bash
colcon build
~~~

5. Launch allegro main node.
~~~bash
source install/setup.bash
ros2 launch allegro_hand_controllers allegro_hand.launch.py
~~~
**You need to write your password to open CAN port**


**Please check 'Launch file instructions below'.**

6. Run allegro hand keyboard node.
~~~bash
cd ~/allegro_ws
source install/setup.bash
ros2 run allegro_hand_keyboards allegro_hand_keyboard
~~~

7. Control Hand using Keyboard command.

## Launch file instructions

Same as the ROS1 package, you can simply control Allegro Hand V5 by launching *allegro_hand.launch.py*.
~~~bash
ros2 launch allegro_hand_controllers allegro_hand.launch.py
~~~

Optional arguments:
~~~
VISUALIZE:=true|false (default is false)
MOVEIT:=true|false (default is false)
GUI:=true|false (default is false)
RS485:=true|false (default is false)
~~~

- If you want to visualize Allegro Hand on Rviz2:
~~~bash
ros2 launch allegro_hand_controllers allegro_hand.launch.py VISUALIZE:=true
~~~

- If you want to use Allegro Hand with MOVEIT2:
~~~bash
ros2 launch allegro_hand_controllers allegro_hand.launch.py MOVEIT:=true
~~~

- If you want to control Allegro Hand with GUI:
~~~bash
ros2 launch allegro_hand_controllers allegro_hand.launch.py GUI:=true
~~~

- If you want to control Allegro Hand through RS-485:
~~~bash
ros2 launch allegro_hand_controllers allegro_hand.launch.py RS485:=true
~~~  

## Control more than one hand

### CAN Communication
To control more than one hand, you must specify CAN port number and Allegro Hand number.

Terminal 1:
~~~bash
ros2 launch allegro_hand_controllers allegro_hand.launch.py CAN_DEVICE:=can0 NUM:=0
~~~

Termianl 2:
~~~bash
ros2 launch allegro_hand_controllers allegro_hand.launch.py CAN_DEVICE:=can1 NUM:=1
~~~

To control first hand,

Terminal 3:
~~~bash
ros2 run allegro_hand_keyboards allegro_hand_keyboard --ros-args allegroHand_0/lib_cmd:=allegroHand_0/lib_cmd
~~~

To control second hand,

Terminal 4:
~~~bash
ros2 run allegro_hand_keyboards allegro_hand_keyboard --ros-args allegroHand_0/lib_cmd:=allegroHand_1/lib_cmd
~~~

### RS-485 Communication
To control more than one hand, you must specify Serial port number and Allegro Hand number.
~~~bash
ls -l /dev/ttyUSB*
~~~

Terminal 1:
~~~bash
ros2 launch allegro_hand_controllers allegro_hand.launch.py RS485:=true PORT:=/dev/ttyUSB0 NUM:=0
~~~

Termianl 2:
~~~bash
ros2 launch allegro_hand_controllers allegro_hand.launch.py RS485:=true PORT:=/dev/ttyUSB1 NUM:=0
~~~

To control first hand,

Terminal 3:
~~~bash
ros2 run allegro_hand_keyboards allegro_hand_keyboard --ros-args allegroHand_0/lib_cmd:=allegroHand_0/lib_cmd
~~~

To control second hand,

Terminal 4:
~~~bash
ros2 run allegro_hand_keyboards allegro_hand_keyboard --ros-args allegroHand_0/lib_cmd:=allegroHand_1/lib_cmd
~~~

**These are example commands.You may need to change CAN_DEVICE, PORT and NUM arguments accroding to your system.**
## MOVEIT2 

These newly added feature function identically to their ROS1 counterparts. Please refer to the ROS1 manual for guidance.
Our latest Allegro Hand V5-3Finger ROS1 package : [ROS1](https://github.com/Wonikrobotics-git/allegro_hand_ros_v5-3Finger)

**Make sure to install MOVEIT2 first in your PC and should install several additional packages to operate it**
You can find installation guide in here : [MOVEIT2](https://moveit.picknik.ai/main/doc/tutorials/getting_started/getting_started.html)

## GUI

These newly added feature function identically to their ROS1 counterparts. Please refer to the ROS1 manual for guidance.
Our latest Allegro Hand V5 ROS1 package : [ROS1](https://github.com/Wonikrobotics-git/allegro_hand_ros_v5-3Finger)
