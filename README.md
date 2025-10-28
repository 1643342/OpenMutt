# OpenMutt-ROS2

For missing control libraries, go to https://github.com/ros-controls/gz_ros2_control




# Requirements and Background

This repository contains all of the necessary commands and information to configure and operate the OpenMutt system.

The system is build within Ubuntu 22.04 Linux utilizing ROS2 running on a Raspberry Pi 4. Motor communication is run through the RS485 CAN HAT module. There are 12 motors, those being the MAD M6C10 200KV motors. The system was trained on IsaacLabEureka program with a custom developed [URDF].


# Hardware Setup
- 3D printed parts
- Different 3D print materials
- MAD M6C10 motors [3]
- RS485 CAN HAT module
- Rasp Pi 4


# First-time system development overview

To create your own version, the process starts with flashing a microSD with ubuntu 22.04 and installing all the required packages. The following will dictate the whole procedure for such. It is recommended to use a microSD with at least 16GB free space, where a 32GB is more suitable.


# 1. Setting up Ubuntu 22.04

The first step is to install and use the [Raspberry Pi Imager program]. For this, you will need a microSD card reader or SD-to-USB adapter to flash your microSD.

Select your chosen Raspberry Pi device. It is recommended to use at least the RPi4 with 4GB RAM to properly run the systems.

Insert your microSD into your device and select it for the storage option.

The Operating system has more steps to it than the others, so be mindful of the exact OS you will be installing. After pressing the 'Select OS' option, scroll down to find the "Other general-purpose OS" category. From here, select the "Ubuntu" category, then "Ubuntu Desktop 22.04.5 LTS (64-bit)" OS.

Proceed to flash the microSD with this exact Ubuntu 22.04 OS. Once it has finished flashing, you may remove the card and continue the process with the Raspberry Pi.


# 2. Downloading Relevant Ubuntu Systems and Tools

The system requires both the use of ROS2 and the O-drive framework to function. This section will describe how to install such packages and how to use them appropriately.

ROS2 is the easier of the two due to the straightforward nature of installing as well as the ample amount of resources to help if any problems occur. As such, it is highly recommended to follow the official install guide for ROS2 on Ubuntu 22.04, also known as ROS2 Humble. The link can be found [here][2]. Be wary of the difference between ROS and ROS2, since ROS is the older, deprecated version of the two. It is crucial that ROS2 is installed.

Installing the O-Drive framework will be complicated and convoluted due to how it was not designed to be used on a ROS2 system. Because of this, most of the work will be focused on manually identifying and creating the framework to communicate properly.






# Initial Startup Procedure
O-Drive startup:
- sudo ip link set can0 up type can bitrate 1000000
- candump can0 (Optional: confirms if CAN is communicating)
- colcon build
- source install/setup.bash (Optional: use for every new terminal instead of colcon build)


# References
[1] Raspberry Pi Imager:  https://www.raspberrypi.com/software/
[2] ROS2 for Ubuntu 22.04.5:  https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
[3] MAD M6C10 Motor Link:  https://mad-motor.com/products/mad-components-m6c10-eee?VariantsId=10490


# Contributors
Openmutt:
Bryan Gonzalez, Jeremy Niemec, Aleiya Holyoak, Zachary Nadeau, Gabriel Alkire

Locomotion 1:
Marcus Targonski, Dylan Ballback

Locomotion 2:
James Frisbie, Sam Curcuro, Andrew Vandergrift, Kellie Coppola, Harry Cardillo

Openarms/Payloads:
Avery Cuenin, Max , Manuel, Killian Embler, Elijah ,Zenat    

Eeyore:
Daniel, Colin, Andrew,

Other:
Dr. Christopher Hockley, Dr. Monica Garcia


# Thanks
Dr. Christopher Hockley, Bryan Gonzalez, Dr. Monica Garcia
