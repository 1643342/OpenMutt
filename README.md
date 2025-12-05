# OpenMutt-ROS2

For missing control libraries, go to https://github.com/ros-controls/gz_ros2_control


# Requirements and Background

This repository contains all of the necessary commands and information to configure and operate the OpenMutt system.

The system is build within Ubuntu 22.04 Linux utilizing ROS2 running on a Raspberry Pi 4. Motor communication is run through the RS485 CAN HAT module. There are 12 motors, those being the MAD M6C10 200KV motors. The system was trained on IsaacLabEureka program with a custom developed [URDF][4].


# Hardware Setup
- 3D printed parts
- Different 3D print materials
- [MAD M6C10 motors][3]
- [RS485 CAN HAT module][5]
- [Rasp Pi 4][6]
- [O-Drive S1][7]
- [Jetson Nano][8]


# First-time system development overview

To create your own version, the process starts with flashing a microSD with ubuntu 22.04 and installing all the required packages. The following will dictate the whole procedure for such. It is recommended to use a microSD with at least 16GB free space, where a 32GB is more suitable.


## 1. Setting up Ubuntu 22.04

The first step is to install and use the [Raspberry Pi Imager program][1]. For this, you will need a microSD card reader or SD-to-USB adapter to flash your microSD.

![Raspberry Pi Imager](https://github.com/JamesFrisbie/OpenMutt/blob/patch-2/Images/raspPi1.PNG "Raspberry Pi Imager")

Select your chosen Raspberry Pi device. It is recommended to use at least the RPi4 with 4GB RAM to properly run the systems.

![Select Raspberry Pi](https://github.com/JamesFrisbie/OpenMutt/blob/patch-2/Images/raspPi2.PNG "Select Rasperry Pi")

Insert your microSD into your device and select it for the storage option.

![Select Storage Option](https://github.com/JamesFrisbie/OpenMutt/blob/patch-2/Images/raspPi3.PNG "Select Storage Option")

The Operating system has more steps to it than the others, so be mindful of the exact OS you will be installing. After pressing the 'Select OS' option, scroll down to find the "Other general-purpose OS" category. 

![Select OS](https://github.com/JamesFrisbie/OpenMutt/blob/patch-2/Images/raspPi4.PNG "Select OS")

From here, select the "Ubuntu" category, then "Ubuntu Desktop 22.04.5 LTS (64-bit)" OS.

![Ubuntu Category](https://github.com/JamesFrisbie/OpenMutt/blob/patch-2/Images/raspPi5.PNG "Ubuntu Category")

![Select Ubuntu 22.04.5](https://github.com/JamesFrisbie/OpenMutt/blob/patch-2/Images/raspPi6.PNG "Select Ubuntu 22.04.5 LTS 64-Bit")

Proceed to flash the microSD with this exact Ubuntu 22.04 OS by hitting the "next" button. Once it has finished flashing, you may remove the card and continue the process with the Raspberry Pi.


# 2. Downloading Relevant Ubuntu Systems and Tools

The system requires both the use of ROS2 and the O-drive framework to function. This section will describe how to install such packages and how to use them appropriately.

## Installing ROS2 Humble 
ROS2 Humble, the version for Ubuntu 22.04, has an [official installation guide][2] that will be echoed in the following steps. Be wary of the difference between ROS and ROS2, since ROS is the older, deprecated version of the two. It is **crucial** that the correct version of ROS2 is installed as well, that being the Humble Hawksbill.

### Step 1: Set Locale
When copying these codes, make sure to press Ctrl+Shift+V to paste within the terminal window.
If you choose to type everything by hand, keep in mind that the commands are *case-sensitive*.
```
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```
### Step 2: Setup Sources
Enable the Ubuntu Universe repository.
```
sudo apt install software-properties-common
sudo add-apt-repository universe
```
Install the ros2-apt-source package and configure ROS2 repositories.
```
sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
```
### Step 3: Install ROS2 Packages
Update the apt repository caches.
```
sudo apt update
sudo apt-get update
```
Ensure system is up to date.
```
sudo apt upgrade
```
Install the desktop tools (ROS, RViz, demos, tutorials).
```
sudo apt install ros-humble-desktop
```
Source the setup script.
```
source /opt/ros/humble/setup.bash
```
### Step 4: Try an example
The following will create a simple talker-listener node to confirm that the systems are installed properly.

Run the talker.
```
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker
```
***Create a new terminal***, then run the listener.
```
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_py listener
```

The system should look like this.

![Talker and Listener](https://github.com/JamesFrisbie/OpenMutt/blob/patch-2/Images/TalkerAndListener.PNG "Talker and Listener")


## Installing O-Drive and CAN Systems

### Step 1: See if CAN communication is set up
The CAN module is used to communicate with the multiple O-Drive motor controllers, where this project uses the RS485 CAN HAT module. Try to see if there is any existing CAN systems before beginning.
```
sudo apt install net-tools
ifconfig -a
```
or
```
ip link show
```
Look for a "can0" anywhere. It will most likely not appear, but is in good practice to check either way. 

If it appears, that means your system can recognize and communicate with the CAN module. Thus, you can skip the following steps that relate to creating the CAN systems.


### Step 2: Edit config file for CAN
The config file is responsible for the startup procedure for the CAN system. In this case, it will be edited to add a name and communication frequency to the CAN system.
```
sudo gedit /boot/firmware/config.txt
```
A text file should open with plenty of text in it already.

![config file open](https://github.com/JamesFrisbie/OpenMutt/blob/patch-2/Images/ConfigFile1.PNG "config file open")

Copy the following text to **Line 17** of the file.
```
dtoverlay=mcp2515-can0,oscillator=12000000,interrupt=25,spimaxfrequency=2000000
```

The result should look like this:

![config file line 17](https://github.com/JamesFrisbie/OpenMutt/blob/patch-2/Images/ConfigFile2.PNG "config file Line 17")

Make sure to rerun the "sudo gedit ..." command again to see if the text file properly saved.

For the edit to come into effect, the raspberry pi needs to be reset. Run the following code to do so.
```
sudo reboot
```

After rebooting, run the following line of code to confirm if everything is working as intended.
```
sudo dmesg | grep -i '\(can\|spi\)'
```


### Step 3: Install CAN systems
Install CAN utilities.
```
sudo apt install can-utils
sudo ip link set can0 up type can bitrate 1000000
candump can0
```
If working correctly, the command 'candump can0' should be posting encoded motor information to the terminal. To end this, press Ctrl+C to terminate the command and stop the candump.

Create the Python nodes.
```
sudo apt install python3-can
pip3 install can
```
and then creat the ROS2 Python package.
```
ros2 pkg create --build-type ament_python can_ros2_example
```
Build the package in a new terminal to run the nodes and confirm if they work. You can create a new terminal by pressing Ctrl+Alt+T.
```
colcon build
source install/setup.bash
```
Create a new window and run these commands for the talker.
```
source install/setup.bash
ros2 run can_ros2_example can_talker
```
...and for the listener.
```
source install/setup.bash
ros2 run can_ros2_example can_listener
```

This is a similar step to before, where a talker and listener will confirm that the systems are installed properly. If so, then the output will appear the same as before, and it is safe to close both windows at any time.


# JETSON Software
The Jetson is the microcontroller component responsible for housing the CHAMP controls system. This component is seperate from the Raspberry Pi and is necessary for its increased performance, which is critical in running the resource-intensive controls system.

This section is for installing all JETSON related systems onto the JETSON. This includes Ubuntu 22.04/JetPack SKD 6, ROS2, OpenMutt Repository, and CHAMP controller systems.

## Install JETSON [JetPack SDK 6][9]
The JETSON has its [own version][10] of Ubuntu 22.04 that is specially curated to the JETSON. As such, it has a very similar, yet slightly different download process to the Raspberry Pi.

### Flash the microSD card using an imager.
Use the following [Imager][11] to flash the microSD card. This is the same process for flashing the Ubuntu 22.04 systems.

The imager should look as follows:
![JetPack SDK 6](https://github.com/JamesFrisbie/OpenMutt/blob/patch-2/Images/JetPack6.PNG "JetPack SDK 6")

Select the option "Jetson Linux (L4T) 24.04 LTS". The process for flashing the microSD card is the same as with the Raspberry Pi from this point on.

### Install ROS2 Systems
Refer to the process dictated before for downloading ROS2 on Ubuntu.

### Clone the OpenMutt Repository
In order to get the tools and systems required to run the OpenMutt, it is necessary to clone the repo for OpenMutt.
```
sudo gh repo clone 1643342/OpenMutt
```

### Format systems
```
colcon build 
source install/setup.bash
ros2 launch champ_config bringup.launch.py rviz:=true
```

Run the following code **_ON THE RASPBERRY PI_** to operate the joints. This line gives control over the joint positions and can be cancelled with Ctrl+C.
```
ros2 launch champ_teleop teleop.launch.py
```

After confirming the champ materials were installed, install the additional required systems.
```
sudo apt install ros-humble-nav2-bringup
sudo apt install ros-humble-ros-gz-sim
sudo apt install ros-humble-ign-ros2-control
```

# Setting up JETSON
The following is the required steps to properly connecting the JETSON to the Raspberry Pi.

Connect the JETSON to the Raspberry Pi through the RS232 Ethernet port.
![Raspberry Pi 5 Ethernet](https://github.com/JamesFrisbie/OpenMutt/blob/patch-2/Images/RaspberryPiEthernet.PNG "Raspberry Pi 5 Ethernet")

## Enable the connection through the Raspberry Pi
For this step, you will need to go through the internet settings on the Raspberry Pi. This process requires that the network IP4s to be defined.

Open the settings for Ubuntu.
![Open Settings](https://github.com/JamesFrisbie/OpenMutt/blob/patch-2/Images/network1.PNG "network1")

Go to the "Network" tab and select the settings gear.
![Network Settings](https://github.com/JamesFrisbie/OpenMutt/blob/patch-2/Images/network2.PNG "network2")

Go to the IP4 tab.
![IP4 Settings](https://github.com/JamesFrisbie/OpenMutt/blob/patch-2/Images/network3.PNG "network3")

For the three spots shown, manually set the IP to a unique address (ie: 192.168.1.105), and set netmask to 255.255.255.0, _leave gateway empty_.

Go to the previous "Network" settings tab and toggle the wifi off and on again. This will reset the connection with the new address and netmask.

## Set up wifi for JETSON
Repeat the previous step for the Jetson. Be sure to use the same netmask, but slightly modify the address (ie: 192.168.1.106). This new address will have the same first three numbers, but **MUST HAVE** a different number for the last section. This allows the two devices to communicate on the same network while maintaining unique addresses.

## Test the connection
When both the Raspberry Pi and the Jetson are set up, use the following command to test if the connection is stable and functioning.
```
ping 192.168.1.105
```

If the system is pinged, then the two systems are able to communicate properly.


# Initial Startup Procedure
O-Drive startup:
```
sudo ip link set can0 up type can bitrate 1000000
candump can0 #(Optional: confirms if CAN is communicating)
colcon build
```
Press Ctrl+C to terminate the command

Run this when you make a new terminal:
```
source install/setup.bash
```

# References
[1]: https://www.raspberrypi.com/software/

[2]: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

[3]: https://mad-motor.com/products/mad-components-m6c10-eee?VariantsId=10490

[4]: OpenMutt URDF

[5]: https://www.pishop.us/product/rs485-can-hat-for-raspberry-pi/?srsltid=AfmBOoqb5Yxrp95_a0az1asbqc6uk3X2CKgEbUejP65qDG63-N3vvCE2

[6]: https://www.raspberrypi.com/products/raspberry-pi-4-model-b/

[7]: https://shop.odriverobotics.com/products/odrive-s1

[8]: https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-nano/product-development/

[9]: https://docs.nvidia.com/jetson/jetpack/install-setup/index.html

[10]: https://developer.nvidia.com/embedded/jetpack-sdk-60

[11]: https://developer.nvidia.com/embedded/jetpack

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




