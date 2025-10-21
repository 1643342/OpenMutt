# OpenMutt-ROS2

For missing control libraries, go to https://github.com/ros-controls/gz_ros2_control

# Requirements and Background
[Insert Locomotion 2 requirements here]

[include entire life of openmutt]
- Bryan's team
- side work from individuals
- our team

# Hardware Setup
- 3D printed parts
- Different 3D print materials
- MAD M6C10 motors [1]
- RS485 CAN HAT module
- Rasp Pi 4
- 

# Initial Startup Procedure
O-Drive startup:
- sudo ip link set can0 up type can bitrate 1000000
- candump can0 (Optional: confirms if CAN is communicating)
- colcon build
- source install/setup.bash (Optional: use for every new terminal instead of colcon build)


# References and Citation
[1] https://mad-motor.com/products/mad-components-m6c10-eee?VariantsId=10490

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

