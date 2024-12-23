**Project Robohand**

This repository contains source code and other files related to the "Manipulator" project, made by a team of second-year UCU students. 
Its main purpose was to create a custom-made robohand, along with movement and detections systems, to be used for specific purposes such as screw tightening.
The exact contents are as follows:

  - The "3d_models" folder contains .stl and .FCStd files representing elements of the manipulator, which were later printed and used in the actual project;
    
  - The "YOLO_detection" folder contains files pertaining to the robohand's detection system, which is based on the "You Only Look Once" model used along with a custom set of weights;
  
  - The "ros_movements" folder contains all ros and esp configuration and scripts for robotic arm movemnts.
	  + The "src" subfolder, contains two ROS packages: **robot\_arm\_urdf**, which contains urdf model of robotic arm with some launch and configuration files and **robot\_arm\_moveit**,    which uses MoveIt Motion Planning Framework to coordinate the manipulator's movements, it also has python scripts that coordinate robotic arm to different positions.
	  + The "arduino_scripts" folder, which contains our early-stage attempts at robohand movements and now mostly used only as a reference

For further details, please refer to the project report: https://www.overleaf.com/read/xgnspjnydprb#3b3b8f
